/*
 * 觉之瞳 v2 → ESP32-S3 + OV2640 人脸跟随
 * ------------------------------------------------
 * 链路：OV2640 → esp32-camera → ESP-DL HumanFaceDetect
 *       → 视觉伺服（增量式）→ LEDC PWM → MG90S ×2(/3)
 *
 * 说明：
 *   1. 摄像头 XCLK 占用 LEDC_TIMER_0 / CHANNEL_0，舵机必须用 TIMER_1，
 *      否则 esp_camera_init() 会报 ledc 冲突。
 *   2. 控制律是"增量式"：角度 += Kp * 误差 * dt。
 *      摄像头装在眼球上会随动，用"角度 = K * 误差"的位置式必然震荡。
 *   3. 先开环、后闭环；先单轴、后双轴。方向反了改 YAW_SIGN / PITCH_SIGN。
 */

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "driver/ledc.h"
#include "human_face_detect.hpp"

static const char *TAG = "face_follow";

/* ======================= 可改参数 ======================= */

/* 舵机信号脚（避开 GPIO33-37 Octal PSRAM / 19-20 USB / 43-44 UART0 / 0,3,45,46 strapping） */
#define PIN_SERVO_YAW    38
#define PIN_SERVO_PITCH  39
#define PIN_SERVO_IRIS   40   /* 第三轴（瞳孔/眼睑），不用可忽略 */

/* 舵机行程与方向（开环测试后按实际机构标定） */
static const float YAW_MIN = -45.0f, YAW_MAX = 45.0f;
static const float PITCH_MIN = -25.0f, PITCH_MAX = 25.0f;
static const int   YAW_SIGN = 1, PITCH_SIGN = 1;
static const int   SERVO_US_MIN = 500, SERVO_US_MAX = 2500;   /* MG90S 180° 行程 */

/* 视觉伺服参数（QVGA，水平 FOV ≈ 60°） */
static const float KP             = 120.0f;  /* °/s 每单位归一化误差 */
static const float DEADZONE       = 0.06f;   /* 画面半宽的 6% */
static const float MAX_RATE_DPS   = 90.0f;   /* 最大角速度 °/s */
static const float SMOOTH         = 0.35f;   /* 输出平滑（EMA） */
static const float LOST_TIMEOUT_S = 1.5f;    /* 丢脸多久后回中 */
static const float STALE_S        = 0.30f;   /* 检测结果超过多久视为过期 */

/* ======================= 共享状态 ======================= */

typedef struct {
    volatile float ex;         /* 归一化水平误差 -1..1 */
    volatile float ey;         /* 归一化垂直误差 -1..1 */
    volatile float box_w;      /* 框宽 / 画面宽 */
    volatile bool  valid;
    volatile int64_t stamp_us;
} follow_in_t;

static follow_in_t g_in;
static SemaphoreHandle_t g_lock;

/* ======================= 舵机 ======================= */

static void servo_init(void)
{
    ledc_timer_config_t t = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_14_BIT,   /* 50Hz/16384 = 1.22us 步进 */
        .timer_num       = LEDC_TIMER_1,        /* 不要用 TIMER_0（摄像头 XCLK） */
        .freq_hz         = 50,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&t));

    const int pins[3] = {PIN_SERVO_YAW, PIN_SERVO_PITCH, PIN_SERVO_IRIS};
    for (int i = 0; i < 3; i++) {
        ledc_channel_config_t c = {
            .gpio_num   = pins[i],
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel    = (ledc_channel_t)(LEDC_CHANNEL_1 + i),
            .intr_type  = LEDC_INTR_DISABLE,
            .timer_sel  = LEDC_TIMER_1,
            .duty       = 0,
            .hpoint     = 0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&c));
    }
}

static void servo_set_us(ledc_channel_t ch, int us)
{
    if (us < SERVO_US_MIN) us = SERVO_US_MIN;
    if (us > SERVO_US_MAX) us = SERVO_US_MAX;
    uint32_t duty = (uint32_t)((float)us * 16383.0f / 20000.0f);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
}

/* 角度（-90..90）→ 脉宽 */
static int ang_to_us(float ang)
{
    return 1500 + (int)(ang * 1000.0f / 90.0f);
}

static void servo_write(float yaw, float pitch, float iris_deg)
{
    servo_set_us(LEDC_CHANNEL_1, ang_to_us(yaw * YAW_SIGN));
    servo_set_us(LEDC_CHANNEL_2, ang_to_us(pitch * PITCH_SIGN));
    servo_set_us(LEDC_CHANNEL_3, ang_to_us(iris_deg));
}

/* ======================= 摄像头 ======================= */

static esp_err_t camera_init(void)
{
    camera_config_t cfg = {
        .pin_pwdn     = -1,
        .pin_reset    = -1,
        .pin_xclk     = 15,
        .pin_sccb_sda = 4,
        .pin_sccb_scl = 5,
        .pin_d7       = 16,
        .pin_d6       = 17,
        .pin_d5       = 18,
        .pin_d4       = 12,
        .pin_d3       = 10,
        .pin_d2       = 8,
        .pin_d1       = 9,
        .pin_d0       = 11,
        .pin_vsync    = 6,
        .pin_href     = 7,
        .pin_pclk     = 13,
        .xclk_freq_hz = 20000000,
        .ledc_timer   = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,
        .pixel_format = PIXFORMAT_RGB565,
        .frame_size   = FRAMESIZE_QVGA,
        .jpeg_quality = 12,
        .fb_count     = 2,
        .fb_location  = CAMERA_FB_IN_PSRAM,
        .grab_mode    = CAMERA_GRAB_WHEN_EMPTY,
    };

    esp_err_t err = esp_camera_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_camera_init failed: %s", esp_err_to_name(err));
        return err;
    }
    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        s->set_vflip(s, 0);      /* 按实际装配方向调整 */
        s->set_hmirror(s, 0);
        s->set_brightness(s, 0);
        s->set_gainceiling(s, GAINCEILING_2X);
    }
    return ESP_OK;
}

/* ======================= 视觉任务 ======================= */

static void vision_task(void *arg)
{
    HumanFaceDetect *detector = new HumanFaceDetect();
    ESP_LOGI(TAG, "detector ready");

    int64_t last_log = 0;
    int frames = 0;

    while (true) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        dl::image::img_t img = {
            .data     = fb->buf,
            .width    = (uint16_t)fb->width,
            .height   = (uint16_t)fb->height,
            .pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB565LE,
        };

        auto &results = detector->run(img);

        /* 选分数最高的一张脸 */
        const dl::detect::result_t *best = nullptr;
        float best_score = 0.0f;
        for (const auto &r : results) {
            if (r.score > best_score) {
                best_score = r.score;
                best = &r;
            }
        }

        if (best) {
            float w = (float)fb->width, h = (float)fb->height;
            float cx = (best->box[0] + best->box[2]) * 0.5f;
            float cy = (best->box[1] + best->box[3]) * 0.5f;

            xSemaphoreTake(g_lock, portMAX_DELAY);
            g_in.ex    = (cx - w * 0.5f) / (w * 0.5f);
            g_in.ey    = (cy - h * 0.5f) / (h * 0.5f);
            g_in.box_w = (best->box[2] - best->box[0]) / w;
            g_in.valid = true;
            g_in.stamp_us = esp_timer_get_time();
            xSemaphoreGive(g_lock);
        } else {
            xSemaphoreTake(g_lock, portMAX_DELAY);
            g_in.valid = false;
            xSemaphoreGive(g_lock);
        }

        esp_camera_fb_return(fb);

        frames++;
        int64_t now = esp_timer_get_time();
        if (now - last_log > 2000000) {
            ESP_LOGI(TAG, "%.1f fps, face=%d", frames * 1e6 / (now - last_log), best ? 1 : 0);
            frames = 0;
            last_log = now;
        }
        /* 检测约 37ms/帧，这里不用额外延时 */
    }
}

/* ======================= 控制任务 ======================= */

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

static void control_task(void *arg)
{
    const float dt = 0.02f;                 /* 50 Hz */
    float yaw = 0.0f, pitch = 0.0f;         /* 指令角 */
    float yaw_out = 0.0f, pitch_out = 0.0f; /* 平滑后输出 */
    float iris = 0.0f;
    bool  lost = false;
    int64_t lost_at = 0;

    TickType_t last_wake = xTaskGetTickCount();

    while (true) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(20));

        xSemaphoreTake(g_lock, portMAX_DELAY);
        float ex = g_in.ex, ey = g_in.ey, bw = g_in.box_w;
        bool  valid = g_in.valid;
        int64_t stamp = g_in.stamp_us;
        xSemaphoreGive(g_lock);

        int64_t now = esp_timer_get_time();
        bool fresh = valid && ((now - stamp) < (int64_t)(STALE_S * 1e6f));

        if (fresh) {
            lost = false;

            /* 增量式视觉伺服：死区外才动，按角速度限幅 */
            float dx = 0.0f, dy = 0.0f;
            if (fabsf(ex) > DEADZONE) {
                dx = KP * (fabsf(ex) - DEADZONE) * (ex > 0 ? 1.0f : -1.0f) * dt;
            }
            if (fabsf(ey) > DEADZONE) {
                dy = KP * (fabsf(ey) - DEADZONE) * (ey > 0 ? 1.0f : -1.0f) * dt;
            }

            float lim = MAX_RATE_DPS * dt;
            dx = clampf(dx, -lim, lim);
            dy = clampf(dy, -lim, lim);

            yaw   = clampf(yaw + dx, YAW_MIN, YAW_MAX);
            pitch = clampf(pitch + dy, PITCH_MIN, PITCH_MAX);

            /* 第三轴：脸越近（框越大）瞳孔越大，可删 */
            float t = clampf((0.5f - bw) / 0.4f, -1.0f, 1.0f);
            iris += 0.15f * (t * 45.0f - iris);
        } else {
            if (!lost) {
                lost = true;
                lost_at = now;
            }
            /* 丢脸 LOST_TIMEOUT_S 后缓慢回中 */
            if ((now - lost_at) > (int64_t)(LOST_TIMEOUT_S * 1e6f)) {
                yaw   += (0.0f - yaw) * 0.05f;
                pitch += (0.0f - pitch) * 0.05f;
                iris  += (0.0f - iris) * 0.05f;
            }
        }

        /* 输出平滑，抑制舵机抖动 */
        yaw_out   += SMOOTH * (yaw - yaw_out);
        pitch_out += SMOOTH * (pitch - pitch_out);
        servo_write(yaw_out, pitch_out, iris);
    }
}

/* ======================= 入口 ======================= */

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "觉之瞳 face-follow boot");
    g_lock = xSemaphoreCreateMutex();

    servo_init();
    servo_write(0, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(300));

    if (camera_init() != ESP_OK) {
        ESP_LOGE(TAG, "camera init failed, halt");
        return;
    }

    xTaskCreatePinnedToCore(vision_task, "vision", 12 * 1024, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(control_task, "control", 4 * 1024, NULL, 6, NULL, 0);
}
