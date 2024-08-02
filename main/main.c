#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_camera.h"
#include "usb_device_uvc.h"
#include "uvc_frame_config.h"

#define CAM_PIN_PWDN -1  //power down is not used
#define CAM_PIN_RESET -1 //software reset will be performed
#define CAM_PIN_XCLK 4
#define CAM_PIN_SIOD 33
#define CAM_PIN_SIOC 34
#define CAM_PIN_D7 18
#define CAM_PIN_D6 17
#define CAM_PIN_D5 16
#define CAM_PIN_D4 15
#define CAM_PIN_D3 14
#define CAM_PIN_D2 13
#define CAM_PIN_D1 12
#define CAM_PIN_D0 11
#define CAM_PIN_VSYNC 5
#define CAM_PIN_HREF 41
#define CAM_PIN_PCLK 42
#define LED_GPIO_NUM 21

static const char *TAG = "Test";

static camera_config_t camera_config = {
    .pin_pwdn = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sccb_sda = CAM_PIN_SIOD,
    .pin_sccb_scl = CAM_PIN_SIOC,
    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,
    .pixel_format = PIXFORMAT_JPEG,
    .frame_size = FRAMESIZE_QVGA,
    .jpeg_quality = 30,
    .fb_count = 1,
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

static esp_err_t init_camera(void) {
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera Init Failed");
        return err;
    }
    return ESP_OK;
}

static esp_err_t camera_start_cb(uvc_format_t format, int width, int height, int rate, void *cb_ctx) {
    ESP_LOGI(TAG, "Camera Start");
    return ESP_OK;
}

static void camera_stop_cb(void *cb_ctx) {
    ESP_LOGI(TAG, "Camera Stop");
}

static uvc_fb_t* camera_fb_get_cb(void *cb_ctx) {
    camera_fb_t *cam_fb = esp_camera_fb_get();
    if (!cam_fb) {
        return NULL;
    }
    static uvc_fb_t uvc_fb;
    uvc_fb.buf = cam_fb->buf;
    uvc_fb.len = cam_fb->len;
    uvc_fb.width = cam_fb->width;
    uvc_fb.height = cam_fb->height;
    uvc_fb.format = cam_fb->format;
    uvc_fb.timestamp = cam_fb->timestamp;
    return &uvc_fb;
}

static void camera_fb_return_cb(uvc_fb_t *fb, void *cb_ctx) {
    esp_camera_fb_return((camera_fb_t *)fb->buf);
}

void app_main(void) {
    if (ESP_OK != init_camera()) {
        return;
    }

    uint8_t *uvc_buffer = (uint8_t *)malloc(64 * 1024);
    if (uvc_buffer == NULL) {
        ESP_LOGE(TAG, "malloc frame buffer fail");
        return;
    }

    uvc_device_config_t config = {
        .uvc_buffer = uvc_buffer,
        .uvc_buffer_size = 64 * 1024,
        .start_cb = camera_start_cb,
        .fb_get_cb = camera_fb_get_cb,
        .fb_return_cb = camera_fb_return_cb,
        .stop_cb = camera_stop_cb,
    };

    ESP_ERROR_CHECK(uvc_device_config(0, &config));
    ESP_ERROR_CHECK(uvc_device_init());

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
