#include <stdio.h>
#include "esp_camera.h"
#include "esp_http_server.h"
#include "nvs_flash.h"
#include "esp_log.h"

static const char *TAG = "camera_stream";

// Camera pin configuration for your ESP32-S3
#define CAM_PIN_PWDN  -1
#define CAM_PIN_RESET -1
#define CAM_PIN_XCLK  10
#define CAM_PIN_SIOD  40
#define CAM_PIN_SIOC  39
#define CAM_PIN_D7    48
#define CAM_PIN_D6    11
#define CAM_PIN_D5    12
#define CAM_PIN_D4    14
#define CAM_PIN_D3    16
#define CAM_PIN_D2    18
#define CAM_PIN_D1    17
#define CAM_PIN_D0    15
#define CAM_PIN_VSYNC 38
#define CAM_PIN_HREF  47
#define CAM_PIN_PCLK  13

#define MIN(a, b) ((a) < (b) ? (a) : (b))

static esp_err_t init_camera() {
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = CAM_PIN_D0;
    config.pin_d1 = CAM_PIN_D1;
    config.pin_d2 = CAM_PIN_D2;
    config.pin_d3 = CAM_PIN_D3;
    config.pin_d4 = CAM_PIN_D4;
    config.pin_d5 = CAM_PIN_D5;
    config.pin_d6 = CAM_PIN_D6;
    config.pin_d7 = CAM_PIN_D7;
    config.pin_xclk = CAM_PIN_XCLK;
    config.pin_pclk = CAM_PIN_PCLK;
    config.pin_vsync = CAM_PIN_VSYNC;
    config.pin_href = CAM_PIN_HREF;
    config.pin_sccb_sda = CAM_PIN_SIOD;
    config.pin_sccb_scl = CAM_PIN_SIOC;
    config.pin_pwdn = CAM_PIN_PWDN;
    config.pin_reset = CAM_PIN_RESET;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;

    // Initialize the camera
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera Init Failed");
        return err;
    }
    return ESP_OK;
}

static esp_err_t start_stream_handler(httpd_req_t *req) {
    camera_fb_t *fb = NULL;
    esp_err_t res = ESP_OK;
    res = httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=123456789000000000000987654321");
    if (res != ESP_OK) {
        return res;
    }

    while (true) {
        fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "Camera capture failed");
            res = ESP_FAIL;
        } else {
            char part_buf[128]; // Increase buffer size
            size_t hlen = snprintf(part_buf, sizeof(part_buf), "\r\n--123456789000000000000987654321\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", fb->len);
            res = httpd_resp_send_chunk(req, part_buf, hlen);
            if (res == ESP_OK) {
                res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
            }
            esp_camera_fb_return(fb);
            if (res != ESP_OK) {
                break;
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    return res;
}

static esp_err_t stop_stream_handler(httpd_req_t *req) {
    httpd_resp_sendstr(req, "Stream Stopped");
    return ESP_OK;
}

static esp_err_t settings_handler(httpd_req_t *req) {
    char buf[100];
    int ret = httpd_req_recv(req, buf, MIN(req->content_len, sizeof(buf)));
    if (ret <= 0) {
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            httpd_resp_send_408(req);
        }
        return ESP_FAIL;
    }
    buf[ret] = '\0';
    ESP_LOGI(TAG, "Received settings: %s", buf);
    // Handle settings change here, e.g., frame size, quality
    httpd_resp_sendstr(req, "Settings Updated");
    return ESP_OK;
}

static const httpd_uri_t start_stream_uri = {
    .uri       = "/start_stream",
    .method    = HTTP_GET,
    .handler   = start_stream_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t stop_stream_uri = {
    .uri       = "/stop_stream",
    .method    = HTTP_GET,
    .handler   = stop_stream_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t settings_uri = {
    .uri       = "/settings",
    .method    = HTTP_POST,
    .handler   = settings_handler,
    .user_ctx  = NULL
};

static esp_err_t root_get_handler(httpd_req_t *req) {
    const char resp[] = "<!DOCTYPE html>"
                        "<html>"
                        "<body>"
                        "<h1>ESP32 Camera Stream</h1>"
                        "<button onclick=\"fetch('/start_stream')\">Start Stream</button>"
                        "<button onclick=\"fetch('/stop_stream')\">Stop Stream</button>"
                        "<form action=\"/settings\" method=\"post\">"
                        "Format: <select name=\"format\">"
                        "<option value=\"JPEG\">JPEG</option>"
                        "<option value=\"PNG\">PNG</option>"
                        "</select><br>"
                        "Quality: <input type=\"number\" name=\"quality\" min=\"1\" max=\"63\" value=\"12\"><br>"
                        "<input type=\"submit\" value=\"Apply\">"
                        "</form>"
                        "</body>"
                        "</html>";
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static const httpd_uri_t root_uri = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = root_get_handler,
    .user_ctx  = NULL
};

static esp_err_t start_webserver(httpd_handle_t *server) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    if (httpd_start(server, &config) == ESP_OK) {
        httpd_register_uri_handler(*server, &root_uri);
        httpd_register_uri_handler(*server, &start_stream_uri);
        httpd_register_uri_handler(*server, &stop_stream_uri);
        httpd_register_uri_handler(*server, &settings_uri);
        return ESP_OK;
    }
    return ESP_FAIL;
}

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(init_camera());

    httpd_handle_t server = NULL;
    start_webserver(&server);
}
