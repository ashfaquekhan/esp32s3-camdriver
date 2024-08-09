#include <esp_system.h>
#include <nvs_flash.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_camera.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "camera_pins.h"
#include "connect_wifi.h"

static const char *TAG = "esp32-cam Webserver";

#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

#define CONFIG_XCLK_FREQ 20000000 

int selected_camera = 1;  // Global variable to keep track of the selected camera

// Function to initialize the first camera
static esp_err_t init_camera_1(void)
{
    camera_config_t camera_config = {
        .ledc_timer = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,
        .pin_pwdn  = CAM_PIN_PWDN,
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
        .xclk_freq_hz = CONFIG_XCLK_FREQ,
        .frame_size = FRAMESIZE_96X96,
        .pixel_format = PIXFORMAT_JPEG,
        .fb_location = CAMERA_FB_IN_DRAM,
        .jpeg_quality = 20,
        .fb_count = 1,
        .grab_mode = CAMERA_GRAB_WHEN_EMPTY
    };

    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK)
    {
        return err;
    }
    return ESP_OK;
}

// Function to initialize the second camera
static esp_err_t init_camera_2(void)
{
    camera_config_t camera_config = {
        .ledc_timer = LEDC_TIMER_1,
        .ledc_channel = LEDC_CHANNEL_1,
        .pin_pwdn  = CAM_PIN_PWDN_2,
        .pin_reset = CAM_PIN_RESET_2,
        .pin_xclk = CAM_PIN_XCLK_2,
        .pin_sccb_sda = CAM_PIN_SIOD_2,
        .pin_sccb_scl = CAM_PIN_SIOC_2,
        .pin_d7 = CAM_PIN_D7_2,
        .pin_d6 = CAM_PIN_D6_2,
        .pin_d5 = CAM_PIN_D5_2,
        .pin_d4 = CAM_PIN_D4_2,
        .pin_d3 = CAM_PIN_D3_2,
        .pin_d2 = CAM_PIN_D2_2,
        .pin_d1 = CAM_PIN_D1_2,
        .pin_d0 = CAM_PIN_D0_2,
        .pin_vsync = CAM_PIN_VSYNC_2,
        .pin_href = CAM_PIN_HREF_2,
        .pin_pclk = CAM_PIN_PCLK_2,
        .xclk_freq_hz = CONFIG_XCLK_FREQ,
        .frame_size = FRAMESIZE_96X96,
        .pixel_format = PIXFORMAT_JPEG,
        .fb_location = CAMERA_FB_IN_DRAM,
        .jpeg_quality = 20,
        .fb_count = 1,
        .grab_mode = CAMERA_GRAB_WHEN_EMPTY
    };

    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK)
    {
        return err;
    }
    return ESP_OK;
}

// Function to deinitialize the camera
static void deinit_camera()
{
    esp_camera_deinit();
}

esp_err_t jpg_stream_httpd_handler(httpd_req_t *req){
    camera_fb_t * fb = NULL;
    esp_err_t res = ESP_OK;
    size_t _jpg_buf_len;
    uint8_t * _jpg_buf;
    char * part_buf[64];
    static int64_t last_frame = 0;
    static int last_camera = 0; // Track the last selected camera

    if(!last_frame) {
        last_frame = esp_timer_get_time();
    }

    // Initialize the selected camera if it has changed
    if (selected_camera != last_camera) {
        deinit_camera();
        if (selected_camera == 1) {
            init_camera_1();
        } else {
            init_camera_2();
        }
        last_camera = selected_camera;
    }

    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if(res != ESP_OK){
        return res;
    }

    while(true){
        fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "Camera %d capture failed", selected_camera);
            res = ESP_FAIL;
            break;
        }
        if(fb->format != PIXFORMAT_JPEG){
            bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
            if(!jpeg_converted){
                ESP_LOGE(TAG, "JPEG compression failed");
                esp_camera_fb_return(fb);
                res = ESP_FAIL;
            }
        } else {
            _jpg_buf_len = fb->len;
            _jpg_buf = fb->buf;
        }

        if(res == ESP_OK){
            res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
        }
        if(res == ESP_OK){
            size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);

            res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
        }
        if(res == ESP_OK){
            res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
        }
        if(fb->format != PIXFORMAT_JPEG){
            free(_jpg_buf);
        }
        esp_camera_fb_return(fb);
        if(res != ESP_OK){
            break;
        }

        int64_t fr_end = esp_timer_get_time();
        int64_t frame_time = fr_end - last_frame;
        last_frame = fr_end;
        frame_time /= 1000;
    }

    last_frame = 0;
    return res;
}

// Handler for switching cameras
esp_err_t camera_switch_handler(httpd_req_t *req)
{
    char buf[32];
    int ret = httpd_req_get_url_query_str(req, buf, sizeof(buf));
    if (ret == ESP_OK)
    {
        char param[32];
        if (httpd_query_key_value(buf, "camera", param, sizeof(param)) == ESP_OK)
        {
            selected_camera = atoi(param);  // Set the selected camera
        }
    }

    // Redirect to the stream page
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

// Handler for the root URL
esp_err_t index_html_handler(httpd_req_t *req)
{
    const char* response = 
    "<html>"
    "<body>"
        "<h1>ESP32-CAM Web Server</h1>"
        "<button onclick=\"switchCamera(1)\">Camera 1</button>"
        "<button onclick=\"switchCamera(2)\">Camera 2</button>"
        "<img src=\"/stream\" id=\"stream\" style=\"width: 320px; height: 240px;\"/>"
        "<script>"
            "function switchCamera(camera) {"
                "window.location.href = `/camera_switch?camera=${camera}`;"
            "}"
        "</script>"
    "</body>"
    "</html>";

    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, response, strlen(response));
    return ESP_OK;
}

httpd_uri_t uri_get = {
    .uri = "/stream",
    .method = HTTP_GET,
    .handler = jpg_stream_httpd_handler,
    .user_ctx = NULL
};

httpd_uri_t uri_camera_switch = {
    .uri = "/camera_switch",
    .method = HTTP_GET,
    .handler = camera_switch_handler,
    .user_ctx = NULL
};

httpd_uri_t uri_index_html = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = index_html_handler,
    .user_ctx = NULL
};

httpd_handle_t setup_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t stream_httpd = NULL;

    if (httpd_start(&stream_httpd, &config) == ESP_OK)
    {
        httpd_register_uri_handler(stream_httpd, &uri_index_html);
        httpd_register_uri_handler(stream_httpd, &uri_get);
        httpd_register_uri_handler(stream_httpd, &uri_camera_switch);
    }

    return stream_httpd;
}

void app_main()
{
    esp_err_t err;

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    connect_wifi();

    if (wifi_connect_status)
    {
        // Initialize the default camera
        err = init_camera_1();
        if (err != ESP_OK)
        {
            printf("err: %s\n", esp_err_to_name(err));
            return;
        }
        setup_server();
        ESP_LOGI(TAG, "ESP32 CAM Web Server is up and running\n");
    }
    else
    {
        ESP_LOGI(TAG, "Failed to connect to Wi-Fi, check your network Credentials\n");
    }
}
