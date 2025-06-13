#include <esp_system.h>
#include <nvs_flash.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
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

// #define CONFIG_XCLK_FREQ 20000000
#define CONFIG_XCLK_FREQ 10000000 

// Camera mutex to prevent conflicts between streaming and capturing
static SemaphoreHandle_t camera_mutex = NULL; 

// HTML webpage with stream and capture functionality
static const char* html_page = 
"<!DOCTYPE html>"
"<html>"
"<head>"
"    <title>ESP32-CAM Stream & Capture</title>"
"    <meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"
"    <style>"
"        body { font-family: Arial, sans-serif; margin: 0; padding: 20px; background-color: #f0f0f0; }"
"        h1 { text-align: center; color: #333; }"
"        .container { max-width: 1200px; margin: 0 auto; }"
"        .stream-section, .capture-section { background: white; padding: 20px; margin: 20px 0; border-radius: 10px; box-shadow: 0 2px 5px rgba(0,0,0,0.1); }"
"        .stream-container { text-align: center; }"
"        #stream { max-width: 100%; height: auto; border: 2px solid #ddd; border-radius: 5px; }"
"        .capture-controls { text-align: center; margin: 20px 0; }"
"        button { background-color: #4CAF50; color: white; padding: 12px 24px; border: none; border-radius: 5px; cursor: pointer; font-size: 16px; margin: 0 10px; }"
"        button:hover { background-color: #45a049; }"
"        button:disabled { background-color: #cccccc; cursor: not-allowed; }"
"        #capturedImage { max-width: 100%; height: auto; border: 2px solid #ddd; border-radius: 5px; display: none; }"
"        .image-info { text-align: center; margin: 10px 0; color: #666; }"
"        .loading { text-align: center; color: #666; font-style: italic; }"
"    </style>"
"</head>"
"<body>"
"    <div class=\"container\">"
"        <h1>ESP32-CAM Stream & HD Capture</h1>"
"        <div class=\"stream-section\">"
"            <h2>Live Stream</h2>"
"            <div class=\"stream-container\">"
"                <img id=\"stream\" src=\"/stream\" alt=\"Camera Stream\">"
"            </div>"
"            <div class=\"image-info\">Real-time camera streaming</div>"
"        </div>"
"        <div class=\"capture-section\">"
"            <h2>Image Capture</h2>"
"            <div class=\"capture-controls\">"
"                <button onclick=\"captureHD()\" id=\"captureBtn\">Capture Image</button>"
"                <button onclick=\"downloadImage()\" id=\"downloadBtn\" style=\"display: none;\">Download Image</button>"
"            </div>"
"            <div id=\"loadingMsg\" class=\"loading\" style=\"display: none;\">Capturing image...</div>"
"            <div class=\"stream-container\">"
"                <img id=\"capturedImage\" alt=\"Captured Image\">"
"            </div>"
"            <div id=\"captureInfo\" class=\"image-info\" style=\"display: none;\"></div>"
"        </div>"
"    </div>"
"    <script>"
"        let capturedImageData = null;"
"        function captureHD() {"
"            const captureBtn = document.getElementById('captureBtn');"
"            const loadingMsg = document.getElementById('loadingMsg');"
"            const capturedImage = document.getElementById('capturedImage');"
"            const captureInfo = document.getElementById('captureInfo');"
"            const downloadBtn = document.getElementById('downloadBtn');"
"            captureBtn.disabled = true;"
"            captureBtn.textContent = 'Capturing...';"
"            loadingMsg.style.display = 'block';"
"            capturedImage.style.display = 'none';"
"            captureInfo.style.display = 'none';"
"            downloadBtn.style.display = 'none';"
"            fetch('/capture')"
"                .then(response => {"
"                    if (!response.ok) {"
"                        throw new Error('Failed to capture image');"
"                    }"
"                    return response.blob();"
"                })"
"                .then(blob => {"
"                    capturedImageData = blob;"
"                    const imageUrl = URL.createObjectURL(blob);"
"                    capturedImage.src = imageUrl;"
"                    capturedImage.style.display = 'block';"
"                    const sizeKB = Math.round(blob.size / 1024);"
"                    captureInfo.textContent = 'Image captured successfully (' + sizeKB + ' KB)';"
"                    captureInfo.style.display = 'block';"
"                    downloadBtn.style.display = 'inline-block';"
"                })"
"                .catch(error => {"
"                    console.error('Error capturing image:', error);"
"                    captureInfo.textContent = 'Failed to capture image. Please try again.';"
"                    captureInfo.style.display = 'block';"
"                })"
"                .finally(() => {"
"                    captureBtn.disabled = false;"
"                    captureBtn.textContent = 'Capture Image';"
"                    loadingMsg.style.display = 'none';"
"                });"
"        }"
"        function downloadImage() {"
"            if (capturedImageData) {"
"                const url = URL.createObjectURL(capturedImageData);"
"                const a = document.createElement('a');"
"                a.href = url;"
"                a.download = 'esp32cam_hd_' + new Date().toISOString().replace(/[:]/g, '-') + '.jpg';"
"                document.body.appendChild(a);"
"                a.click();"
"                document.body.removeChild(a);"
"                URL.revokeObjectURL(url);"
"            }"
"        }"
"    </script>"
"</body>"
"</html>";

static esp_err_t init_camera(void)
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

        .frame_size = FRAMESIZE_QVGA,    // Start with smaller size (320x240)
        .pixel_format = PIXFORMAT_JPEG,
        .fb_location = CAMERA_FB_IN_DRAM,  // Use DRAM instead of PSRAM
        .jpeg_quality = 15,              // Lower quality to reduce buffer size
        .fb_count = 1,                   // Single buffer only
        .grab_mode = CAMERA_GRAB_WHEN_EMPTY
    };

    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed with DRAM, trying PSRAM with smaller settings...");
        
        // Try PSRAM with even smaller settings
        camera_config.fb_location = CAMERA_FB_IN_DRAM;
        camera_config.frame_size = FRAMESIZE_QQVGA;  // Even smaller (160x120)
        camera_config.jpeg_quality = 20;             // Lower quality
        
        err = esp_camera_init(&camera_config);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Camera init failed with all configurations");
            return err;
        }
        ESP_LOGI(TAG, "Camera initialized with QQVGA in PSRAM");
    } else {
        ESP_LOGI(TAG, "Camera initialized with QVGA in DRAM");
    }

    // Try to gradually improve settings after successful initialization
    sensor_t * s = esp_camera_sensor_get();
    if (s != NULL) {
        // Wait a bit for camera to stabilize
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        
        // Try to set a better frame size
        if (s->set_framesize(s, FRAMESIZE_VGA) != 0) {
            ESP_LOGW(TAG, "VGA failed, trying HVGA");
            if (s->set_framesize(s, FRAMESIZE_HVGA) != 0) {
                ESP_LOGW(TAG, "HVGA failed, trying CIF");
                if (s->set_framesize(s, FRAMESIZE_CIF) != 0) {
                    ESP_LOGW(TAG, "CIF failed, staying with QVGA");
                    s->set_framesize(s, FRAMESIZE_QVGA);
                }
            }
        }
        
        // Apply OV2640-specific settings for stability
        s->set_brightness(s, 0);     // -2 to 2
        s->set_contrast(s, 0);       // -2 to 2
        s->set_saturation(s, 0);     // -2 to 2
        s->set_special_effect(s, 0); // 0 to 6 (0 - No Effect)
        s->set_whitebal(s, 1);       // 0 = disable , 1 = enable
        s->set_awb_gain(s, 1);       // 0 = disable , 1 = enable
        s->set_wb_mode(s, 0);        // 0 to 4 - if awb_gain enabled
        s->set_exposure_ctrl(s, 1);  // 0 = disable , 1 = enable
        s->set_aec2(s, 0);           // 0 = disable , 1 = enable
        s->set_ae_level(s, 0);       // -2 to 2
        s->set_aec_value(s, 300);    // 0 to 1200
        s->set_gain_ctrl(s, 1);      // 0 = disable , 1 = enable
        s->set_agc_gain(s, 0);       // 0 to 30
        s->set_gainceiling(s, (gainceiling_t)0);  // 0 to 6
        s->set_bpc(s, 0);            // 0 = disable , 1 = enable
        s->set_wpc(s, 1);            // 0 = disable , 1 = enable
        s->set_raw_gma(s, 1);        // 0 = disable , 1 = enable
        s->set_lenc(s, 1);           // 0 = disable , 1 = enable
        s->set_hmirror(s, 0);        // 0 = disable , 1 = enable
        s->set_vflip(s, 0);          // 0 = disable , 1 = enable
        s->set_dcw(s, 1);            // 0 = disable , 1 = enable
        s->set_colorbar(s, 0);       // 0 = disable , 1 = enable
        
        // Improve quality for streaming
        s->set_quality(s, 12);
        
        ESP_LOGI(TAG, "Camera settings applied successfully");
    }

    return ESP_OK;
}

// Simple streaming handler with minimal camera stress
esp_err_t jpg_stream_httpd_handler(httpd_req_t *req) {
    camera_fb_t * fb = NULL;
    esp_err_t res = ESP_OK;
    size_t _jpg_buf_len;
    uint8_t * _jpg_buf;
    char * part_buf[64];
    
    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if(res != ESP_OK){
        return res;
    }

    while(true){
        // Very short mutex timeout - skip frame if busy
        if (xSemaphoreTake(camera_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            fb = esp_camera_fb_get();
            xSemaphoreGive(camera_mutex);
        } else {
            // Skip this frame if camera is busy
            vTaskDelay(200 / portTICK_PERIOD_MS);
            continue;
        }
        
        if (!fb) {
            // Skip frame and wait longer
            vTaskDelay(200 / portTICK_PERIOD_MS);
            continue;
        }
        
        if(fb->format != PIXFORMAT_JPEG){
            bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
            if(!jpeg_converted){
                esp_camera_fb_return(fb);
                vTaskDelay(200 / portTICK_PERIOD_MS);
                continue;
            }
        } else {
            _jpg_buf_len = fb->len;
            _jpg_buf = fb->buf;
        }

        // Send stream boundary
        if(res == ESP_OK){
            res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
        }
        
        // Send content headers
        if(res == ESP_OK){
            size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
            res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
        }
        
        // Send image data
        if(res == ESP_OK){
            res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
        }
        
        // Cleanup
        if(fb->format != PIXFORMAT_JPEG){
            free(_jpg_buf);
        }
        esp_camera_fb_return(fb);
        
        // Exit on send error
        if(res != ESP_OK){
            ESP_LOGW(TAG, "Stream connection closed");
            break;
        }
        
        // Long delay between frames to reduce camera stress
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }

    return res;
}

// Simplified capture handler - no quality changes, just capture current frame
esp_err_t capture_hd_handler(httpd_req_t *req) {
    camera_fb_t * fb = NULL;
    esp_err_t res = ESP_OK;
    
    ESP_LOGI(TAG, "Capture request received");
    
    // Very short timeout to avoid blocking
    if (xSemaphoreTake(camera_mutex, pdMS_TO_TICKS(500)) != pdTRUE) {
        ESP_LOGW(TAG, "Camera busy, please try again");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Camera busy");
        return ESP_FAIL;
    }
    
    // Just capture current frame without any changes
    fb = esp_camera_fb_get();
    
    // Release mutex immediately
    xSemaphoreGive(camera_mutex);
    
    if (!fb) {
        ESP_LOGE(TAG, "Camera capture failed");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Capture failed");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Image captured: %dx%d, %d bytes", 
             fb->width, fb->height, fb->len);
    
    // Set response headers
    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    
    // Send the image data
    res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
    
    // Clean up
    esp_camera_fb_return(fb);
    
    if (res == ESP_OK) {
        ESP_LOGI(TAG, "Image sent successfully");
    } else {
        ESP_LOGE(TAG, "Failed to send image");
    }
    
    return res;
}

// Handler for the main webpage
esp_err_t index_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, html_page, strlen(html_page));
}

httpd_uri_t uri_get = {
    .uri = "/stream",
    .method = HTTP_GET,
    .handler = jpg_stream_httpd_handler,
    .user_ctx = NULL
};

httpd_uri_t capture_uri = {
    .uri = "/capture",
    .method = HTTP_GET,
    .handler = capture_hd_handler,
    .user_ctx = NULL
};

httpd_uri_t index_uri = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = index_handler,
    .user_ctx = NULL
};

httpd_handle_t setup_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 8;
    config.task_priority = 5;           // Lower priority
    config.stack_size = 8192;           // Increased stack size
    config.server_port = 80;
    config.ctrl_port = 32768;
    config.max_resp_headers = 8;
    config.recv_wait_timeout = 5;       // Shorter timeout
    config.send_wait_timeout = 5;       // Shorter timeout
    
    httpd_handle_t stream_httpd = NULL;

    if (httpd_start(&stream_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(stream_httpd, &index_uri);
        httpd_register_uri_handler(stream_httpd, &uri_get);
        httpd_register_uri_handler(stream_httpd, &capture_uri);
        ESP_LOGI(TAG, "HTTP server started successfully");
    } else {
        ESP_LOGE(TAG, "Failed to start HTTP server");
        return NULL;
    }

    return stream_httpd;
}

void app_main()
{
    esp_err_t err;

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    // Create camera mutex
    camera_mutex = xSemaphoreCreateMutex();
    if (camera_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create camera mutex");
        return;
    }

    connect_wifi();

    if (wifi_connect_status) {
        err = init_camera();
        if (err != ESP_OK) {
            printf("err: %s\n", esp_err_to_name(err));
            return;
        }
        setup_server();
        ESP_LOGI(TAG, "ESP32 CAM Web Server is up and running\n");
        ESP_LOGI(TAG, "Stream: http://[IP_ADDRESS]/stream\n");
        ESP_LOGI(TAG, "Web Interface: http://[IP_ADDRESS]/\n");
    }
    else {
        ESP_LOGI(TAG, "Failed to connected with Wi-Fi, check your network Credentials\n");
    }
}