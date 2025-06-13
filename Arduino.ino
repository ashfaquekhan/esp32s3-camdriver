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

// #define CONFIG_XCLK_FREQ 20000000
#define CONFIG_XCLK_FREQ 10000000 

// HTML webpage with stream and capture functionality
static const char* html_page = R"(
<!DOCTYPE html>
<html>
<head>
    <title>ESP32-CAM Stream & Capture</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { 
            font-family: Arial, sans-serif; 
            margin: 0; 
            padding: 20px; 
            background-color: #f0f0f0; 
        }
        h1 { 
            text-align: center; 
            color: #333; 
        }
        .container { 
            max-width: 1200px; 
            margin: 0 auto; 
        }
        .stream-section, .capture-section { 
            background: white; 
            padding: 20px; 
            margin: 20px 0; 
            border-radius: 10px; 
            box-shadow: 0 2px 5px rgba(0,0,0,0.1); 
        }
        .stream-container { 
            text-align: center; 
        }
        #stream { 
            max-width: 100%; 
            height: auto; 
            border: 2px solid #ddd; 
            border-radius: 5px; 
        }
        .capture-controls { 
            text-align: center; 
            margin: 20px 0; 
        }
        button { 
            background-color: #4CAF50; 
            color: white; 
            padding: 12px 24px; 
            border: none; 
            border-radius: 5px; 
            cursor: pointer; 
            font-size: 16px; 
            margin: 0 10px; 
        }
        button:hover { 
            background-color: #45a049; 
        }
        button:disabled { 
            background-color: #cccccc; 
            cursor: not-allowed; 
        }
        #capturedImage { 
            max-width: 100%; 
            height: auto; 
            border: 2px solid #ddd; 
            border-radius: 5px; 
            display: none; 
        }
        .image-info { 
            text-align: center; 
            margin: 10px 0; 
            color: #666; 
        }
        .loading { 
            text-align: center; 
            color: #666; 
            font-style: italic; 
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>ESP32-CAM Stream & HD Capture</h1>
        
        <div class="stream-section">
            <h2>Live Stream (Low Resolution)</h2>
            <div class="stream-container">
                <img id="stream" src="/stream" alt="Camera Stream">
            </div>
            <div class="image-info">Real-time streaming at VGA resolution</div>
        </div>
        
        <div class="capture-section">
            <h2>HD Image Capture</h2>
            <div class="capture-controls">
                <button onclick="captureHD()" id="captureBtn">Capture HD Image</button>
                <button onclick="downloadImage()" id="downloadBtn" style="display: none;">Download Image</button>
            </div>
            <div id="loadingMsg" class="loading" style="display: none;">Capturing HD image...</div>
            <div class="stream-container">
                <img id="capturedImage" alt="Captured HD Image">
            </div>
            <div id="captureInfo" class="image-info" style="display: none;"></div>
        </div>
    </div>

    <script>
        let capturedImageData = null;
        
        function captureHD() {
            const captureBtn = document.getElementById('captureBtn');
            const loadingMsg = document.getElementById('loadingMsg');
            const capturedImage = document.getElementById('capturedImage');
            const captureInfo = document.getElementById('captureInfo');
            const downloadBtn = document.getElementById('downloadBtn');
            
            // Show loading state
            captureBtn.disabled = true;
            captureBtn.textContent = 'Capturing...';
            loadingMsg.style.display = 'block';
            capturedImage.style.display = 'none';
            captureInfo.style.display = 'none';
            downloadBtn.style.display = 'none';
            
            // Capture HD image
            fetch('/capture')
                .then(response => {
                    if (!response.ok) {
                        throw new Error('Failed to capture image');
                    }
                    return response.blob();
                })
                .then(blob => {
                    // Create object URL for the image
                    capturedImageData = blob;
                    const imageUrl = URL.createObjectURL(blob);
                    
                    // Display the captured image
                    capturedImage.src = imageUrl;
                    capturedImage.style.display = 'block';
                    
                    // Show image info
                    const sizeKB = Math.round(blob.size / 1024);
                    captureInfo.textContent = `HD image captured successfully (${sizeKB} KB)`;
                    captureInfo.style.display = 'block';
                    
                    // Show download button
                    downloadBtn.style.display = 'inline-block';
                })
                .catch(error => {
                    console.error('Error capturing image:', error);
                    captureInfo.textContent = 'Failed to capture HD image. Please try again.';
                    captureInfo.style.display = 'block';
                })
                .finally(() => {
                    // Reset button state
                    captureBtn.disabled = false;
                    captureBtn.textContent = 'Capture HD Image';
                    loadingMsg.style.display = 'none';
                });
        }
        
        function downloadImage() {
            if (capturedImageData) {
                const url = URL.createObjectURL(capturedImageData);
                const a = document.createElement('a');
                a.href = url;
                a.download = `esp32cam_hd_${new Date().toISOString().replace(/[:.]/g, '-')}.jpg`;
                document.body.appendChild(a);
                a.click();
                document.body.removeChild(a);
                URL.revokeObjectURL(url);
            }
        }
    </script>
</body>
</html>
)";

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

        .frame_size = FRAMESIZE_VGA,  // Stream in VGA for better performance
        .pixel_format = PIXFORMAT_JPEG,
        .fb_location = CAMERA_FB_IN_PSRAM,
        .jpeg_quality = 15,  // Lower quality for streaming
        .fb_count = 2,       // Double buffering for smoother stream
        .grab_mode = CAMERA_GRAB_WHEN_EMPTY
    };

    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        return err;
    }
    return ESP_OK;
}

// Stream handler for low-resolution real-time video
esp_err_t jpg_stream_httpd_handler(httpd_req_t *req) {
    camera_fb_t * fb = NULL;
    esp_err_t res = ESP_OK;
    size_t _jpg_buf_len;
    uint8_t * _jpg_buf;
    char * part_buf[64];
    static int64_t last_frame = 0;
    
    if(!last_frame) {
        last_frame = esp_timer_get_time();
    }

    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if(res != ESP_OK){
        return res;
    }

    while(true){
        fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "Camera capture failed");
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

// Handler for HD image capture
esp_err_t capture_hd_handler(httpd_req_t *req) {
    camera_fb_t * fb = NULL;
    esp_err_t res = ESP_OK;
    
    // Get current sensor settings
    sensor_t * s = esp_camera_sensor_get();
    if (s == NULL) {
        ESP_LOGE(TAG, "Failed to get camera sensor");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    
    // Save current settings
    framesize_t current_framesize = s->status.framesize;
    int current_quality = s->status.quality;
    
    // Set high resolution and quality for capture
    s->set_framesize(s, FRAMESIZE_UXGA);  // 1600x1200 HD resolution
    s->set_quality(s, 5);  // High quality (lower number = better quality)
    
    // Give camera time to adjust
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
    // Capture HD frame
    fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGE(TAG, "HD Camera capture failed");
        // Restore original settings
        s->set_framesize(s, current_framesize);
        s->set_quality(s, current_quality);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    
    // Send HD image
    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    
    res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
    
    // Clean up
    esp_camera_fb_return(fb);
    
    // Restore original settings for streaming
    s->set_framesize(s, current_framesize);
    s->set_quality(s, current_quality);
    
    // Give camera time to adjust back
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
    ESP_LOGI(TAG, "HD image captured and sent");
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
    config.max_uri_handlers = 8;  // Increase to handle multiple endpoints
    httpd_handle_t stream_httpd = NULL;

    if (httpd_start(&stream_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(stream_httpd, &index_uri);
        httpd_register_uri_handler(stream_httpd, &uri_get);
        httpd_register_uri_handler(stream_httpd, &capture_uri);
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