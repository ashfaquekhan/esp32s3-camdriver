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

#include "esp_heap_caps.h"  // For heap capabilities (DMA-capable memory)

static const char *TAG = "esp32-cam Webserver";

#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

#define PIN1 GPIO_NUM_47 
#define PIN2 GPIO_NUM_48

static bool state; //left-0, right=1


void rCam(int delay_ms)
{
    state=1;
    gpio_set_level(PIN1, 1);
    gpio_set_level(PIN2, 0);

    vTaskDelay(delay_ms / portTICK_PERIOD_MS);
}
void lCam(int delay_ms)
{
    state=0;
    gpio_set_level(PIN1, 0);
    gpio_set_level(PIN2, 1);

    vTaskDelay(delay_ms / portTICK_PERIOD_MS);
}

void log_frame_data(uint8_t *frame_data, size_t width, size_t height) {
    // Log a subset of the frame data (e.g., first few rows/columns)
    for (size_t y = 0; y < height; y += 10) {  // Log every 10th row
        for (size_t x = 0; x < width; x += 10) {  // Log every 10th column
            // Log the value of the pixel at (x, y)
            ESP_LOGI(TAG, "Pixel (%zu, %zu): %u", x, y, frame_data[y * width + x]);
        }
    }
}

void swapInit()
{
    esp_rom_gpio_pad_select_gpio(PIN1);
    esp_rom_gpio_pad_select_gpio(PIN2);
    gpio_set_direction(PIN1, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN2, GPIO_MODE_OUTPUT);

    lCam(10);
    rCam(10);
}


#define CONFIG_XCLK_FREQ 20000000  //20000000 / 8000000

#define THRESHOLD 40 

void copy_frame_buffer(uint8_t *from, uint8_t *to, size_t len) {
    if (from == NULL || to == NULL) {
        return;  // Handle invalid pointers
    }
    // Use standard memcpy for copying frame buffer
    // memcpy(output_buf, frame->buf, len);
    memcpy(to,from,len);
}



void binary_sketcher_on_roi(camera_fb_t *frame, size_t x, size_t y, size_t roi_width, size_t roi_height) {
    if (!frame || !frame->buf) {
        return; // If frame or buffer is NULL, do nothing
    }

    size_t width = frame->width;
    size_t height = frame->height;

    // Ensure the ROI is within the frame bounds
    if (x >= width || y >= height) {
        return; // If the top-left corner is out of bounds, do nothing
    }

    size_t max_x = x + roi_width;
    size_t max_y = y + roi_height;

    if (max_x > width) {
        max_x = width; // Adjust width if it exceeds frame bounds
    }
    if (max_y > height) {
        max_y = height; // Adjust height if it exceeds frame bounds
    }

    // Perform binary conversion on the selected ROI
    for (size_t j = y; j < max_y; ++j) {
        for (size_t i = x; i < max_x; ++i) {
            uint8_t pixel_value = frame->buf[j * width + i];
            if (pixel_value >= THRESHOLD) {
                frame->buf[j * width + i] = 255; // Set to white
            } else {
                frame->buf[j * width + i] = 0; // Set to black
            }
        }
    }
}

SemaphoreHandle_t xBinarySemaphore;

void binary_sketcher_task(void *arg) 
{
    // camera_fb_t *fb = (camera_fb_t *)arg;
    binary_sketcher_on_roi((camera_fb_t *)arg, 40, 20, 80, 60);
    xSemaphoreGive(xBinarySemaphore);
    vTaskDelete(NULL);
}


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

        .frame_size = FRAMESIZE_96X96,
        .pixel_format = PIXFORMAT_GRAYSCALE,
        // .fb_location = CAMERA_FB_IN_PSRAM,
        .fb_location = CAMERA_FB_IN_DRAM,
        // .jpeg_quality = 20,
        .fb_count = 1,
        .grab_mode = CAMERA_GRAB_WHEN_EMPTY
        };
        //CAMERA_GRAB_LATEST. Sets when buffers should be filled
    lCam(10);    
    esp_err_t err = esp_camera_init(&camera_config);
    // sensor_t * s = esp_camera_sensor_get();
    // s->set_special_effect(s, 2);
    esp_camera_deinit();
    rCam(10); 
    err = esp_camera_init(&camera_config);
    // s = esp_camera_sensor_get();
    // s->set_special_effect(s, 2);

    if (err != ESP_OK)
    {
        return err;
    }
    return ESP_OK;

}

#define MAX_DISPARITY 8  // Maximum disparity range (pixels)
#define BLOCK_SIZE 2      // Block size for block matching

uint8_t* calculate_disparity_map(uint8_t *left_frame, uint8_t *right_frame, size_t width, size_t height) {
    // Allocate memory for the disparity map
    uint8_t *disparity_map = (uint8_t *)heap_caps_malloc(width * height, MALLOC_CAP_DMA);
    if (disparity_map == NULL) {
        return NULL;  // Handle memory allocation failure
    }

    // Initialize disparity map to zero
    memset(disparity_map, 0, width * height);

    // Loop over each pixel in the image
    for (size_t y = BLOCK_SIZE; y < height - BLOCK_SIZE; y++) {
        for (size_t x = BLOCK_SIZE; x < width - BLOCK_SIZE; x++) {
            int min_ssd = INT_MAX;
            int best_disparity = 0;

            // Search for the best disparity within the max range
            for (int d = 0; d < MAX_DISPARITY; d++) {
                int ssd = 0;  // Sum of Squared Differences

                // Ensure we don't go out of bounds on the right frame
                if (x - d < 0) {
                    break;
                }

                // Calculate the Sum of Squared Differences (SSD) for the block
                for (int v = -BLOCK_SIZE; v <= BLOCK_SIZE; v++) {
                    for (int u = -BLOCK_SIZE; u <= BLOCK_SIZE; u++) {
                        int left_pixel = left_frame[(y + v) * width + (x + u)];
                        int right_pixel = right_frame[(y + v) * width + (x + u - d)];
                        int diff = left_pixel - right_pixel;
                        ssd += diff * diff;
                    }
                }

                // Find the disparity with the minimum SSD
                if (ssd < min_ssd) {
                    min_ssd = ssd;
                    best_disparity = d;
                }
            }

            // Set the disparity value for the current pixel
            disparity_map[y * width + x] = best_disparity;
        }
    }

    return disparity_map;
}

esp_err_t jpg_stream_httpd_handler(httpd_req_t *req){
    camera_fb_t * fb = NULL;
    esp_err_t res = ESP_OK;
    size_t _jpg_buf_len;
    uint8_t * _jpg_buf;
    char * part_buf[64];
    static int64_t last_frame = 0;
    if(!last_frame) {
        last_frame = esp_timer_get_time();
    }
    uint8_t *leftImage = (uint8_t *)heap_caps_malloc(1024, MALLOC_CAP_DMA);
    uint8_t *rightImage = (uint8_t *)heap_caps_malloc(1024, MALLOC_CAP_DMA);

    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if(res != ESP_OK){
        return res;
    }

    while(true){
        fb = esp_camera_fb_get();
        // copy_frame_buffer(fb->buf,rightImage,fb->len);
        // esp_camera_fb_return(fb);
        // lCam(5);
        // fb = esp_camera_fb_get();
        // copy_frame_buffer(fb->buf,leftImage,fb->len);
        // rCam(5);
        // fb->buf= calculate_disparity_map(rightImage,leftImage,fb->width,fb->height);
        // log_frame_data(fb->buf,fb->width,fb->height);
        

        // xTaskCreatePinnedToCore(binary_sketcher_task,"binary_sketcher",2048,fb,1,NULL,1);
        // while(!xSemaphoreTake(xBinarySemaphore,portMAX_DELAY))
        // {

        // }
        // draw_bounding_box(fb,45,45,20,20);
        //  binary_sketcher_on_roi(fb, 10,10,80,80);
        // swap(10);
        // fbt = esp_camera_fb_get();
        // swap(10);
        // fb = calculate_depth_map(fb,fbt);
        // esp_camera_fb_return(fbt);

        if (!fb) {
            ESP_LOGE(TAG, "Camera capture failed");
            res = ESP_FAIL;
            break;
        }
        
        if(fb->format != PIXFORMAT_JPEG){
            // swap(0);
            ESP_LOGI(TAG, "State = %d",state);
            bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
            if(!jpeg_converted){
                ESP_LOGE(TAG, "JPEG compression failed");
                esp_camera_fb_return(fb);
                res = ESP_FAIL;
            }
        } 
        else {
            
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
        if(res == ESP_OK ){
            res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
        }
        if(fb->format != PIXFORMAT_JPEG){
            free(_jpg_buf);
        }
        esp_camera_fb_return(fb);
        // esp_camera_fb_return(fbt);
        if(res != ESP_OK){
            break;
        }
        int64_t fr_end = esp_timer_get_time();
        int64_t frame_time = fr_end - last_frame;
        last_frame = fr_end;
        frame_time /= 1000;
        // ESP_LOGI(TAG, "MJPG: %uKB %ums (%.1ffps)",(uint32_t)(_jpg_buf_len/1024),(uint32_t)frame_time, 1000.0 / (uint32_t)frame_time);

    }

    last_frame = 0;
    return res;
}

httpd_uri_t uri_get = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = jpg_stream_httpd_handler,
    .user_ctx = NULL};
httpd_handle_t setup_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t stream_httpd  = NULL;

    if (httpd_start(&stream_httpd , &config) == ESP_OK)
    {
        httpd_register_uri_handler(stream_httpd , &uri_get);
    }

    return stream_httpd;
}

void app_main()
{
    // Create the binary semaphore
    xBinarySemaphore = xSemaphoreCreateBinary();
    if (xBinarySemaphore == NULL) {
        // Semaphore creation failed
        return;
    }

    swapInit();
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
        err = init_camera();
        if (err != ESP_OK)
        {
            printf("err: %s\n", esp_err_to_name(err));
            return;
        }
        setup_server();
        ESP_LOGI(TAG, "ESP32 CAM Web Server is up and running\n");
    }
    else
        ESP_LOGI(TAG, "Failed to connected with Wi-Fi, check your network Credentials\n");
}