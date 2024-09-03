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



bool copy_frame_buffer(camera_fb_t *fb, uint8_t *dest_buf, size_t buf_size)
{
    // Check if the provided buffer is large enough
    if (buf_size < fb->len) {
        return false;  // Buffer is not large enough
    }

    // Copy the frame buffer to the destination buffer
    memcpy(dest_buf, fb->buf, fb->len);
    
    return true;  // Copy successful
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

        .frame_size = FRAMESIZE_QQVGA,
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
typedef struct {
    uint8_t *imgL;             // Pointer to left image
    uint8_t *imgR;             // Pointer to right image
    uint8_t *disparity;        // Pointer to disparity map
    int img_width;             // Width of the images
    int img_height;            // Height of the images
    int max_disparity;         // Maximum disparity
} DisparityTaskParams;

void calculate_disparity(uint8_t* imgL, uint8_t* imgR, uint8_t* disparity, int width, int height, int max_disparity) {
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int min_ssd = INT_MAX;  // Initialize with a large value
            int best_disparity = 0;

            // Search range for disparity
            for (int d = 0; d < max_disparity; d++) {
                int ssd = 0;  // Sum of Squared Differences

                // Calculate SSD between blocks in left and right images
                for (int j = -1; j <= 1; j++) {
                    for (int i = -1; i <= 1; i++) {
                        int l_x = x + i;
                        int l_y = y + j;
                        int r_x = x + i - d;

                        // Check bounds
                        if (l_x >= 0 && l_x < width && l_y >= 0 && l_y < height && r_x >= 0 && r_x < width) {
                            int diff = imgL[l_y * width + l_x] - imgR[l_y * width + r_x];
                            ssd += diff * diff;
                        }
                    }
                }

                // Update best disparity
                if (ssd < min_ssd) {
                    min_ssd = ssd;
                    best_disparity = d;
                }
            }

            // Store the best disparity
            disparity[y * width + x] = (uint8_t)best_disparity;
        }
    }
}

SemaphoreHandle_t xDisparitySemaphore;
// Function to be run as a task
void disparity_task(void* arg) {
    // Assuming imgL, imgR, and disparity are globally defined
    DisparityTaskParams *params = (DisparityTaskParams *)arg;
    calculate_disparity(params->imgL, params->imgR, params->disparity, params->img_width, params->img_height, params->max_disparity);
    xSemaphoreGive(xDisparitySemaphore);
    vTaskDelete(NULL);
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

    int buf_len = 176*144;
    uint8_t * imgL = (uint8_t *)malloc(buf_len);
    uint8_t * imgR = (uint8_t *)malloc(buf_len);
    uint8_t * disparity = (uint8_t *)malloc(buf_len);
    int img_width = 160;   
    int img_height = 120;  
    int max_disparity = 12; 

    DisparityTaskParams *params=(DisparityTaskParams *)malloc(sizeof(DisparityTaskParams));
    params->imgL = imgL;
    params->imgR = imgR;
    params->disparity = disparity;
    params->img_width = img_width;
    params->img_height = img_height;
    params->max_disparity = max_disparity;

    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if(res != ESP_OK){
        return res;
    }

    while(true){
        rCam(1);
        fb = esp_camera_fb_get();
        copy_frame_buffer(fb,imgR,buf_len);
        esp_camera_fb_return(fb);
        
        lCam(1);
        fb = esp_camera_fb_get();
        copy_frame_buffer(fb,imgL,buf_len);
        esp_camera_fb_return(fb);
        xTaskCreatePinnedToCore(disparity_task, "Disparity Task", 4096, (void *)params, 1, NULL, 1);
        while(xSemaphoreTake(xDisparitySemaphore,portMAX_DELAY) ==pdFALSE){}
        

        if (!fb) {
            ESP_LOGE(TAG, "Camera capture failed");
            res = ESP_FAIL;
            break;
        }
        
        if(fb->format != PIXFORMAT_JPEG){
            // swap(0);
            // ESP_LOGI(TAG, "State = %d",state);
            // bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
            bool jpeg_converted= fmt2jpg(disparity, buf_len, img_width, img_height, PIXFORMAT_GRAYSCALE, 80, &_jpg_buf, &_jpg_buf_len);
            if(!jpeg_converted){
                ESP_LOGE(TAG, "JPEG compression failed");
                // esp_camera_fb_return(fb);
                // res = ESP_FAIL;
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
        // esp_camera_fb_return(fb);
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
    xDisparitySemaphore = xSemaphoreCreateBinary();
    if (xDisparitySemaphore == NULL) {
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