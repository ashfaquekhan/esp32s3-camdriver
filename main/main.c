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
#include <math.h>
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

typedef struct {
    uint8_t *imgL;             // Pointer to left image
    uint8_t *imgR;             // Pointer to right image
    int buf_len;
    int img_width;             // Width of the images
    int img_height;            // Height of the images
    int num_points;
    int *points_x;
    int *points_y;

} OpticalFlowParams;

#define WIN_SIZE 3  // Window size for Lucas-Kanade
#define MAX_INTENSITY 255  // Max grayscale intensity for drawing

// Function to set a pixel value in the image (ensuring bounds checking)
void set_pixel(uint8_t *img, int width, int height, int x, int y, uint8_t value) {
    if (x >= 0 && x < width && y >= 0 && y < height) {
        img[y * width + x] = value;
    }
}

// Basic line drawing function using Bresenham's algorithm to draw optical flow lines
void draw_line(uint8_t *img, int width, int height, int x0, int y0, int x1, int y1, uint8_t color) {
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy, e2;  // error value

    while (1) {
        set_pixel(img, width, height, x0, y0, color);  // Set pixel color to draw the line
        if (x0 == x1 && y0 == y1) break;
        e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

// Function to calculate optical flow and merge results into the left image
void calculate_optical_flow_and_merge(uint8_t *imgL, uint8_t *imgR, int width, int height, int num_points, int *points_x, int *points_y) {
    // Sobel filter kernels for calculating image gradients
    int Gx[3][3] = {{-1, 0, 1}, {-2, 0, 2}, {-1, 0, 1}};
    int Gy[3][3] = {{-1, -2, -1}, {0, 0, 0}, {1, 2, 1}};

    for (int p = 0; p < num_points; p++) {
        int x = points_x[p];
        int y = points_y[p];

        // Skip if the point is too close to the border
        if (x < WIN_SIZE / 2 || x >= width - WIN_SIZE / 2 || y < WIN_SIZE / 2 || y >= height - WIN_SIZE / 2) {
            continue;
        }

        // Variables to store summation values for solving the optical flow equations
        double sum_IxIx = 0, sum_IyIy = 0, sum_IxIy = 0;
        double sum_IxIt = 0, sum_IyIt = 0;

        // Compute gradients within the window
        for (int i = -WIN_SIZE / 2; i <= WIN_SIZE / 2; i++) {
            for (int j = -WIN_SIZE / 2; j <= WIN_SIZE / 2; j++) {
                int imgX = x + i;
                int imgY = y + j;

                // Calculate image gradients Ix, Iy using Sobel operators
                double Ix = 0, Iy = 0;
                for (int kx = 0; kx < 3; kx++) {
                    for (int ky = 0; ky < 3; ky++) {
                        int pixel_x = imgL[(imgY + ky - 1) * width + (imgX + kx - 1)];
                        Ix += Gx[kx][ky] * pixel_x;
                        Iy += Gy[kx][ky] * pixel_x;
                    }
                }

                // Calculate It (image intensity difference between imgL and imgR)
                double It = imgR[imgY * width + imgX] - imgL[imgY * width + imgX];

                // Update sums for solving the optical flow equations
                sum_IxIx += Ix * Ix;
                sum_IyIy += Iy * Iy;
                sum_IxIy += Ix * Iy;
                sum_IxIt += Ix * It;
                sum_IyIt += Iy * It;
            }
        }

        // Solve the optical flow linear equations:
        double det = (sum_IxIx * sum_IyIy) - (sum_IxIy * sum_IxIy);
        int vx = 0, vy = 0;
        if (fabs(det) > 1e-6) {
            vx = (int)((sum_IyIy * (-sum_IxIt) - sum_IxIy * (-sum_IyIt)) / det);
            vy = (int)((sum_IxIx * (-sum_IyIt) - sum_IxIy * (-sum_IxIt)) / det);
        }

        // Calculate the new position of the point after optical flow
        int new_x = x + vx;
        int new_y = y + vy;

        // Draw the optical flow line on the left image
        draw_line(imgL, width, height, x, y, new_x, new_y, MAX_INTENSITY);  // Draw white line

        // Optionally, mark the new position with a brighter point
        set_pixel(imgL, width, height, new_x, new_y, MAX_INTENSITY);  // Mark the end point
    }
}

size_t _jpg_buf_len;
uint8_t * _jpg_buf;
SemaphoreHandle_t xOpticalFlowSemaphore;
// Function to be run as a task
void opticalFlow_task(void* arg) {
    // Assuming imgL, imgR, and disparity are globally defined
    OpticalFlowParams *params = (OpticalFlowParams *)arg;
    
    calculate_optical_flow_and_merge(params->imgL, params->imgR, params->img_width,params->img_height,params->num_points,params->points_x,params->points_y);

    bool jpeg_converted= fmt2jpg(params->imgL,params->buf_len, params->img_width, params->img_height, PIXFORMAT_GRAYSCALE, 80, &_jpg_buf, &_jpg_buf_len);
    
    if(!jpeg_converted){
        ESP_LOGE(TAG, "JPEG compression failed");
        // esp_camera_fb_return(fb);
        // res = ESP_FAIL;
    }
    xSemaphoreGive(xOpticalFlowSemaphore);
    vTaskDelete(NULL);
}


esp_err_t jpg_stream_httpd_handler(httpd_req_t *req){
    camera_fb_t * fb = NULL;
    esp_err_t res = ESP_OK;
    // size_t _jpg_buf_len;
    // uint8_t * _jpg_buf;
    char * part_buf[64];
    static int64_t last_frame = 0;
    if(!last_frame) {
        last_frame = esp_timer_get_time();
    }

    int img_width = 96;   
    int img_height = 96;  
    int buf_len = img_width * img_height;
    uint8_t * imgL = (uint8_t *)malloc(buf_len);
    uint8_t * imgR = (uint8_t *)malloc(buf_len);
    int num_points =10;
    int points_x[10]= {5,10,15,20,25,30,35,40,45,50};
    int points_y[10]= {5,10,15,20,25,30,35,40,45,50};

    // uint8_t * disparity = (uint8_t *)malloc(buf_len);   

    OpticalFlowParams *params=(OpticalFlowParams *)malloc(sizeof(OpticalFlowParams));
    params->imgL = imgL;
    params->imgR = imgR;
    params->buf_len= buf_len;
    params->img_width = img_width;
    params->img_height = img_height;
    params->num_points = num_points;
    params->points_x = points_x;
    params->points_y = points_y;



    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if(res != ESP_OK){
        return res;
    }

    while(true){
        
        // fb = esp_camera_fb_get();
        lCam(1);
        // copy_frame_buffer(fb,imgR,buf_len);
        // esp_camera_fb_return(fb);
        
        fb = esp_camera_fb_get();
        // rCam(1);
        copy_frame_buffer(fb,imgL,buf_len);
        esp_camera_fb_return(fb);
        
        xTaskCreatePinnedToCore(opticalFlow_task, "OpticalFlow Task", 4096, (void *)params, 1, NULL, 1);
        
        while(xSemaphoreTake(xOpticalFlowSemaphore,portMAX_DELAY) ==pdFALSE){}
        

        if (!fb) {
            ESP_LOGE(TAG, "Camera capture failed");
            res = ESP_FAIL;
            break;
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
    xOpticalFlowSemaphore = xSemaphoreCreateBinary();
    if (xOpticalFlowSemaphore == NULL) {
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