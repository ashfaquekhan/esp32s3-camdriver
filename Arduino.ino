#include "CameraWrapper.h"

#define CAM_PIN_PWDN    -1
#define CAM_PIN_RESET   -1
#define CAM_PIN_XCLK    4
#define CAM_PIN_SIOD    9
#define CAM_PIN_SIOC    10

#define CAM_PIN_D7      18
#define CAM_PIN_D6      17
#define CAM_PIN_D5      16
#define CAM_PIN_D4      15
#define CAM_PIN_D3      14
#define CAM_PIN_D2      13
#define CAM_PIN_D1      12
#define CAM_PIN_D0      11
#define CAM_PIN_VSYNC   5
#define CAM_PIN_HREF    6
#define CAM_PIN_PCLK    7

#define CAM_PIN_PWDN_2  -1
#define CAM_PIN_RESET_2 -1
#define CAM_PIN_XCLK_2  47
#define CAM_PIN_SIOD_2  19
#define CAM_PIN_SIOC_2  20

#define CAM_PIN_D7_2    42
#define CAM_PIN_D6_2    41
#define CAM_PIN_D5_2    40
#define CAM_PIN_D4_2    39
#define CAM_PIN_D3_2    38
#define CAM_PIN_D2_2    37
#define CAM_PIN_D1_2    36
#define CAM_PIN_D0_2    35
#define CAM_PIN_VSYNC_2 26
#define CAM_PIN_HREF_2  33
#define CAM_PIN_PCLK_2  48

camera_config_t camera_config1 = {
    .pin_pwdn       = CAM_PIN_PWDN,
    .pin_reset      = CAM_PIN_RESET,
    .pin_xclk       = CAM_PIN_XCLK,
    .pin_sccb_sda   = CAM_PIN_SIOD,
    .pin_sccb_scl   = CAM_PIN_SIOC,
    .pin_d7         = CAM_PIN_D7,
    .pin_d6         = CAM_PIN_D6,
    .pin_d5         = CAM_PIN_D5,
    .pin_d4         = CAM_PIN_D4,
    .pin_d3         = CAM_PIN_D3,
    .pin_d2         = CAM_PIN_D2,
    .pin_d1         = CAM_PIN_D1,
    .pin_d0         = CAM_PIN_D0,
    .pin_vsync      = CAM_PIN_VSYNC,
    .pin_href       = CAM_PIN_HREF,
    .pin_pclk       = CAM_PIN_PCLK,
    .xclk_freq_hz   = 20000000,
    .ledc_timer     = LEDC_TIMER_0,
    .ledc_channel   = LEDC_CHANNEL_0,
    .pixel_format   = PIXFORMAT_JPEG,
    .frame_size     = FRAMESIZE_SVGA,
    .jpeg_quality   = 20,
    .fb_count       = 1,
    .fb_location    = CAMERA_FB_IN_DRAM,
    .grab_mode      = CAMERA_GRAB_WHEN_EMPTY
};

//camera_config_t camera_config2 = {
//    .pin_pwdn       = CAM_PIN_PWDN_2,
//    .pin_reset      = CAM_PIN_RESET_2,
//    .pin_xclk       = CAM_PIN_XCLK_2,
//    .pin_sccb_sda   = CAM_PIN_SIOD_2,
//    .pin_sccb_scl   = CAM_PIN_SIOC_2,
//    .pin_d7         = CAM_PIN_D7_2,
//    .pin_d6         = CAM_PIN_D6_2,
//    .pin_d5         = CAM_PIN_D5_2,
//    .pin_d4         = CAM_PIN_D4_2,
//    .pin_d3         = CAM_PIN_D3_2,
//    .pin_d2         = CAM_PIN_D2_2,
//    .pin_d1         = CAM_PIN_D1_2,
//    .pin_d0         = CAM_PIN_D0_2,
//    .pin_vsync      = CAM_PIN_VSYNC_2,
//    .pin_href       = CAM_PIN_HREF_2,
//    .pin_pclk       = CAM_PIN_PCLK_2,
//    .xclk_freq_hz   = 20000000,
//    .ledc_timer     = LEDC_TIMER_1,
//    .ledc_channel   = LEDC_CHANNEL_1,
//    .pixel_format   = PIXFORMAT_JPEG,
//    .frame_size     = FRAMESIZE_SVGA,
//    .jpeg_quality   = 20,
//    .fb_count       = 1,
//    .grab_mode      = CAMERA_GRAB_WHEN_EMPTY
//};

Camera cam1(camera_config1);
//Camera cam2(camera_config2);

void setup() {
    Serial.begin(115200);

    if (cam1.init() != ESP_OK) {
        Serial.println("Failed to initialize camera 1");
        return;
    } else {
        Serial.println("Camera 1 initialized successfully");
    }
//
//    if (cam2.init() != ESP_OK) {
//        Serial.println("Failed to initialize camera 2");
//        return;
//    } else {
//        Serial.println("Camera 2 initialized successfully");
//    }
}

void loop() {
    camera_fb_t* fb1 = cam1.capture();
//    camera_fb_t* fb2 = cam2.capture();

    if (fb1) {
        Serial.println("Captured a frame from camera 1");
        cam1.returnFrame(fb1);
    } else {
        Serial.println("Failed to capture a frame from camera 1");
    }

//    if (fb2) {
//        Serial.println("Captured a frame from camera 2");
//        cam2.returnFrame(fb2);
//    } else {
//        Serial.println("Failed to capture a frame from camera 2");
//    }

    delay(1000);  // Delay between captures
}