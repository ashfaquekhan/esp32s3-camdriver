#ifndef CAMERA_PINS_H_
#define CAMERA_PINS_H_

#define CAMERA_MODEL_XIAO_ESP32S3 1
// #define CUSTOM_BOARD 1


#if defined(CUSTOM_BOARD)
    #define CAM_PIN_PWDN  -1
    #define CAM_PIN_RESET -1
    #define CAM_PIN_XCLK  4
    #define CAM_PIN_SIOD  9
    #define CAM_PIN_SIOC  10

    #define CAM_PIN_D7    18
    #define CAM_PIN_D6    17
    #define CAM_PIN_D5    16
    #define CAM_PIN_D4    15
    #define CAM_PIN_D3    14
    #define CAM_PIN_D2    13
    #define CAM_PIN_D1    12
    #define CAM_PIN_D0    11
    #define CAM_PIN_VSYNC 5
    #define CAM_PIN_HREF  6
    #define CAM_PIN_PCLK  7

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

#elif defined(CAMERA_MODEL_XIAO_ESP32S3)

  #define CAM_PIN_PWDN    -1
  #define CAM_PIN_RESET   -1
  #define CAM_PIN_XCLK    10
  #define CAM_PIN_SIOD    40
  #define CAM_PIN_SIOC    39
  
  #define CAM_PIN_D7      48
  #define CAM_PIN_D6      11
  #define CAM_PIN_D5      12
  #define CAM_PIN_D4      14
  #define CAM_PIN_D3      16
  #define CAM_PIN_D2      18
  #define CAM_PIN_D1      17
  #define CAM_PIN_D0      15
  #define CAM_PIN_VSYNC   38
  #define CAM_PIN_HREF    47
  #define CAM_PIN_PCLK    13

#define CAMERA_MODULE_NAME "ESP32-S3"
#define CAMERA_MODULE_SOC "CAMERA_MODEL_XIAO_ESP32S3"

#else
    #error "Camera model not selected"
#endif

#endif