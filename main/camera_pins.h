#ifndef CAMERA_PINS_H_
#define CAMERA_PINS_H_

// #define CAMERA_MODEL_XIAO_ESP32S3 1
#define CUSTOM_BOARD 1


#if defined(CUSTOM_BOARD)
    // #define CAM_PIN_PWDN  -1
    // #define CAM_PIN_RESET -1
    // #define CAM_PIN_XCLK  4
    // #define CAM_PIN_SIOD  33
    // #define CAM_PIN_SIOC  34

    // #define CAM_PIN_D7    18
    // #define CAM_PIN_D6    17
    // #define CAM_PIN_D5    16
    // #define CAM_PIN_D4    15
    // #define CAM_PIN_D3    14
    // #define CAM_PIN_D2    13
    // #define CAM_PIN_D1    12
    // #define CAM_PIN_D0    11
    // #define CAM_PIN_VSYNC 5
    // #define CAM_PIN_HREF  41
    // #define CAM_PIN_PCLK  42
    #define CAM_PIN_PWDN  -1
    #define CAM_PIN_RESET -1
    #define CAM_PIN_XCLK  4
    #define CAM_PIN_SIOD  -1//9
    #define CAM_PIN_SIOC  -1//10

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

#else
    #error "Camera model not selected"
#endif

#endif