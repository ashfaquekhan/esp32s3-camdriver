#pragma once

#include "esp_camera.h"
#include <string>

class Camera {
public:
    Camera(const camera_config_t& config) : config_(config), initialized_(false) {}

    ~Camera() {
        if (initialized_) {
            esp_camera_deinit();
        }
    }

    esp_err_t init() {
        esp_err_t err = esp_camera_init(&config_);
        if (err == ESP_OK) {
            initialized_ = true;
        }
        return err;
    }

    esp_err_t deinit() {
        if (initialized_) {
            esp_err_t err = esp_camera_deinit();
            if (err == ESP_OK) {
                initialized_ = false;
            }
            return err;
        }
        return ESP_ERR_INVALID_STATE;
    }

    camera_fb_t* capture() {
        return esp_camera_fb_get();
    }

    void returnFrame(camera_fb_t* fb) {
        if (fb) {
            esp_camera_fb_return(fb);
        }
    }

    sensor_t* getSensor() {
        return esp_camera_sensor_get();
    }

    esp_err_t saveSettingsToNVS(const std::string& key) {
        return esp_camera_save_to_nvs(key.c_str());
    }

    esp_err_t loadSettingsFromNVS(const std::string& key) {
        return esp_camera_load_from_nvs(key.c_str());
    }

    void returnAllFrames() {
        esp_camera_return_all();
    }

private:
    camera_config_t config_;
    bool initialized_;
};

