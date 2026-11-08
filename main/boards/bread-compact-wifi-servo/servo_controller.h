#ifndef __SERVO_CONTROLLER_H__
#define __SERVO_CONTROLLER_H__

#include "mcp_server.h"

#include <driver/ledc.h>
#include <esp_log.h>

// SG90 等 9g 舵机：50Hz PWM，脉宽 0.5ms~2.5ms 对应 0~180 度
class ServoController {
private:
    static constexpr const char* kTag = "ServoController";
    static constexpr ledc_mode_t kSpeedMode = LEDC_LOW_SPEED_MODE;
    static constexpr ledc_timer_t kTimer = LEDC_TIMER_1;
    static constexpr ledc_channel_t kChannel = LEDC_CHANNEL_1;
    static constexpr ledc_timer_bit_t kDutyResolution = LEDC_TIMER_14_BIT;
    static constexpr uint32_t kFrequencyHz = 50;
    static constexpr uint32_t kPeriodUs = 1000000 / kFrequencyHz;
    static constexpr uint32_t kMinPulseUs = 500;
    static constexpr uint32_t kMaxPulseUs = 2500;
    static constexpr int kMaxAngle = 180;
    static constexpr int kInitialAngle = 90;

    gpio_num_t gpio_num_;
    int angle_ = kInitialAngle;

    void SetAngle(int angle) {
        if (angle < 0) {
            angle = 0;
        } else if (angle > kMaxAngle) {
            angle = kMaxAngle;
        }
        angle_ = angle;

        uint32_t pulse_us = kMinPulseUs + (kMaxPulseUs - kMinPulseUs) * angle / kMaxAngle;
        uint32_t duty = pulse_us * (1 << kDutyResolution) / kPeriodUs;
        ESP_ERROR_CHECK(ledc_set_duty(kSpeedMode, kChannel, duty));
        ESP_ERROR_CHECK(ledc_update_duty(kSpeedMode, kChannel));
        ESP_LOGI(kTag, "Angle set to %d (pulse %lu us)", angle_, (unsigned long)pulse_us);
    }

public:
    ServoController(gpio_num_t gpio_num) : gpio_num_(gpio_num) {
        if (gpio_num_ == GPIO_NUM_NC) {
            return;
        }

        ledc_timer_config_t timer_config = {.speed_mode = kSpeedMode,
                                            .duty_resolution = kDutyResolution,
                                            .timer_num = kTimer,
                                            .freq_hz = kFrequencyHz,
                                            .clk_cfg = LEDC_AUTO_CLK};
        ESP_ERROR_CHECK(ledc_timer_config(&timer_config));

        ledc_channel_config_t channel_config = {.gpio_num = gpio_num_,
                                                .speed_mode = kSpeedMode,
                                                .channel = kChannel,
                                                .intr_type = LEDC_INTR_DISABLE,
                                                .timer_sel = kTimer,
                                                .duty = 0,
                                                .hpoint = 0};
        ESP_ERROR_CHECK(ledc_channel_config(&channel_config));
        SetAngle(kInitialAngle);

        auto& mcp_server = McpServer::GetInstance();
        mcp_server.AddTool("self.servo.get_angle",
                           "Get the current angle of the servo motor (0-180 degrees)",
                           PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
                               return "{\"angle\": " + std::to_string(angle_) + "}";
                           });

        mcp_server.AddTool("self.servo.set_angle",
                           "Rotate the servo motor to an absolute angle. 0 is one end, 90 is the "
                           "center, 180 is the other end. "
                           "To turn left/right by some amount, call `self.servo.get_angle` first "
                           "and compute the target angle.",
                           PropertyList({Property("angle", kPropertyTypeInteger, 0, kMaxAngle)}),
                           [this](const PropertyList& properties) -> ReturnValue {
                               SetAngle(properties["angle"].value<int>());
                               return true;
                           });
    }
};

#endif  // __SERVO_CONTROLLER_H__
