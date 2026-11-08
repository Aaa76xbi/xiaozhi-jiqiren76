#include "dual_servo_controller.h"

#include "application.h"
#include "mcp_server.h"

#include <driver/ledc.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <cstdio>
#include <cstring>
#include <string>

namespace {

constexpr const char* kTag = "DualServo";
constexpr ledc_mode_t kSpeedMode = LEDC_LOW_SPEED_MODE;
// Face demo timing. This board's LED is WS2812 on RMT and the lamp is plain GPIO,
// so timer 0 / channel 0 (tilt) / channel 1 (pan) does not collide with other PWM.
constexpr ledc_timer_t kTimer = LEDC_TIMER_0;
constexpr ledc_channel_t kTiltChannel = LEDC_CHANNEL_0;
constexpr ledc_channel_t kPanChannel = LEDC_CHANNEL_1;
constexpr ledc_timer_bit_t kDutyResolution = LEDC_TIMER_14_BIT;
constexpr uint32_t kFrequencyHz = 50;
constexpr int64_t kBootGuardUs = 3000000;
constexpr int kTaskStack = 4096;
constexpr UBaseType_t kTaskPriority = 1;

bool NamesEqual(const char* a, const char* b) {
    if (a == nullptr || b == nullptr) {
        return false;
    }
    return std::strcmp(a, b) == 0;
}

}  // namespace

DualServoController* DualServoController::instance_ = nullptr;

DualServoController::DualServoController(gpio_num_t pan_gpio, gpio_num_t tilt_gpio)
    : pan_gpio_(pan_gpio), tilt_gpio_(tilt_gpio) {
    InitPwm();
    instance_ = this;
    RegisterTools();

    auto alive = dual_servo::LookupAction("alive");
    PostGesture(alive.frames, alive.count, alive.after, alive.name);
    boot_alive_ = true;
    boot_deadline_us_ = esp_timer_get_time() + kBootGuardUs;
    StartTask();
}

void DualServoController::InitPwm() {
    pan_ok_ = pan_gpio_ != GPIO_NUM_NC;
    tilt_ok_ = tilt_gpio_ != GPIO_NUM_NC;
    if (!pan_ok_ && !tilt_ok_) {
        ESP_LOGW(kTag, "Servo pins are NC; head motion disabled");
        return;
    }

    uint32_t center_duty = dual_servo::DutyForPulse(
        dual_servo::PulseUsForAngle(static_cast<float>(dual_servo::kPanCenter)));

    ledc_timer_config_t timer = {.speed_mode = kSpeedMode,
                                 .duty_resolution = kDutyResolution,
                                 .timer_num = kTimer,
                                 .freq_hz = kFrequencyHz,
                                 .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    if (tilt_ok_) {
        ledc_channel_config_t channel = {.gpio_num = tilt_gpio_,
                                         .speed_mode = kSpeedMode,
                                         .channel = kTiltChannel,
                                         .intr_type = LEDC_INTR_DISABLE,
                                         .timer_sel = kTimer,
                                         .duty = center_duty,
                                         .hpoint = 0};
        ESP_ERROR_CHECK(ledc_channel_config(&channel));
        tilt_duty_ = center_duty;
    }
    if (pan_ok_) {
        ledc_channel_config_t channel = {.gpio_num = pan_gpio_,
                                         .speed_mode = kSpeedMode,
                                         .channel = kPanChannel,
                                         .intr_type = LEDC_INTR_DISABLE,
                                         .timer_sel = kTimer,
                                         .duty = center_duty,
                                         .hpoint = 0};
        ESP_ERROR_CHECK(ledc_channel_config(&channel));
        pan_duty_ = center_duty;
    }

    pan_ = static_cast<float>(dual_servo::kPanCenter);
    tilt_ = static_cast<float>(dual_servo::kTiltCenter);
    pan_deg_.store(dual_servo::kPanCenter);
    tilt_deg_.store(dual_servo::kTiltCenter);
    ESP_LOGI(kTag, "Centered pan GPIO%d / tilt GPIO%d (50Hz, 500-2500us)",
             static_cast<int>(pan_gpio_), static_cast<int>(tilt_gpio_));
}

void DualServoController::RegisterTools() {
    auto& mcp = McpServer::GetInstance();
    mcp.AddTool("self.servo.get_pan",
                "Get the current pan (yaw) angle. 50 is left, 90 is center, 130 is right.",
                PropertyList(), [this](const PropertyList&) -> ReturnValue {
                    char json[32];
                    std::snprintf(json, sizeof(json), "{\"pan\":%d}", pan_deg_.load());
                    return std::string(json);
                });
    mcp.AddTool(
        "self.servo.set_pan",
        "Ease the pan (yaw) servo to an absolute angle. 50 is left, 90 is center, 130 is right. "
        "The move is queued and does not block.",
        PropertyList(
            {Property("angle", kPropertyTypeInteger, dual_servo::kPanMin, dual_servo::kPanMax)}),
        [this](const PropertyList& properties) -> ReturnValue {
            PostPose(true, properties["angle"].value<int>(), false, 0);
            return true;
        });
    mcp.AddTool("self.servo.get_tilt",
                "Get the current tilt (pitch) angle. 55 is up, 90 is center, 125 is down.",
                PropertyList(), [this](const PropertyList&) -> ReturnValue {
                    char json[32];
                    std::snprintf(json, sizeof(json), "{\"tilt\":%d}", tilt_deg_.load());
                    return std::string(json);
                });
    mcp.AddTool(
        "self.servo.set_tilt",
        "Ease the tilt (pitch) servo to an absolute angle. 55 is up, 90 is center, 125 is down. "
        "The move is queued and does not block.",
        PropertyList(
            {Property("angle", kPropertyTypeInteger, dual_servo::kTiltMin, dual_servo::kTiltMax)}),
        [this](const PropertyList& properties) -> ReturnValue {
            PostPose(false, 0, true, properties["angle"].value<int>());
            return true;
        });
    mcp.AddTool(
        "self.servo.play_action",
        "Queue a named head action: center, shake, nod, curious, dizzy, look_left, look_right, "
        "alive, idle, idle_off. Chat emotions already move the head; use this only when a "
        "specific movement is requested. Returns as soon as the action is queued.",
        PropertyList({Property("action", kPropertyTypeString).SetMaxLength(16)}),
        [this](const PropertyList& properties) -> ReturnValue {
            const std::string& action = properties["action"].value<std::string>();
            if (!PlayNamedAction(action.c_str())) {
                return std::string(
                    "Unknown action. Use center, shake, nod, curious, dizzy, look_left, "
                    "look_right, alive, idle, or idle_off.");
            }
            return true;
        });
    mcp.AddTool(
        "self.servo.set_idle",
        "Enable or disable idle head motion (slow breath and an occasional glance). Emotion "
        "gestures, the listening peek, and speaking nods still run when idle motion is off.",
        PropertyList({Property("enabled", kPropertyTypeBoolean)}),
        [this](const PropertyList& properties) -> ReturnValue {
            idle_enabled_.store(properties["enabled"].value<bool>());
            return true;
        });
}

void DualServoController::StartTask() {
    BaseType_t created = xTaskCreatePinnedToCore(TaskEntry, "dual_servo", kTaskStack, this,
                                                 kTaskPriority, nullptr, 0);
    if (created != pdPASS) {
        ESP_LOGE(kTag, "Failed to start the servo motion task");
    }
}

void DualServoController::NotifyEmotion(const char* emotion) {
    if (instance_ == nullptr) {
        return;
    }
    auto hit = dual_servo::LookupEmotion(emotion);
    if (hit.kind == dual_servo::EmotionHit::Kind::kUnknown) {
        return;
    }
    if (hit.kind == dual_servo::EmotionHit::Kind::kNeutral) {
        instance_->PostRelease(true);
        return;
    }
    instance_->PostGesture(hit.frames, hit.count, hit.after, hit.name);
}

void DualServoController::PostGesture(const dual_servo::Keyframe* frames, uint8_t count,
                                      dual_servo::AmbientStyle after, const char* name) {
    std::lock_guard<std::mutex> lock(mu_);
    pending_.has_pose = false;
    pending_.set_pan = false;
    pending_.set_tilt = false;
    pending_.release = false;
    pending_.soft_release = false;
    pending_.has_gesture = frames != nullptr && count > 0;
    pending_.frames = frames;
    pending_.count = count;
    pending_.after = after;
    dual_servo::CopyKey(pending_.name, sizeof(pending_.name), name);
}

void DualServoController::PostRelease(bool soft) {
    std::lock_guard<std::mutex> lock(mu_);
    // A standby "neutral" must not erase the boot sequence before the task starts it.
    if (soft && pending_.has_gesture && NamesEqual(pending_.name, "alive")) {
        return;
    }
    pending_.has_gesture = false;
    pending_.has_pose = false;
    pending_.set_pan = false;
    pending_.set_tilt = false;
    pending_.frames = nullptr;
    pending_.count = 0;
    pending_.name[0] = '\0';
    pending_.release = true;
    pending_.soft_release = soft;
}

void DualServoController::PostPose(bool set_pan, int pan, bool set_tilt, int tilt) {
    std::lock_guard<std::mutex> lock(mu_);
    pending_.has_gesture = false;
    pending_.release = false;
    pending_.soft_release = false;
    pending_.name[0] = '\0';
    pending_.has_pose = true;
    if (set_pan) {
        pending_.set_pan = true;
        pending_.pan = dual_servo::ClampPanInt(pan);
    }
    if (set_tilt) {
        pending_.set_tilt = true;
        pending_.tilt = dual_servo::ClampTiltInt(tilt);
    }
}

bool DualServoController::PlayNamedAction(const char* name) {
    auto hit = dual_servo::LookupAction(name);
    if (hit.kind == dual_servo::ActionHit::Kind::kUnknown) {
        return false;
    }
    if (hit.kind == dual_servo::ActionHit::Kind::kIdleOn) {
        idle_enabled_.store(true);
        PostRelease(false);
        return true;
    }
    if (hit.kind == dual_servo::ActionHit::Kind::kIdleOff) {
        idle_enabled_.store(false);
        PostRelease(false);
        return true;
    }
    PostGesture(hit.frames, hit.count, hit.after, hit.name);
    return true;
}

DualServoController::Pending DualServoController::TakePending() {
    std::lock_guard<std::mutex> lock(mu_);
    Pending snap = pending_;
    pending_.has_gesture = false;
    pending_.release = false;
    pending_.soft_release = false;
    pending_.has_pose = false;
    pending_.set_pan = false;
    pending_.set_tilt = false;
    pending_.name[0] = '\0';
    return snap;
}

void DualServoController::ApplyPending() {
    if (boot_alive_ && esp_timer_get_time() > boot_deadline_us_) {
        boot_alive_ = false;
    }

    Pending pending = TakePending();
    if (pending.release) {
        bool keep_boot = pending.soft_release && boot_alive_ && player_.active &&
                         NamesEqual(active_name_, "alive");
        if (!keep_boot) {
            player_.Cancel();
            ambient_style_ = dual_servo::AmbientStyle::kNeutral;
            active_name_[0] = '\0';
            boot_alive_ = false;
        }
        return;
    }
    if (pending.has_gesture) {
        bool same = player_.active && NamesEqual(active_name_, pending.name);
        if (!same) {
            player_.Start(pending.frames, pending.count, pending.after, pan_, tilt_);
            dual_servo::CopyKey(active_name_, sizeof(active_name_), pending.name);
            if (!NamesEqual(active_name_, "alive")) {
                boot_alive_ = false;
            }
            ESP_LOGI(kTag, "Gesture %s", active_name_);
        }
        return;
    }
    if (pending.has_pose) {
        int pan = pending.set_pan ? pending.pan : dual_servo::RoundToInt(pan_);
        int tilt = pending.set_tilt ? pending.tilt : dual_servo::RoundToInt(tilt_);
        pose_frame_ =
            dual_servo::Keyframe{pan, tilt, dual_servo::kPoseMoveMs, dual_servo::Ease::kSmooth};
        player_.Start(&pose_frame_, 1, ambient_style_, pan_, tilt_);
        active_name_[0] = '\0';
        boot_alive_ = false;
        ESP_LOGI(kTag, "Move to pan %d tilt %d", pan, tilt);
    }
}

void DualServoController::TaskEntry(void* arg) {
    static_cast<DualServoController*>(arg)->TaskLoop();
    vTaskDelete(nullptr);
}

void DualServoController::TaskLoop() {
    int64_t last_us = esp_timer_get_time();
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(20));
        int64_t now_us = esp_timer_get_time();
        float dt_ms = static_cast<float>(now_us - last_us) / 1000.f;
        last_us = now_us;
        if (dt_ms < 0.f) {
            dt_ms = 0.f;
        } else if (dt_ms > 40.f) {
            dt_ms = 40.f;
        }

        ApplyPending();
        if (player_.active) {
            dual_servo::AmbientStyle after = player_.after;
            if (!player_.Step(dt_ms, pan_, tilt_)) {
                ambient_style_ = after;
                if (NamesEqual(active_name_, "alive")) {
                    boot_alive_ = false;
                }
            }
        } else {
            DeviceState state = Application::GetInstance().GetDeviceState();
            dual_servo::Pose target =
                dual_servo::AmbientPose(state, ambient_style_, idle_enabled_.load(), now_us / 1000);
            float step = dual_servo::kTrackDegPerSec * (dt_ms / 1000.f);
            pan_ = dual_servo::MoveTowards(pan_, target.pan, step);
            tilt_ = dual_servo::MoveTowards(tilt_, target.tilt, step);
            pan_ = dual_servo::ClampPan(pan_);
            tilt_ = dual_servo::ClampTilt(tilt_);
        }
        WriteOutputs();
    }
}

void DualServoController::WriteOutputs() {
    auto write_channel = [](bool enabled, ledc_channel_t channel, float angle,
                            uint32_t& last_duty) {
        if (!enabled) {
            return;
        }
        uint32_t duty = dual_servo::DutyForPulse(dual_servo::PulseUsForAngle(angle));
        if (duty == last_duty) {
            return;
        }
        if (ledc_set_duty(kSpeedMode, channel, duty) != ESP_OK) {
            return;
        }
        if (ledc_update_duty(kSpeedMode, channel) != ESP_OK) {
            return;
        }
        last_duty = duty;
    };

    write_channel(pan_ok_, kPanChannel, dual_servo::ClampPan(pan_), pan_duty_);
    write_channel(tilt_ok_, kTiltChannel, dual_servo::ClampTilt(tilt_), tilt_duty_);
    pan_deg_.store(dual_servo::RoundToInt(pan_));
    tilt_deg_.store(dual_servo::RoundToInt(tilt_));
}
