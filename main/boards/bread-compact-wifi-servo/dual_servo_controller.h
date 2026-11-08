#ifndef DUAL_SERVO_CONTROLLER_H
#define DUAL_SERVO_CONTROLLER_H

#include "dual_servo_gestures.h"

#include <driver/gpio.h>

#include <atomic>
#include <cstdint>
#include <mutex>

// Pan + tilt SG90 driver. PWM writes and interpolation run on a low-priority task.
// Emotion, MCP, and ambient requests only post a one-deep mailbox, so a newer
// gesture supersedes one that has not started and retargets from the current pose.
class DualServoController {
public:
    DualServoController(gpio_num_t pan_gpio, gpio_num_t tilt_gpio);
    DualServoController(const DualServoController&) = delete;
    DualServoController& operator=(const DualServoController&) = delete;

    static void NotifyEmotion(const char* emotion);

private:
    struct Pending {
        bool has_gesture = false;
        bool release = false;
        bool soft_release = false;
        const dual_servo::Keyframe* frames = nullptr;
        uint8_t count = 0;
        dual_servo::AmbientStyle after = dual_servo::AmbientStyle::kNeutral;
        char name[24] = {};
        bool has_pose = false;
        bool set_pan = false;
        bool set_tilt = false;
        int pan = dual_servo::kPanCenter;
        int tilt = dual_servo::kTiltCenter;
    };

    static DualServoController* instance_;
    static void TaskEntry(void* arg);

    void InitPwm();
    void RegisterTools();
    void StartTask();
    void TaskLoop();
    void ApplyPending();
    void WriteOutputs();
    Pending TakePending();
    void PostGesture(const dual_servo::Keyframe* frames, uint8_t count,
                     dual_servo::AmbientStyle after, const char* name);
    void PostRelease(bool soft);
    void PostPose(bool set_pan, int pan, bool set_tilt, int tilt);
    bool PlayNamedAction(const char* name);

    gpio_num_t pan_gpio_;
    gpio_num_t tilt_gpio_;
    bool pan_ok_ = false;
    bool tilt_ok_ = false;
    uint32_t pan_duty_ = 0xffffffffu;
    uint32_t tilt_duty_ = 0xffffffffu;

    float pan_ = static_cast<float>(dual_servo::kPanCenter);
    float tilt_ = static_cast<float>(dual_servo::kTiltCenter);
    std::atomic<int> pan_deg_{dual_servo::kPanCenter};
    std::atomic<int> tilt_deg_{dual_servo::kTiltCenter};
    std::atomic<bool> idle_enabled_{true};

    std::mutex mu_;
    Pending pending_;
    dual_servo::ScriptPlayer player_;
    dual_servo::Keyframe pose_frame_{};
    dual_servo::AmbientStyle ambient_style_ = dual_servo::AmbientStyle::kNeutral;
    char active_name_[24] = {};
    bool boot_alive_ = false;
    int64_t boot_deadline_us_ = 0;
};

#endif  // DUAL_SERVO_CONTROLLER_H
