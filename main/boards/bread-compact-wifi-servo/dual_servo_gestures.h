#ifndef DUAL_SERVO_GESTURES_H
#define DUAL_SERVO_GESTURES_H

// Head choreography for the bread-compact-wifi-servo SG90 pan/tilt rig.
// Motion ideas follow the face demo: smoothstep, a little overshoot, named
// actions, and ambient motion that a non-blocking task can interrupt.
//
// Mechanical convention (center both horns at 90 before tightening):
//   pan  50 = look left,  90 = center, 130 = look right
//   tilt 55 = look up,    90 = center, 125 = look down

#include "device_state.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace dual_servo {

constexpr int kPanMin = 50;
constexpr int kPanMax = 130;
constexpr int kPanCenter = 90;
constexpr int kTiltMin = 55;
constexpr int kTiltMax = 125;
constexpr int kTiltCenter = 90;

constexpr uint32_t kMinPulseUs = 500;
constexpr uint32_t kMaxPulseUs = 2500;
constexpr uint32_t kPwmPeriodUs = 20000;  // 50 Hz
constexpr int kPwmBits = 14;
constexpr uint16_t kPoseMoveMs = 280;
constexpr float kTrackDegPerSec = 70.f;
constexpr float kPi = 3.14159265f;

enum class Ease : uint8_t { kSmooth, kOvershoot };

enum class AmbientStyle : uint8_t { kNeutral, kSleepy, kSad };

struct Keyframe {
    int pan;
    int tilt;
    uint16_t duration_ms;
    Ease ease;
};

struct Pose {
    float pan;
    float tilt;
};

struct EmotionHit {
    enum class Kind : uint8_t { kUnknown, kNeutral, kGesture } kind = Kind::kUnknown;
    const Keyframe* frames = nullptr;
    uint8_t count = 0;
    AmbientStyle after = AmbientStyle::kNeutral;
    char name[24] = {};
};

struct ActionHit {
    enum class Kind : uint8_t { kUnknown, kGesture, kIdleOn, kIdleOff } kind = Kind::kUnknown;
    const Keyframe* frames = nullptr;
    uint8_t count = 0;
    AmbientStyle after = AmbientStyle::kNeutral;
    char name[24] = {};
};

inline float ClampPan(float angle) {
    if (angle < static_cast<float>(kPanMin)) {
        return static_cast<float>(kPanMin);
    }
    if (angle > static_cast<float>(kPanMax)) {
        return static_cast<float>(kPanMax);
    }
    return angle;
}

inline float ClampTilt(float angle) {
    if (angle < static_cast<float>(kTiltMin)) {
        return static_cast<float>(kTiltMin);
    }
    if (angle > static_cast<float>(kTiltMax)) {
        return static_cast<float>(kTiltMax);
    }
    return angle;
}

inline int ClampPanInt(int angle) {
    if (angle < kPanMin) {
        return kPanMin;
    }
    if (angle > kPanMax) {
        return kPanMax;
    }
    return angle;
}

inline int ClampTiltInt(int angle) {
    if (angle < kTiltMin) {
        return kTiltMin;
    }
    if (angle > kTiltMax) {
        return kTiltMax;
    }
    return angle;
}

inline int RoundToInt(float value) {
    return static_cast<int>(value >= 0.f ? value + 0.5f : value - 0.5f);
}

inline constexpr uint32_t PulseUsForAngle(float angle) {
    float clamped = angle;
    if (clamped < 0.f) {
        clamped = 0.f;
    } else if (clamped > 180.f) {
        clamped = 180.f;
    }
    return static_cast<uint32_t>(kMinPulseUs + (kMaxPulseUs - kMinPulseUs) * (clamped / 180.f) +
                                 0.5f);
}

inline constexpr uint32_t DutyForPulse(uint32_t pulse_us) {
    constexpr uint32_t kMaxDuty = (1u << kPwmBits) - 1u;
    return pulse_us * kMaxDuty / kPwmPeriodUs;
}

static_assert(PulseUsForAngle(0.f) == 500u);
static_assert(PulseUsForAngle(90.f) == 1500u);
static_assert(PulseUsForAngle(180.f) == 2500u);
static_assert(DutyForPulse(1500u) == 1228u);

inline float Smoothstep(float t) {
    if (t <= 0.f) {
        return 0.f;
    }
    if (t >= 1.f) {
        return 1.f;
    }
    return t * t * (3.f - 2.f * t);
}

// Slightly past the target, then settle. The result may leave 0..1; callers clamp angles.
inline float EaseOutBack(float t) {
    if (t <= 0.f) {
        return 0.f;
    }
    if (t >= 1.f) {
        return 1.f;
    }
    constexpr float kC1 = 1.70158f;
    constexpr float kC3 = kC1 + 1.f;
    float u = t - 1.f;
    return 1.f + kC3 * u * u * u + kC1 * u * u;
}

inline float MoveTowards(float current, float target, float max_step) {
    if (max_step < 0.f) {
        max_step = 0.f;
    }
    float delta = target - current;
    if (delta > max_step) {
        return current + max_step;
    }
    if (delta < -max_step) {
        return current - max_step;
    }
    return target;
}

template <size_t N>
constexpr bool FramesInRange(const Keyframe (&frames)[N]) {
    for (size_t i = 0; i < N; ++i) {
        if (frames[i].pan < kPanMin || frames[i].pan > kPanMax) {
            return false;
        }
        if (frames[i].tilt < kTiltMin || frames[i].tilt > kTiltMax) {
            return false;
        }
        if (frames[i].duration_ms == 0) {
            return false;
        }
    }
    return N > 0;
}

// clang-format off
inline constexpr Keyframe kCenter[] = {
    {90, 90, 400, Ease::kSmooth},
};
inline constexpr Keyframe kShake[] = {
    {72, 90, 90, Ease::kOvershoot},
    {108, 90, 90, Ease::kOvershoot},
    {76, 90, 80, Ease::kOvershoot},
    {104, 90, 80, Ease::kOvershoot},
    {84, 90, 70, Ease::kSmooth},
    {90, 90, 140, Ease::kSmooth},
};
inline constexpr Keyframe kNod[] = {
    {90, 78, 140, Ease::kOvershoot},
    {90, 98, 130, Ease::kSmooth},
    {90, 76, 140, Ease::kOvershoot},
    {90, 90, 180, Ease::kSmooth},
};
inline constexpr Keyframe kCurious[] = {
    {70, 80, 350, Ease::kSmooth},
    {110, 78, 400, Ease::kSmooth},
    {90, 70, 300, Ease::kSmooth},
    {90, 90, 350, Ease::kSmooth},
};
inline constexpr Keyframe kDizzy[] = {
    {70, 100, 120, Ease::kSmooth},
    {110, 80, 120, Ease::kSmooth},
    {68, 82, 120, Ease::kSmooth},
    {112, 104, 120, Ease::kSmooth},
    {80, 96, 120, Ease::kSmooth},
    {100, 84, 120, Ease::kSmooth},
    {90, 90, 280, Ease::kSmooth},
};
inline constexpr Keyframe kLookLeft[] = {
    {58, 84, 320, Ease::kOvershoot},
    {58, 84, 700, Ease::kSmooth},
    {90, 90, 350, Ease::kSmooth},
};
inline constexpr Keyframe kLookRight[] = {
    {122, 84, 320, Ease::kOvershoot},
    {122, 84, 700, Ease::kSmooth},
    {90, 90, 350, Ease::kSmooth},
};
inline constexpr Keyframe kAlive[] = {
    {90, 72, 280, Ease::kOvershoot},
    {68, 86, 260, Ease::kSmooth},
    {112, 86, 280, Ease::kSmooth},
    {90, 80, 180, Ease::kOvershoot},
    {90, 96, 160, Ease::kSmooth},
    {90, 90, 280, Ease::kSmooth},
};
inline constexpr Keyframe kHappy[] = {
    {90, 76, 150, Ease::kOvershoot},
    {90, 94, 120, Ease::kSmooth},
    {88, 74, 150, Ease::kOvershoot},
    {92, 90, 180, Ease::kSmooth},
};
inline constexpr Keyframe kLaughing[] = {
    {86, 74, 110, Ease::kOvershoot},
    {96, 96, 100, Ease::kOvershoot},
    {84, 72, 110, Ease::kOvershoot},
    {98, 94, 100, Ease::kOvershoot},
    {90, 78, 140, Ease::kOvershoot},
    {90, 90, 160, Ease::kSmooth},
};
inline constexpr Keyframe kFunny[] = {
    {108, 80, 180, Ease::kOvershoot},
    {108, 96, 140, Ease::kSmooth},
    {100, 78, 160, Ease::kOvershoot},
    {90, 90, 200, Ease::kSmooth},
};
inline constexpr Keyframe kLoving[] = {
    {98, 108, 320, Ease::kSmooth},
    {96, 100, 200, Ease::kSmooth},
    {100, 112, 220, Ease::kOvershoot},
    {94, 104, 240, Ease::kSmooth},
    {90, 96, 280, Ease::kSmooth},
};
inline constexpr Keyframe kKissy[] = {
    {104, 110, 280, Ease::kSmooth},
    {100, 102, 180, Ease::kSmooth},
    {106, 114, 200, Ease::kOvershoot},
    {90, 94, 300, Ease::kSmooth},
};
inline constexpr Keyframe kEmbarrassed[] = {
    {114, 116, 300, Ease::kSmooth},
    {116, 118, 400, Ease::kSmooth},
    {90, 90, 450, Ease::kSmooth},
};
inline constexpr Keyframe kSad[] = {
    {90, 116, 500, Ease::kSmooth},
    {78, 114, 650, Ease::kSmooth},
    {102, 118, 650, Ease::kSmooth},
    {90, 112, 500, Ease::kSmooth},
};
inline constexpr Keyframe kCrying[] = {
    {90, 120, 600, Ease::kSmooth},
    {74, 118, 800, Ease::kSmooth},
    {106, 122, 800, Ease::kSmooth},
    {82, 120, 700, Ease::kSmooth},
    {90, 116, 600, Ease::kSmooth},
};
inline constexpr Keyframe kSurprised[] = {
    {90, 60, 110, Ease::kOvershoot},
    {90, 62, 420, Ease::kSmooth},
    {90, 84, 260, Ease::kSmooth},
};
inline constexpr Keyframe kShocked[] = {
    {80, 78, 80, Ease::kSmooth},
    {100, 60, 100, Ease::kOvershoot},
    {90, 58, 480, Ease::kSmooth},
    {90, 86, 280, Ease::kSmooth},
};
inline constexpr Keyframe kThinking[] = {
    {108, 78, 420, Ease::kSmooth},
    {114, 74, 360, Ease::kSmooth},
    {108, 80, 280, Ease::kSmooth},
    {90, 90, 400, Ease::kSmooth},
};
inline constexpr Keyframe kConfused[] = {
    {72, 80, 300, Ease::kSmooth},
    {112, 76, 340, Ease::kSmooth},
    {100, 100, 280, Ease::kSmooth},
    {90, 90, 320, Ease::kSmooth},
};
inline constexpr Keyframe kSleepy[] = {
    {90, 102, 700, Ease::kSmooth},
    {90, 96, 700, Ease::kSmooth},
    {90, 106, 800, Ease::kSmooth},
    {90, 100, 700, Ease::kSmooth},
};
inline constexpr Keyframe kRelaxed[] = {
    {86, 100, 800, Ease::kSmooth},
    {94, 104, 800, Ease::kSmooth},
    {90, 100, 700, Ease::kSmooth},
};
inline constexpr Keyframe kWinking[] = {
    {112, 82, 180, Ease::kOvershoot},
    {112, 82, 260, Ease::kSmooth},
    {90, 90, 240, Ease::kSmooth},
};
inline constexpr Keyframe kSilly[] = {
    {74, 98, 140, Ease::kSmooth},
    {108, 80, 140, Ease::kSmooth},
    {76, 86, 140, Ease::kSmooth},
    {104, 100, 140, Ease::kSmooth},
    {90, 90, 220, Ease::kSmooth},
};
inline constexpr Keyframe kCool[] = {
    {118, 86, 420, Ease::kSmooth},
    {118, 88, 500, Ease::kSmooth},
    {90, 90, 380, Ease::kSmooth},
};
inline constexpr Keyframe kConfident[] = {
    {90, 74, 260, Ease::kOvershoot},
    {94, 80, 200, Ease::kSmooth},
    {90, 90, 280, Ease::kSmooth},
};
inline constexpr Keyframe kDelicious[] = {
    {96, 108, 280, Ease::kSmooth},
    {96, 98, 160, Ease::kOvershoot},
    {98, 110, 180, Ease::kOvershoot},
    {90, 94, 260, Ease::kSmooth},
};
// clang-format on

static_assert(FramesInRange(kCenter));
static_assert(FramesInRange(kShake));
static_assert(FramesInRange(kNod));
static_assert(FramesInRange(kCurious));
static_assert(FramesInRange(kDizzy));
static_assert(FramesInRange(kLookLeft));
static_assert(FramesInRange(kLookRight));
static_assert(FramesInRange(kAlive));
static_assert(FramesInRange(kHappy));
static_assert(FramesInRange(kLaughing));
static_assert(FramesInRange(kFunny));
static_assert(FramesInRange(kLoving));
static_assert(FramesInRange(kKissy));
static_assert(FramesInRange(kEmbarrassed));
static_assert(FramesInRange(kSad));
static_assert(FramesInRange(kCrying));
static_assert(FramesInRange(kSurprised));
static_assert(FramesInRange(kShocked));
static_assert(FramesInRange(kThinking));
static_assert(FramesInRange(kConfused));
static_assert(FramesInRange(kSleepy));
static_assert(FramesInRange(kRelaxed));
static_assert(FramesInRange(kWinking));
static_assert(FramesInRange(kSilly));
static_assert(FramesInRange(kCool));
static_assert(FramesInRange(kConfident));
static_assert(FramesInRange(kDelicious));

inline void CopyKey(char* dst, size_t dst_len, const char* src) {
    if (dst_len == 0) {
        return;
    }
    size_t i = 0;
    if (src != nullptr) {
        for (; src[i] != '\0' && i + 1 < dst_len; ++i) {
            dst[i] = src[i];
        }
    }
    dst[i] = '\0';
}

inline bool NormalizeKey(const char* in, char* out, size_t out_len) {
    if (out == nullptr || out_len == 0) {
        return false;
    }
    out[0] = '\0';
    if (in == nullptr) {
        return false;
    }
    size_t n = 0;
    for (size_t i = 0; in[i] != '\0'; ++i) {
        unsigned char c = static_cast<unsigned char>(in[i]);
        if (c == ' ' || c == '\t' || c == '\n' || c == '\r') {
            continue;
        }
        if (c >= 'A' && c <= 'Z') {
            c = static_cast<unsigned char>(c - 'A' + 'a');
        }
        if (n + 1 >= out_len) {
            out[0] = '\0';
            return false;
        }
        out[n++] = static_cast<char>(c);
    }
    out[n] = '\0';
    return n > 0;
}

inline bool KeyEq(const char* key, const char* literal) { return std::strcmp(key, literal) == 0; }

template <size_t N>
inline EmotionHit MakeGesture(const char* name, const Keyframe (&frames)[N], AmbientStyle after) {
    EmotionHit hit;
    hit.kind = EmotionHit::Kind::kGesture;
    hit.frames = frames;
    hit.count = static_cast<uint8_t>(N);
    hit.after = after;
    CopyKey(hit.name, sizeof(hit.name), name);
    return hit;
}

template <size_t N>
inline ActionHit MakeAction(const char* name, const Keyframe (&frames)[N], AmbientStyle after) {
    ActionHit hit;
    hit.kind = ActionHit::Kind::kGesture;
    hit.frames = frames;
    hit.count = static_cast<uint8_t>(N);
    hit.after = after;
    CopyKey(hit.name, sizeof(hit.name), name);
    return hit;
}

inline EmotionHit LookupEmotion(const char* emotion) {
    char key[24];
    if (!NormalizeKey(emotion, key, sizeof(key))) {
        return {};
    }
    if (KeyEq(key, "neutral")) {
        EmotionHit hit;
        hit.kind = EmotionHit::Kind::kNeutral;
        CopyKey(hit.name, sizeof(hit.name), "neutral");
        return hit;
    }
    if (KeyEq(key, "happy")) {
        return MakeGesture("happy", kHappy, AmbientStyle::kNeutral);
    }
    if (KeyEq(key, "laughing")) {
        return MakeGesture("laughing", kLaughing, AmbientStyle::kNeutral);
    }
    if (KeyEq(key, "funny")) {
        return MakeGesture("funny", kFunny, AmbientStyle::kNeutral);
    }
    if (KeyEq(key, "loving")) {
        return MakeGesture("loving", kLoving, AmbientStyle::kNeutral);
    }
    if (KeyEq(key, "kissy")) {
        return MakeGesture("kissy", kKissy, AmbientStyle::kNeutral);
    }
    if (KeyEq(key, "embarrassed")) {
        return MakeGesture("embarrassed", kEmbarrassed, AmbientStyle::kNeutral);
    }
    if (KeyEq(key, "sad")) {
        return MakeGesture("sad", kSad, AmbientStyle::kSad);
    }
    if (KeyEq(key, "crying")) {
        return MakeGesture("crying", kCrying, AmbientStyle::kSad);
    }
    if (KeyEq(key, "angry")) {
        return MakeGesture("angry", kShake, AmbientStyle::kNeutral);
    }
    if (KeyEq(key, "surprised")) {
        return MakeGesture("surprised", kSurprised, AmbientStyle::kNeutral);
    }
    if (KeyEq(key, "shocked")) {
        return MakeGesture("shocked", kShocked, AmbientStyle::kNeutral);
    }
    if (KeyEq(key, "thinking")) {
        return MakeGesture("thinking", kThinking, AmbientStyle::kNeutral);
    }
    if (KeyEq(key, "confused")) {
        return MakeGesture("confused", kConfused, AmbientStyle::kNeutral);
    }
    if (KeyEq(key, "sleepy")) {
        return MakeGesture("sleepy", kSleepy, AmbientStyle::kSleepy);
    }
    if (KeyEq(key, "relaxed")) {
        return MakeGesture("relaxed", kRelaxed, AmbientStyle::kSleepy);
    }
    if (KeyEq(key, "winking")) {
        return MakeGesture("winking", kWinking, AmbientStyle::kNeutral);
    }
    if (KeyEq(key, "silly")) {
        return MakeGesture("silly", kSilly, AmbientStyle::kNeutral);
    }
    if (KeyEq(key, "cool")) {
        return MakeGesture("cool", kCool, AmbientStyle::kNeutral);
    }
    if (KeyEq(key, "confident")) {
        return MakeGesture("confident", kConfident, AmbientStyle::kNeutral);
    }
    if (KeyEq(key, "delicious")) {
        return MakeGesture("delicious", kDelicious, AmbientStyle::kNeutral);
    }
    return {};
}

inline ActionHit LookupAction(const char* action) {
    char key[24];
    if (!NormalizeKey(action, key, sizeof(key))) {
        return {};
    }
    if (KeyEq(key, "center")) {
        return MakeAction("center", kCenter, AmbientStyle::kNeutral);
    }
    if (KeyEq(key, "shake")) {
        return MakeAction("shake", kShake, AmbientStyle::kNeutral);
    }
    if (KeyEq(key, "nod")) {
        return MakeAction("nod", kNod, AmbientStyle::kNeutral);
    }
    if (KeyEq(key, "curious")) {
        return MakeAction("curious", kCurious, AmbientStyle::kNeutral);
    }
    if (KeyEq(key, "dizzy")) {
        return MakeAction("dizzy", kDizzy, AmbientStyle::kNeutral);
    }
    if (KeyEq(key, "look_left")) {
        return MakeAction("look_left", kLookLeft, AmbientStyle::kNeutral);
    }
    if (KeyEq(key, "look_right")) {
        return MakeAction("look_right", kLookRight, AmbientStyle::kNeutral);
    }
    if (KeyEq(key, "alive")) {
        return MakeAction("alive", kAlive, AmbientStyle::kNeutral);
    }
    if (KeyEq(key, "idle") || KeyEq(key, "idle_on")) {
        ActionHit hit;
        hit.kind = ActionHit::Kind::kIdleOn;
        CopyKey(hit.name, sizeof(hit.name), "idle");
        return hit;
    }
    if (KeyEq(key, "idle_off")) {
        ActionHit hit;
        hit.kind = ActionHit::Kind::kIdleOff;
        CopyKey(hit.name, sizeof(hit.name), "idle_off");
        return hit;
    }
    return {};
}

inline float WrappedSine(int64_t now_ms, int64_t period_ms) {
    if (period_ms <= 0) {
        return 0.f;
    }
    int64_t wrapped = now_ms % period_ms;
    if (wrapped < 0) {
        wrapped += period_ms;
    }
    float turns = static_cast<float>(wrapped) / static_cast<float>(period_ms);
    return std::sin(turns * 2.f * kPi);
}

inline float GlancePan(int64_t now_ms, int64_t period_ms, float amount) {
    float pan = static_cast<float>(kPanCenter);
    if (period_ms <= 0) {
        return pan;
    }
    int64_t wrapped = now_ms % period_ms;
    if (wrapped < 0) {
        wrapped += period_ms;
    }
    int64_t cycle = now_ms / period_ms;
    if (cycle < 0) {
        cycle = -cycle;
    }
    if (wrapped > (period_ms * 8) / 10 && wrapped < (period_ms * 19) / 20) {
        pan += (cycle % 2 == 0) ? -amount : amount;
    }
    return ClampPan(pan);
}

inline Pose CenterPose() {
    return Pose{static_cast<float>(kPanCenter), static_cast<float>(kTiltCenter)};
}

inline Pose IdlePose(AmbientStyle style, int64_t now_ms) {
    Pose pose = CenterPose();
    if (style == AmbientStyle::kSleepy) {
        pose.tilt = 102.f + 2.f * WrappedSine(now_ms, 5200);
    } else if (style == AmbientStyle::kSad) {
        pose.tilt = 114.f + 1.5f * WrappedSine(now_ms, 5000);
        pose.pan = static_cast<float>(kPanCenter) + 8.f * WrappedSine(now_ms, 8000);
    } else {
        pose.tilt = static_cast<float>(kTiltCenter) + 2.5f * WrappedSine(now_ms, 3800);
        pose.pan = GlancePan(now_ms, 9000, 10.f);
    }
    pose.pan = ClampPan(pose.pan);
    pose.tilt = ClampTilt(pose.tilt);
    return pose;
}

inline Pose ListeningPose(int64_t now_ms) {
    Pose pose;
    pose.tilt = 76.f + 1.5f * WrappedSine(now_ms, 2800);
    pose.pan = GlancePan(now_ms, 6000, 8.f);
    pose.pan = ClampPan(pose.pan);
    pose.tilt = ClampTilt(pose.tilt);
    return pose;
}

inline Pose SpeakingPose(int64_t now_ms) {
    Pose pose = CenterPose();
    pose.tilt = static_cast<float>(kTiltCenter) + 4.f * WrappedSine(now_ms, 900);
    pose.tilt = ClampTilt(pose.tilt);
    return pose;
}

inline Pose AmbientPose(DeviceState state, AmbientStyle style, bool idle_enabled, int64_t now_ms) {
    switch (state) {
        case kDeviceStateListening:
        case kDeviceStateConnecting:
            return ListeningPose(now_ms);
        case kDeviceStateSpeaking:
        case kDeviceStateNotifying:
            // A lingering sad/sleepy pose stays while the line is spoken. Otherwise nod softly.
            if (style != AmbientStyle::kNeutral) {
                return IdlePose(style, now_ms);
            }
            return SpeakingPose(now_ms);
        case kDeviceStateIdle:
            if (!idle_enabled) {
                return CenterPose();
            }
            return IdlePose(style, now_ms);
        default:
            return CenterPose();
    }
}

struct ScriptPlayer {
    const Keyframe* frames = nullptr;
    uint8_t count = 0;
    uint8_t index = 0;
    float elapsed_ms = 0.f;
    float pan_start = static_cast<float>(kPanCenter);
    float tilt_start = static_cast<float>(kTiltCenter);
    bool active = false;
    AmbientStyle after = AmbientStyle::kNeutral;

    void Start(const Keyframe* script, uint8_t script_count, AmbientStyle style, float pan,
               float tilt) {
        frames = script;
        count = script_count;
        index = 0;
        elapsed_ms = 0.f;
        pan_start = pan;
        tilt_start = tilt;
        after = style;
        active = script != nullptr && script_count > 0;
    }

    void Cancel() {
        active = false;
        frames = nullptr;
        count = 0;
        index = 0;
        elapsed_ms = 0.f;
    }

    // Advances the script by dt_ms. Writes the new pose and returns false once it has finished.
    bool Step(float dt_ms, float& pan, float& tilt) {
        if (!active || frames == nullptr || count == 0) {
            active = false;
            return false;
        }
        float left = dt_ms > 0.f ? dt_ms : 0.f;
        int guard = static_cast<int>(count) + 2;
        while (active && guard-- > 0) {
            const Keyframe& frame = frames[index];
            float duration = frame.duration_ms > 0 ? static_cast<float>(frame.duration_ms) : 1.f;
            float need = duration - elapsed_ms;
            if (need > 0.05f && left + 0.01f < need) {
                elapsed_ms += left;
                float u = elapsed_ms / duration;
                float eased = frame.ease == Ease::kOvershoot ? EaseOutBack(u) : Smoothstep(u);
                pan = ClampPan(pan_start + (static_cast<float>(frame.pan) - pan_start) * eased);
                tilt =
                    ClampTilt(tilt_start + (static_cast<float>(frame.tilt) - tilt_start) * eased);
                return true;
            }
            if (need > 0.f) {
                left -= need;
            }
            pan = ClampPan(static_cast<float>(frame.pan));
            tilt = ClampTilt(static_cast<float>(frame.tilt));
            elapsed_ms = 0.f;
            pan_start = pan;
            tilt_start = tilt;
            if (static_cast<size_t>(index) + 1 >= count) {
                active = false;
                return false;
            }
            index++;
            if (left <= 0.05f) {
                return true;
            }
        }
        return active;
    }
};

}  // namespace dual_servo

#endif  // DUAL_SERVO_GESTURES_H
