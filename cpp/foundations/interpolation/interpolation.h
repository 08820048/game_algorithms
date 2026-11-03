#pragma once

#include <algorithm>
#include <cmath>

#include "vec2.h"

namespace ga {

// #region lerp
// 线性插值：t = 0 得到 a，t = 1 得到 b
constexpr float lerp(float a, float b, float t) { return a + (b - a) * t; }
constexpr Vec2 lerp(Vec2 a, Vec2 b, float t) { return a + (b - a) * t; }

// 反向插值：v 落在 [a, b] 的哪个比例上
constexpr float inverseLerp(float a, float b, float v) { return (v - a) / (b - a); }

// 把 v 从区间 [inMin, inMax] 映射到 [outMin, outMax]，例如血量 -> 血条宽度
constexpr float remap(float v, float inMin, float inMax, float outMin, float outMax) {
    return lerp(outMin, outMax, inverseLerp(inMin, inMax, v));
}

constexpr float clamp01(float t) { return t < 0 ? 0 : (t > 1 ? 1 : t); }
// #endregion

// #region lerp-angle
// 角度插值：先把差值规整到 (-π, π]，保证走"短的那一边"
inline float lerpAngle(float a, float b, float t) {
    float delta = std::remainder(b - a, 2.0f * kPi);
    return a + delta * t;
}

// 二维方向的球面插值（slerp）：在两个单位向量之间匀速转动
inline Vec2 slerp(Vec2 from, Vec2 to, float t) { return rotate(from, signedAngle(from, to) * t); }
// #endregion

// #region easing
// 缓动函数：输入 t ∈ [0,1]，输出"进度"。都满足 f(0) = 0、f(1) = 1
namespace ease {
inline float linear(float t) { return t; }
inline float inQuad(float t) { return t * t; }
inline float outQuad(float t) { return 1 - (1 - t) * (1 - t); }
inline float inOutQuad(float t) { return t < 0.5f ? 2 * t * t : 1 - std::pow(-2 * t + 2, 2.0f) / 2; }
inline float inCubic(float t) { return t * t * t; }
inline float outCubic(float t) { return 1 - std::pow(1 - t, 3.0f); }
inline float inOutCubic(float t) { return t < 0.5f ? 4 * t * t * t : 1 - std::pow(-2 * t + 2, 3.0f) / 2; }
inline float inOutSine(float t) { return -(std::cos(kPi * t) - 1) / 2; }
inline float smoothstep(float t) { return t * t * (3 - 2 * t); }

// 先略微超过终点再回来，适合弹出的 UI
inline float outBack(float t) {
    const float c1 = 1.70158f, c3 = c1 + 1;
    return 1 + c3 * std::pow(t - 1, 3.0f) + c1 * std::pow(t - 1, 2.0f);
}

// 像弹簧一样来回震荡后停下
inline float outElastic(float t) {
    if (t <= 0) return 0;
    if (t >= 1) return 1;
    const float c4 = 2 * kPi / 3;
    return std::pow(2.0f, -10 * t) * std::sin((t * 10 - 0.75f) * c4) + 1;
}

// 落地弹跳
inline float outBounce(float t) {
    const float n1 = 7.5625f, d1 = 2.75f;
    if (t < 1 / d1) return n1 * t * t;
    if (t < 2 / d1) { t -= 1.5f / d1; return n1 * t * t + 0.75f; }
    if (t < 2.5f / d1) { t -= 2.25f / d1; return n1 * t * t + 0.9375f; }
    t -= 2.625f / d1;
    return n1 * t * t + 0.984375f;
}
}  // namespace ease

using EaseFn = float (*)(float);

// 用缓动函数驱动一段从 a 到 b、时长 duration 的补间（tween）
inline float tween(float a, float b, float elapsed, float duration, EaseFn ease) {
    return lerp(a, b, ease(clamp01(elapsed / duration)));
}
// #endregion

// #region naive-follow
// 常见但有问题的写法：每帧向目标靠近 10%。效果和帧率绑定！
inline Vec2 naiveFollow(Vec2 current, Vec2 target, float factorPerFrame) {
    return lerp(current, target, factorPerFrame);
}
// #endregion

// #region exp-decay
// 与帧率无关的平滑跟随：剩余距离按指数衰减，decay 越大跟得越紧
inline float expDecay(float current, float target, float decay, float dt) {
    return target + (current - target) * std::exp(-decay * dt);
}
inline Vec2 expDecay(Vec2 current, Vec2 target, float decay, float dt) {
    return target + (current - target) * std::exp(-decay * dt);
}
// #endregion

// #region smooth-damp
// 临界阻尼弹簧（Game Programming Gems 4, 1.10）：带速度，起步和停下都很柔和，不会过冲
// smoothTime 约等于追上目标所需的时间；velocity 由调用方保存，每帧传入
inline float smoothDamp(float current, float target, float& velocity, float smoothTime, float dt) {
    smoothTime = std::max(0.0001f, smoothTime);
    float omega = 2.0f / smoothTime;
    float x = omega * dt;
    float decay = 1.0f / (1.0f + x + 0.48f * x * x + 0.235f * x * x * x);  // ≈ e^-x
    float change = current - target;
    float temp = (velocity + omega * change) * dt;
    velocity = (velocity - omega * temp) * decay;
    return target + (change + temp) * decay;
}

inline Vec2 smoothDamp(Vec2 current, Vec2 target, Vec2& velocity, float smoothTime, float dt) {
    return {smoothDamp(current.x, target.x, velocity.x, smoothTime, dt),
            smoothDamp(current.y, target.y, velocity.y, smoothTime, dt)};
}
// #endregion

}  // namespace ga
