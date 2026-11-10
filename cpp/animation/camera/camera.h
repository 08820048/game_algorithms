#pragma once

// 2D 相机：跟随、死区、平台吸附、前视、震屏
//   - 跟随：相机中心追一个"期望位置"。期望位置怎么定，比怎么追更重要
//   - 死区（窗口）：角色在屏幕中间的一个框里走动时相机不动，走出框时相机被框边推着走
//   - 平台吸附：竖直方向只在角色落地时更新期望高度，跳跃时相机不上下晃
//   - 前视：期望位置朝角色前进的方向偏一段，让玩家看到前面；转身时偏移要换边，这一下是最大的晃动来源
//   - 震屏：trauma ∈ [0, 1] 随时间线性衰减，抖动幅度 = trauma^exponent；抖动的来源是每帧随机数或连续噪声
// 平滑用第 0.3 节的 smoothDamp（临界阻尼弹簧）和指数衰减

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "../../foundations/interpolation/interpolation.h"
#include "../../foundations/random/random.h"
#include "../../procgen/noise/noise.h"
#include "vec2.h"

namespace ga::camera {

// #region deadzone
// 死区：角色超出以相机为中心、半宽 half 的框时，把相机推到角色刚好在框边上的位置。框内时相机不动
inline float deadZoneAxis(float cam, float target, float half) {
    if (target > cam + half) return target - half;
    if (target < cam - half) return target + half;
    return cam;
}
inline Vec2 deadZone(Vec2 cam, Vec2 target, Vec2 half) { return {deadZoneAxis(cam.x, target.x, half.x), deadZoneAxis(cam.y, target.y, half.y)}; }
// #endregion

// #region rig
struct RigOptions {
    bool smooth = true;          // 用 smoothDamp 追期望位置；否则直接跳过去
    float smoothTime = 0.25f;
    Vec2 deadZone{0, 0};         // 半宽；0 = 没有死区
    bool platformSnap = false;   // 竖直方向只在落地时更新
    float snapMargin = 120;      // 吸附时，角色离相机中心超过这么远（掉下悬崖、跳得太高）仍然跟
    float lookAhead = 0;         // 前视距离；0 = 不前视
    float lookAheadTime = 0.5f;  // 前视偏移换边所需的时间（smoothDamp 的 smoothTime）
    float turnHysteresis = 0;    // 角色往回走超过这段距离才承认转身（避免原地左右小碎步导致相机来回甩）
};

struct PlayerSample {
    Vec2 pos;
    Vec2 vel;
    bool grounded;
};

class Rig {
public:
    explicit Rig(const RigOptions& o = {}) : o_(o) {}
    void reset(Vec2 p) {
        cam_ = goal_ = p, vel_ = {0, 0};
        groundY_ = p.y, facing_ = 1, look_ = o_.lookAhead, lookVel_ = 0, turnAnchor_ = p.x;
        initialised_ = true;
    }

    void update(const PlayerSample& s, float dt) {
        if (!initialised_) reset(s.pos);
        // 1. 竖直方向的跟随目标：平台吸附时用最近一次落地的高度
        float ty = s.pos.y;
        if (o_.platformSnap) {
            if (s.grounded) groundY_ = s.pos.y;
            ty = std::fabs(s.pos.y - groundY_) > o_.snapMargin ? s.pos.y - std::copysign(o_.snapMargin, s.pos.y - groundY_) : groundY_;
        }
        // 2. 前视：朝向由移动方向决定；有滞回时，反向走出 turnHysteresis 才换边
        if (o_.lookAhead > 0) {
            if (o_.turnHysteresis > 0) {
                if ((s.pos.x - turnAnchor_) * float(facing_) > 0) turnAnchor_ = s.pos.x;  // 一直沿当前朝向走：锚点跟着走
                else if (std::fabs(s.pos.x - turnAnchor_) > o_.turnHysteresis) facing_ = -facing_, turnAnchor_ = s.pos.x;
            } else if (std::fabs(s.vel.x) > 1) {
                facing_ = s.vel.x > 0 ? 1 : -1;
            }
            look_ = smoothDamp(look_, o_.lookAhead * float(facing_), lookVel_, o_.lookAheadTime, dt);
        }
        Vec2 target{s.pos.x + look_ * (o_.lookAhead > 0 ? 1.0f : 0.0f), ty};
        // 3. 死区：期望位置只在目标走出框时移动
        goal_ = o_.deadZone.x > 0 || o_.deadZone.y > 0 ? deadZone(goal_, target, o_.deadZone) : target;
        // 4. 平滑地追期望位置
        if (o_.smooth) cam_ = smoothDamp(cam_, goal_, vel_, o_.smoothTime, dt);
        else cam_ = goal_;
    }

    Vec2 position() const { return cam_; }
    Vec2 goal() const { return goal_; }
    float lookOffset() const { return look_; }
    int facing() const { return facing_; }
    float groundY() const { return groundY_; }
    const RigOptions& options() const { return o_; }
    RigOptions& options() { return o_; }

private:
    RigOptions o_;
    Vec2 cam_{}, goal_{}, vel_{};
    float groundY_ = 0, look_ = 0, lookVel_ = 0, turnAnchor_ = 0;
    int facing_ = 1;
    bool initialised_ = false;
};
// #endregion

// #region shake
// 一维梯度噪声：整数格点上放随机斜率，格点之间用 quintic 缓和插值（第 4.1 节）。值域约 [-0.5, 0.5]，这里放大到约 [-1, 1]
inline float noise1(uint32_t seed, float t) {
    float fl = std::floor(t);
    int i = int(fl);
    auto grad = [&](int k) { return float(noise::mix32(uint32_t(k) ^ noise::mix32(seed)) >> 8) * (2.0f / 16777215.0f) - 1; };
    return 2 * noise::gradientSegment(grad(i), grad(i + 1), t - fl, noise::FadeCurve::Quintic);
}

enum class ShakeSource { Random, Noise };
struct ShakeOptions {
    ShakeSource source = ShakeSource::Noise;
    float maxOffset = 24;     // 像素
    float maxAngle = 0.05f;   // 弧度
    float frequency = 18;     // 噪声每秒穿过多少个格点
    float decay = 1.2f;       // trauma 每秒减少多少
    float exponent = 2;       // 抖动幅度 = trauma^exponent
    bool staggered = true;    // 三个通道在时间上错开（梯度噪声在整数格点上为 0，不错开时三个通道同时归零）
    uint64_t seed = 1;
};

class Shake {
public:
    explicit Shake(const ShakeOptions& o = {}) : o_(o), rng_(o.seed) {}
    void add(float amount) { trauma_ = std::min(1.0f, trauma_ + amount); }
    void update(float dt) {
        time_ += dt;
        trauma_ = std::max(0.0f, trauma_ - o_.decay * dt);
        float a = std::pow(trauma_, o_.exponent);
        float u, v, w;
        if (o_.source == ShakeSource::Random) {
            // 每帧取一个新的随机数：帧率越高，抖得越快
            u = rng_.nextFloat() * 2 - 1, v = rng_.nextFloat() * 2 - 1, w = rng_.nextFloat() * 2 - 1;
        } else {
            // 连续噪声：按时间采样，与帧率无关；三个通道用不同的种子
            float t = time_ * o_.frequency;
            uint32_t s = uint32_t(o_.seed) * 3;
            float d = o_.staggered ? 1.0f / 3 : 0;
            u = noise1(s, t), v = noise1(s + 1, t + d), w = noise1(s + 2, t + 2 * d);
        }
        offset_ = Vec2{u, v} * (o_.maxOffset * a);
        angle_ = w * o_.maxAngle * a;
    }
    float trauma() const { return trauma_; }
    Vec2 offset() const { return offset_; }
    float angle() const { return angle_; }
    ShakeOptions& options() { return o_; }

private:
    ShakeOptions o_;
    Pcg32 rng_;
    float trauma_ = 0, time_ = 0, angle_ = 0;
    Vec2 offset_{};
};
// #endregion

}  // namespace ga::camera
