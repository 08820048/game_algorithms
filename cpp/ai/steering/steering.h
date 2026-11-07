#pragma once

// 转向行为（steering behaviors，Craig Reynolds 1999）：
//   每个行为根据自己和目标的状态，算出一个"想要的速度"（desired velocity），
//   转向力就是把当前速度扭向它所需的加速度。力受上限约束，速度也受上限约束，
//   所以角色不会瞬间掉头，而是画出一条自然的弧线。
//
//   多个行为的力可以加权相加，也可以按优先级分配力的额度（见 accumulate）。

#include <algorithm>
#include <cmath>
#include <vector>

#include "../../foundations/random/random.h"
#include "vec2.h"

namespace ga::steering {

// #region agent
struct Agent {
    Vec2 pos, vel;
    float maxSpeed = 120;  // 像素 / 秒
    float maxForce = 360;  // 像素 / 秒²（质量取 1，力就是加速度）
    float reaction = 0.1f; // 反应时间：不受力上限约束时，用多少秒把速度扭到想要的速度
    float radius = 8;
};

// 长度超过 max 时缩短到 max，方向不变
inline Vec2 truncate(Vec2 v, float max) {
    float l2 = lengthSq(v);
    return l2 > max * max ? v * (max / std::sqrt(l2)) : v;
}

// 转向力 = (想要的速度 - 当前速度) / 反应时间
inline Vec2 steerToward(const Agent& a, Vec2 desired) { return (desired - a.vel) / a.reaction; }

// 半隐式欧拉（见 2.7）：先更新速度，再用新速度更新位置
inline void integrate(Agent& a, Vec2 force, float dt) {
    a.vel = truncate(a.vel + truncate(force, a.maxForce) * dt, a.maxSpeed);
    a.pos += a.vel * dt;
}
// #endregion

// #region seek-flee
// 寻找：全速冲向目标。到达后会冲过头，再折返，绕着目标打转
inline Vec2 seek(const Agent& a, Vec2 target) { return steerToward(a, normalized(target - a.pos) * a.maxSpeed); }

// 逃离：与寻找方向相反。panic 以外的威胁不予理会
inline Vec2 flee(const Agent& a, Vec2 threat, float panic = 1e9f) {
    if (lengthSq(a.pos - threat) > panic * panic) return {};
    return steerToward(a, normalized(a.pos - threat) * a.maxSpeed);
}
// #endregion

// #region arrive
// 抵达：进入减速半径后，想要的速度与剩余距离成正比，到目标时正好为 0
inline Vec2 arrive(const Agent& a, Vec2 target, float slowRadius) {
    Vec2 to = target - a.pos;
    float d = length(to);
    if (d < 1e-3f) return steerToward(a, {});
    float speed = a.maxSpeed * std::min(1.0f, d / slowRadius);
    return steerToward(a, to * (speed / d));
}
// #endregion

// #region pursue-evade
// 追逐：不追目标现在的位置，而是追它 T 秒后的位置。T 取"以最高速度跑完当前距离所需的时间"，
// 并限制在 maxPredict 以内：离得越近，预测得越短
inline Vec2 predictPosition(const Agent& a, Vec2 targetPos, Vec2 targetVel, float maxPredict) {
    float t = std::min(distance(a.pos, targetPos) / a.maxSpeed, maxPredict);
    return targetPos + targetVel * t;
}
inline Vec2 pursue(const Agent& a, Vec2 targetPos, Vec2 targetVel, float maxPredict = 2) {
    return seek(a, predictPosition(a, targetPos, targetVel, maxPredict));
}
inline Vec2 evade(const Agent& a, Vec2 threatPos, Vec2 threatVel, float panic = 1e9f, float maxPredict = 2) {
    if (lengthSq(a.pos - threatPos) > panic * panic) return {};
    return flee(a, predictPosition(a, threatPos, threatVel, maxPredict));
}
// #endregion

// #region wander
// 漫游：在角色前方 distance 处放一个半径为 radius 的圆，圆上有一个目标点。
// 目标点的角度每秒随机游走一点，角色朝它前进。因为每次只挪一点，路线是连续的，不会抽搐。
// 随机游走的步长按 sqrt(dt) 缩放，帧率不同时，转弯的统计特性相同
struct Wander {
    float distance = 60, radius = 30, rate = 4;  // rate：角度随机游走的强度（弧度 / sqrt(秒)）
    float angle = 0;
};
inline Vec2 wanderTarget(const Agent& a, const Wander& w) {
    Vec2 fwd = lengthSq(a.vel) > 1e-6f ? normalized(a.vel) : Vec2{1, 0};
    return a.pos + fwd * w.distance + fromAngle(angleOf(fwd) + w.angle) * w.radius;
}
inline Vec2 wander(const Agent& a, Wander& w, float dt, Pcg32& rng) {
    w.angle += (rng.nextFloat() * 2 - 1) * w.rate * std::sqrt(dt);
    return seek(a, wanderTarget(a, w));
}
// #endregion

// #region avoid
struct Circle {
    Vec2 c;
    float r;
};

// 避障：沿速度方向伸出一个"探测盒"，长度随速度增加。找出与盒子相交的最近障碍，
// 产生一个侧向的力把角色推开，越近推得越用力，同时稍微刹车
inline Vec2 avoidObstacles(const Agent& a, const std::vector<Circle>& obstacles, float lookAhead = 0.3f,
                           const Circle** hitOut = nullptr) {
    float speed = length(a.vel);
    if (speed < 1e-3f) return {};
    Vec2 fwd = a.vel / speed, side = perp(fwd);
    float len = a.radius + speed * lookAhead;
    const Circle* hit = nullptr;
    float hitAhead = 1e9f, hitLateral = 0;
    for (const Circle& o : obstacles) {
        Vec2 d = o.c - a.pos;
        float ahead = dot(d, fwd), lateral = dot(d, side), r = o.r + a.radius;
        if (ahead < -r || ahead - r > len || std::abs(lateral) >= r) continue;  // 不在盒子里
        if (ahead < hitAhead) hit = &o, hitAhead = ahead, hitLateral = lateral;
    }
    if (hitOut) *hitOut = hit;
    if (!hit) return {};
    float r = hit->r + a.radius;
    float closeness = std::clamp(1 - (hitAhead - r) / len, 0.0f, 1.0f);   // 0：刚碰到盒子尖端，1：贴上了
    float away = hitLateral > 0 ? -1.0f : 1.0f;                            // 障碍在右边就往左
    float push = (1 - std::abs(hitLateral) / r) * 0.5f + 0.5f;             // 正对着障碍时推得最用力
    return side * (away * a.maxForce * push * (0.5f + closeness)) - fwd * (a.maxForce * 0.4f * closeness);
}
// #endregion

// #region combine
// 加权相加：每个行为乘以权重后相加，最后截断到力的上限。
// 问题是几个力可能互相抵消：避障向左推，寻找向右拉，结果谁也没赢，角色径直撞上去
inline Vec2 weightedSum(std::initializer_list<std::pair<Vec2, float>> forces, float maxForce) {
    Vec2 total{};
    for (auto [f, w] : forces) total += f * w;
    return truncate(total, maxForce);
}

// 按优先级累加：从最重要的行为开始，每个力只能使用剩下的额度。
// 避障用满了额度，寻找就分不到力，直到危险解除
inline bool accumulate(Vec2& total, Vec2 force, float maxForce) {
    float remaining = maxForce - length(total);
    if (remaining <= 0) return false;
    total += truncate(force, remaining);
    return true;
}
// #endregion

}  // namespace ga::steering
