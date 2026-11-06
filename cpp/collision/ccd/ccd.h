#pragma once

// 连续碰撞检测（CCD）：不只检查这一步的终点，而是求出物体在这一步之内最早何时接触（碰撞时间 TOI）
//   - sweep(Aabb, d, Aabb)            移动的盒子 vs 静止的盒子：射线 vs 闵可夫斯基和（扩大后的盒子）
//   - sweep(Circle, d, a, b)          移动的圆 vs 线段：射线 vs 胶囊体
//   - sweep(Circle, da, Circle, db)   两个都在移动的圆：相对运动，解一元二次方程
//   - conservativeAdvancement         任意凸形状，可以旋转：用 GJK 的距离一步步安全地前进
//
// 所有函数的 t 都是这一步之内的比例：t = 0 是起点，t = 1 是终点

#include <algorithm>
#include <cmath>
#include <optional>

#include "../gjk/gjk.h"
#include "../intersect/intersect.h"

namespace ga {

struct Toi {
    float t;      // 碰撞时刻（这一步的比例，0 ~ 1）
    Vec2 normal;  // 被撞物体（B）表面的外法线，指向移动的物体（A）
    int iterations = 1;
};

// #region sweep-aabb
// 盒子 a 这一步移动 d，b 静止。把 b 向四周扩大 a 的半边长，问题就变成：a 的中心沿 d 的射线是否穿过扩大后的盒子
inline std::optional<Toi> sweep(const Aabb& a, Vec2 d, const Aabb& b) {
    Vec2 h = a.half();
    Aabb expanded{b.min - h, b.max + h};
    auto hit = raycast(Ray{a.center(), d}, expanded, 1.0f);
    if (!hit) return std::nullopt;
    return Toi{hit->t, hit->normal};
}

// 两个盒子都在移动：换到 b 的参考系，a 的位移是 da − db
inline std::optional<Toi> sweep(const Aabb& a, Vec2 da, const Aabb& b, Vec2 db) { return sweep(a, da - db, b); }
// #endregion

// #region sweep-circle-segment
// 圆（半径 r）这一步移动 d，线段 ab 静止。圆心离线段不到 r 时相交，所以问题是：
// 圆心的射线何时进入"线段向外扩大 r"的区域——一个胶囊体（两条平行边 + 两端的半圆）
inline std::optional<Toi> sweep(const Circle& c, Vec2 d, Vec2 a, Vec2 b) {
    Vec2 p = c.center, ab = b - a;
    float len2 = dot(ab, ab);
    // 已经相交：t = 0
    float u0 = len2 > 0 ? std::clamp(dot(p - a, ab) / len2, 0.0f, 1.0f) : 0.0f;
    Vec2 q = a + ab * u0;
    if (lengthSq(p - q) < c.radius * c.radius) {
        Vec2 n = normalized(p - q);
        return Toi{0, lengthSq(n) > 0 ? n : Vec2{0, -1}};
    }
    std::optional<Toi> best;
    auto consider = [&](float t, Vec2 n) {
        if (t >= 0 && t <= 1 && (!best || t < best->t)) best = Toi{t, n};
    };
    // 1. 两条平行边：圆心到直线的有向距离变成 r 的时刻，且接触点落在线段范围内
    if (len2 > 0) {
        Vec2 n = normalized(perp(ab));
        float side = dot(p - a, n) >= 0 ? 1.0f : -1.0f;
        n = n * side;                   // 朝向圆心所在的一侧
        float dist = dot(p - a, n);     // > r（已排除相交）
        float speed = dot(d, n);        // < 0 才是在靠近
        if (speed < 0) {
            float t = (c.radius - dist) / speed;
            float u = dot(p + d * t - a, ab) / len2;
            if (u >= 0 && u <= 1) consider(t, n);
        }
    }
    // 2. 两端的半圆：射线 vs 以端点为圆心、半径为 r 的圆
    for (Vec2 e : {a, b})
        if (auto hit = raycast(Ray{p, d}, Circle{e, c.radius}, 1.0f)) consider(hit->t, hit->normal);
    return best;
}
// #endregion

// #region sweep-circles
// 两个圆都在移动：换到 b 的参考系，a 的圆心沿 da − db 运动，撞上以 b 的圆心为圆心、半径 ra + rb 的圆
inline std::optional<Toi> sweep(const Circle& a, Vec2 da, const Circle& b, Vec2 db) {
    auto hit = raycast(Ray{a.center, da - db}, Circle{b.center, a.radius + b.radius}, 1.0f);
    if (!hit) return std::nullopt;
    // raycast 的法线从 b 指向 a 的圆心；起点就在内部时为零向量
    Vec2 n = hit->normal;
    if (lengthSq(n) == 0) n = normalized(a.center - b.center);
    return Toi{hit->t, n};
}
// #endregion

// #region motion
// 物体在这一步内的运动：位置和角度都从起点线性变化到终点
struct Motion {
    Vec2 pos;      // 起点位置（形状局部坐标的原点）
    float angle;   // 起点角度
    Vec2 dpos;     // 这一步的位移
    float dangle;  // 这一步转过的角度
};

// 形状在 t 时刻的世界坐标
inline ConvexShape poseAt(const ConvexShape& local, const Motion& m, float t) {
    ConvexShape s{{}, local.radius};
    float angle = m.angle + m.dangle * t;
    Vec2 pos = m.pos + m.dpos * t;
    for (Vec2 p : local.points) s.points.push_back(pos + rotate(p, angle));
    return s;
}

// 形状上离局部原点最远的点有多远（旋转时，任何一点的速度都不超过 |ω| 乘以它）
inline float boundingRadius(const ConvexShape& local) {
    float r = 0;
    for (Vec2 p : local.points) r = std::max(r, length(p));
    return r + local.radius;
}
// #endregion

// #region conservative-advancement
// 保守前进：当前距离为 d，两个物体互相靠近的速度不超过 bound，
// 那么在 d / bound 这段时间内它们一定不会相撞，可以放心地前进这么多。重复直到距离小于 tolerance
inline std::optional<Toi> conservativeAdvancement(const ConvexShape& a, const Motion& ma, const ConvexShape& b, const Motion& mb,
                                                  float tolerance, int maxIterations = 64) {
    float ra = boundingRadius(a), rb = boundingRadius(b);
    float t = 0;
    for (int it = 1; it <= maxIterations; ++it) {
        Proximity p = query(poseAt(a, ma, t), poseAt(b, mb, t));
        if (p.intersecting || p.distance < tolerance) return Toi{t, -p.normal, it};
        // 靠近速度的上界：相对平移在法线上的分量 + 两个物体旋转时最快的点的速度
        float bound = dot(ma.dpos - mb.dpos, p.normal) + std::abs(ma.dangle) * ra + std::abs(mb.dangle) * rb;
        if (bound <= 0) return std::nullopt;  // 正在远离，这一步内不会相撞
        t += p.distance / bound;
        if (t > 1) return std::nullopt;
    }
    return Toi{t, {0, 0}, maxIterations};  // 迭代次数用完：保守地返回当前时刻
}
// #endregion

}  // namespace ga
