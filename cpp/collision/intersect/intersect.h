#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>

#include "vec2.h"

namespace ga {

// #region shapes
// 轴对齐包围盒（Axis-Aligned Bounding Box）：边与坐标轴平行的矩形
struct Aabb {
    Vec2 min, max;
    Vec2 center() const { return (min + max) * 0.5f; }
    Vec2 half() const { return (max - min) * 0.5f; }
};

struct Circle {
    Vec2 center;
    float radius;
};

// 射线：origin + t * dir，t ≥ 0。dir 不要求是单位向量，t 以 dir 的长度为单位
struct Ray {
    Vec2 origin, dir;
    Vec2 at(float t) const { return origin + dir * t; }
};
// #endregion

// #region overlap
// 两个区间 [a0, a1] 和 [b0, b1] 有重叠，当且仅当谁都不完全在对方的一侧
inline bool overlaps(const Aabb& a, const Aabb& b) {
    return a.min.x < b.max.x && b.min.x < a.max.x &&  // x 方向有重叠
           a.min.y < b.max.y && b.min.y < a.max.y;    // y 方向有重叠
}

// 比较距离的平方，省掉一次开方
inline bool overlaps(const Circle& a, const Circle& b) {
    Vec2 d = b.center - a.center;
    float r = a.radius + b.radius;
    return dot(d, d) < r * r;
}

// 盒子上离 p 最近的点：把 p 的每个坐标夹到盒子的范围内
inline Vec2 closestPoint(const Aabb& box, Vec2 p) {
    return {std::clamp(p.x, box.min.x, box.max.x), std::clamp(p.y, box.min.y, box.max.y)};
}

inline bool overlaps(const Circle& c, const Aabb& box) {
    Vec2 d = c.center - closestPoint(box, c.center);
    return dot(d, d) < c.radius * c.radius;
}
inline bool overlaps(const Aabb& box, const Circle& c) { return overlaps(c, box); }
// #endregion

inline bool contains(const Aabb& box, Vec2 p) {
    return p.x > box.min.x && p.x < box.max.x && p.y > box.min.y && p.y < box.max.y;
}
inline bool contains(const Circle& c, Vec2 p) {
    Vec2 d = p - c.center;
    return dot(d, d) < c.radius * c.radius;
}

// #region contact
// 接触信息：把 b 沿 normal 移动 depth，两者就恰好分开（最小分离向量，MTV）。
// normal 是单位向量，从 a 指向 b。
struct Contact {
    Vec2 normal;
    float depth;
    Vec2 point;  // 一个代表性的接触点，用于显示和计算力矩
};

inline std::optional<Contact> contact(const Aabb& a, const Aabb& b) {
    // 每个轴上，b 可以往正方向推出（推到 a.max 外）或往负方向推出（推到 a.min 外），取较短的一个。
    // 注意：这不等于两个区间的重叠长度 —— 一个区间包含另一个时，要推出的距离比重叠长度更长
    float px = std::min(a.max.x - b.min.x, b.max.x - a.min.x);
    float py = std::min(a.max.y - b.min.y, b.max.y - a.min.y);
    if (px <= 0 || py <= 0) return std::nullopt;
    float sx = (a.max.x - b.min.x) < (b.max.x - a.min.x) ? 1.0f : -1.0f;
    float sy = (a.max.y - b.min.y) < (b.max.y - a.min.y) ? 1.0f : -1.0f;
    Vec2 lo{std::max(a.min.x, b.min.x), std::max(a.min.y, b.min.y)};
    Vec2 hi{std::min(a.max.x, b.max.x), std::min(a.max.y, b.max.y)};
    Vec2 mid = (lo + hi) * 0.5f;  // 重叠区域的中心
    // 沿推出距离较短的轴分开
    if (px < py) return Contact{{sx, 0}, px, mid};
    return Contact{{0, sy}, py, mid};
}

inline std::optional<Contact> contact(const Circle& a, const Circle& b) {
    Vec2 d = b.center - a.center;
    float dist2 = dot(d, d), r = a.radius + b.radius;
    if (dist2 >= r * r) return std::nullopt;
    float dist = std::sqrt(dist2);
    Vec2 n = dist > 1e-6f ? d / dist : Vec2{1, 0};  // 圆心重合时随便选一个方向
    return Contact{n, r - dist, a.center + n * a.radius};
}

// 盒子 a 与圆 b
inline std::optional<Contact> contact(const Aabb& a, const Circle& b) {
    Vec2 q = closestPoint(a, b.center);
    Vec2 d = b.center - q;
    float dist2 = dot(d, d);
    if (dist2 > 1e-12f) {  // 圆心在盒子外：沿"最近点 → 圆心"分开
        if (dist2 >= b.radius * b.radius) return std::nullopt;
        float dist = std::sqrt(dist2);
        return Contact{d / dist, b.radius - dist, q};
    }
    // 圆心在盒子内：找最近的一条边，从那里推出去
    Vec2 p = b.center;
    float left = p.x - a.min.x, right = a.max.x - p.x, top = p.y - a.min.y, bottom = a.max.y - p.y;
    float m = std::min({left, right, top, bottom});
    Vec2 n = m == left ? Vec2{-1, 0} : m == right ? Vec2{1, 0} : m == top ? Vec2{0, -1} : Vec2{0, 1};
    return Contact{n, b.radius + m, p + n * m};
}
// #endregion

// #region raycast
struct RayHit {
    float t;      // 命中参数：命中点 = ray.at(t)
    Vec2 point;
    Vec2 normal;  // 命中表面的外法线；射线起点在形状内部时为零向量
};

// slab 法：盒子是 x 方向和 y 方向两条"板"（slab）的交集。
// 分别求出射线在每条板内的参数区间 [tEnter, tExit]，两个区间的交集非空就命中
inline std::optional<RayHit> raycast(const Ray& ray, const Aabb& box,
                                     float tMax = std::numeric_limits<float>::infinity()) {
    float tMin = 0;
    Vec2 normal{0, 0};
    for (int axis = 0; axis < 2; ++axis) {
        float o = axis == 0 ? ray.origin.x : ray.origin.y;
        float d = axis == 0 ? ray.dir.x : ray.dir.y;
        float lo = axis == 0 ? box.min.x : box.min.y;
        float hi = axis == 0 ? box.max.x : box.max.y;
        if (std::abs(d) < 1e-9f) {  // 与这条板平行：起点必须在板内，否则永远碰不到
            if (o < lo || o > hi) return std::nullopt;
            continue;
        }
        float t1 = (lo - o) / d, t2 = (hi - o) / d;  // 进入、离开这条板的参数
        float sign = -1;                             // 从 lo 一侧进入时，法线指向负方向
        if (t1 > t2) {
            std::swap(t1, t2);
            sign = 1;
        }
        if (t1 > tMin) {  // 最晚进入的那条板决定了命中面
            tMin = t1;
            normal = axis == 0 ? Vec2{sign, 0} : Vec2{0, sign};
        }
        tMax = std::min(tMax, t2);
        if (tMin > tMax) return std::nullopt;  // 两个区间不再重叠
    }
    return RayHit{tMin, ray.at(tMin), normal};
}

// 解 |origin + t·dir - center|² = r²，取较小的根
inline std::optional<RayHit> raycast(const Ray& ray, const Circle& c,
                                     float tMax = std::numeric_limits<float>::infinity()) {
    Vec2 m = ray.origin - c.center;
    float a = dot(ray.dir, ray.dir);
    float b = dot(m, ray.dir);
    float k = dot(m, m) - c.radius * c.radius;  // k > 0：起点在圆外
    if (k > 0 && b > 0) return std::nullopt;    // 起点在圆外且背对圆心
    float disc = b * b - a * k;
    if (disc < 0) return std::nullopt;  // 射线所在的直线与圆不相交
    float t = (-b - std::sqrt(disc)) / a;
    if (t < 0) return RayHit{0, ray.origin, {0, 0}};  // 起点在圆内
    if (t > tMax) return std::nullopt;
    Vec2 p = ray.at(t);
    return RayHit{t, p, (p - c.center) / c.radius};
}
// #endregion

}  // namespace ga
