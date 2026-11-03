#pragma once

// 二维向量：本系列所有章节共用。详细讲解见 0.2《向量与矩阵》。
//
// 约定：与屏幕一致，x 向右、y 向下。角度以弧度计，从 +x 轴开始顺时针为正（因为 y 朝下）。

#include <cmath>

namespace ga {

inline constexpr float kPi = 3.14159265358979323846f;

// #region vec2
struct Vec2 {
    float x = 0.0f;
    float y = 0.0f;

    constexpr Vec2() = default;
    constexpr Vec2(float x_, float y_) : x(x_), y(y_) {}

    constexpr Vec2 operator+(Vec2 o) const { return {x + o.x, y + o.y}; }
    constexpr Vec2 operator-(Vec2 o) const { return {x - o.x, y - o.y}; }
    constexpr Vec2 operator*(float s) const { return {x * s, y * s}; }
    constexpr Vec2 operator/(float s) const { return {x / s, y / s}; }
    constexpr Vec2 operator-() const { return {-x, -y}; }
    constexpr Vec2& operator+=(Vec2 o) { x += o.x; y += o.y; return *this; }
    constexpr Vec2& operator-=(Vec2 o) { x -= o.x; y -= o.y; return *this; }
    constexpr Vec2& operator*=(float s) { x *= s; y *= s; return *this; }
    friend constexpr Vec2 operator*(float s, Vec2 v) { return v * s; }
    friend constexpr bool operator==(Vec2 a, Vec2 b) = default;
};
// #endregion

// #region dot-cross
// 点积：|a||b|cosθ。正 = 大致同向，0 = 垂直，负 = 大致反向
constexpr float dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }

// 二维叉积（也叫 perp-dot）：|a||b|sinθ。符号告诉你 b 在 a 的哪一侧
constexpr float cross(Vec2 a, Vec2 b) { return a.x * b.y - a.y * b.x; }
// #endregion

// #region length
constexpr float lengthSq(Vec2 v) { return dot(v, v); }  // 比较距离时用它，省掉开方
inline float length(Vec2 v) { return std::sqrt(dot(v, v)); }
inline float distance(Vec2 a, Vec2 b) { return length(b - a); }

inline Vec2 normalized(Vec2 v) {
    float len = length(v);
    return len > 1e-6f ? v / len : Vec2{};  // 零向量没有方向，返回零向量而不是 NaN
}
// #endregion

// #region rotate
// 垂直向量：在 y 朝下的屏幕坐标里，这是"顺时针转 90°"，也就是朝向 v 时的右手方向
constexpr Vec2 perp(Vec2 v) { return {-v.y, v.x}; }

inline Vec2 fromAngle(float radians) { return {std::cos(radians), std::sin(radians)}; }
inline float angleOf(Vec2 v) { return std::atan2(v.y, v.x); }

inline Vec2 rotate(Vec2 v, float radians) {
    float c = std::cos(radians), s = std::sin(radians);
    return {v.x * c - v.y * s, v.x * s + v.y * c};
}

// 从 a 转到 b 的有符号角度，范围 (-π, π]
inline float signedAngle(Vec2 a, Vec2 b) { return std::atan2(cross(a, b), dot(a, b)); }
// #endregion

// #region project-reflect
// v 在 onto 方向上的投影（onto 不必是单位向量）
inline Vec2 project(Vec2 v, Vec2 onto) {
    float d = lengthSq(onto);
    return d > 1e-12f ? onto * (dot(v, onto) / d) : Vec2{};
}

// 以单位法线 n 反射 v：子弹弹墙、光线反射都是它
constexpr Vec2 reflect(Vec2 v, Vec2 n) { return v - n * (2.0f * dot(v, n)); }
// #endregion

}  // namespace ga
