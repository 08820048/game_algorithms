#pragma once

// 用点积、叉积回答游戏里最常见的几个"空间问题"：
// 目标在我前面还是后面？在左边还是右边？在不在我的视野里？

#include <cmath>

#include "vec2.h"

namespace ga {

// #region in-front
// 点积 > 0：目标与朝向的夹角小于 90°，在"前方"
inline bool isInFront(Vec2 facing, Vec2 toTarget) { return dot(facing, toTarget) > 0.0f; }
// #endregion

// #region side
enum class Side { Left, Right, Straight };

// 叉积的符号判断左右。注意：屏幕坐标 y 朝下，结论与数学课本（y 朝上）相反
inline Side sideOf(Vec2 facing, Vec2 toTarget, float epsilon = 1e-4f) {
    float c = cross(facing, toTarget);
    if (c > epsilon) return Side::Right;
    if (c < -epsilon) return Side::Left;
    return Side::Straight;
}
// #endregion

// #region vision-cone
struct VisionCone {
    Vec2 position;
    Vec2 facing;      // 单位向量
    float halfAngle;  // 视野半角（弧度）
    float range;
};

// 不需要 acos，也不需要先把 toTarget 归一化：
//   dot(f, to) >= |to| · cos(halfAngle)  ⇔  夹角 <= halfAngle
inline bool canSee(const VisionCone& cone, Vec2 target) {
    Vec2 to = target - cone.position;
    float d2 = lengthSq(to);
    if (d2 > cone.range * cone.range) return false;  // 先做便宜的距离判断
    if (d2 < 1e-8f) return true;
    return dot(cone.facing, to) >= std::cos(cone.halfAngle) * std::sqrt(d2);
}
// #endregion

}  // namespace ga
