#pragma once

// 2×2 矩阵与二维变换：把"世界坐标"和"某个物体的局部坐标"互相转换

#include <cmath>

#include "vec2.h"

namespace ga {

// #region mat2
// 用两列表示：col0 是变换后的 x 轴，col1 是变换后的 y 轴
struct Mat2 {
    Vec2 col0{1, 0};
    Vec2 col1{0, 1};

    static Mat2 rotation(float radians) {
        float c = std::cos(radians), s = std::sin(radians);
        return {{c, s}, {-s, c}};
    }

    // 矩阵 × 向量 = 两列按分量加权求和
    Vec2 operator*(Vec2 v) const { return col0 * v.x + col1 * v.y; }

    Mat2 operator*(const Mat2& o) const { return {(*this) * o.col0, (*this) * o.col1}; }

    Mat2 transposed() const { return {{col0.x, col1.x}, {col0.y, col1.y}}; }
};
// #endregion

// #region transform
// 位置 + 朝向：游戏物体最常见的变换（这里省略了缩放）
struct Transform2D {
    Vec2 position;
    float rotation = 0.0f;

    Vec2 forward() const { return fromAngle(rotation); }
    Vec2 right() const { return perp(forward()); }

    // 局部 -> 世界：先旋转，再平移
    Vec2 toWorld(Vec2 local) const { return position + Mat2::rotation(rotation) * local; }

    // 世界 -> 局部：先平移回去，再反向旋转。旋转矩阵的逆就是它的转置
    Vec2 toLocal(Vec2 world) const { return Mat2::rotation(rotation).transposed() * (world - position); }
};
// #endregion

// #region to-local-dot
// 等价写法：局部坐标就是在"前方"和"右方"两个轴上各做一次点积
inline Vec2 toLocalByDot(const Transform2D& t, Vec2 world) {
    Vec2 d = world - t.position;
    return {dot(d, t.forward()), dot(d, t.right())};
}
// #endregion

}  // namespace ga
