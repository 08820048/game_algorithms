#include <cmath>

#include "check.h"
#include "transform.h"
#include "vision.h"

using namespace ga;

static bool near(float a, float b, float eps = 1e-4f) { return std::abs(a - b) <= eps; }
static bool near(Vec2 a, Vec2 b, float eps = 1e-4f) { return near(a.x, b.x, eps) && near(a.y, b.y, eps); }

int main() {
    // 点积 / 叉积基本性质
    {
        CHECK(dot({1, 0}, {0, 1}) == 0.0f);
        CHECK(dot({2, 3}, {4, 5}) == 23.0f);
        CHECK(cross({1, 0}, {0, 1}) == 1.0f);
        CHECK(cross({0, 1}, {1, 0}) == -1.0f);
        CHECK(cross({3, 3}, {6, 6}) == 0.0f);  // 平行
    }
    // 长度与归一化
    {
        CHECK(length({3, 4}) == 5.0f);
        CHECK(near(length(normalized({3, 4})), 1.0f));
        CHECK(normalized({0, 0}) == (Vec2{0, 0}));  // 零向量不产生 NaN
    }
    // 旋转、角度、垂直向量
    {
        CHECK(near(rotate({1, 0}, kPi / 2), {0, 1}));
        CHECK(near(fromAngle(kPi), {-1, 0}));
        CHECK(near(signedAngle({1, 0}, {0, 1}), kPi / 2));
        CHECK(near(signedAngle({1, 0}, {0, -1}), -kPi / 2));
        CHECK(perp(Vec2{1, 0}) == (Vec2{0, 1}));
    }
    // 投影与反射
    {
        CHECK(near(project({3, 4}, {10, 0}), {3, 0}));
        CHECK(near(reflect({1, 1}, {0, -1}), {1, -1}));  // 打在地面上弹起
    }
    // 前后、左右（屏幕坐标 y 朝下）
    {
        Vec2 facing{1, 0};  // 朝右看
        CHECK(isInFront(facing, {5, 1}));
        CHECK(!isInFront(facing, {-5, 1}));
        CHECK(sideOf(facing, {5, 3}) == Side::Right);  // 屏幕下方 = 朝右看时的右手边
        CHECK(sideOf(facing, {5, -3}) == Side::Left);
        CHECK(sideOf(facing, {5, 0}) == Side::Straight);
    }
    // 视野锥
    {
        VisionCone cone{{0, 0}, {1, 0}, kPi / 4, 100};  // 90° 视野
        CHECK(canSee(cone, {50, 0}));
        CHECK(canSee(cone, {50, 49}));    // 约 44.4°，在视野内
        CHECK(!canSee(cone, {50, 51}));   // 约 45.6°，刚好出界
        CHECK(!canSee(cone, {150, 0}));   // 太远
        CHECK(!canSee(cone, {-10, 0}));   // 身后
    }
    // 矩阵与变换
    {
        Mat2 r = Mat2::rotation(0.7f);
        Mat2 identity = r * r.transposed();
        CHECK(near(identity.col0, {1, 0}) && near(identity.col1, {0, 1}));  // 旋转矩阵的逆 = 转置

        Transform2D t{{100, 50}, kPi / 2};  // 位于 (100,50)，朝下
        CHECK(near(t.toWorld({10, 0}), {100, 60}));
        Vec2 world{130, 80};
        CHECK(near(t.toWorld(t.toLocal(world)), world));
        CHECK(near(t.toLocal(world), toLocalByDot(t, world)));
    }

    return report("vectors_test");
}
