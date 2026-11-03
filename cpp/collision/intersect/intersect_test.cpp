#include "intersect.h"

#include <cmath>

#include "../../foundations/random/random.h"
#include "check.h"

using namespace ga;

static float rnd(Pcg32& r, float lo, float hi) { return lo + r.nextFloat() * (hi - lo); }
static Aabb randomBox(Pcg32& r) {
    Vec2 c{rnd(r, 0, 100), rnd(r, 0, 100)}, h{rnd(r, 2, 30), rnd(r, 2, 30)};
    return {c - h, c + h};
}
static Circle randomCircle(Pcg32& r) { return {{rnd(r, 0, 100), rnd(r, 0, 100)}, rnd(r, 2, 30)}; }

static Aabb moved(Aabb b, Vec2 d) { return {b.min + d, b.max + d}; }
static Circle moved(Circle c, Vec2 d) { return {c.center + d, c.radius}; }

// 暴力对照：在网格上采样，看是否存在同时落在两个形状里的点
template <class A, class B>
static bool sampledOverlap(const A& a, const B& b) {
    for (float y = -40; y <= 140; y += 0.25f)
        for (float x = -40; x <= 140; x += 0.25f)
            if (contains(a, {x, y}) && contains(b, {x, y})) return true;
    return false;
}

// 接触信息是否正确：沿 normal 移开 depth 后不再重叠；只移开一半时仍然重叠
template <class A, class B>
static void checkContact(const A& a, const B& b) {
    auto c = contact(a, b);
    CHECK(c.has_value() == overlaps(a, b));
    if (!c) return;
    CHECK(std::abs(length(c->normal) - 1) < 1e-4f);
    CHECK(c->depth > 0);
    CHECK(!overlaps(a, moved(b, c->normal * (c->depth + 1e-3f))));
    if (c->depth > 0.01f) CHECK(overlaps(a, moved(b, c->normal * (c->depth * 0.5f))));
}

// 暴力对照：沿射线以很小的步长前进，返回第一次进入形状的 t
template <class S>
static float marchedT(const Ray& ray, const S& s, float tMax) {
    if (contains(s, ray.origin)) return 0;
    for (float t = 0; t <= tMax; t += 1e-3f)
        if (contains(s, ray.at(t))) return t;
    return -1;
}

static Aabb inflate(Aabb b, float e) { return {b.min - Vec2{e, e}, b.max + Vec2{e, e}}; }
static Circle inflate(Circle c, float e) { return {c.center, c.radius + e}; }

template <class S>
static void checkRay(const Ray& ray, const S& s) {
    const float kMax = 3;
    auto hit = raycast(ray, s, kMax);
    // 射线恰好擦过边缘时，浮点误差可能让两种方法得出不同结论：这类情况跳过
    bool grazing = raycast(ray, inflate(s, 0.05f), kMax).has_value() != raycast(ray, inflate(s, -0.05f), kMax).has_value();
    if (grazing) return;
    float ref = marchedT(ray, s, kMax);
    CHECK(hit.has_value() == (ref >= 0));
    if (!hit || ref < 0) return;
    CHECK(std::abs(hit->t - ref) < 2e-3f + 0.05f / length(ray.dir));
    if (hit->t > 0) CHECK(std::abs(length(hit->normal) - 1) < 1e-4f);
}

int main() {
    // 几个手算的例子
    {
        Aabb a{{0, 0}, {10, 10}}, b{{8, 2}, {20, 5}};
        auto c = contact(a, b);
        CHECK(c && c->normal == (Vec2{1, 0}) && std::abs(c->depth - 2) < 1e-5f);
        CHECK(!overlaps(a, Aabb{{10, 0}, {20, 10}}));  // 只碰到边：不算重叠

        auto hit = raycast(Ray{{-5, 5}, {1, 0}}, a);
        CHECK(hit && std::abs(hit->t - 5) < 1e-5f && hit->normal == (Vec2{-1, 0}));
        CHECK(!raycast(Ray{{-5, 5}, {-1, 0}}, a));     // 背对盒子
        CHECK(!raycast(Ray{{-5, 15}, {1, 0}}, a));     // 平行且在板外
        auto inside = raycast(Ray{{5, 5}, {1, 1}}, a);
        CHECK(inside && inside->t == 0 && inside->normal == (Vec2{0, 0}));

        Circle ci{{0, 0}, 2};
        auto h2 = raycast(Ray{{-10, 0}, {2, 0}}, ci);  // dir 长度为 2：t 以 2 为单位
        CHECK(h2 && std::abs(h2->t - 4) < 1e-5f && h2->normal == (Vec2{-1, 0}));
    }
    // 随机：相交判断与采样一致（排除几乎相切的情况）
    Pcg32 rng(2);
    for (int i = 0; i < 300; ++i) {
        Aabb a = randomBox(rng), b = randomBox(rng);
        Circle c = randomCircle(rng), d = randomCircle(rng);
        auto nearBoundary = [](auto ct) { return ct && ct->depth < 0.5f; };
        if (!nearBoundary(contact(a, b))) CHECK(overlaps(a, b) == sampledOverlap(a, b));
        if (!nearBoundary(contact(c, d))) CHECK(overlaps(c, d) == sampledOverlap(c, d));
        if (!nearBoundary(contact(a, c))) CHECK(overlaps(c, a) == sampledOverlap(c, a));
    }
    // 随机：接触信息
    for (int i = 0; i < 5000; ++i) {
        checkContact(randomBox(rng), randomBox(rng));
        checkContact(randomCircle(rng), randomCircle(rng));
        checkContact(randomBox(rng), randomCircle(rng));
    }
    // 圆心在盒子内部的情况
    for (int i = 0; i < 2000; ++i) {
        Aabb a = randomBox(rng);
        Circle c{{rnd(rng, a.min.x, a.max.x), rnd(rng, a.min.y, a.max.y)}, rnd(rng, 1, 10)};
        checkContact(a, c);
    }
    // 随机：射线检测与逐步前进的结果一致
    for (int i = 0; i < 1500; ++i) {
        Ray ray{{rnd(rng, -20, 120), rnd(rng, -20, 120)}, {rnd(rng, -60, 60), rnd(rng, -60, 60)}};
        if (i % 7 == 0) ray.dir.y = 0;  // 平行于坐标轴的射线
        if (length(ray.dir) < 1) continue;
        checkRay(ray, randomBox(rng));
        checkRay(ray, randomCircle(rng));
    }
    return report("intersect_test");
}
