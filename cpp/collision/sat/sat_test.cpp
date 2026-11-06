#include "sat.h"

#include <cmath>

#include "../../foundations/random/random.h"
#include "check.h"

using namespace ga;

static float rnd(Pcg32& r, float lo, float hi) { return lo + r.nextFloat() * (hi - lo); }

// 随机凸多边形：在圆上随机取若干角度，按角度排序后连起来（不规则，但一定是凸的）
static ConvexPolygon randomPolygon(Pcg32& r) {
    int n = 3 + static_cast<int>(r.nextFloat() * 6);
    Vec2 c{rnd(r, 0, 100), rnd(r, 0, 100)};
    float radius = rnd(r, 4, 30);
    std::vector<float> angles;
    for (int i = 0; i < n; ++i) angles.push_back(rnd(r, 0, 2 * kPi));
    std::sort(angles.begin(), angles.end());
    std::vector<Vec2> pts;
    for (float a : angles) pts.push_back(c + fromAngle(a) * radius);
    // 去掉几乎重合的顶点，避免退化的边
    std::vector<Vec2> clean;
    for (Vec2 p : pts)
        if (clean.empty() || distance(clean.back(), p) > 0.5f) clean.push_back(p);
    while (clean.size() > 2 && distance(clean.back(), clean.front()) <= 0.5f) clean.pop_back();
    if (clean.size() < 3 || std::abs(signedArea2(clean)) < 10) return makeBox(c, {radius, radius * 0.5f}, angles[0]);
    return makePolygon(clean);
}
static Circle randomCircle(Pcg32& r) { return {{rnd(r, 0, 100), rnd(r, 0, 100)}, rnd(r, 2, 30)}; }

static ConvexPolygon moved(const ConvexPolygon& p, Vec2 d) { return transformed(p, d, 0); }
static Circle moved(Circle c, Vec2 d) { return {c.center + d, c.radius}; }

// ---- 暴力对照：不用分离轴，直接看边是否相交、顶点是否在对方内部 ----
static bool segmentsCross(Vec2 a, Vec2 b, Vec2 c, Vec2 d) {
    float d1 = cross(b - a, c - a), d2 = cross(b - a, d - a);
    float d3 = cross(d - c, a - c), d4 = cross(d - c, b - c);
    return ((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0));
}
static bool bruteOverlap(const ConvexPolygon& a, const ConvexPolygon& b) {
    for (int i = 0; i < a.size(); ++i)
        for (int j = 0; j < b.size(); ++j)
            if (segmentsCross(a.vertex(i), a.vertex(i + 1), b.vertex(j), b.vertex(j + 1))) return true;
    return contains(a, b.v[0]) || contains(b, a.v[0]) || contains(a, centroid(b)) || contains(b, centroid(a));
}
static float pointSegmentDistance(Vec2 p, Vec2 a, Vec2 b) {
    Vec2 ab = b - a;
    float t = std::clamp(dot(p - a, ab) / dot(ab, ab), 0.0f, 1.0f);
    return distance(p, a + ab * t);
}
static float distanceToBoundary(const ConvexPolygon& p, Vec2 q) {
    float d = std::numeric_limits<float>::infinity();
    for (int i = 0; i < p.size(); ++i) d = std::min(d, pointSegmentDistance(q, p.vertex(i), p.vertex(i + 1)));
    return d;
}
static bool bruteOverlap(const ConvexPolygon& a, const Circle& c) {
    return contains(a, c.center) || distanceToBoundary(a, c.center) < c.radius;
}

// 沿方向 dir 至少要把 b 推多远才能分开（二分查找）
template <class B>
static float separationAlong(const ConvexPolygon& a, const B& b, Vec2 dir) {
    float lo = 0, hi = 200;
    for (int it = 0; it < 40; ++it) {
        float mid = (lo + hi) / 2;
        (bruteOverlap(a, moved(b, dir * mid)) ? lo : hi) = mid;
    }
    return hi;
}

template <class B>
static void checkContact(const ConvexPolygon& a, const B& b, Pcg32& rng) {
    auto c = contact(a, b);
    CHECK(c.has_value() == overlaps(a, b));
    if (!c) return;
    CHECK(std::abs(length(c->normal) - 1) < 1e-4f);
    CHECK(c->depth > 0);
    // 沿 MTV 推开后分离；只推一半仍然重叠
    CHECK(!bruteOverlap(a, moved(b, c->normal * (c->depth + 1e-2f))));
    if (c->depth > 0.05f) CHECK(bruteOverlap(a, moved(b, c->normal * (c->depth * 0.5f))));
    // "最小"：随便换一个方向，需要推的距离都不会更短
    for (int k = 0; k < 8; ++k) {
        Vec2 dir = fromAngle(rnd(rng, 0, 2 * kPi));
        CHECK(separationAlong(a, b, dir) > c->depth - 2e-2f);
    }
}

// 接触点：参考边裁剪得到的点落在入射边上，也就是某个多边形的边界上。
// 穿透很浅时（物理引擎里的常态），它们还应该落在两个多边形的重叠区域附近
static void checkManifold(const ConvexPolygon& a, const ConvexPolygon& b, bool shallow) {
    auto m = collide(a, b);
    if (!m) return;
    CHECK(m->count == 1 || m->count == 2);
    for (int i = 0; i < m->count; ++i) {
        Vec2 p = m->points[i];
        CHECK(std::min(distanceToBoundary(a, p), distanceToBoundary(b, p)) < 1e-2f);
        CHECK(m->depths[i] >= 0);
        if (shallow) {
            CHECK(contains(a, p) || distanceToBoundary(a, p) < m->depth + 1e-2f);
            CHECK(contains(b, p) || distanceToBoundary(b, p) < m->depth + 1e-2f);
            CHECK(m->depths[i] <= m->depth + 1e-2f);
        }
    }
}

int main() {
    // 手算的例子
    {
        ConvexPolygon a = makeBox({0, 0}, {10, 10});
        ConvexPolygon b = makeBox({18, 2}, {10, 3});
        auto c = contact(a, b);
        CHECK(c && std::abs(c->normal.x - 1) < 1e-5f && std::abs(c->depth - 2) < 1e-4f);
        CHECK(!overlaps(a, makeBox({20, 0}, {10, 10})));  // 只碰到边：不算重叠

        // 旋转 45° 的正方形：轴对齐包围盒重叠，但实际不相交
        ConvexPolygon d = makeBox({19, 19}, {10, 10}, kPi / 4);
        CHECK(overlaps(Aabb{{-10, -10}, {10, 10}}, Aabb{{19 - 14.2f, 19 - 14.2f}, {19 + 14.2f, 19 + 14.2f}}));
        CHECK(!overlaps(a, d));

        // 面贴面：两个接触点，是 b 的底边被 a 的顶边裁剪后的两端
        ConvexPolygon top = makeBox({4, -18}, {6, 9});  // 底边 y = -9，压进 a（顶边 y = -10）1 个单位
        auto m = collide(a, top);
        CHECK(m && m->count == 2 && std::abs(m->depth - 1) < 1e-4f);
        CHECK(m && std::abs(m->normal.y + 1) < 1e-5f);

        // 多边形与圆：圆靠近角点，分离方向沿"角点 → 圆心"
        auto cc = contact(a, Circle{{14, 14}, 6});
        CHECK(cc && std::abs(cc->normal.x - cc->normal.y) < 1e-4f && std::abs(cc->depth - (6 - 4 * std::sqrt(2.0f))) < 1e-4f);
    }

    Pcg32 rng(22);
    // 随机：相交判断与暴力方法一致
    for (int i = 0; i < 3000; ++i) {
        ConvexPolygon a = randomPolygon(rng), b = randomPolygon(rng);
        Circle c = randomCircle(rng);
        auto near = [](auto ct) { return ct && ct->depth < 1e-2f; };
        if (!near(contact(a, b))) CHECK(overlaps(a, b) == bruteOverlap(a, b));
        if (!near(contact(a, c))) CHECK(overlaps(a, c) == bruteOverlap(a, c));
    }
    // 随机：接触信息正确，并且是最小的
    for (int i = 0; i < 1500; ++i) {
        checkContact(randomPolygon(rng), randomPolygon(rng), rng);
        checkContact(randomPolygon(rng), randomCircle(rng), rng);
    }
    // 随机：接触点
    for (int i = 0; i < 5000; ++i) {
        ConvexPolygon a = randomPolygon(rng), b = randomPolygon(rng);
        checkManifold(a, b, false);
        // 沿 MTV 把 b 推回去，只留下 0.5 的穿透
        if (auto c = contact(a, b); c && c->depth > 0.5f) checkManifold(a, moved(b, c->normal * (c->depth - 0.5f)), true);
    }
    // 分离轴缓存：结果与不缓存时完全一致
    {
        std::vector<SatCache> caches(50);
        std::vector<ConvexPolygon> as, bs;
        for (int i = 0; i < 50; ++i) as.push_back(randomPolygon(rng)), bs.push_back(randomPolygon(rng));
        for (int frame = 0; frame < 200; ++frame) {
            for (int i = 0; i < 50; ++i) {
                bs[i] = transformed(bs[i], {std::sin(frame * 0.05f + i) * 0.8f, std::cos(frame * 0.03f + i) * 0.8f}, 0);
                CHECK(overlapsCached(as[i], bs[i], caches[i]) == overlaps(as[i], bs[i]));
            }
        }
    }
    return report("sat_test");
}
