#include "gjk.h"

#include <cmath>
#include <cstdio>

#include "../../foundations/random/random.h"
#include "../sat/sat.h"
#include "check.h"

using namespace ga;

static float rnd(Pcg32& r, float lo, float hi) { return lo + r.nextFloat() * (hi - lo); }

static std::vector<Vec2> randomConvex(Pcg32& r, Vec2 c, float radius) {
    int n = 3 + static_cast<int>(r.nextFloat() * 6);
    std::vector<float> angles;
    for (int i = 0; i < n; ++i) angles.push_back(rnd(r, 0, 2 * kPi));
    std::sort(angles.begin(), angles.end());
    std::vector<Vec2> pts;
    for (float a : angles) {
        Vec2 p = c + fromAngle(a) * radius;
        if (pts.empty() || distance(pts.back(), p) > 0.5f) pts.push_back(p);
    }
    while (pts.size() > 2 && distance(pts.back(), pts.front()) <= 0.5f) pts.pop_back();
    if (pts.size() < 3 || std::abs(signedArea2(pts)) < 10) return makeBox(c, {radius, radius * 0.5f}, angles[0]).v;
    return pts;
}

// 随机形状：多边形、圆、胶囊体、圆角多边形
static ConvexShape randomShape(Pcg32& r, int kind) {
    Vec2 c{rnd(r, 0, 100), rnd(r, 0, 100)};
    switch (kind) {
        case 0: return {randomConvex(r, c, rnd(r, 4, 30)), 0};
        case 1: return {{c}, rnd(r, 2, 25)};
        case 2: {
            Vec2 h = fromAngle(rnd(r, 0, 2 * kPi)) * rnd(r, 2, 25);
            return {{c - h, c + h}, rnd(r, 2, 12)};
        }
        default: return {randomConvex(r, c, rnd(r, 4, 25)), rnd(r, 1, 8)};
    }
}

static ConvexShape moved(ConvexShape s, Vec2 d) {
    for (Vec2& p : s.points) p += d;
    return s;
}

// ---- 暴力对照：核心点的凸包之间的距离，再减去两个半径 ----
static float pointSegment(Vec2 p, Vec2 a, Vec2 b) {
    Vec2 ab = b - a;
    float l2 = dot(ab, ab);
    float t = l2 > 0 ? std::clamp(dot(p - a, ab) / l2, 0.0f, 1.0f) : 0.0f;
    return distance(p, a + ab * t);
}
static bool segmentsCross(Vec2 a, Vec2 b, Vec2 c, Vec2 d) {
    float d1 = cross(b - a, c - a), d2 = cross(b - a, d - a);
    float d3 = cross(d - c, a - c), d4 = cross(d - c, b - c);
    return ((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0));
}
static std::vector<std::pair<Vec2, Vec2>> edges(const std::vector<Vec2>& pts) {
    std::vector<std::pair<Vec2, Vec2>> e;
    if (pts.size() == 1) e.push_back({pts[0], pts[0]});
    else if (pts.size() == 2) e.push_back({pts[0], pts[1]});
    else for (size_t i = 0; i < pts.size(); ++i) e.push_back({pts[i], pts[(i + 1) % pts.size()]});
    return e;
}
static bool insideHull(const std::vector<Vec2>& hull, Vec2 p) {
    return hull.size() >= 3 && contains(makePolygon(hull), p);
}
// 带符号的距离：核心凸包相交时返回 -1（只用来判断"相交"）
static float bruteDistance(const ConvexShape& a, const ConvexShape& b) {
    auto ea = edges(a.points), eb = edges(b.points);
    float core = std::numeric_limits<float>::infinity();
    for (auto [p, q] : ea)
        for (auto [s, t] : eb) {
            if (segmentsCross(p, q, s, t)) return -1;
            core = std::min({core, pointSegment(p, s, t), pointSegment(q, s, t), pointSegment(s, p, q), pointSegment(t, p, q)});
        }
    if (insideHull(a.points, b.points[0]) || insideHull(b.points, a.points[0])) return -1;
    return core - a.radius - b.radius;
}
static bool bruteOverlap(const ConvexShape& a, const ConvexShape& b) { return bruteDistance(a, b) < 0; }

int main() {
    // 手算的例子
    {
        ConvexShape box{makeBox({0, 0}, {10, 10}).v, 0};
        ConvexShape circle{{{25, 0}}, 5};
        Proximity p = query(box, circle);
        CHECK(!p.intersecting && std::abs(p.distance - 10) < 1e-3f);
        CHECK(distance(p.pointA, {10, 0}) < 1e-2f && distance(p.pointB, {20, 0}) < 1e-2f);

        ConvexShape capsule{{{-30, 8}, {30, 8}}, 4};  // 覆盖 y ∈ [4, 12]，与盒子（y ∈ [-10, 10]）重叠 6
        Proximity q = query(box, capsule);
        CHECK(q.intersecting && std::abs(q.distance + 6) < 1e-2f && std::abs(q.normal.y - 1) < 1e-3f);
    }

    Pcg32 rng(23);
    int checked = 0;
    // 多边形与多边形：与 SAT 对照
    for (int i = 0; i < 4000; ++i) {
        ConvexPolygon pa = makePolygon(randomConvex(rng, {rnd(rng, 0, 100), rnd(rng, 0, 100)}, rnd(rng, 4, 30)));
        ConvexPolygon pb = makePolygon(randomConvex(rng, {rnd(rng, 0, 100), rnd(rng, 0, 100)}, rnd(rng, 4, 30)));
        ConvexShape a{pa.v, 0}, b{pb.v, 0};
        auto sat = contact(pa, pb);
        Proximity p = query(a, b);
        if (sat && sat->depth < 1e-2f) continue;  // 几乎恰好接触
        if (!sat && p.distance < 1e-2f) continue;
        ++checked;
        CHECK(p.intersecting == sat.has_value());
        if (sat) {
            CHECK(std::abs(-p.distance - sat->depth) < 1e-2f);  // EPA 深度 = SAT 的最小推出距离
            CHECK(!overlaps(pa, transformed(pb, p.normal * (-p.distance + 1e-2f), 0)));
        } else {
            CHECK(std::abs(p.distance - bruteDistance(a, b)) < 1e-2f);
            CHECK(std::abs(distance(p.pointA, p.pointB) - p.distance) < 1e-2f);
        }
    }
    CHECK(checked > 3500);

    // 任意形状组合：与暴力距离对照
    for (int i = 0; i < 6000; ++i) {
        ConvexShape a = randomShape(rng, i % 4), b = randomShape(rng, (i / 4) % 4);
        float ref = bruteDistance(a, b);
        Proximity p = query(a, b);
        if (std::abs(ref) < 2e-2f) continue;
        CHECK(p.intersecting == (ref < 0));
        if (p.intersecting != (ref < 0) || (!p.intersecting && std::abs(p.distance - ref) >= 2e-2f))
            std::fprintf(stderr, "kinds %d %d: ref %.4f gjk %.4f inter %d | na %zu ra %.2f nb %zu rb %.2f\n", i % 4, (i / 4) % 4, ref,
                         p.distance, p.intersecting, a.points.size(), a.radius, b.points.size(), b.radius);
        if (!p.intersecting) {
            CHECK(std::abs(p.distance - ref) < 2e-2f);
            continue;
        }
        float depth = -p.distance;
        CHECK(std::abs(length(p.normal) - 1) < 1e-3f);
        // 沿 normal 推开 depth 后分离；只推一半仍然相交
        CHECK(bruteDistance(a, moved(b, p.normal * (depth + 2e-2f))) > -1e-3f);
        if (depth > 0.1f) CHECK(bruteOverlap(a, moved(b, p.normal * (depth * 0.5f))));
        // 随便换个方向，需要推的距离都不会更短
        for (int k = 0; k < 4; ++k) {
            Vec2 dir = fromAngle(rnd(rng, 0, 2 * kPi));
            float lo = 0, hi = 200;
            for (int it = 0; it < 30; ++it) {
                float mid = (lo + hi) / 2;
                (bruteOverlap(a, moved(b, dir * mid)) ? lo : hi) = mid;
            }
            CHECK(hi > depth - 3e-2f);
        }
    }
    return report("gjk_test");
}
