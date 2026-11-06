#include "ccd.h"

#include <cmath>
#include <cstdio>

#include "../../foundations/random/random.h"
#include "../sat/sat.h"  // makeBox、regularPolygon
#include "check.h"

using namespace ga;

static float rnd(Pcg32& r, float lo, float hi) { return lo + r.nextFloat() * (hi - lo); }

// 暴力求 TOI：密集采样找到第一个相交的时刻，再在前后两个采样之间二分
template <class Overlap>
static std::optional<float> bruteToi(Overlap&& overlapAt, int samples = 4000) {
    if (overlapAt(0.0f)) return 0.0f;
    for (int i = 1; i <= samples; ++i) {
        float t1 = float(i) / samples;
        if (!overlapAt(t1)) continue;
        float lo = float(i - 1) / samples, hi = t1;
        for (int k = 0; k < 40; ++k) {
            float mid = (lo + hi) / 2;
            (overlapAt(mid) ? hi : lo) = mid;
        }
        return hi;
    }
    return std::nullopt;
}

static float pointSegment(Vec2 p, Vec2 a, Vec2 b) {
    Vec2 ab = b - a;
    float l2 = dot(ab, ab);
    float u = l2 > 0 ? std::clamp(dot(p - a, ab) / l2, 0.0f, 1.0f) : 0.0f;
    return distance(p, a + ab * u);
}

int main() {
    // ---- 隧穿：每帧只检查终点，高速的子弹会直接穿过薄墙 ----
    {
        Aabb wall{{100, 0}, {102, 200}};  // 2 像素厚的墙
        Aabb bullet{{0, 98}, {4, 102}};
        Vec2 d{300, 0};  // 一帧移动 300 像素
        Aabb end{bullet.min + d, bullet.max + d};
        CHECK(!overlaps(bullet, wall) && !overlaps(end, wall));  // 起点和终点都不相交：离散检测漏掉了
        auto hit = sweep(bullet, d, wall);
        CHECK(hit && std::abs(hit->t - 96.0f / 300) < 1e-5f && hit->normal == Vec2(-1, 0));
    }

    Pcg32 rng(77);
    int hits = 0;
    // ---- 盒子 vs 盒子 ----
    for (int i = 0; i < 3000; ++i) {
        Vec2 pa{rnd(rng, 0, 100), rnd(rng, 0, 100)}, pb{rnd(rng, 0, 100), rnd(rng, 0, 100)};
        Aabb a{pa, pa + Vec2{rnd(rng, 1, 20), rnd(rng, 1, 20)}}, b{pb, pb + Vec2{rnd(rng, 1, 20), rnd(rng, 1, 20)}};
        Vec2 da{rnd(rng, -150, 150), rnd(rng, -150, 150)}, db{rnd(rng, -50, 50), rnd(rng, -50, 50)};
        auto got = sweep(a, da, b, db);
        auto ref = bruteToi([&](float t) {
            return overlaps(Aabb{a.min + da * t, a.max + da * t}, Aabb{b.min + db * t, b.max + db * t});
        });
        if (ref) {
            ++hits;
            CHECK(got && std::abs(got->t - *ref) < 1e-3f);
        } else if (got) {
            // 采样没发现、解析解却说撞了：只能是擦边而过（重叠的时间极短）
            Aabb ea{a.min + da * got->t, a.max + da * got->t}, eb{b.min + db * got->t, b.max + db * got->t};
            float gap = std::max({eb.min.x - ea.max.x, ea.min.x - eb.max.x, eb.min.y - ea.max.y, ea.min.y - eb.max.y});
            CHECK(std::abs(gap) < 1e-2f);
        }
    }

    // ---- 圆 vs 线段 ----
    for (int i = 0; i < 3000; ++i) {
        Circle c{{rnd(rng, 0, 100), rnd(rng, 0, 100)}, rnd(rng, 1, 15)};
        Vec2 a{rnd(rng, 0, 100), rnd(rng, 0, 100)}, b = i % 10 == 0 ? a : Vec2{rnd(rng, 0, 100), rnd(rng, 0, 100)};
        Vec2 d{rnd(rng, -150, 150), rnd(rng, -150, 150)};
        auto got = sweep(c, d, a, b);
        auto ref = bruteToi([&](float t) { return pointSegment(c.center + d * t, a, b) < c.radius; });
        if (ref) {
            ++hits;
            CHECK(got && std::abs(got->t - *ref) < 1e-3f);
            if (got && got->t > 0) {
                // 接触时圆心离线段正好是 r，法线指向圆心
                Vec2 p = c.center + d * got->t;
                CHECK(std::abs(pointSegment(p, a, b) - c.radius) < 1e-2f);
                CHECK(std::abs(length(got->normal) - 1) < 1e-3f && dot(got->normal, d) < 0);
            }
        } else if (got) {
            CHECK(std::abs(pointSegment(c.center + d * got->t, a, b) - c.radius) < 1e-2f);
        }
    }

    // ---- 两个移动的圆 ----
    for (int i = 0; i < 3000; ++i) {
        Circle a{{rnd(rng, 0, 100), rnd(rng, 0, 100)}, rnd(rng, 1, 15)}, b{{rnd(rng, 0, 100), rnd(rng, 0, 100)}, rnd(rng, 1, 15)};
        Vec2 da{rnd(rng, -150, 150), rnd(rng, -150, 150)}, db{rnd(rng, -150, 150), rnd(rng, -150, 150)};
        auto got = sweep(a, da, b, db);
        auto ref = bruteToi([&](float t) { return distance(a.center + da * t, b.center + db * t) < a.radius + b.radius; });
        if (ref) {
            ++hits;
            CHECK(got && std::abs(got->t - *ref) < 1e-3f);
        } else if (got) {
            CHECK(std::abs(distance(a.center + da * got->t, b.center + db * got->t) - a.radius - b.radius) < 1e-2f);
        }
    }

    // ---- 保守前进：旋转的凸形状 ----
    int caHits = 0;
    long caIters = 0;
    for (int i = 0; i < 3000; ++i) {
        auto shape = [&](int kind) {
            if (kind == 0) return ConvexShape{makeBox({0, 0}, {rnd(rng, 2, 12), rnd(rng, 1, 6)}).v, 0};
            if (kind == 1) return ConvexShape{{{0, 0}}, rnd(rng, 2, 10)};
            return ConvexShape{regularPolygon({0, 0}, rnd(rng, 3, 10), 3 + i % 4).v, rnd(rng, 0, 3)};
        };
        ConvexShape a = shape(i % 3), b = shape((i / 3) % 3);
        Motion ma{{rnd(rng, 0, 100), rnd(rng, 0, 100)}, rnd(rng, 0, 6), {rnd(rng, -120, 120), rnd(rng, -120, 120)}, rnd(rng, -6, 6)};
        Motion mb{{rnd(rng, 0, 100), rnd(rng, 0, 100)}, rnd(rng, 0, 6), {rnd(rng, -40, 40), rnd(rng, -40, 40)}, rnd(rng, -3, 3)};
        const float tol = 0.05f;
        auto got = conservativeAdvancement(a, ma, b, mb, tol);
        auto ref = bruteToi([&](float t) { return query(poseAt(a, ma, t), poseAt(b, mb, t)).distance < 0; }, 2000);
        if (ref && *ref == 0) continue;  // 一开始就相交
        if (got) caIters += got->iterations;
        if (ref) {
            ++caHits;
            CHECK(got.has_value());                // 不会漏掉（不会隧穿）
            if (got) CHECK(got->t <= *ref + 1e-4f);  // 保守：不会晚于真正的碰撞时刻
        }
        if (got) {
            float dist = query(poseAt(a, ma, got->t), poseAt(b, mb, got->t)).distance;
            CHECK(dist > -1e-2f && dist < tol + 1e-3f);  // 停在接触前一点点
        }
    }
    std::printf("swept tests: %d hits checked; conservative advancement: %d hits, %.1f iterations on average\n", hits, caHits,
                double(caIters) / std::max(1, caHits));
    CHECK(hits > 1500 && caHits > 200);
    return report("ccd_test");
}
