#include "spline.h"

#include <cmath>
#include <cstdio>
#include <vector>

#include "../../foundations/random/random.h"
#include "check.h"

using namespace ga;
using namespace ga::spline;

// 双精度点，作参照
struct D2 {
    double x = 0, y = 0;
    D2 operator+(D2 o) const { return {x + o.x, y + o.y}; }
    D2 operator-(D2 o) const { return {x - o.x, y - o.y}; }
    D2 operator*(double s) const { return {x * s, y * s}; }
};
static D2 d2(Vec2 v) { return {v.x, v.y}; }
static double dist(D2 a, D2 b) { return std::hypot(a.x - b.x, a.y - b.y); }

static Vec2 rnd(Pcg32& r, float s = 100) { return {(r.nextFloat() * 2 - 1) * s, (r.nextFloat() * 2 - 1) * s}; }
static Cubic randomCubic(Pcg32& r) { return {{rnd(r), rnd(r), rnd(r), rnd(r)}}; }

// ---- 求值 ----
static void testEvaluatorsAgree() {
    Pcg32 r(1);
    for (int trial = 0; trial < 500; ++trial) {
        int n = 2 + int(r.below(10));
        std::vector<D2> p;
        for (int i = 0; i < n; ++i) p.push_back(d2(rnd(r)));
        auto a = toPowerBasis<D2, double>(p.data(), n);
        for (int k = 0; k <= 20; ++k) {
            double t = k / 20.0;
            D2 x = deCasteljau(p.data(), n, t), y = bernstein(p.data(), n, t), z = horner(a, t);
            CHECK(dist(x, y) < 1e-9 && dist(x, z) < 1e-9);
        }
        CHECK(dist(deCasteljau(p.data(), n, 0.0), p[0]) < 1e-12);       // 经过首尾两点
        CHECK(dist(deCasteljau(p.data(), n, 1.0), p.back()) < 1e-12);
    }
    // 三次的展开式和 de Casteljau 一致
    for (int trial = 0; trial < 200; ++trial) {
        Cubic c = randomCubic(r);
        for (int k = 0; k <= 10; ++k) {
            float t = k / 10.0f;
            CHECK(length(c.at(t) - deCasteljau(c.p, 4, t)) < 1e-3f);
        }
    }
}

static void testSplit() {
    Pcg32 r(2);
    for (int trial = 0; trial < 300; ++trial) {
        Cubic c = randomCubic(r), L, R;
        float t = 0.05f + 0.9f * r.nextFloat();
        c.split(t, &L, &R);
        for (int k = 0; k <= 10; ++k) {
            float u = k / 10.0f;
            CHECK(length(L.at(u) - c.at(u * t)) < 2e-3f);
            CHECK(length(R.at(u) - c.at(t + (1 - t) * u)) < 2e-3f);
        }
    }
}

// 导数和差分对照；四分之一圆的近似（控制柄 0.5523）曲率约等于 1/r
static void testDerivatives() {
    Pcg32 r(3);
    for (int trial = 0; trial < 300; ++trial) {
        Cubic c = randomCubic(r);
        std::vector<D2> p = {d2(c.p[0]), d2(c.p[1]), d2(c.p[2]), d2(c.p[3])};
        double t = 0.05 + 0.9 * r.nextDouble(), h = 1e-5;
        D2 fd = (deCasteljau(p.data(), 4, t + h) - deCasteljau(p.data(), 4, t - h)) * (1 / (2 * h));
        D2 fd2 = (deCasteljau(p.data(), 4, t + h) - deCasteljau(p.data(), 4, t) * 2 + deCasteljau(p.data(), 4, t - h)) * (1 / (h * h));
        CHECK(dist(d2(c.d1(float(t))), fd) < 1e-2 * (1 + std::hypot(fd.x, fd.y)));
        CHECK(dist(d2(c.d2(float(t))), fd2) < 0.5);
    }
    const float k = 0.5522847f, R = 50;
    Cubic arc{{{R, 0}, {R, R * k}, {R * k, R}, {0, R}}};
    for (int i = 0; i <= 10; ++i) {
        float t = i / 10.0f;
        CHECK(std::fabs(length(arc.at(t)) - R) < 0.02f);
        CHECK(std::fabs(arc.curvature(t) * R - 1) < 0.025f);  // 这个近似本身的曲率偏差约 2%
    }
    Cubic cw{{{0, R}, {R * k, R}, {R, R * k}, {R, 0}}};  // 顺时针：曲率为负
    CHECK(cw.curvature(0.5f) < 0);
}

// ---- 求积与弧长 ----
static void testGauss() {
    for (int n = 1; n <= 16; ++n) {
        Quadrature q = gaussLegendre(n);
        double ws = 0;
        for (double w : q.w) ws += w;
        CHECK(std::fabs(ws - 2) < 1e-12);
        for (int k = 0; k <= 2 * n - 1; ++k) {  // ∫₀¹ tᵏ dt = 1 / (k + 1)
            double v = integrate(q, 0, 1, [&](double t) { return std::pow(t, k); });
            CHECK(std::fabs(v - 1.0 / (k + 1)) < 1e-12);
        }
        double v = integrate(q, 0, 1, [&](double t) { return std::pow(t, 2 * n); });
        if (n <= 8) CHECK(std::fabs(v - 1.0 / (2 * n + 1)) > 1e-14);  // 2n 次就不再精确
    }
}

// 参照：双精度折线，一百万段
static double chordLength(const Cubic& c, double t0, double t1, int n = 1000000) {
    std::vector<D2> p = {d2(c.p[0]), d2(c.p[1]), d2(c.p[2]), d2(c.p[3])};
    double s = 0;
    D2 prev = deCasteljau(p.data(), 4, t0);
    for (int i = 1; i <= n; ++i) {
        D2 q = deCasteljau(p.data(), 4, t0 + (t1 - t0) * i / n);
        s += dist(prev, q), prev = q;
    }
    return s;
}
static void testArcLength() {
    Cubic line{{{0, 0}, {10, 0}, {11, 0}, {30, 0}}};  // 共线但控制点不均匀：t 不是弧长，长度仍然是 30
    CHECK(std::fabs(arcLength(line) - 30) < 1e-4);
    const float k = 0.5522847f, R = 50;
    Cubic arc{{{R, 0}, {R, R * k}, {R * k, R}, {0, R}}};
    CHECK(std::fabs(arcLength(arc) - M_PI / 2 * R) < 0.02);  // 近似比真圆弧长 0.011
    CHECK(std::fabs(arcLength(arc) - chordLength(arc, 0, 1)) < 1e-4);
    Pcg32 r(4);
    for (int trial = 0; trial < 40; ++trial) {
        Cubic c = randomCubic(r);
        double ref = chordLength(c, 0, 1);
        CHECK(std::fabs(arcLength(c, 0, 1, 32, 8) - ref) < 1e-4 * ref + 1e-3);
    }
}

static void testArcLengthTable() {
    Pcg32 r(5);
    double worstPlain = 0, worstNewton = 0;
    for (int trial = 0; trial < 30; ++trial) {
        Cubic c = randomCubic(r);
        ArcLengthTable tab(c, 32);
        CHECK(std::fabs(tab.length() - chordLength(c, 0, 1, 200000)) < 1e-3 * tab.length() + 1e-3);
        CHECK(tab.paramAt(-1) == 0 && tab.paramAt(1e9) == 1);
        for (int k = 1; k < 20; ++k) {
            double s = tab.length() * k / 20;
            float t0 = tab.paramAt(s), t1 = tab.paramAt(s, 2);
            worstPlain = std::max(worstPlain, std::fabs(arcLength(c, 0, t0, 32, 8) - s) / tab.length());
            worstNewton = std::max(worstNewton, std::fabs(arcLength(c, 0, t1, 32, 8) - s) / tab.length());
        }
        // 单调：s 越大 t 越大
        float prev = -1;
        for (int k = 0; k <= 100; ++k) {
            float t = tab.paramAt(tab.length() * k / 100, 1);
            CHECK(t >= prev);
            prev = t;
        }
    }
    // 牛顿法多走几步要越来越准，不能在收敛后又跳开
    Pcg32 r2(15);
    double worst3 = 0;
    for (int trial = 0; trial < 60; ++trial) {
        Cubic c = randomCubic(r2);
        for (int K : {32, 64}) {
            ArcLengthTable tab(c, K);
            for (int k = 1; k < 200; ++k) {
                double s = tab.length() * k / 200;
                worst3 = std::max(worst3, std::fabs(arcLength(c, 0, tab.paramAt(s, 3), 32, 8) - s) / tab.length());
            }
        }
    }
    CHECK(worst3 < 5e-5);
    CHECK(worstPlain < 5e-3);
    CHECK(worstNewton < 5e-5);  // float 的 t 和导数限制了精度
    CHECK(worstNewton * 30 < worstPlain);
}

// ---- 折线化 ----
// 曲线上密集采样的点到折线的最大距离（对整条折线取最近）
static double polylineError(const Cubic& c, const std::vector<Vec2>& poly, int samples = 2000) {
    double worst = 0;
    for (int i = 0; i <= samples; ++i) {
        Vec2 p = c.at(float(i) / samples);
        float best = 1e30f;
        for (size_t k = 0; k + 1 < poly.size(); ++k) best = std::min(best, distToSegment(p, poly[k], poly[k + 1]));
        worst = std::max(worst, double(best));
    }
    return worst;
}
static void testFlatten() {
    Pcg32 r(6);
    for (int trial = 0; trial < 60; ++trial) {
        Cubic c = randomCubic(r);
        for (float tol : {2.0f, 0.5f, 0.1f}) {
            std::vector<Vec2> a, u;
            flattenAdaptive(c, tol, &a);
            flattenUniform(c, wangSegments(c, tol), &u);
            CHECK(polylineError(c, a) <= tol * 1.001 + 1e-4);
            CHECK(polylineError(c, u) <= tol * 1.001 + 1e-4);
            CHECK(a.front() == c.p[0] && a.back() == c.p[3]);
        }
    }
    CHECK(std::fabs(distToSegment({5, 0}, {0, 0}, {1, 0}) - 4) < 1e-6f);  // 线段外的点到最近的端点
    CHECK(std::fabs(distToSegment({0.5f, 2}, {0, 0}, {1, 0}) - 2) < 1e-6f);
    // 直线只要一段
    std::vector<Vec2> a;
    flattenAdaptive({{{0, 0}, {1, 0}, {2, 0}, {3, 0}}}, 0.1f, &a);
    CHECK(a.size() == 2);
    CHECK(wangSegments({{{0, 0}, {1, 0}, {2, 0}, {3, 0}}}, 0.1f) == 1);
}

// ---- Catmull-Rom ----
static void testCatmullRom() {
    Pcg32 r(7);
    for (int trial = 0; trial < 500; ++trial) {
        Vec2 p0 = rnd(r), p1 = rnd(r), p2 = rnd(r), p3 = rnd(r);
        for (float alpha : {0.0f, 0.5f, 1.0f}) {
            Cubic b = catmullRomToBezier(p0, p1, p2, p3, alpha);
            float scale = std::max({length(p1 - p0), length(p2 - p1), length(p3 - p2)});
            for (int k = 0; k <= 10; ++k) {
                float u = k / 10.0f;
                CHECK(length(b.at(u) - catmullRom(p0, p1, p2, p3, alpha, u)) < 1e-4f * scale + 1e-3f);
            }
            CHECK(length(b.p[0] - p1) == 0 && length(b.p[3] - p2) == 0);
        }
        // 均匀参数化就是经典公式：切线 = (p2 - p0) / 2
        Cubic u = catmullRomToBezier(p0, p1, p2, p3, 0);
        CHECK(length(u.d1(0) - (p2 - p0) * 0.5f) < 1e-3f);
        CHECK(length(u.d1(1) - (p3 - p1) * 0.5f) < 1e-3f);
    }
    // 链：经过每个点，相邻两段在接点处对节点参数的导数相等（C1）
    for (int trial = 0; trial < 100; ++trial) {
        std::vector<Vec2> pts;
        for (int i = 0; i < 6; ++i) pts.push_back(rnd(r));
        for (float alpha : {0.0f, 0.5f, 1.0f})
            for (bool closed : {false, true}) {
                auto segs = catmullRomChain(pts, alpha, closed);
                CHECK(int(segs.size()) == (closed ? 6 : 5));
                for (size_t i = 0; i < segs.size(); ++i) {
                    CHECK(segs[i].p[0] == pts[i]);
                    if (i + 1 >= segs.size() && !closed) break;
                    const Cubic& a = segs[i];
                    const Cubic& b = segs[(i + 1) % segs.size()];
                    float da = knotStep(a.p[0], a.p[3], alpha), db = knotStep(b.p[0], b.p[3], alpha);
                    Vec2 va = a.d1(1) / da, vb = b.d1(0) / db;
                    CHECK(length(va - vb) < 1e-3f * (1 + length(va)));
                }
            }
    }
    // 开放的链：两端外推虚拟点 p₋₁ = 2p₀ − p₁，均匀参数化时起点切线 = p₁ − p₀
    {
        std::vector<Vec2> pts = {{0, 0}, {10, 5}, {20, -3}, {35, 0}};
        auto segs = catmullRomChain(pts, 0, false);
        CHECK(length(segs[0].d1(0) - (pts[1] - pts[0])) < 1e-3f);
        CHECK(length(segs[2].d1(1) - (pts[3] - pts[2])) < 1e-3f);
    }
    // 重合的点不产生 NaN
    Cubic z = catmullRomToBezier({0, 0}, {0, 0}, {1, 0}, {1, 0}, 0.5f);
    CHECK(std::isfinite(z.p[1].x) && std::isfinite(z.p[2].y));
}

// 向心参数化的段内没有尖点和自交（Yuksel 等 2011），均匀参数化会有。端点处速度为零不算尖点
static void testCentripetalHasNoDefects() {
    Defects e = findDefects({{{0, 0}, {1, 1}, {0, 1}, {1, 0}}});
    CHECK(e.cusp && !e.loop);
    e = findDefects({{{0, 0}, {3, 2}, {-2, 2}, {1, 0}}});
    CHECK(e.loop);
    e = findDefects({{{0, 0}, {1, 1}, {2, 1}, {3, 0}}});
    CHECK(!e.loop && !e.cusp);
    // 端点处速度为零（P2 = P3）不算尖点
    CHECK(!hasCusp({{{0, 0}, {1, 1}, {2, 0}, {2, 0}}}));
    // 急转弯但不自交。这里原来断言"120 段的折线会相交"，那其实是舍入造成的误报：急转弯处相邻的线段几乎共线，
    // 叉积接近 0，结果取决于编译器是否把 a*b − c*d 合并成 FMA。关掉合并（6.4）后，任何采样数都不相交。
    // 随机 6 万条曲线里，"稀疏相交、10 倍采样不相交"的有 2 条（合并时 17 条），留出 1e-3 的余量再判，一条都不剩
    Cubic sharp = catmullRomToBezier({11.6457f, 12.1852f}, {3.83368f, -4.3235f}, {3.54312f, -5.09548f}, {-24.9963f, -93.6298f}, 1);
    CHECK(!polylineSelfCrosses(sharp, 1200));
    CHECK(!findDefects(sharp, 120).loop);

    Pcg32 r(8);
    int bad[3] = {};
    for (int trial = 0; trial < 3000; ++trial) {
        // 点间距相差悬殊：中间两点很近，两边很远
        Vec2 p1 = rnd(r, 10), p2 = p1 + rnd(r, 3), p0 = p1 + rnd(r, 100), p3 = p2 + rnd(r, 100);
        float alphas[3] = {0, 0.5f, 1};
        for (int k = 0; k < 3; ++k) {
            Defects d = findDefects(catmullRomToBezier(p0, p1, p2, p3, alphas[k]), 120);
            bad[k] += d.cusp || d.loop;
        }
    }
    CHECK(bad[0] > 300);
    CHECK(bad[1] == 0);
    // 弦长参数化（α = 1）在这种分布下也没出现，但它没有这个保证：随机点集上偶尔会自交（见 spline_bench catmull）
}

int main() {
    testEvaluatorsAgree();
    testSplit();
    testDerivatives();
    testGauss();
    testArcLength();
    testArcLengthTable();
    testFlatten();
    testCatmullRom();
    testCentripetalHasNoDefects();
    return report("spline_test");
}
