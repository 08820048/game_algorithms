// 贝塞尔与 Catmull-Rom：求值的精度与开销、弧长求积、匀速移动、折线化的段数、参数化与尖点自交（不由 CMake 构建）：
//   c++ -std=c++20 -O2 -Icpp/common cpp/animation/spline/spline_bench.cpp -o spline_bench && ./spline_bench [section]
//   section：precision cost arclength speed flatten catmull（缺省全部）
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

#include "../../foundations/random/random.h"
#include "spline.h"

using namespace ga;
using namespace ga::spline;
using Clock = std::chrono::steady_clock;

static bool want(int argc, char** argv, const char* s) { return argc < 2 || std::strcmp(argv[1], s) == 0; }
static double msSince(Clock::time_point t0) { return std::chrono::duration<double, std::milli>(Clock::now() - t0).count(); }
static Vec2 rnd(Pcg32& r, float s = 100) { return {(r.nextFloat() * 2 - 1) * s, (r.nextFloat() * 2 - 1) * s}; }

template <class T>
struct P2 {
    T x = 0, y = 0;
    P2 operator+(P2 o) const { return {x + o.x, y + o.y}; }
    P2 operator-(P2 o) const { return {x - o.x, y - o.y}; }
    P2 operator*(T s) const { return {x * s, y * s}; }
};
using PF = P2<float>;
using PL = P2<long double>;

// 浮点下三种求值方法的误差（以长双精度的 de Casteljau 为准），单位 = 控制点坐标的范围
static void precision() {
    std::printf("== precision: float evaluation error vs long double, max over 200 curves x 101 t, relative to coordinate range\n");
    for (float offset : {0.0f, 10000.0f}) {
        std::printf("control points in [-100,100]^2 + %.0f\n", offset);
        for (int n : {4, 6, 9, 13, 17, 21}) {
            Pcg32 r{uint64_t(n)};
            double eC = 0, eB = 0, eH = 0;
            for (int trial = 0; trial < 200; ++trial) {
                std::vector<PF> pf;
                std::vector<PL> pl;
                for (int i = 0; i < n; ++i) {
                    Vec2 v = rnd(r);
                    pf.push_back({v.x + offset, v.y + offset});
                    pl.push_back({(long double)v.x + offset, (long double)v.y + offset});
                }
                auto a = toPowerBasis<PF, float>(pf.data(), n);
                for (int k = 0; k <= 100; ++k) {
                    float t = k / 100.0f;
                    PL ref = deCasteljau(pl.data(), n, (long double)t);
                    auto err = [&](PF p) { return double(std::hypot((long double)p.x - ref.x, (long double)p.y - ref.y)); };
                    eC = std::max(eC, err(deCasteljau(pf.data(), n, t)));
                    eB = std::max(eB, err(bernstein(pf.data(), n, t)));
                    eH = std::max(eH, err(horner(a, t)));
                }
            }
            std::printf("  degree %-2d  de Casteljau %.2e  Bernstein %.2e  power basis %.2e\n", n - 1, eC / 200, eB / 200, eH / 200);
        }
    }
}

static void cost() {
    std::printf("== cost: ns per cubic evaluation (best of 7 rounds of 1M)\n");
    Pcg32 r(1);
    std::vector<Cubic> cs;
    for (int i = 0; i < 64; ++i) cs.push_back({{rnd(r), rnd(r), rnd(r), rnd(r)}});
    std::vector<std::vector<Vec2>> pw;
    for (const Cubic& c : cs) pw.push_back(toPowerBasis<Vec2, float>(c.p, 4));
    const int N = 1000000;
    volatile float sink = 0;
    auto bench = [&](const char* name, auto&& f) {
        double best = 1e30;
        for (int round = 0; round < 7; ++round) {
            auto t0 = Clock::now();
            float acc = 0;
            for (int i = 0; i < N; ++i) {
                Vec2 p = f(i & 63, float(i & 1023) * (1.0f / 1023));
                acc += p.x + p.y;
            }
            sink = sink + acc;
            best = std::min(best, msSince(t0));
        }
        std::printf("  %-22s %.2f ns\n", name, best * 1e6 / N);
    };
    bench("expanded (Cubic::at)", [&](int k, float t) { return cs[size_t(k)].at(t); });
    bench("de Casteljau", [&](int k, float t) { return deCasteljau(cs[size_t(k)].p, 4, t); });
    bench("Bernstein", [&](int k, float t) { return bernstein(cs[size_t(k)].p, 4, t); });
    bench("power basis (Horner)", [&](int k, float t) { return horner(pw[size_t(k)], t); });
}

// 双精度的速度 |B'(t)|，作弧长的参照
static double speedD(const Cubic& c, double t) {
    double u = 1 - t;
    double x = 3 * u * u * (c.p[1].x - c.p[0].x) + 6 * u * t * (c.p[2].x - c.p[1].x) + 3 * t * t * (c.p[3].x - c.p[2].x);
    double y = 3 * u * u * (c.p[1].y - c.p[0].y) + 6 * u * t * (c.p[2].y - c.p[1].y) + 3 * t * t * (c.p[3].y - c.p[2].y);
    return std::hypot(x, y);
}
static double refLength(const Cubic& c) {
    static Quadrature q = gaussLegendre(20);
    double s = 0;
    for (int i = 0; i < 4096; ++i) s += integrate(q, i / 4096.0, (i + 1) / 4096.0, [&](double t) { return speedD(c, t); });
    return s;
}
static void arclength() {
    std::printf("== arclength: relative error (median / max over 300 random cubics), double integrand\n");
    Pcg32 r(2);
    std::vector<Cubic> cs;
    for (int i = 0; i < 300; ++i) cs.push_back({{rnd(r), rnd(r), rnd(r), rnd(r)}});
    std::vector<double> ref;
    for (const Cubic& c : cs) ref.push_back(refLength(c));
    auto report = [&](const char* name, auto&& f) {
        std::vector<double> e;
        for (size_t i = 0; i < cs.size(); ++i) e.push_back(std::fabs(f(cs[i]) - ref[i]) / ref[i]);
        std::sort(e.begin(), e.end());
        std::printf("  %-26s median %.1e  max %.1e\n", name, e[e.size() / 2], e.back());
    };
    for (int order : {2, 3, 5, 8})
        for (int pieces : {1, 4, 16}) {
            Quadrature q = gaussLegendre(order);
            char name[64];
            std::snprintf(name, sizeof name, "Gauss %d pts x %d pieces", order, pieces);
            report(name, [&](const Cubic& c) {
                double s = 0;
                for (int i = 0; i < pieces; ++i) s += integrate(q, double(i) / pieces, double(i + 1) / pieces, [&](double t) { return speedD(c, t); });
                return s;
            });
        }
    for (int n : {8, 32, 128, 1024}) {
        char name[64];
        std::snprintf(name, sizeof name, "chords x %d (%d evals)", n, n + 1);
        report(name, [&](const Cubic& c) {
            double s = 0;
            Vec2 prev = c.p[0];
            for (int i = 1; i <= n; ++i) {
                Vec2 q = c.at(float(i) / n);
                s += std::hypot(double(q.x) - prev.x, double(q.y) - prev.y), prev = q;
            }
            return s;
        });
    }
    // 接近尖点的曲线：速度在 t = 0.5 附近几乎为零，|B'| 在那里不光滑
    Cubic nc{{{0, 0}, {100, 100}, {0, 100}, {100.5f, 0}}};
    double rl = refLength(nc);
    std::printf("  near-cusp curve (min speed %.3g):", speedD(nc, 0.5));
    for (int pieces : {1, 4, 16, 64}) {
        Quadrature q = gaussLegendre(5);
        double s = 0;
        for (int i = 0; i < pieces; ++i) s += integrate(q, double(i) / pieces, double(i + 1) / pieces, [&](double t) { return speedD(nc, t); });
        std::printf("  5x%d %.1e", pieces, std::fabs(s - rl) / rl);
    }
    std::printf("\n");
}

// 匀速移动：每帧走 L/1000，实际走过的距离和目标的比值
static void speed() {
    std::printf("== speed: naive t vs arc-length table; per-step distance / target, 500 random cubics\n");
    Pcg32 r(3);
    std::vector<Cubic> cs;
    for (int i = 0; i < 500; ++i) cs.push_back({{rnd(r), rnd(r), rnd(r), rnd(r)}});
    std::vector<double> ratio;
    for (const Cubic& c : cs) {
        double lo = 1e30, hi = 0;
        for (int k = 0; k <= 1000; ++k) {
            double v = speedD(c, k / 1000.0);
            lo = std::min(lo, v), hi = std::max(hi, v);
        }
        ratio.push_back(hi / std::max(lo, 1e-9));
    }
    std::sort(ratio.begin(), ratio.end());
    std::printf("  naive t: max/min speed  median %.1f  p90 %.1f  p99 %.0f\n", ratio[250], ratio[450], ratio[495]);
    for (int K : {4, 8, 16, 32, 64})
        for (int newton : {0, 1, 3}) {
            std::vector<double> dev;
            for (const Cubic& c : cs) {
                ArcLengthTable tab(c, K);
                // 每一步实际走过的弧长（双精度积分，而不是弦长：急转弯处弦比弧短得多）
                static Quadrature q8 = gaussLegendre(8);
                double L = refLength(c), worst = 0, prevT = 0;
                for (int k = 1; k <= 1000; ++k) {
                    double t = tab.paramAt(tab.length() * k / 1000, newton);
                    double d = 0;
                    for (int j = 0; j < 4; ++j) d += integrate(q8, prevT + (t - prevT) * j / 4, prevT + (t - prevT) * (j + 1) / 4, [&](double u) { return speedD(c, u); });
                    worst = std::max(worst, std::fabs(d / (L / 1000) - 1));
                    prevT = t;
                }
                dev.push_back(worst);
            }
            std::sort(dev.begin(), dev.end());
            std::printf("  table %-3d newton %d: worst step deviation  median %5.1f%%  p90 %5.1f%%  max %6.1f%%\n", K, newton, dev[250] * 100, dev[450] * 100, dev.back() * 100);
        }
}

static double maxError(const Cubic& c, const std::vector<Vec2>& poly) {
    // 均匀切分时，每段弦对应的参数区间已知，按区间比较即可；自适应的折线用整体最近距离
    double worst = 0;
    for (int i = 0; i <= 4000; ++i) {
        Vec2 p = c.at(i / 4000.0f);
        float best = 1e30f;
        for (size_t k = 0; k + 1 < poly.size(); ++k) best = std::min(best, distToSegment(p, poly[k], poly[k + 1]));
        worst = std::max(worst, double(best));
    }
    return worst;
}
static void flatten() {
    std::printf("== flatten: segments needed (mean over 200 random cubics) and actual max error / tol\n");
    Pcg32 r(4);
    std::vector<Cubic> cs;
    for (int i = 0; i < 200; ++i) cs.push_back({{rnd(r), rnd(r), rnd(r), rnd(r)}});
    for (float tol : {1.0f, 0.25f, 0.05f}) {
        double nA = 0, nW = 0, nO = 0, eA = 0, eW = 0, eAmin = 1e9;
        for (const Cubic& c : cs) {
            std::vector<Vec2> a, w;
            flattenAdaptive(c, tol, &a);
            int nw = wangSegments(c, tol);
            flattenUniform(c, nw, &w);
            nA += double(a.size()) - 1, nW += nw;
            double ea = maxError(c, a) / tol;
            eA = std::max(eA, ea), eAmin = std::min(eAmin, ea), eW = std::max(eW, maxError(c, w) / tol);
            // 均匀切分最少需要几段：从 1 往上试（二分）
            int lo = 1, hi = nw;
            while (lo < hi) {
                int mid = (lo + hi) / 2;
                std::vector<Vec2> u;
                flattenUniform(c, mid, &u);
                if (maxError(c, u) <= tol) hi = mid;
                else lo = mid + 1;
            }
            nO += lo;
        }
        double N = double(cs.size());
        std::printf("  tol %-5.2f adaptive %6.1f (error/tol max %.2f, min %.2f) | Wang %6.1f (error/tol max %.2f) | best uniform %6.1f\n", tol, nA / N, eA, eAmin, nW / N, eW, nO / N);
    }
}

// Catmull-Rom 的参数化：段内尖点 / 自交的比例，以及曲线离开弦 p1p2 有多远
static void catmull() {
    std::printf("== catmull: share of segments with a cusp or self-intersection, and bulge (max distance from chord p1p2 / |p1p2|)\n");
    for (int scenario = 0; scenario < 2; ++scenario) {
        std::printf(scenario == 0 ? "uneven spacing: |p1p2| ~ 3, neighbours ~ 100\n" : "random points in a square\n");
        for (float alpha : {0.0f, 0.25f, 0.4f, 0.45f, 0.5f, 0.75f, 1.0f}) {
            Pcg32 r(8);
            int cusp = 0, loop = 0;
            std::vector<double> bulge;
            const int N = 5000;
            for (int trial = 0; trial < N; ++trial) {
                Vec2 p0, p1, p2, p3;
                if (scenario == 0) p1 = rnd(r, 10), p2 = p1 + rnd(r, 3), p0 = p1 + rnd(r, 100), p3 = p2 + rnd(r, 100);
                else p0 = rnd(r), p1 = rnd(r), p2 = rnd(r), p3 = rnd(r);
                Cubic c = catmullRomToBezier(p0, p1, p2, p3, alpha);
                Defects d = findDefects(c, 120);
                cusp += d.cusp, loop += d.loop;
                float worst = 0;
                for (int k = 0; k <= 100; ++k) worst = std::max(worst, distToSegment(c.at(k / 100.0f), p1, p2));
                bulge.push_back(worst / std::max(1e-6f, length(p2 - p1)));
            }
            std::sort(bulge.begin(), bulge.end());
            std::printf("  alpha %.2f  cusp %5.2f%%  loop %5.2f%%  bulge median %.2f p90 %.2f p99 %.1f\n", alpha, 100.0 * cusp / N, 100.0 * loop / N, bulge[N / 2], bulge[N * 9 / 10], bulge[N * 99 / 100]);
        }
    }
}

int main(int argc, char** argv) {
    if (want(argc, argv, "precision")) precision();
    if (want(argc, argv, "cost")) cost();
    if (want(argc, argv, "arclength")) arclength();
    if (want(argc, argv, "speed")) speed();
    if (want(argc, argv, "flatten")) flatten();
    if (want(argc, argv, "catmull")) catmull();
}
