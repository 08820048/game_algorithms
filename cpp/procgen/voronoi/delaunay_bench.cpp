// Delaunay 与 Voronoi：插入顺序与定位步数、空腔大小、超级三角形的缺失、谓词精度、邻居数分布、Lloyd 收敛、开销（不由 CMake 构建）：
//   c++ -std=c++20 -O2 -Icpp/common cpp/procgen/voronoi/delaunay_bench.cpp -o delaunay_bench && ./delaunay_bench [section]
//   section：order cavity super precision cells lloyd speed（缺省全部）
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <map>
#include <set>
#include <vector>

#include "../poisson/poisson.h"
#include "delaunay.h"

using namespace ga;
using namespace ga::delaunay;
using Clock = std::chrono::steady_clock;
using Tri3 = std::array<int, 3>;

static bool want(int argc, char** argv, const char* s) { return argc < 2 || std::strcmp(argv[1], s) == 0; }
static double msSince(Clock::time_point t0) { return std::chrono::duration<double, std::milli>(Clock::now() - t0).count(); }
static Tri3 canon(Tri3 t) {
    while (!(t[0] < t[1] && t[0] < t[2])) t = {t[1], t[2], t[0]};
    return t;
}
static std::vector<Vec2> uniform(int n, float w, float h, uint64_t seed) {
    Pcg32 rng{seed};
    std::vector<Vec2> p(n);
    for (Vec2& a : p) a = {rng.nextFloat() * w, rng.nextFloat() * h};
    return p;
}
static long long violations(const std::vector<Vec2>& p, const std::vector<Tri3>& ts) {
    long long bad = 0;
    for (const Tri3& t : ts)
        for (size_t d = 0; d < p.size(); ++d) bad += inCircle(p[t[0]], p[t[1]], p[t[2]], p[d], Precision::Exact) > 0;
    return bad;
}

// 内部单元（不碰边界框）的统计
struct CellStats {
    double frac5 = 0, frac6 = 0, frac7 = 0, other = 0, mean = 0, areaCv = 0, minNN = 0;
};
static CellStats cellStats(const std::vector<Vec2>& p, double w, double h) {
    Triangulation tr = triangulate(p);
    auto nb = tr.neighbors();
    auto cells = voronoiCells(p, nb, w, h);
    CellStats s;
    int cnt = 0;
    double sa = 0, sa2 = 0;
    for (size_t i = 0; i < p.size(); ++i) {
        bool edge = false;
        for (P2 v : cells[i]) edge |= v.x < 1e-9 || v.y < 1e-9 || v.x > w - 1e-9 || v.y > h - 1e-9;
        if (edge) continue;
        int k = int(nb[i].size());
        ++cnt, s.mean += k, s.frac5 += k == 5, s.frac6 += k == 6, s.frac7 += k == 7, s.other += k < 5 || k > 7;
        double a = area(cells[i]);
        sa += a, sa2 += a * a;
    }
    s.mean /= cnt, s.frac5 /= cnt, s.frac6 /= cnt, s.frac7 /= cnt, s.other /= cnt;
    double m = sa / cnt;
    s.areaCv = std::sqrt(std::max(0.0, sa2 / cnt - m * m)) / m;
    double spacing = std::sqrt(w * h / p.size());
    float lo = INFINITY;
    for (size_t i = 0; i < p.size(); ++i)
        for (int j : nb[i]) lo = std::min(lo, std::hypot(p[i].x - p[j].x, p[i].y - p[j].y));
    s.minNN = lo / spacing;
    return s;
}

int main(int argc, char** argv) {
    if (want(argc, argv, "order")) {
        std::printf("== order: uniform points in a square, double predicates; per point ==\n");
        std::printf("  n        order    walk steps  circle tests  cavity  ms\n");
        for (int n : {1000, 10000, 100000, 1000000}) {
            std::vector<Vec2> p = uniform(n, 1000, 1000, 7);
            for (Order o : {Order::Given, Order::SortX, Order::Hilbert}) {
                if (o == Order::Given && n > 100000) {
                    std::printf("  %-8d %-8s (skipped)\n", n, "random");
                    continue;
                }
                auto t0 = Clock::now();
                Triangulation tr = triangulate(p, o);
                double ms = msSince(t0);
                const Stats& s = tr.stats();
                const char* nm = o == Order::Given ? "random" : o == Order::SortX ? "sort-x" : "hilbert";
                std::printf("  %-8d %-8s %10.1f  %12.2f  %6.2f  %8.1f\n", n, nm, double(s.walkSteps) / n, double(s.circleTests) / n, double(s.cavity) / n, ms);
            }
        }
    }

    if (want(argc, argv, "cavity")) {
        std::printf("== cavity: triangles removed per insertion, n = 100000 uniform, hilbert order ==\n");
        std::vector<Vec2> p = uniform(100000, 1000, 1000, 3);
        Pcg32 rng{1};
        Triangulation tr(p);
        std::map<int, long long> hist;
        long long total = 0, ghosts = 0;
        for (int i : insertionOrder(p, Order::Hilbert, rng)) {
            Trace trace;
            long long before = tr.stats().cavity;
            tr.insert(i, &trace);
            const std::vector<Tri3>& cav = trace.cavity;
            long long c = tr.stats().cavity - before;
            if (c > 0) ++hist[int(c)], ++total, ghosts += c - (long long)cav.size();
        }
        for (auto [k, v] : hist)
            if (v * 1000 >= total) std::printf("  removed %2d: %6.2f%%\n", k, 100.0 * v / total);
        std::printf("  mean removed %.3f, created %.3f; ghost triangles removed per insertion %.4f\n", double(tr.stats().cavity) / total,
                    double(tr.stats().cavity) / total + 2, double(ghosts) / total);
    }

    if (want(argc, argv, "super")) {
        std::printf("== super triangle: 2000 random integer points in [0,1000)^2, 10 seeds ==\n");
        std::printf("  scale  precision  triangles  missing  wrong  missing hull edges\n");
        for (Precision pr : {Precision::Exact, Precision::Double})
            for (float sc : {1.0f, 2.0f, 10.0f, 100.0f, 1000.0f, 10000.0f, 1e5f, 1e6f, 1e7f}) {
                if (pr == Precision::Exact && sc > 1e4f) continue;  // 精确版的 128 位整数放不下
                double tris = 0, miss = 0, wrong = 0, hullMiss = 0, refN = 0;
                for (int s = 0; s < 10; ++s) {
                    Pcg32 rng{uint64_t(s + 1)};
                    std::vector<Vec2> p(2000);
                    for (Vec2& a : p) a = {float(rng.below(1000)), float(rng.below(1000))};
                    Triangulation ref = triangulate(p, Order::Hilbert, Precision::Exact);
                    std::set<Tri3> rs;
                    for (auto& t : ref.triangles()) rs.insert(canon(t));
                    std::vector<Tri3> got = superTriangleDelaunay(p, sc, pr);
                    std::set<Tri3> gs;
                    for (auto& t : got) gs.insert(canon(t));
                    int w = 0;
                    for (auto& t : gs) w += !rs.count(t);
                    tris += gs.size(), miss += double(rs.size()) - (gs.size() - w), wrong += w, refN += rs.size();
                    // 凸包边是否还在（作为某个三角形的边）
                    std::set<std::pair<int, int>> edges;
                    for (auto& t : got)
                        for (int e = 0; e < 3; ++e) edges.insert({t[e], t[(e + 1) % 3]});
                    std::vector<int> h = ref.hull();
                    for (size_t k = 0; k < h.size(); ++k) {
                        int a = h[k], b = h[(k + 1) % h.size()];
                        hullMiss += !edges.count({a, b}) && !edges.count({b, a});
                    }
                }
                std::printf("  %-6.0f %-9s  %9.1f  %7.1f  %5.1f  %18.1f   (ref %.0f)\n", sc, pr == Precision::Exact ? "exact" : "double", tris / 10, miss / 10,
                            wrong / 10, hullMiss / 10, refN / 10);
            }
    }

    if (want(argc, argv, "precision")) {
        for (int withCentre = 1; withCentre >= 0; --withCentre) {
        std::printf("== precision: up to 2000 points on a circle of radius R (rounded to integers)%s ==\n", withCentre ? " plus its centre" : "");
        std::printf("  R        precision  points  inserted  broken  fallbacks  valid  empty-circle violations  incircle sign errors\n");
        for (int bits : {6, 10, 14, 18, 22}) {
            double R = std::ldexp(1.0, bits);
            std::vector<Vec2> p;
            std::set<std::pair<int, int>> seen;
            Pcg32 rng{5};
            for (int k = 0; k < 2000; ++k) {
                double th = rng.nextDouble() * 2 * M_PI;
                int x = int(std::lround(R * std::cos(th))) + (1 << 23), y = int(std::lround(R * std::sin(th))) + (1 << 23);
                if (seen.insert({x, y}).second) p.push_back({float(x), float(y)});
            }
            if (withCentre) p.push_back({float(1 << 23), float(1 << 23)});
            // 谓词本身：随机四元组的 inCircle 符号和精确值不一致的比例
            for (Precision pr : {Precision::Float, Precision::Double, Precision::Exact}) {
                long long err = 0, tot = 0;
                Pcg32 r2{9};
                for (int q = 0; q < 200000; ++q) {
                    uint32_t ia = r2.below(uint32_t(p.size())), ib = r2.below(uint32_t(p.size())), ic = r2.below(uint32_t(p.size())), id = r2.below(uint32_t(p.size()));
                    if (ia == ib || ia == ic || ia == id || ib == ic || ib == id || ic == id) continue;  // 四个不同的点
                    const Vec2 &a = p[ia], &b = p[ib], &c = p[ic], &d = p[id];
                    err += inCircle(a, b, c, d, pr) != inCircle(a, b, c, d, Precision::Exact), ++tot;
                }
                Triangulation tr = triangulate(p, Order::Random, pr, 3);
                const Stats& s = tr.stats();
                bool valid = !s.broken && tr.valid();
                long long viol = valid ? violations(p, tr.triangles()) : -1;
                const char* nm = pr == Precision::Float ? "float" : pr == Precision::Double ? "double" : "exact";
                std::printf("  2^%-6d %-9s  %6zu  %8lld  %6s  %9d  %5s  %23lld  %18.3f%%\n", bits, nm, p.size(), s.located, s.broken ? "yes" : "no", s.fallbacks,
                            valid ? "yes" : "no", viol, 100.0 * err / tot);
            }
        }
        }
        std::printf("== precision: a, b, c random integers in [0, 2^k), d = integer point nearest to their circumcircle; 10^6 quadruples ==\n");
        std::printf("  k    exactly cocircular  float wrong  double wrong\n");
        for (int k : {8, 12, 16, 20, 23}) {
            Pcg32 rng{17};
            long long tot = 0, zero = 0, ef = 0, ed = 0;
            uint32_t range = 1u << k;
            while (tot < 1000000) {
                Vec2 a{float(rng.below(range)), float(rng.below(range))}, b{float(rng.below(range)), float(rng.below(range))};
                Vec2 c{float(rng.below(range)), float(rng.below(range))};
                double bx = double(b.x) - a.x, by = double(b.y) - a.y, cx = double(c.x) - a.x, cy = double(c.y) - a.y, D = 2 * (bx * cy - by * cx);
                if (std::abs(D) < 1) continue;
                double ux = (cy * (bx * bx + by * by) - by * (cx * cx + cy * cy)) / D, uy = (bx * (cx * cx + cy * cy) - cx * (bx * bx + by * by)) / D;
                double R = std::hypot(ux, uy), th = rng.nextDouble() * 2 * M_PI;
                double dx = std::round(a.x + ux + R * std::cos(th)), dy = std::round(a.y + uy + R * std::sin(th));
                if (dx < 0 || dy < 0 || dx >= range || dy >= range) continue;
                Vec2 d{float(dx), float(dy)};
                int ex = inCircle(a, b, c, d, Precision::Exact);
                ++tot, zero += ex == 0, ef += inCircle(a, b, c, d, Precision::Float) != ex, ed += inCircle(a, b, c, d, Precision::Double) != ex;
            }
            std::printf("  %-4d %17.3f%%  %10.3f%%  %11.4f%%\n", k, 100.0 * zero / tot, 100.0 * ef / tot, 100.0 * ed / tot);
        }
        std::printf("== precision: uniform random float points in [0,1000)^2, n = 20000 ==\n");
        for (Precision pr : {Precision::Float, Precision::Double}) {
            std::vector<Vec2> p = uniform(20000, 1000, 1000, 11);
            Triangulation tr = triangulate(p, Order::Hilbert, pr);
            std::printf("  %-6s broken %s after %lld points  fallbacks %d  valid %s  triangles %zu\n", pr == Precision::Float ? "float" : "double",
                        tr.stats().broken ? "yes" : "no", tr.stats().located, tr.stats().fallbacks, tr.valid() ? "yes" : "no", tr.triangles().size());
        }
    }

    if (want(argc, argv, "cells")) {
        std::printf("== cells: interior Voronoi cells, 60x60, ~3000 points, 5 seeds ==\n");
        std::printf("  points                 5-gon   6-gon   7-gon   other   mean   area CV   min neighbour dist / spacing\n");
        auto row = [](const char* nm, CellStats s) {
            std::printf("  %-21s %6.1f%%  %5.1f%%  %5.1f%%  %5.1f%%  %5.3f  %7.3f  %8.3f\n", nm, 100 * s.frac5, 100 * s.frac6, 100 * s.frac7, 100 * s.other, s.mean, s.areaCv,
                        s.minNN);
        };
        const float W = 60, H = 60;
        for (int m = 0; m < 7; ++m) {
            CellStats acc;
            for (int s = 0; s < 5; ++s) {
                Pcg32 rng{uint64_t(s + 1)};
                poisson::BridsonParams prm;
                std::vector<Vec2> ref = poisson::bridson(W, H, 1, rng, prm);
                std::vector<Vec2> p;
                if (m == 0) p = poisson::whiteNoise(W, H, int(ref.size()), rng);
                else if (m == 1) p = poisson::jitteredGrid(W, H, std::sqrt(W * H / ref.size()), rng);
                else if (m == 2) p = ref;
                else {
                    p = poisson::whiteNoise(W, H, int(ref.size()), rng);
                    int its = m == 3 ? 1 : m == 4 ? 5 : m == 5 ? 30 : 300;
                    for (int it = 0; it < its; ++it) lloyd(p, W, H);
                }
                CellStats c = cellStats(p, W, H);
                acc.frac5 += c.frac5 / 5, acc.frac6 += c.frac6 / 5, acc.frac7 += c.frac7 / 5, acc.other += c.other / 5, acc.mean += c.mean / 5;
                acc.areaCv += c.areaCv / 5, acc.minNN += c.minNN / 5;
            }
            const char* names[] = {"white noise", "jittered grid", "bridson k30", "white + lloyd 1", "white + lloyd 5", "white + lloyd 30", "white + lloyd 300"};
            row(names[m], acc);
        }
    }

    if (want(argc, argv, "lloyd")) {
        std::printf("== lloyd: 2000 points, 50x40, energy / hexagonal bound, mean move / spacing, 5 seeds ==\n");
        const double W = 50, H = 40;
        const int n = 2000;
        double hex = 5 * std::sqrt(3.0) / 54 * (W * H / n) * W * H, spacing = std::sqrt(W * H / n);
        for (int start = 0; start < 2; ++start) {
            std::printf("  start: %s\n", start ? "bridson" : "white noise");
            std::vector<int> report = {0, 1, 2, 5, 10, 20, 50, 100, 200, 500, 1000};
            std::vector<double> e(1001, 0), mv(1001, 0);
            for (int s = 0; s < 5; ++s) {
                Pcg32 rng{uint64_t(s + 1)};
                std::vector<Vec2> p;
                if (start) {
                    poisson::BridsonParams prm;
                    p = poisson::bridson(float(W), float(H), 1.0f * float(spacing), rng, prm);
                    p.resize(std::min<size_t>(p.size(), n));
                } else p = uniform(n, float(W), float(H), uint64_t(s + 1));
                double hx = 5 * std::sqrt(3.0) / 54 * (W * H / p.size()) * W * H;
                for (int it = 0; it <= 1000; ++it) {
                    LloydStep r = lloyd(p, W, H);
                    e[it] += r.energy / hx / 5, mv[it] += r.meanMove / spacing / 5;
                }
            }
            (void)hex;
            for (int it : report) std::printf("    iter %4d  energy %.5f  move %.2e\n", it, e[it], mv[it]);
        }
    }

    if (want(argc, argv, "speed")) {
        std::printf("== speed ==\n");
        for (int n : {1000, 4000}) {
            std::vector<Vec2> p = uniform(n, 100, 100, 1);
            auto t0 = Clock::now();
            Triangulation tr = triangulate(p);
            auto cells = voronoiCells(p, tr.neighbors(), 100, 100);
            double fast = msSince(t0);
            t0 = Clock::now();
            double sum = 0;
            for (size_t i = 0; i < p.size(); ++i) {
                Poly c = boxPoly(100, 100);
                for (size_t j = 0; j < p.size(); ++j)
                    if (j != i) c = clipBisector(c, p[i], p[j]);
                sum += area(c);
            }
            double brute = msSince(t0);
            std::printf("  n=%d  voronoi via delaunay %.2f ms, clip by all points %.1f ms (area %.1f)\n", n, fast, brute, sum);
        }
        for (Precision pr : {Precision::Double, Precision::Exact}) {
            Pcg32 rng{1};
            std::vector<Vec2> p(1000000);
            for (Vec2& a : p) a = {float(rng.below(1 << 20)), float(rng.below(1 << 20))};
            auto t0 = Clock::now();
            Triangulation tr = triangulate(p, Order::Hilbert, pr);
            std::printf("  1e6 integer points, %s predicates: %.0f ms\n", pr == Precision::Double ? "double" : "exact", msSince(t0));
        }
        std::vector<Vec2> p = uniform(2000, 50, 40, 1);
        auto t0 = Clock::now();
        for (int it = 0; it < 100; ++it) lloyd(p, 50, 40);
        std::printf("  lloyd, 2000 points: %.2f ms per iteration\n", msSince(t0) / 100);
    }
}
