// GJK / EPA 与 SAT 的性能对比（不由 CMake 构建）：
//   c++ -std=c++20 -O2 -Icpp/common cpp/collision/gjk/gjk_bench.cpp -o gjk_bench && ./gjk_bench
#include <chrono>
#include <cstdio>

#include "../../foundations/random/random.h"
#include "../sat/sat.h"
#include "gjk.h"

using namespace ga;
using Clock = std::chrono::steady_clock;

static float rnd(Pcg32& r, float lo, float hi) { return lo + r.nextFloat() * (hi - lo); }

template <class F>
static double bestOf(int runs, F&& f) {
    double best = 1e30;
    for (int r = 0; r < runs; ++r) {
        auto t0 = Clock::now();
        f();
        best = std::min(best, std::chrono::duration<double, std::nano>(Clock::now() - t0).count());
    }
    return best;
}

static volatile float sink;

int main() {
    Pcg32 rng(9);
    const int kPairs = 20000;
    std::printf("overlap test, ns per pair (regular n-gons)\n");
    std::printf("%-6s %-10s %8s %8s %10s | %8s %8s %8s\n", "verts", "pairs", "SAT", "GJK", "GJK iters", "SAT MTV", "GJK+EPA", "EPA iters");
    for (int n : {4, 8, 16, 32, 64}) {
        for (int kind = 0; kind < 2; ++kind) {  // 0：约 30% 相交 1：全部相交
            std::vector<ConvexPolygon> pa, pb;
            std::vector<ConvexShape> sa, sb;
            while (static_cast<int>(pa.size()) < kPairs) {
                ConvexPolygon a = regularPolygon({0, 0}, 10, n, rnd(rng, 0, 6.3f));
                float d = kind == 0 ? rnd(rng, 0, 60) : rnd(rng, 0, 15);
                ConvexPolygon b = regularPolygon(fromAngle(rnd(rng, 0, 6.3f)) * d, 10, n, rnd(rng, 0, 6.3f));
                if (kind == 1 && !overlaps(a, b)) continue;
                pa.push_back(a), pb.push_back(b), sa.push_back({a.v, 0}), sb.push_back({b.v, 0});
            }
            long gjkIters = 0, epaIters = 0, hits = 0;
            for (int i = 0; i < kPairs; ++i) {
                Gjk g(sa[i], sb[i]);
                bool hit = g.run() == Gjk::Status::Intersecting;
                gjkIters += g.iterations();
                if (hit) {
                    ++hits;
                    Epa e(sa[i], sb[i], g.simplex());
                    e.run();
                    epaIters += e.iterations();
                }
            }
            double tSat = bestOf(11, [&] { int s = 0; for (int i = 0; i < kPairs; ++i) s += overlaps(pa[i], pb[i]); sink = s; });
            double tGjk = bestOf(11, [&] { int s = 0; for (int i = 0; i < kPairs; ++i) s += overlaps(sa[i], sb[i]); sink = s; });
            double tSatC = bestOf(11, [&] {
                float s = 0;
                for (int i = 0; i < kPairs; ++i)
                    if (auto c = contact(pa[i], pb[i])) s += c->depth;
                sink = s;
            });
            double tEpa = bestOf(11, [&] { float s = 0; for (int i = 0; i < kPairs; ++i) s += query(sa[i], sb[i]).distance; sink = s; });
            std::printf("%-6d %3ld%% hit   %6.0fns %6.0fns %10.2f | %6.0fns %6.0fns %8.2f\n", n, hits * 100 / kPairs, tSat / kPairs,
                        tGjk / kPairs, double(gjkIters) / kPairs, tSatC / kPairs, tEpa / kPairs, hits ? double(epaIters) / hits : 0.0);
        }
    }

    // 曲线形状：GJK 只能逐步逼近圆弧，迭代次数取决于容差
    std::printf("\ncurved shapes: circle vs box, 50%% overlapping\n");
    for (float tol : {1e-1f, 1e-2f, 1e-3f, 1e-4f}) {
        long gi = 0, ei = 0, hits = 0;
        double errSep = 0, errPen = 0;
        int nSep = 0;
        Pcg32 r2(3);
        for (int i = 0; i < kPairs; ++i) {
            ConvexShape box{makeBox({0, 0}, {20, 12}, rnd(r2, 0, 6.3f)).v, 0};
            Vec2 c = fromAngle(rnd(r2, 0, 6.3f)) * rnd(r2, 0, 50);
            ConvexShape circle{{c}, 10};
            Gjk g(box, circle, tol);
            bool hit = g.run() == Gjk::Status::Intersecting;
            gi += g.iterations();
            // 精确答案：圆心到盒子的距离 - 半径
            ConvexPolygon bp = makePolygon(box.points);
            float best = 1e9f;
            for (int k = 0; k < 4; ++k) {
                Vec2 p = bp.vertex(k), q = bp.vertex(k + 1), e = q - p;
                float t = std::clamp(dot(c - p, e) / dot(e, e), 0.0f, 1.0f);
                best = std::min(best, distance(c, p + e * t));
            }
            bool inside = contains(bp, c);
            float exact = inside ? -(best + 10) : best - 10;
            if (hit) {
                ++hits;
                Epa e(box, circle, g.simplex(), tol);
                e.run();
                ei += e.iterations();
                errPen = std::max(errPen, double(std::abs(-e.depth() - exact)));
            } else {
                errSep = std::max(errSep, double(std::abs(g.distance() - exact)));
                ++nSep;
            }
        }
        std::printf("tol %-7g GJK iters %.2f  EPA iters %.2f  max error: distance %.4f, depth %.4f\n", tol, double(gi) / kPairs,
                    double(ei) / hits, errSep, errPen);
    }
}
