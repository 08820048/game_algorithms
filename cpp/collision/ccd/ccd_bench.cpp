// 连续碰撞检测：隧穿率与开销（不由 CMake 构建）：
//   c++ -std=c++20 -O2 -Icpp/common cpp/collision/ccd/ccd_bench.cpp -o ccd_bench && ./ccd_bench
#include <chrono>
#include <cmath>
#include <cstdio>
#include <vector>

#include "../../foundations/random/random.h"
#include "../sat/sat.h"
#include "ccd.h"

using namespace ga;
using Clock = std::chrono::steady_clock;

static float rnd(Pcg32& r, float lo, float hi) { return lo + r.nextFloat() * (hi - lo); }
static volatile float sink;

template <class F>
static double bestNs(int runs, int n, F&& f) {
    double best = 1e30;
    for (int r = 0; r < runs; ++r) {
        auto t0 = Clock::now();
        f();
        best = std::min(best, std::chrono::duration<double, std::nano>(Clock::now() - t0).count() / n);
    }
    return best;
}

int main() {
    std::printf("1) tunneling: a bullet (radius 2) flies at a wall 4 px thick; %% of bullets that pass through undetected\n");
    std::printf("  %-10s %10s %10s %10s %10s %10s %10s\n", "px/frame", "theory", "discrete", "2 substeps", "4 substeps", "8 substeps", "swept");
    for (float v : {4.0f, 8.0f, 16.0f, 32.0f, 64.0f, 128.0f}) {
        Pcg32 r(1);
        const int n = 100000;
        int miss[4] = {}, sweptMiss = 0;
        Aabb wall{{100, -1000}, {104, 1000}};
        for (int i = 0; i < n; ++i) {
            // 子弹从墙前随机位置出发，飞行若干帧，直到越过墙
            float x0 = rnd(r, 100 - 2 - v, 100 - 2);  // 第一帧的起点恰好在墙前一帧之内
            int k = 0;
            for (int sub : {1, 2, 4, 8}) {
                bool hit = false;
                for (float x = x0; x < 120 + v && !hit; x += v / sub)
                    hit = overlaps(Circle{{x, 0}, 2}, wall);
                miss[k++] += !hit;
            }
            sweptMiss += !sweep(Circle{{x0, 0}, 2}, {v, 0}, {100, -1000}, {100, 1000}) &&
                         !sweep(Circle{{x0 + v, 0}, 2}, {v, 0}, {100, -1000}, {100, 1000});
        }
        float theory = std::max(0.0f, 1 - (4 + 2 * 2) / v);  // 墙厚 + 子弹直径 小于一帧的位移时才会漏
        std::printf("  %-10.0f %9.1f%% %9.1f%% %9.1f%% %9.1f%% %9.1f%% %9.1f%%\n", v, theory * 100, miss[0] * 100.0 / n, miss[1] * 100.0 / n,
                    miss[2] * 100.0 / n, miss[3] * 100.0 / n, sweptMiss * 100.0 / n);
    }

    std::printf("\n2) cost per test, ns\n");
    {
        const int n = 200000;
        Pcg32 r(2);
        std::vector<Circle> cs;
        std::vector<Vec2> ds, as, bs;
        std::vector<Aabb> ba, bb;
        for (int i = 0; i < n; ++i) {
            cs.push_back({{rnd(r, 0, 100), rnd(r, 0, 100)}, rnd(r, 1, 10)});
            ds.push_back({rnd(r, -100, 100), rnd(r, -100, 100)});
            as.push_back({rnd(r, 0, 100), rnd(r, 0, 100)});
            bs.push_back({rnd(r, 0, 100), rnd(r, 0, 100)});
            Vec2 p{rnd(r, 0, 100), rnd(r, 0, 100)}, q{rnd(r, 0, 100), rnd(r, 0, 100)};
            ba.push_back({p, p + Vec2{rnd(r, 1, 10), rnd(r, 1, 10)}});
            bb.push_back({q, q + Vec2{rnd(r, 1, 10), rnd(r, 1, 10)}});
        }
        auto run = [&](const char* name, auto&& f) {
            double ns = bestNs(9, n, [&] {
                float s = 0;
                for (int i = 0; i < n; ++i) s += f(i);
                sink = s;
            });
            std::printf("  %-34s %6.1f ns\n", name, ns);
        };
        run("discrete AABB overlap", [&](int i) { return float(overlaps(ba[i], bb[i])); });
        run("swept AABB (moving vs static)", [&](int i) { auto h = sweep(ba[i], ds[i], bb[i]); return h ? h->t : 2.0f; });
        run("discrete circle vs segment", [&](int i) {
            Vec2 ab = bs[i] - as[i], p = cs[i].center;
            float u = std::clamp(dot(p - as[i], ab) / std::max(dot(ab, ab), 1e-9f), 0.0f, 1.0f);
            return float(lengthSq(p - as[i] - ab * u) < cs[i].radius * cs[i].radius);
        });
        run("swept circle vs segment", [&](int i) { auto h = sweep(cs[i], ds[i], as[i], bs[i]); return h ? h->t : 2.0f; });
        run("swept circle vs moving circle", [&](int i) {
            auto h = sweep(cs[i], ds[i], Circle{as[i], 5}, ds[(i + 1) % n]);
            return h ? h->t : 2.0f;
        });
    }

    std::printf("\n3) conservative advancement: two boxes (12 x 4 and 8 x 8) that collide, by rotation per step\n");
    std::printf("  %-14s %12s %12s %12s\n", "rotation", "iterations", "ns per call", "GJK share");
    for (float spin : {0.0f, 0.5f, 2.0f, 6.0f, 20.0f}) {
        Pcg32 r(3);
        ConvexShape a{makeBox({0, 0}, {6, 2}).v, 0}, b{makeBox({0, 0}, {4, 4}).v, 0};
        std::vector<Motion> ma, mb;
        while (ma.size() < 20000) {
            Motion x{{0, rnd(r, -6, 6)}, rnd(r, 0, 6), {rnd(r, 80, 120), rnd(r, -10, 10)}, spin * (r.nextFloat() < 0.5f ? -1 : 1)};
            Motion y{{60, 0}, rnd(r, 0, 6), {0, 0}, 0};
            // 只保留真的会在这一步内相撞的
            bool hit = false;
            for (int k = 0; k <= 400 && !hit; ++k) hit = query(poseAt(a, x, k / 400.0f), poseAt(b, y, k / 400.0f)).distance < 0;
            if (hit && query(poseAt(a, x, 0), poseAt(b, y, 0)).distance > 0) ma.push_back(x), mb.push_back(y);
        }
        long iters = 0;
        for (size_t i = 0; i < ma.size(); ++i)
            if (auto t = conservativeAdvancement(a, ma[i], b, mb[i], 0.05f)) iters += t->iterations;
        double ns = bestNs(5, int(ma.size()), [&] {
            float s = 0;
            for (size_t i = 0; i < ma.size(); ++i)
                if (auto t = conservativeAdvancement(a, ma[i], b, mb[i], 0.05f)) s += t->t;
            sink = s;
        });
        double gjkNs = bestNs(5, int(ma.size()), [&] {
            float s = 0;
            for (size_t i = 0; i < ma.size(); ++i) s += query(poseAt(a, ma[i], 0.3f), poseAt(b, mb[i], 0.3f)).distance;
            sink = s;
        });
        double avg = double(iters) / ma.size();
        std::printf("  %5.1f rad/step %12.2f %10.0fns %11.0f%%\n", spin, avg, ns, 100 * gjkNs * avg / ns);
    }

    std::printf("\n4) conservative advancement: tolerance vs iterations (no rotation / 2 rad per step)\n");
    for (float tol : {1.0f, 0.1f, 0.01f, 0.001f}) {
        Pcg32 r(4);
        ConvexShape a{makeBox({0, 0}, {6, 2}).v, 0}, b{makeBox({0, 0}, {4, 4}).v, 0};
        double avg[2];
        for (int s = 0; s < 2; ++s) {
            long iters = 0, count = 0;
            for (int i = 0; i < 5000; ++i) {
                Motion x{{0, rnd(r, -3, 3)}, rnd(r, 0, 6), {100, 0}, s ? 2.0f : 0.0f};
                Motion y{{60, 0}, rnd(r, 0, 6), {0, 0}, 0};
                if (auto t = conservativeAdvancement(a, x, b, y, tol)) iters += t->iterations, ++count;
            }
            avg[s] = double(iters) / std::max(1L, count);
        }
        std::printf("  tolerance %-6g iterations %.2f / %.2f\n", tol, avg[0], avg[1]);
    }
}
