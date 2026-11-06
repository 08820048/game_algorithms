// 宽相检测的性能测试（不由 CMake 构建）：
//   c++ -std=c++20 -O2 -Icpp/common cpp/collision/spatial_hash/spatial_hash_bench.cpp -o sh_bench && ./sh_bench
#include <chrono>
#include <cstdio>

#include "../../foundations/random/random.h"
#include "spatial_hash.h"

using namespace ga;
using Clock = std::chrono::steady_clock;

static float rnd(Pcg32& r, float lo, float hi) { return lo + r.nextFloat() * (hi - lo); }

template <class F>
static double bestMs(int runs, F&& f) {
    double best = 1e30;
    for (int r = 0; r < runs; ++r) {
        auto t0 = Clock::now();
        f();
        best = std::min(best, std::chrono::duration<double, std::milli>(Clock::now() - t0).count());
    }
    return best;
}

// n 个边长 [lo, hi] 的盒子，均匀撒在边长 side 的正方形里
static std::vector<Aabb> scene(int n, float side, float lo, float hi, uint64_t seed, float bigFraction = 0, float bigSize = 0) {
    Pcg32 r(seed);
    std::vector<Aabb> boxes;
    for (int i = 0; i < n; ++i) {
        Vec2 p{rnd(r, 0, side), rnd(r, 0, side)};
        float s = r.nextFloat() < bigFraction ? bigSize : rnd(r, lo, hi);
        boxes.push_back({p, p + Vec2{s, s}});
    }
    return boxes;
}

// 平均每个物体占 kArea 的面积：物体数变化时密度不变
constexpr float kArea = 400;
static float sideFor(int n) { return std::sqrt(n * kArea); }

int main() {
    std::vector<Pair> pairs;
    const int runs = 9;

    std::printf("1) scaling, boxes 4..12 wide, cell 12, ms per frame (build + find pairs)\n");
    std::printf("%-7s %8s %10s %10s %10s %10s %12s\n", "n", "pairs", "brute", "naive", "grid", "hash", "grid tests");
    for (int n : {1000, 4000, 16000, 64000}) {
        float side = sideFor(n);
        auto boxes = scene(n, side, 4, 12, n);
        double tBrute = n <= 16000 ? bestMs(n <= 4000 ? runs : 3, [&] { bruteForcePairs(boxes, pairs); }) : -1;
        NaiveGrid naive({{0, 0}, {side + 12, side + 12}}, 12);
        double tNaive = bestMs(runs, [&] { naive.findPairs(boxes, pairs); });
        UniformGrid grid(GridIndexer({{0, 0}, {side + 12, side + 12}}, 12));
        long tests = 0;
        double tGrid = bestMs(runs, [&] { grid.build(boxes); tests = grid.findPairs(pairs); });
        SpatialHash hash(HashIndexer(12, 2 * n));
        double tHash = bestMs(runs, [&] { hash.build(boxes); hash.findPairs(pairs); });
        if (tBrute >= 0) std::printf("%-7d %8zu %8.2fms %8.2fms %8.2fms %8.2fms %12ld\n", n, pairs.size(), tBrute, tNaive, tGrid, tHash, tests);
        else std::printf("%-7d %8zu %10s %8.2fms %8.2fms %8.2fms %12ld\n", n, pairs.size(), "-", tNaive, tGrid, tHash, tests);
    }

    std::printf("\n1b) where the time goes (n = 16000): build vs find pairs\n");
    {
        int n = 16000;
        float side = sideFor(n);
        auto boxes = scene(n, side, 4, 12, n);
        UniformGrid grid(GridIndexer({{0, 0}, {side + 12, side + 12}}, 12));
        double tBuild = bestMs(runs, [&] { grid.build(boxes); });
        double tFind = bestMs(runs, [&] { grid.findPairs(pairs); });
        std::printf("grid build %.2fms, find %.2fms\n", tBuild, tFind);
    }

    std::printf("\n2) cell size (n = 16000, boxes 4..12 wide)\n");
    std::printf("%-6s %10s %12s %14s %10s\n", "cell", "ms", "tests", "cells per box", "cells");
    {
        int n = 16000;
        float side = sideFor(n);
        auto boxes = scene(n, side, 4, 12, n);
        for (float cell : {3.0f, 6.0f, 12.0f, 24.0f, 48.0f, 96.0f}) {
            UniformGrid grid(GridIndexer({{0, 0}, {side + 12, side + 12}}, cell));
            long tests = 0;
            double t = bestMs(runs, [&] { grid.build(boxes); tests = grid.findPairs(pairs); });
            std::printf("%-6.0f %8.2fms %12ld %14.2f %10d\n", cell, t, tests, double(grid.entryCount()) / n, grid.indexer().slots());
        }
    }

    std::printf("\n3) a few big objects (n = 16000, 1%% of boxes 200 wide, the rest 4..12)\n");
    std::printf("%-6s %10s %12s %14s\n", "cell", "ms", "tests", "cells per box");
    {
        int n = 16000;
        float side = sideFor(n);
        auto boxes = scene(n, side, 4, 12, n, 0.01f, 200);
        double tBrute = bestMs(3, [&] { bruteForcePairs(boxes, pairs); });
        std::printf("brute force %.2fms, %zu pairs\n", tBrute, pairs.size());
        for (float cell : {12.0f, 24.0f, 48.0f, 96.0f, 200.0f}) {
            UniformGrid grid(GridIndexer({{0, 0}, {side + 200, side + 200}}, cell));
            long tests = 0;
            double t = bestMs(runs, [&] { grid.build(boxes); tests = grid.findPairs(pairs); });
            std::printf("%-6.0f %8.2fms %12ld %14.2f\n", cell, t, tests, double(grid.entryCount()) / n);
        }
    }

    std::printf("\n4) hash bucket count (n = 16000, cell 12)\n");
    std::printf("%-12s %10s %12s\n", "buckets", "ms", "tests");
    {
        int n = 16000;
        float side = sideFor(n);
        auto boxes = scene(n, side, 4, 12, n);
        UniformGrid grid(GridIndexer({{0, 0}, {side + 12, side + 12}}, 12));
        long gt = 0;
        double tg = bestMs(runs, [&] { grid.build(boxes); gt = grid.findPairs(pairs); });
        std::printf("%-12s %8.2fms %12ld  (%d cells)\n", "grid", tg, gt, grid.indexer().slots());
        for (double f : {0.25, 0.5, 1.0, 2.0, 4.0, 16.0}) {
            int buckets = static_cast<int>(n * f);
            SpatialHash hash(HashIndexer(12, buckets));
            long tests = 0;
            double t = bestMs(runs, [&] { hash.build(boxes); tests = hash.findPairs(pairs); });
            char label[32];
            std::snprintf(label, sizeof label, "%.2gn", f);
            std::printf("%-12s %8.2fms %12ld\n", label, t, tests);
        }
    }

    std::printf("\n5) sparse world: 16000 boxes clustered in one corner of a world 16x larger in area\n");
    {
        int n = 16000;
        float side = sideFor(n);
        auto boxes = scene(n, side, 4, 12, n);
        for (float scale : {1.0f, 4.0f, 16.0f}) {
            float world = side * scale + 12;
            UniformGrid grid(GridIndexer({{0, 0}, {world, world}}, 12));
            double tg = bestMs(runs, [&] { grid.build(boxes); grid.findPairs(pairs); });
            std::printf("world side x%-3.0f grid %8.2fms (%9d cells, %5.1f MB)\n", scale, tg, grid.indexer().slots(),
                        grid.indexer().slots() * 8.0 / 1e6);
        }
        SpatialHash hash(HashIndexer(12, 2 * n));
        double th = bestMs(runs, [&] { hash.build(boxes); hash.findPairs(pairs); });
        std::printf("hash (any world size)  %8.2fms (%9d buckets)\n", th, 2 * n);
    }
}
