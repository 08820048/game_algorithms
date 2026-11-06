// 四叉树 / 八叉树与均匀网格、两两测试的对比（不由 CMake 构建）：
//   c++ -std=c++20 -O2 -Icpp/common cpp/collision/quadtree/quadtree_bench.cpp -o qt_bench && ./qt_bench
#include <chrono>
#include <cmath>
#include <cstdio>

#include "../../foundations/random/random.h"
#include "../spatial_hash/spatial_hash.h"
#include "quadtree.h"

using namespace ga;
using Clock = std::chrono::steady_clock;

static volatile long sink;

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

constexpr int kN = 16000;
static const float kSide = std::sqrt(kN * 400.0f);  // 与 2.4 相同的密度

enum class Layout { Uniform, Big, Clustered };

static std::vector<Aabb> scene(Layout layout, uint64_t seed) {
    Pcg32 r(seed);
    std::vector<Aabb> boxes;
    Vec2 centers[8];
    for (Vec2& c : centers) c = {rnd(r, 0.1f * kSide, 0.9f * kSide), rnd(r, 0.1f * kSide, 0.9f * kSide)};
    for (int i = 0; i < kN; ++i) {
        Vec2 p{rnd(r, 0, kSide), rnd(r, 0, kSide)};
        float s = rnd(r, 4, 12);
        if (layout == Layout::Big && r.nextFloat() < 0.01f) s = 200;
        if (layout == Layout::Clustered) {
            // 8 个团，每个团里的物体按近似正态分布聚在中心附近（三个均匀数相加）
            Vec2 c = centers[i % 8];
            float sx = (r.nextFloat() + r.nextFloat() + r.nextFloat() - 1.5f) * 0.06f * kSide;
            float sy = (r.nextFloat() + r.nextFloat() + r.nextFloat() - 1.5f) * 0.06f * kSide;
            p = c + Vec2{sx, sy};
        }
        boxes.push_back({p, p + Vec2{s, s}});
    }
    return boxes;
}

struct Result {
    double ms;
    long tests;
};

int main() {
    const int runs = 9;
    std::vector<Pair> gp;
    std::vector<TreePair> tp;
    const char* names[] = {"uniform 4..12", "1% are 200 wide", "8 clusters"};

    std::printf("1) n = %d, ms per frame (build + find pairs), tests in parentheses\n", kN);
    std::printf("%-17s %8s %10s %20s %20s %20s\n", "layout", "pairs", "brute", "grid (cell 12)", "strict quadtree", "loose quadtree");
    for (Layout layout : {Layout::Uniform, Layout::Big, Layout::Clustered}) {
        auto boxes = scene(layout, 7);
        std::vector<BoxN<2>> nb;
        for (auto& b : boxes) nb.push_back(toBox(b));
        Aabb world{{0, 0}, {kSide + 12, kSide + 12}};
        double tBrute = bestMs(3, [&] { bruteForcePairs(boxes, gp); });
        size_t pairs = gp.size();
        UniformGrid grid(GridIndexer(world, 12));
        long gt = 0;
        double tg = bestMs(runs, [&] { grid.build(boxes); gt = grid.findPairs(gp); });
        Result r[2];
        for (int l = 0; l < 2; ++l) {
            Quadtree tree(toBox(world), {10, 8, l == 0 ? 1.0f : 2.0f});
            long t = 0;
            double ms = bestMs(runs, [&] { tree.build(nb); t = tree.findPairs(tp); });
            r[l] = {ms, t};
            if (tp.size() != pairs) std::printf("MISMATCH\n");
        }
        std::printf("%-17s %8zu %8.1fms %8.2fms (%7ld) %8.2fms (%7ld) %8.2fms (%7ld)\n", names[int(layout)], pairs, tBrute, tg, gt,
                    r[0].ms, r[0].tests, r[1].ms, r[1].tests);
    }

    std::printf("\n2) where objects end up (uniform layout): depth histogram\n");
    {
        auto boxes = scene(Layout::Uniform, 7);
        std::vector<BoxN<2>> nb;
        for (auto& b : boxes) nb.push_back(toBox(b));
        for (float loose : {1.0f, 2.0f}) {
            Quadtree tree({{0, 0}, {kSide + 12, kSide + 12}}, {10, 8, loose});
            tree.build(nb);
            int hist[11] = {};
            int leafObjects = 0;
            for (int i = 0; i < kN; ++i) {
                ++hist[tree.depthOf(i)];
                leafObjects += tree.nodes()[tree.nodeOf(i)].firstChild < 0;
            }
            std::printf("%s: nodes %zu, objects in leaves %.0f%%, depth:", loose == 1 ? "strict" : "loose ", tree.nodes().size(),
                        100.0 * leafObjects / kN);
            for (int d = 0; d <= 10; ++d)
                if (hist[d]) std::printf(" %d:%d", d, hist[d]);
            std::printf("\n");
        }
    }

    std::printf("\n3) build vs find pairs (uniform layout)\n");
    {
        auto boxes = scene(Layout::Uniform, 7);
        std::vector<BoxN<2>> nb;
        for (auto& b : boxes) nb.push_back(toBox(b));
        for (float loose : {1.0f, 2.0f}) {
            Quadtree tree({{0, 0}, {kSide + 12, kSide + 12}}, {10, 8, loose});
            double tb = bestMs(runs, [&] { tree.build(nb); });
            double tf = bestMs(runs, [&] { tree.findPairs(tp); });
            double tq = bestMs(runs, [&] { tree.findPairsByQuery(tp); });
            std::printf("%s: build %.2fms, find pairs %.2fms (one query per object: %.2fms)\n", loose == 1 ? "strict" : "loose ", tb, tf, tq);
        }
    }

    std::printf("\n4) split count (uniform layout / clustered), ms\n");
    std::printf("%-6s %10s %10s %10s %10s\n", "split", "strict U", "loose U", "strict C", "loose C");
    {
        std::vector<BoxN<2>> nb[2];
        for (int k = 0; k < 2; ++k)
            for (auto& b : scene(k == 0 ? Layout::Uniform : Layout::Clustered, 7)) nb[k].push_back(toBox(b));
        for (int split : {1, 2, 4, 8, 16, 32, 64}) {
            std::printf("%-6d", split);
            for (int k = 0; k < 2; ++k)
                for (float loose : {1.0f, 2.0f}) {
                    Quadtree tree({{0, 0}, {kSide + 12, kSide + 12}}, {10, split, loose});
                    std::printf(" %8.2fms", bestMs(runs, [&] { tree.build(nb[k]); tree.findPairs(tp); }));
                }
            std::printf("\n");
        }
    }

    std::printf("\n5) clustered layout: grid cell size vs quadtree\n");
    {
        auto boxes = scene(Layout::Clustered, 7);
        Aabb world{{0, 0}, {kSide + 12, kSide + 12}};
        for (float cell : {6.0f, 12.0f, 24.0f}) {
            UniformGrid grid(GridIndexer(world, cell));
            long t = 0;
            double ms = bestMs(runs, [&] { grid.build(boxes); t = grid.findPairs(gp); });
            std::printf("grid cell %-4.0f %8.2fms  tests %8ld\n", cell, ms, t);
        }
    }

    std::printf("\n6) octree, 3D: boxes 4..12 wide, same density per volume\n");
    std::printf("%-7s %8s %10s %20s %20s\n", "n", "pairs", "brute", "strict octree", "loose octree");
    for (int n : {2000, 8000, 32000}) {
        float side = std::cbrt(n * 4000.0f);
        Pcg32 r(n);
        std::vector<BoxN<3>> boxes;
        for (int i = 0; i < n; ++i) {
            BoxN<3> b;
            float s = rnd(r, 4, 12);
            for (int k = 0; k < 3; ++k) b.min[k] = rnd(r, 0, side), b.max[k] = b.min[k] + s;
            boxes.push_back(b);
        }
        double tb = n <= 8000 ? bestMs(3, [&] { bruteForcePairs(boxes, tp); }) : -1;
        BoxN<3> world;
        world.min.fill(0), world.max.fill(side + 12);
        Result res[2];
        for (int l = 0; l < 2; ++l) {
            Octree tree(world, {10, 8, l == 0 ? 1.0f : 2.0f});
            long t = 0;
            double ms = bestMs(runs, [&] { tree.build(boxes); t = tree.findPairs(tp); });
            res[l] = {ms, t};
        }
        char brute[16];
        if (tb >= 0) std::snprintf(brute, sizeof brute, "%.1fms", tb);
        else std::snprintf(brute, sizeof brute, "-");
        std::printf("%-7d %8zu %10s %8.2fms (%7ld) %8.2fms (%7ld)\n", n, tp.size(), brute, res[0].ms, res[0].tests, res[1].ms, res[1].tests);
    }

    std::printf("\n7) sizes spread over two orders of magnitude: side = 2 * 2^(8u), u uniform (2 .. 512)\n");
    {
        Pcg32 r(3);
        std::vector<Aabb> boxes;
        std::vector<BoxN<2>> nb;
        float side = kSide * 2;
        for (int i = 0; i < kN; ++i) {
            float s = 2 * std::exp2(8 * r.nextFloat());
            Vec2 p{rnd(r, 0, side), rnd(r, 0, side)};
            boxes.push_back({p, p + Vec2{s, s}});
            nb.push_back(toBox(boxes.back()));
        }
        Aabb world{{0, 0}, {side + 512, side + 512}};
        double tb = bestMs(3, [&] { bruteForcePairs(boxes, gp); });
        std::printf("brute force %.1fms, %zu pairs\n", tb, gp.size());
        for (float cell : {8.0f, 32.0f, 128.0f, 512.0f}) {
            UniformGrid grid(GridIndexer(world, cell));
            long t = 0;
            double ms = bestMs(runs, [&] { grid.build(boxes); t = grid.findPairs(gp); });
            std::printf("grid cell %-4.0f   %8.2fms  tests %9ld  cells per box %.1f\n", cell, ms, t, double(grid.entryCount()) / kN);
        }
        for (float loose : {1.0f, 2.0f}) {
            Quadtree tree(toBox(world), {10, 8, loose});
            long t = 0;
            double ms = bestMs(runs, [&] { tree.build(nb); t = tree.findPairs(tp); });
            std::printf("%s quadtree  %8.2fms  tests %9ld\n", loose == 1 ? "strict" : "loose ", ms, t);
        }
    }

    std::printf("\n8) region query size (uniform layout), us per query\n");
    std::printf("%-8s %10s %10s %10s\n", "region", "grid", "strict", "loose");
    {
        auto boxes = scene(Layout::Uniform, 7);
        std::vector<BoxN<2>> nb;
        for (auto& b : boxes) nb.push_back(toBox(b));
        Aabb world{{0, 0}, {kSide + 12, kSide + 12}};
        UniformGrid grid(GridIndexer(world, 12));
        grid.build(boxes);
        Quadtree strict(toBox(world), {10, 8, 1}), loose(toBox(world), {10, 8, 2});
        strict.build(nb);
        loose.build(nb);
        for (float q : {10.0f, 40.0f, 160.0f, 640.0f}) {
            Pcg32 r(1);
            std::vector<Aabb> regions;
            int count = q < 600 ? 10000 : 1000;
            for (int i = 0; i < count; ++i) {
                Vec2 p{rnd(r, 0, kSide - q), rnd(r, 0, kSide - q)};
                regions.push_back({p, p + Vec2{q, q}});
            }
            long hits = 0;
            double tg = bestMs(runs, [&] { for (auto& g : regions) grid.query(g, [&](int) { ++hits; }); });
            double ts = bestMs(runs, [&] { for (auto& g : regions) strict.query(toBox(g), [&](int) { ++hits; }); });
            double tl = bestMs(runs, [&] { for (auto& g : regions) loose.query(toBox(g), [&](int) { ++hits; }); });
            sink = hits;
            long st = 0, lt = 0;  // 每次查询做了多少次盒子测试
            for (auto& g : regions) st += strict.query(toBox(g), [](int) {}), lt += loose.query(toBox(g), [](int) {});
            std::printf("%-8.0f %8.2fus %8.2fus %8.2fus   box tests per query: strict %.0f, loose %.0f\n", q, tg * 1000 / count,
                        ts * 1000 / count, tl * 1000 / count, double(st) / count, double(lt) / count);
        }
    }
}
