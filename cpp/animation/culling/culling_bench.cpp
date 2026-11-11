// 视锥剔除与遮挡剔除的测量
//   c++ -std=c++20 -O2 -Icpp/common cpp/animation/culling/culling_bench.cpp -o /tmp/cull && /tmp/cull [section]
//   section: fp | bvh | occl | hiz | select | forest（默认全部）

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <vector>

#include "culling.h"
#include "scene.h"

using namespace ga;
using namespace ga::culling;

template <class F>
static double bestMs(int rounds, F&& f) {
    double best = 1e30;
    for (int r = 0; r < rounds; ++r) {
        auto t0 = std::chrono::steady_clock::now();
        f();
        best = std::min(best, std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count());
    }
    return best;
}
static volatile long sink;

// 平面测试的假阳性：被平面测试接受、但其实和视锥不相交的盒子，占接受总数的比例
static void falsePositives() {
    std::printf("== plane test false positives (accepted but outside) ==\n");
    std::printf("%-10s %-8s %10s %10s %10s %10s\n", "box size", "fov", "accepted", "plane FP", "circle FP", "exact");
    for (float size : {2.0f, 10.0f, 40.0f, 120.0f})
        for (float fovDeg : {60.0f, 90.0f}) {
            Pcg32 r{7};
            long acc = 0, fp = 0, circ = 0, exact = 0;
            for (int trial = 0; trial < 200; ++trial) {
                View v{{0, 0}, fromAngle(r.nextFloat() * 2 * kPi), fovDeg * kPi / 360, 1, 600};
                Frustum f = makeFrustum(v);
                for (int k = 0; k < 2000; ++k) {
                    float hw = size * (0.5f + r.nextFloat()), hh = size * (0.5f + r.nextFloat());
                    Vec2 p{(r.nextFloat() * 2 - 1) * 800, (r.nextFloat() * 2 - 1) * 800};
                    Aabb b{p - Vec2{hw, hh}, p + Vec2{hw, hh}};
                    bool ex = overlapsExact(f, b);
                    exact += ex;
                    if (classify(f, b) != Cull::Outside) acc++, fp += !ex;
                    Vec2 c = (b.min + b.max) * 0.5f;
                    if (!circleOutside(f, c, length(b.max - c))) circ += !ex;
                }
            }
            std::printf("%-10.0f %-8.0f %10ld %9.2f%% %9.2f%% %10ld\n", size, fovDeg, acc, 100.0 * fp / double(acc),
                        100.0 * circ / double(exact + circ), exact);
        }
}

// 逐个测试 vs 层次剔除（有 / 没有平面掩码）
static void bvh() {
    std::printf("\n== brute force vs BVH (world 4000x4000, far 800, fov 90) ==\n");
    std::printf("%-8s %-14s %12s %10s %10s %10s\n", "N", "method", "plane tests", "nodes", "ms", "visible");
    for (int n : {1000, 10000, 100000, 1000000}) {
        Pcg32 r{11};
        auto boxes = randomBoxes(r, {{0, 0}, {4000, 4000}}, n, 1, 12);
        Bvh t;
        double buildMs = bestMs(3, [&] { t.build(boxes, 4); });
        std::vector<View> views;
        for (int k = 0; k < 32; ++k) views.push_back({{r.nextFloat() * 4000, r.nextFloat() * 4000}, fromAngle(r.nextFloat() * 2 * kPi), kPi / 4, 1, 800});
        for (int m = 0; m < 3; ++m) {
            CullStats st;
            long vis = 0;
            double ms = bestMs(5, [&] {
                st = {}, vis = 0;
                for (const View& v : views) {
                    Frustum f = makeFrustum(v);
                    auto visit = [&](int) { ++vis; };
                    if (m == 0) cullBrute(f, boxes, visit, st);
                    else cullBvh(t, f, boxes, m == 1, visit, st);
                }
            });
            const char* names[3] = {"brute", "bvh + masks", "bvh no masks"};
            std::printf("%-8d %-14s %12.0f %10.0f %10.4f %10.0f\n", n, names[m], st.planeTests / 32.0, st.nodes / 32.0, ms / 32, vis / 32.0);
        }
        std::printf("%-8d build %.2f ms\n", n, buildMs);
    }
}

struct CityCase {
    City city;
    std::vector<View> views;
};
static CityCase bigCity(int props) {
    Pcg32 r{21};
    CityCase c{makeCity(r, {{0, 0}, {6000, 6000}}, 40, 40, 30, props), {}};
    for (int k = 0; k < 24; ++k) {
        Vec2 eye = c.city.streets[r.next() % c.city.streets.size()];
        eye = eye + Vec2{(r.nextFloat() - 0.5f) * 20, (r.nextFloat() - 0.5f) * 20};
        // 沿街道看（轴向 ± 一点）或斜着看
        float a = (k % 2 == 0) ? float(r.next() % 4) * kPi / 2 + (r.nextFloat() - 0.5f) * 0.3f : r.nextFloat() * 2 * kPi;
        c.views.push_back({eye, fromAngle(a), kPi / 4, 2, 3000});
    }
    return c;
}

// 遮挡剔除的效果：分辨率、融合、与理想值（逐条视线）对比
static void occlusion() {
    std::printf("\n== occlusion culling: city 40x40 blocks, 60000 props, far 3000 ==\n");
    CityCase cc = bigCity(60000);
    const City& c = cc.city;
    long inFrustum = 0, ideal = 0;
    std::vector<std::vector<char>> inF(cc.views.size());
    for (size_t k = 0; k < cc.views.size(); ++k) {
        Frustum f = makeFrustum(cc.views[k]);
        inF[k].assign(c.props.size(), 0);
        for (size_t i = 0; i < c.props.size(); ++i)
            if (overlapsExact(f, c.props[i])) {
                inF[k][i] = 1, ++inFrustum;
                ideal += !visibleByRays(cc.views[k], c.props[i], c.occluders, 64);
            }
    }
    std::printf("in frustum %.0f per view, hidden by rays (ideal) %.1f%%\n", inFrustum / double(cc.views.size()), 100.0 * ideal / inFrustum);
    std::printf("%-6s %12s %12s %12s %12s\n", "bins", "culled", "of ideal", "individual", "raster ms");
    for (int bins : {16, 64, 256, 1024, 4096}) {
        long culled = 0, indiv = 0;
        double rast = 0;
        DepthBuffer db(bins), scratch(bins);
        for (size_t k = 0; k < cc.views.size(); ++k) {
            rast += bestMs(3, [&] {
                db.begin(cc.views[k]);
                for (const auto& o : c.occluders) db.rasterize(o);
            });
            std::vector<Aabb> items;
            for (size_t i = 0; i < c.props.size(); ++i)
                if (inF[k][i]) items.push_back(c.props[i]), culled += db.occluded(c.props[i]);
            std::vector<char> h;
            if (bins <= 1024) {
                cullIndividually(scratch, cc.views[k], c.occluders, items, h);
                for (char x : h) indiv += x;
            }
        }
        std::printf("%-6d %11.1f%% %11.1f%% %11.1f%% %12.3f\n", bins, 100.0 * culled / inFrustum, 100.0 * culled / ideal,
                    bins <= 1024 ? 100.0 * indiv / inFrustum : 0.0, rast / cc.views.size());
    }
}

// Hi-Z：逐列 vs 只看一层 vs 细分；不同物体大小
static void hiz() {
    std::printf("\n== Hi-Z queries (bins 1024) ==\n");
    CityCase cc = bigCity(2000);
    std::printf("%-10s %-12s %12s %10s %12s\n", "obj size", "method", "texels/query", "culled", "ns/query");
    for (float size : {4.0f, 40.0f, 200.0f}) {
        Pcg32 r{5};
        auto items = randomBoxes(r, {{0, 0}, {6000, 6000}}, 20000, size * 0.5f, size * 1.5f);
        std::vector<DepthBuffer> dbs;
        std::vector<std::vector<int>> lists;
        for (const View& v : cc.views) {
            dbs.emplace_back(1024);
            dbs.back().begin(v);
            for (const auto& o : cc.city.occluders) dbs.back().rasterize(o);
            dbs.back().buildHiZ();
            Frustum f = makeFrustum(v);
            lists.emplace_back();
            for (int i = 0; i < int(items.size()); ++i)
                if (overlapsExact(f, items[i]) && dbs.back().project(items[i]).valid) lists.back().push_back(i);
        }
        long queries = 0;
        for (auto& l : lists) queries += long(l.size());
        for (int m = 0; m < 3; ++m) {
            long tex = 0, culled = 0;
            double ms = bestMs(5, [&] {
                tex = 0, culled = 0;
                for (size_t k = 0; k < dbs.size(); ++k)
                    for (int i : lists[k])
                        culled += m == 0 ? dbs[k].occluded(items[i], &tex) : dbs[k].occludedHiZ(items[i], m == 2, &tex);
            });
            const char* names[3] = {"per column", "hi-z 1 level", "hi-z refine"};
            std::printf("%-10.0f %-12s %12.2f %9.1f%% %12.1f\n", size, names[m], double(tex) / queries, 100.0 * culled / queries,
                        ms * 1e6 / queries);
        }
    }
}

// 只光栅化最近的 K 个遮挡物（按到视点的距离），剔除率怎么变
static void selectOccluders() {
    std::printf("\n== rasterize only the K nearest occluders (bins 1024) ==\n");
    CityCase cc = bigCity(60000);
    const City& c = cc.city;
    std::printf("%-8s %12s %12s\n", "K", "culled", "raster ms");
    for (int K : {4, 16, 64, 256, 100000}) {
        long culled = 0, inF = 0;
        double rast = 0;
        DepthBuffer db(1024);
        for (const View& v : cc.views) {
            Frustum f = makeFrustum(v);
            std::vector<std::pair<float, int>> order;
            for (int i = 0; i < int(c.buildings.size()); ++i)
                if (classify(f, c.buildings[i]) != Cull::Outside)
                    order.push_back({lengthSq(closestPoint(c.buildings[i], v.eye) - v.eye), i});
            std::sort(order.begin(), order.end());
            rast += bestMs(3, [&] {
                db.begin(v);
                for (int j = 0; j < std::min<int>(K, int(order.size())); ++j) db.rasterize(c.occluders[order[j].second]);
            });
            for (const Aabb& b : c.props)
                if (overlapsExact(f, b)) ++inF, culled += db.occluded(b);
        }
        std::printf("%-8d %11.1f%% %12.3f\n", K, 100.0 * culled / inF, rast / cc.views.size());
    }
}

// 森林：很多小遮挡物（树干），单独一棵挡不住什么，合起来才能挡住远处的东西
static void forest() {
    std::printf("\n== forest: many small occluders (bins 1024, far 1500) ==\n");
    std::printf("%-8s %-8s %10s %12s %12s %12s\n", "trees", "objects", "in frustum", "ideal", "fused", "individual");
    for (float osize : {3.0f, 25.0f})
    for (int trees : {250, 1000, 4000}) {
        Pcg32 r{31};
        Aabb world{{0, 0}, {3000, 3000}};
        std::vector<Aabb> trunks = randomBoxes(r, world, trees, 4, 8);
        std::vector<Aabb> props;
        for (const Aabb& b : randomBoxes(r, world, 6000, osize * 0.7f, osize * 1.3f)) {
            bool clash = false;
            for (const Aabb& t : trunks) clash |= overlaps(t, b);
            if (!clash) props.push_back(b);
        }
        long inF = 0, ideal = 0, fused = 0, indiv = 0;
        DepthBuffer db(1024), scratch(1024);
        for (int k = 0; k < 8; ++k) {
            View v{{1500 + (r.nextFloat() - 0.5f) * 400, 1500 + (r.nextFloat() - 0.5f) * 400}, fromAngle(r.nextFloat() * 2 * kPi), kPi / 4, 2, 1500};
            Frustum f = makeFrustum(v);
            std::vector<std::vector<Vec2>> occ;
            for (const Aabb& t : trunks)
                if (classify(f, t) != Cull::Outside && !contains(t, v.eye)) occ.push_back(corners(t));
            db.begin(v);
            for (const auto& o : occ) db.rasterize(o);
            std::vector<Aabb> items;
            for (const Aabb& b : props)
                if (overlapsExact(f, b)) items.push_back(b);
            std::vector<char> h;
            cullIndividually(scratch, v, occ, items, h);
            for (size_t i = 0; i < items.size(); ++i) {
                ++inF;
                ideal += !visibleByRays(v, items[i], occ, 32);
                fused += db.occluded(items[i]);
                indiv += h[i];
            }
        }
        std::printf("%-8d %-8.0f %10.0f %11.1f%% %11.1f%% %11.1f%%\n", trees, osize, inF / 8.0, 100.0 * ideal / inF, 100.0 * fused / inF, 100.0 * indiv / inF);
    }
}

int main(int argc, char** argv) {
    auto want = [&](const char* s) { return argc < 2 || std::strcmp(argv[1], s) == 0; };
    if (want("fp")) falsePositives();
    if (want("bvh")) bvh();
    if (want("occl")) occlusion();
    if (want("hiz")) hiz();
    if (want("select")) selectOccluders();
    if (want("forest")) forest();
    sink = 0;
}
