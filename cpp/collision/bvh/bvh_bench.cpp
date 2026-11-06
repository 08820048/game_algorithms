// Sweep and Prune、动态 BVH 与网格、四叉树的对比（不由 CMake 构建）：
//   c++ -std=c++20 -O2 -Icpp/common cpp/collision/bvh/bvh_bench.cpp -o bvh_bench && ./bvh_bench
// 物体连续运动 60 帧，测量每帧的平均耗时（更新结构 + 找出所有物体对）
#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <memory>
#include <optional>

#include "../../foundations/random/random.h"
#include "../quadtree/quadtree.h"
#include "bvh.h"

using namespace ga;
using Clock = std::chrono::steady_clock;

static float rnd(Pcg32& r, float lo, float hi) { return lo + r.nextFloat() * (hi - lo); }

constexpr int kN = 16000;
constexpr int kFrames = 60;

struct World {
    Vec2 size;
    std::vector<Aabb> boxes;
    std::vector<Vec2> vel;
    void step() {
        for (size_t i = 0; i < boxes.size(); ++i) {
            Aabb& b = boxes[i];
            Vec2& v = vel[i];
            b.min += v, b.max += v;
            if (b.min.x < 0 && v.x < 0) v.x = -v.x;
            if (b.min.y < 0 && v.y < 0) v.y = -v.y;
            if (b.max.x > size.x && v.x > 0) v.x = -v.x;
            if (b.max.y > size.y && v.y > 0) v.y = -v.y;
        }
    }
};

enum class Layout { Uniform, Wide, Clustered };

static World makeWorld(Layout layout, float speed, Vec2 size, uint64_t seed = 7) {
    Pcg32 r(seed);
    World w{size, {}, {}};
    Vec2 centers[8];
    for (Vec2& c : centers) c = {rnd(r, 0.1f, 0.9f) * size.x, rnd(r, 0.1f, 0.9f) * size.y};
    for (int i = 0; i < kN; ++i) {
        float s = layout == Layout::Wide ? 2 * std::exp2(8 * r.nextFloat()) : rnd(r, 4, 12);
        Vec2 p{rnd(r, 0, size.x - s), rnd(r, 0, size.y - s)};
        if (layout == Layout::Clustered) {
            Vec2 c = centers[i % 8];
            p = c + Vec2{(r.nextFloat() + r.nextFloat() + r.nextFloat() - 1.5f) * 0.06f * size.x,
                         (r.nextFloat() + r.nextFloat() + r.nextFloat() - 1.5f) * 0.06f * size.y};
        }
        w.boxes.push_back({p, p + Vec2{s, s}});
        w.vel.push_back(fromAngle(rnd(r, 0, 2 * kPi)) * speed * rnd(r, 0.5f, 1.5f));
    }
    return w;
}

struct Stats {
    double ms = 0;        // 每帧平均
    double extra = 0;     // 方法相关的统计（交换次数、重新插入数……），每帧平均
    long tests = 0;       // 最后一帧的测试次数
    size_t pairs = 0;
};

// 一个方法：init 在第 0 帧之前调用（不计时），frame 每帧调用，返回 {tests, extra}
struct Method {
    const char* name;
    std::function<void(const World&)> init;
    std::function<std::pair<long, long>(const World&, std::vector<Pair>&)> frame;
};

static Stats run(World w, const Method& m) {
    Stats best;
    best.ms = 1e30;
    for (int rep = 0; rep < 3; ++rep) {
        World copy = w;
        m.init(copy);
        std::vector<Pair> pairs;
        double total = 0, extra = 0;
        long tests = 0;
        for (int f = 0; f < kFrames; ++f) {
            copy.step();
            auto t0 = Clock::now();
            auto [t, e] = m.frame(copy, pairs);
            total += std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
            extra += e, tests = t;
        }
        if (total / kFrames < best.ms) best = {total / kFrames, extra / kFrames, tests, pairs.size()};
    }
    return best;
}

// 各种方法。物体编号与 boxes 下标一致
static std::vector<Method> methods(float gridCell, float margin) {
    auto grid = std::make_shared<std::optional<UniformGrid>>();
    auto tree = std::make_shared<std::optional<Quadtree>>();
    auto nb = std::make_shared<std::vector<BoxN<2>>>();
    auto tp = std::make_shared<std::vector<TreePair>>();
    auto sap = std::make_shared<SweepAndPrune>();
    auto sap2 = std::make_shared<SweepAndPrune>();
    auto bvh = std::make_shared<DynamicTree>();
    auto leaves = std::make_shared<std::vector<int>>();
    auto bvh2 = std::make_shared<DynamicTree>();
    auto bp = std::make_shared<std::optional<Broadphase>>();
    return {
        {"grid (2.4)",
         [=](const World& w) { grid->emplace(GridIndexer({{0, 0}, w.size}, gridCell)); },
         [=](const World& w, std::vector<Pair>& out) {
             (*grid)->build(w.boxes);
             return std::pair{(*grid)->findPairs(out), 0L};
         }},
        {"quadtree (2.5)",
         [=](const World& w) { tree->emplace(BoxN<2>{{0, 0}, {w.size.x, w.size.y}}, Quadtree::Options{10, 8, 1}); },
         [=](const World& w, std::vector<Pair>& out) {
             nb->clear();
             for (const Aabb& b : w.boxes) nb->push_back(toBox(b));
             (*tree)->build(*nb);
             long t = (*tree)->findPairs(*tp);
             out.assign(tp->size(), {});
             for (size_t k = 0; k < tp->size(); ++k) out[k] = {(*tp)[k].a, (*tp)[k].b};
             return std::pair{t, 0L};
         }},
        {"SAP, std::sort",
         [=](const World&) {},
         [=](const World& w, std::vector<Pair>& out) {
             sap2->sortFromScratch(w.boxes);
             return std::pair{sap2->findPairs(out), 0L};
         }},
        {"SAP, incremental",
         [=](const World& w) { sap->sortFromScratch(w.boxes); },
         [=](const World& w, std::vector<Pair>& out) {
             long swaps = sap->update(w.boxes);
             return std::pair{sap->findPairs(out), swaps};
         }},
        {"BVH, rebuilt",
         [=](const World&) {},
         [=](const World& w, std::vector<Pair>& out) {
             *bvh2 = DynamicTree(margin);
             for (int i = 0; i < static_cast<int>(w.boxes.size()); ++i) bvh2->insert(w.boxes[i], i);
             return std::pair{bvh2->findPairs(w.boxes, out), 0L};
         }},
        {"BVH, incremental",
         [=](const World& w) {
             *bvh = DynamicTree(margin);
             leaves->clear();
             for (int i = 0; i < static_cast<int>(w.boxes.size()); ++i) leaves->push_back(bvh->insert(w.boxes[i], i));
         },
         [=](const World& w, std::vector<Pair>& out) {
             long moved = 0;
             for (int i = 0; i < static_cast<int>(w.boxes.size()); ++i)
                 if (w.vel[i] != Vec2{0, 0}) moved += bvh->move((*leaves)[i], w.boxes[i]);
             return std::pair{bvh->findPairs(w.boxes, out), moved};
         }},
        {"BVH + kept pairs",
         [=](const World& w) {
             bp->emplace(margin);
             for (int i = 0; i < static_cast<int>(w.boxes.size()); ++i) (*bp)->add(i, w.boxes[i]);
             (*bp)->update();
         },
         [=](const World& w, std::vector<Pair>& out) {
             for (int i = 0; i < static_cast<int>(w.boxes.size()); ++i)
                 if (w.vel[i] != Vec2{0, 0}) (*bp)->move(i, w.boxes[i]);  // 静止的物体不需要通知宽相
             long moved = (*bp)->update();
             return std::pair{(*bp)->findPairs(w.boxes, out), moved};
         }},
    };
}

static void table(const char* title, const World& w, float gridCell, float margin) {
    std::printf("\n%s\n", title);
    std::printf("  %-18s %9s %10s %9s %12s\n", "method", "ms/frame", "tests", "pairs", "extra/frame");
    for (const Method& m : methods(gridCell, margin)) {
        Stats s = run(w, m);
        std::printf("  %-18s %7.2fms %10ld %9zu %12.0f\n", m.name, s.ms, s.tests, s.pairs, s.extra);
    }
}

int main() {
    const float side = std::sqrt(kN * 400.0f);
    std::printf("n = %d, %d frames; extra = swaps (SAP incremental) or reinserted leaves (BVH)\n", kN, kFrames);
    table("1) uniform, boxes 4..12, speed ~1 px/frame", makeWorld(Layout::Uniform, 1, {side, side}), 12, 2);
    table("2) clustered (8 clusters), speed ~1", makeWorld(Layout::Clustered, 1, {side, side}), 12, 2);
    table("3) sizes 2..512 (log-uniform), world 2x side, speed ~1", makeWorld(Layout::Wide, 1, {2 * side, 2 * side}), 128, 2);
    // 横版卷轴式的长条世界：物体在 x 方向铺开
    table("4) long strip 16:1, sweeping along the long axis (x)", makeWorld(Layout::Uniform, 1, {side * 4, side / 4}), 12, 2);
    {
        // 同一个世界转 90°：沿短轴扫描
        World w = makeWorld(Layout::Uniform, 1, {side * 4, side / 4});
        for (Aabb& b : w.boxes) b = {{b.min.y, b.min.x}, {b.max.y, b.max.x}};
        for (Vec2& v : w.vel) v = {v.y, v.x};
        w.size = {w.size.y, w.size.x};
        table("5) same strip rotated 90 degrees: sweeping along the short axis", w, 12, 2);
    }

    {
        // 典型的游戏关卡：大部分物体静止（墙、箱子、睡眠中的刚体），只有 10% 在动
        World w = makeWorld(Layout::Uniform, 1, {side, side});
        Pcg32 r(5);
        for (Vec2& v : w.vel)
            if (r.nextFloat() >= 0.1f) v = {0, 0};
        table("5b) uniform, only 10% of the objects move", w, 12, 4);
    }
    table("1b) uniform, all moving, margin 4", makeWorld(Layout::Uniform, 1, {side, side}), 12, 4);

    std::printf("\n6) speed (uniform): incremental SAP and BVH (margin 2)\n");
    std::printf("  %-8s %10s %12s %10s %12s %10s\n", "px/frame", "SAP inc", "swaps", "BVH inc", "reinserts", "grid");
    for (float speed : {0.25f, 1.0f, 4.0f, 16.0f}) {
        World w = makeWorld(Layout::Uniform, speed, {side, side});
        auto ms = methods(12, 2);
        Stats g = run(w, ms[0]), s = run(w, ms[3]), b = run(w, ms[5]);
        std::printf("  %-8.2f %8.2fms %12.0f %8.2fms %12.0f %8.2fms\n", speed, s.ms, s.extra, b.ms, b.extra, g.ms);
    }

    std::printf("\n7) BVH margin (uniform, speed ~1)\n");
    std::printf("  %-7s %10s %12s %10s\n", "margin", "ms/frame", "reinserts", "tests");
    for (float margin : {0.0f, 0.5f, 1.0f, 2.0f, 4.0f, 8.0f, 16.0f}) {
        World w = makeWorld(Layout::Uniform, 1, {side, side});
        Stats b = run(w, methods(12, margin)[5]);
        std::printf("  %-7.1f %8.2fms %12.0f %10ld\n", margin, b.ms, b.extra, b.tests);
    }

    std::printf("\n8) tree quality: rotations on / off (uniform, static, inserted in random order and in sorted order)\n");
    for (int sortedOrder = 0; sortedOrder < 2; ++sortedOrder) {
        World w = makeWorld(Layout::Uniform, 0, {side, side});
        if (sortedOrder) std::sort(w.boxes.begin(), w.boxes.end(), [](const Aabb& a, const Aabb& b) { return a.min.x < b.min.x; });
        for (bool balance : {true, false}) {
            DynamicTree t(2, balance);
            auto t0 = Clock::now();
            for (int i = 0; i < kN; ++i) t.insert(w.boxes[i], i);
            double build = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
            std::vector<Pair> out;
            long tests = 0;
            double ms = 1e30;
            for (int r = 0; r < 5; ++r) {
                auto t1 = Clock::now();
                tests = t.findPairs(w.boxes, out);
                ms = std::min(ms, std::chrono::duration<double, std::milli>(Clock::now() - t1).count());
            }
            std::printf("  %-7s rotations %-3s: height %5d, internal perimeter %10.0f, build %7.2fms, find pairs %7.2fms (%ld tests)\n",
                        sortedOrder ? "sorted" : "random", balance ? "on" : "off", t.height(), t.internalPerimeter(), build, ms, tests);
        }
    }
}
