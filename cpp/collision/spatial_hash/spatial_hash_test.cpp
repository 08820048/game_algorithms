#include "spatial_hash.h"

#include <cstdio>

#include "../../foundations/random/random.h"
#include "check.h"

using namespace ga;

static float rnd(Pcg32& r, float lo, float hi) { return lo + r.nextFloat() * (hi - lo); }

// 随机场景。kind 0：大小相近 1：大小悬殊（少数很大的物体） 2：落在整数坐标上（边恰好压在格线上、恰好接触）
static std::vector<Aabb> randomScene(Pcg32& r, int n, int kind, Aabb area) {
    std::vector<Aabb> boxes;
    for (int i = 0; i < n; ++i) {
        Vec2 p{rnd(r, area.min.x, area.max.x), rnd(r, area.min.y, area.max.y)};
        Vec2 size;
        if (kind == 0) size = {rnd(r, 2, 12), rnd(r, 2, 12)};
        else if (kind == 1) size = r.nextFloat() < 0.05f ? Vec2{rnd(r, 40, 300), rnd(r, 40, 300)} : Vec2{rnd(r, 0, 6), rnd(r, 0, 6)};
        else {
            p = {std::floor(p.x / 5) * 5, std::floor(p.y / 5) * 5};
            size = {5.0f * r.range(0, 3), 5.0f * r.range(0, 3)};
        }
        boxes.push_back({p, p + size});
    }
    return boxes;
}

static std::vector<Pair> sorted(std::vector<Pair> v) {
    std::sort(v.begin(), v.end());
    return v;
}

// 区域查询：每个重叠的物体恰好回调一次
template <class B>
static bool queryMatches(const B& bp, const std::vector<Aabb>& boxes, const Aabb& region) {
    std::vector<int> hits(boxes.size(), 0);
    bp.query(region, [&](int j) { ++hits[j]; });
    for (size_t j = 0; j < boxes.size(); ++j)
        if (hits[j] != (overlaps(boxes[j], region) ? 1 : 0)) return false;
    return true;
}

int main() {
    // 手算的例子：两个盒子跨越 4 个公共格子，也只报告一次；恰好接触的不算重叠
    {
        std::vector<Aabb> boxes{{{5, 5}, {25, 25}}, {{8, 8}, {30, 30}}, {{25, 0}, {35, 5}}, {{-50, -50}, {-40, -40}}};
        UniformGrid grid(GridIndexer({{0, 0}, {100, 100}}, 10));
        grid.build(boxes);
        std::vector<Pair> pairs;
        grid.findPairs(pairs);
        CHECK((pairs.size() == 1 && pairs[0] == Pair{0, 1}));
        int found = 0;
        grid.query({{-45, -45}, {-44, -44}}, [&](int j) { found += j == 3 ? 1 : 100; });  // 世界外的物体被夹进角落的格子
        CHECK(found == 1);
    }

    Pcg32 rng(31);
    std::vector<Pair> ref, got;
    int scenes = 0;
    for (int n : {0, 1, 2, 30, 300, 1500})
        for (int kind = 0; kind < 3; ++kind)
            for (float cell : {1.5f, 8.0f, 25.0f, 500.0f}) {
                // 物体有一部分落在世界边界之外（网格要把它们夹进边缘的格子），坐标也有负数（空间哈希）
                Aabb world{{0, 0}, {200, 150}};
                std::vector<Aabb> boxes = randomScene(rng, n, kind, {{-30, -30}, {220, 170}});
                long bruteTests = bruteForcePairs(boxes, ref);
                ++scenes;

                NaiveGrid naive(world, cell);
                naive.findPairs(boxes, got);
                CHECK(got == ref);

                UniformGrid grid(GridIndexer(world, cell));
                grid.build(boxes);
                long tests = grid.findPairs(got);
                CHECK(sorted(got) == ref);  // 排序后完全相同：没有遗漏，也没有重复
                if (n == 1500 && kind == 0 && cell == 25.0f) CHECK(tests * 20 < bruteTests);

                // 桶很少时大量格子挤进同一个桶，结果仍然正确
                for (int buckets : {1, 7, 4096}) {
                    SpatialHash hash(HashIndexer(cell, buckets));
                    hash.build(boxes);
                    hash.findPairs(got);
                    CHECK(sorted(got) == ref);
                    for (int q = 0; q < 5; ++q) {
                        Vec2 p{rnd(rng, -60, 240), rnd(rng, -60, 200)};
                        Aabb region{p, p + Vec2{rnd(rng, 0, 80), rnd(rng, 0, 80)}};
                        CHECK(queryMatches(hash, boxes, region));
                    }
                }
                for (int q = 0; q < 10; ++q) {
                    Vec2 p{rnd(rng, -60, 240), rnd(rng, -60, 200)};
                    Aabb region{p, p + Vec2{rnd(rng, 0, 80), rnd(rng, 0, 80)}};
                    CHECK(queryMatches(grid, boxes, region));
                }
            }
    CHECK(scenes == 72);

    // 重建多次：上一帧的内容不会残留
    {
        UniformGrid grid(GridIndexer({{0, 0}, {100, 100}}, 10));
        SpatialHash hash(HashIndexer(10, 64));
        for (int frame = 0; frame < 20; ++frame) {
            std::vector<Aabb> boxes = randomScene(rng, rng.range(0, 200), frame % 3, {{0, 0}, {100, 100}});
            bruteForcePairs(boxes, ref);
            grid.build(boxes);
            grid.findPairs(got);
            CHECK(sorted(got) == ref);
            hash.build(boxes);
            hash.findPairs(got);
            CHECK(sorted(got) == ref);
        }
    }
    return report("spatial_hash_test");
}
