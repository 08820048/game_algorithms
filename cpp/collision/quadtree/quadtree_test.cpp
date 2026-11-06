#include "quadtree.h"

#include <cstdio>

#include "../../foundations/random/random.h"
#include "check.h"

using namespace ga;

static float rnd(Pcg32& r, float lo, float hi) { return lo + r.nextFloat() * (hi - lo); }

// 随机场景。kind 0：大小相近 1：大小悬殊 2：对齐到 25 的整数倍（正好压在分割线上） 3：聚成几团
template <int D>
static std::vector<BoxN<D>> randomScene(Pcg32& r, int n, int kind) {
    std::vector<BoxN<D>> boxes;
    std::array<float, D> centers[4];
    for (auto& c : centers)
        for (int k = 0; k < D; ++k) c[k] = rnd(r, 0, 200);
    for (int i = 0; i < n; ++i) {
        BoxN<D> b;
        for (int k = 0; k < D; ++k) {
            float p = rnd(r, -20, 220), s;  // 一部分落在世界 [0, 200] 之外
            if (kind == 0) s = rnd(r, 1, 10);
            else if (kind == 1) s = r.nextFloat() < 0.05f ? rnd(r, 30, 250) : rnd(r, 0, 5);
            else if (kind == 2) p = std::floor(p / 25) * 25, s = 25.0f * r.range(0, 2);
            else p = centers[i % 4][k] + rnd(r, -8, 8), s = rnd(r, 0.5f, 3);
            b.min[k] = p, b.max[k] = p + s;
        }
        boxes.push_back(b);
    }
    return boxes;
}

template <int D>
static BoxN<D> world() {
    BoxN<D> w;
    w.min.fill(0), w.max.fill(200);
    return w;
}

template <int D>
static bool inside(const BoxN<D>& b, const BoxN<D>& bound) {
    for (int k = 0; k < D; ++k)
        if (b.min[k] < bound.min[k] || b.max[k] > bound.max[k]) return false;
    return true;
}

static std::vector<TreePair> sorted(std::vector<TreePair> v) {
    std::sort(v.begin(), v.end());
    return v;
}

template <int D>
static int run(Pcg32& rng, int* missesWithStackOnLoose) {
    int scenes = 0;
    std::vector<TreePair> ref, got;
    for (int n : {0, 1, 2, 40, 400, 1200})
        for (int kind = 0; kind < 4; ++kind)
            for (float loose : {1.0f, 2.0f})
                for (auto [depth, split] : {std::pair{0, 4}, {3, 1}, {8, 8}, {12, 2}}) {
                    auto boxes = randomScene<D>(rng, n, kind);
                    bruteForcePairs(boxes, ref);
                    Tree<D> tree(world<D>(), {depth, split, loose});
                    tree.build(boxes);
                    ++scenes;

                    tree.findPairs(got);
                    CHECK(sorted(got) == ref);
                    tree.findPairsByQuery(got);
                    CHECK(sorted(got) == ref);
                    if (loose == 2) {
                        tree.findPairsStrict(got);  // 祖先栈的办法用在松散树上：会漏
                        if (got.size() < ref.size()) ++*missesWithStackOnLoose;
                        CHECK(got.size() <= ref.size());
                    }

                    // 结构：非根节点的物体在节点的（松散）边界内；内部节点上的物体装不进任何孩子；
                    // 叶子超过 splitCount 只能是因为到了最大深度
                    const auto& nodes = tree.nodes();
                    for (int i = 0; i < n; ++i) {
                        int nd = tree.nodeOf(i);
                        CHECK(inside(boxes[i], tree.looseBounds(nd)));
                        if (nodes[nd].firstChild >= 0) CHECK(tree.fitChild(nd, boxes[i]) < 0);
                    }
                    int total = 0;
                    for (const auto& node : nodes) {
                        total += node.count;
                        if (node.firstChild < 0 && node.count > split) CHECK(node.depth == depth);
                    }
                    CHECK(total == n);

                    // 区域查询：每个重叠的物体恰好一次
                    for (int q = 0; q < 6; ++q) {
                        BoxN<D> region;
                        for (int k = 0; k < D; ++k) {
                            float p = rnd(rng, -40, 230);
                            region.min[k] = p, region.max[k] = p + rnd(rng, 0, 70);
                        }
                        std::vector<int> hits(boxes.size(), 0);
                        tree.query(region, [&](int j) { ++hits[j]; });
                        bool ok = true;
                        for (int j = 0; j < n; ++j) ok &= hits[j] == (overlaps(boxes[j], region) ? 1 : 0);
                        CHECK(ok);
                    }
                }
    return scenes;
}

int main() {
    // 手算的例子：跨越中线的盒子留在根节点
    {
        std::vector<BoxN<2>> boxes{{{90, 10}, {110, 20}}, {{10, 10}, {20, 20}}, {{12, 12}, {18, 18}}, {{150, 150}, {160, 160}}};
        Quadtree tree({{0, 0}, {200, 200}}, {4, 1, 1});
        tree.build(boxes);
        CHECK(tree.depthOf(0) == 0);
        CHECK(tree.depthOf(1) >= 1 && tree.depthOf(3) == 1);
        std::vector<TreePair> pairs;
        tree.findPairs(pairs);
        CHECK((pairs.size() == 1 && pairs[0] == TreePair{1, 2}));
        // 松散树：同一个盒子按中心（100.0 在右半边）放进孩子
        Quadtree loose({{0, 0}, {200, 200}}, {4, 1, 2});
        loose.build(boxes);
        CHECK(loose.depthOf(0) >= 1);
    }

    Pcg32 rng(5);
    int misses2 = 0, misses3 = 0;
    CHECK(run<2>(rng, &misses2) == 192);
    CHECK(run<3>(rng, &misses3) == 192);
    CHECK(misses2 > 0 && misses3 > 0);  // 确认"祖先栈在松散树上会漏掉物体对"不是空话
    std::printf("stack-on-loose missed pairs in %d (2D) / %d (3D) scenes\n", misses2, misses3);
    return report("quadtree_test");
}
