#include "bvh.h"

#include <cmath>
#include <cstdio>

#include "../../foundations/random/random.h"
#include "check.h"

using namespace ga;

static float rnd(Pcg32& r, float lo, float hi) { return lo + r.nextFloat() * (hi - lo); }

static Aabb randomBox(Pcg32& r, int kind) {
    Vec2 p{rnd(r, -20, 220), rnd(r, -20, 220)};
    Vec2 s;
    if (kind == 0) s = {rnd(r, 1, 10), rnd(r, 1, 10)};
    else if (kind == 1) s = r.nextFloat() < 0.05f ? Vec2{rnd(r, 30, 200), rnd(r, 30, 200)} : Vec2{rnd(r, 0, 5), rnd(r, 0, 5)};
    else p = {std::floor(p.x / 10) * 10, std::floor(p.y / 10) * 10}, s = {10.0f * r.range(0, 2), 10.0f * r.range(0, 2)};
    return {p, p + s};
}

static std::vector<Pair> sorted(std::vector<Pair> v) {
    std::sort(v.begin(), v.end());
    return v;
}

// 检查树的结构：父子链接、包围盒、高度；返回子树里的叶子数
static int checkNode(const DynamicTree& t, int n, int parent, const std::vector<Aabb>& tight, bool& ok) {
    const auto& node = t.nodes()[n];
    ok &= node.parent == parent;
    if (node.leaf()) {
        ok &= node.height == 0 && containsBox(node.box, tight[node.item]);
        return 1;
    }
    const auto &l = t.nodes()[node.left], &r = t.nodes()[node.right];
    ok &= node.height == 1 + std::max(l.height, r.height);
    Aabb m = merge(l.box, r.box);
    ok &= m.min == node.box.min && m.max == node.box.max;
    return checkNode(t, node.left, n, tight, ok) + checkNode(t, node.right, n, tight, ok);
}

int main() {
    std::vector<Pair> ref, got;

    // ---- Sweep and Prune ----
    {
        Pcg32 rng(2);
        for (int kind = 0; kind < 3; ++kind)
            for (int n : {0, 1, 2, 50, 600}) {
                std::vector<Aabb> boxes;
                std::vector<Vec2> vel;
                for (int i = 0; i < n; ++i) boxes.push_back(randomBox(rng, kind)), vel.push_back({rnd(rng, -3, 3), rnd(rng, -3, 3)});
                SweepAndPrune sap, fresh;
                for (int frame = 0; frame < 30; ++frame) {
                    long swaps = sap.update(boxes);
                    if (frame > 0 && kind == 2) CHECK(swaps == 0);  // kind 2 不动：顺序不变，无需交换
                    bruteForcePairs(boxes, ref);
                    sap.findPairs(got);
                    CHECK(sorted(got) == ref);
                    fresh.sortFromScratch(boxes);
                    fresh.findPairs(got);
                    CHECK(sorted(got) == ref);
                    for (size_t k = 1; k < sap.entries().size(); ++k) CHECK(sap.entries()[k - 1].lo <= sap.entries()[k].lo);
                    if (kind != 2)
                        for (int i = 0; i < n; ++i) {
                            Vec2 d = vel[i] * (frame % 10 == 9 ? 20.0f : 1.0f);  // 偶尔大幅跳动
                            boxes[i].min += d, boxes[i].max += d;
                        }
                }
            }
    }

    // ---- 动态 BVH：插入、移动、删除交替进行 ----
    for (bool balance : {true, false})
        for (float margin : {0.0f, 2.0f})
            for (int kind = 0; kind < 3; ++kind) {
                Pcg32 rng(17 + kind);
                DynamicTree tree(margin, balance);
                std::vector<Aabb> tight;  // 按物体编号
                std::vector<int> leafOf;  // 物体 → 叶子；-1 表示已删除
                std::vector<Vec2> vel;
                auto add = [&] {
                    int id = static_cast<int>(tight.size());
                    tight.push_back(randomBox(rng, kind));
                    vel.push_back({rnd(rng, -2, 2), rnd(rng, -2, 2)});
                    leafOf.push_back(tree.insert(tight.back(), id));
                };
                for (int i = 0; i < 400; ++i) add();
                for (int frame = 0; frame < 40; ++frame) {
                    // 移动
                    for (size_t i = 0; i < tight.size(); ++i) {
                        if (leafOf[i] < 0) continue;
                        Vec2 d = vel[i] * (rng.nextFloat() < 0.05f ? 30.0f : 1.0f);
                        tight[i].min += d, tight[i].max += d;
                        tree.move(leafOf[i], tight[i]);
                    }
                    // 删除一些、加入一些
                    for (int k = 0; k < 10; ++k) {
                        int i = rng.range(0, static_cast<int>(tight.size()) - 1);
                        if (leafOf[i] >= 0) tree.remove(leafOf[i]), leafOf[i] = -1;
                    }
                    for (int k = 0; k < 8; ++k) add();

                    // 对照：只看还活着的物体
                    std::vector<Aabb> live;
                    std::vector<int> ids;
                    for (size_t i = 0; i < tight.size(); ++i)
                        if (leafOf[i] >= 0) live.push_back(tight[i]), ids.push_back(static_cast<int>(i));
                    bruteForcePairs(live, ref);
                    for (Pair& p : ref) p = {ids[p.a], ids[p.b]};
                    tree.findPairs(tight, got);
                    CHECK(sorted(got) == sorted(ref));

                    bool ok = true;
                    int leaves = tree.root() < 0 ? 0 : checkNode(tree, tree.root(), -1, tight, ok);
                    CHECK(ok);
                    CHECK(leaves == static_cast<int>(live.size()));
                    int used = 0;
                    for (const auto& node : tree.nodes()) used += node.height >= 0;
                    CHECK(used == 2 * leaves - 1);  // 空闲节点被回收再用，没有泄漏

                    for (int q = 0; q < 5; ++q) {
                        Vec2 p{rnd(rng, -40, 230), rnd(rng, -40, 230)};
                        Aabb region{p, p + Vec2{rnd(rng, 0, 60), rnd(rng, 0, 60)}};
                        std::vector<int> hits(tight.size(), 0);
                        tree.query(region, [&](int j) { ++hits[j]; });
                        bool qok = true;
                        for (size_t j = 0; j < tight.size(); ++j) {
                            if (leafOf[j] < 0) qok &= hits[j] == 0;
                            // 查询比较的是胖盒子：真正重叠的一定被报告，且只报告一次
                            else if (overlaps(tight[j], region)) qok &= hits[j] == 1;
                            else qok &= hits[j] <= 1;
                        }
                        CHECK(qok);
                    }
                }
            }

    // ---- 平衡：按顺序插入一排盒子是二叉树的最坏情况 ----
    {
        int heights[2];
        for (int b = 0; b < 2; ++b) {
            DynamicTree tree(0, b == 0);
            for (int i = 0; i < 1024; ++i) tree.insert({{i * 2.0f, 0}, {i * 2.0f + 1, 1}}, i);
            heights[b] = tree.height();
        }
        std::printf("1024 boxes in a row: height %d with rotations, %d without\n", heights[0], heights[1]);
        CHECK(heights[0] <= 20);
        CHECK(heights[1] > 100);
    }
    // ---- 保留物体对的宽相：移动的物体有快有慢，还有静止的 ----
    for (float margin : {0.0f, 3.0f}) {
        Pcg32 rng(41);
        std::vector<Aabb> tight;
        std::vector<Vec2> vel;
        Broadphase bp(margin);
        for (int i = 0; i < 500; ++i) {
            tight.push_back(randomBox(rng, i % 2));
            float speed = i % 3 == 0 ? 0.0f : i % 3 == 1 ? 0.5f : 6.0f;
            vel.push_back(fromAngle(rnd(rng, 0, 7)) * speed);
            bp.add(i, tight[i]);
        }
        for (int frame = 0; frame < 60; ++frame) {
            bp.update();
            bruteForcePairs(tight, ref);
            bp.findPairs(tight, got);
            CHECK(sorted(got) == ref);
            for (size_t i = 0; i < tight.size(); ++i) {
                Vec2 d = vel[i] * (frame % 20 == 19 ? 15.0f : 1.0f);
                tight[i].min += d, tight[i].max += d;
                bp.move(static_cast<int>(i), tight[i]);
            }
        }
    }
    return report("bvh_test");
}
