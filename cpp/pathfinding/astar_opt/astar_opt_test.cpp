#include <algorithm>
#include <vector>

#include "../../foundations/random/random.h"
#include "binary_heap.h"
#include "check.h"
#include "pathfinder.h"
#include "smoothing.h"

using namespace ga;

static CostGrid randomGrid(Pcg32& rng, int w, int h) {
    CostGrid g(w, h);
    for (int i = 0; i < g.size(); ++i) {
        uint32_t r = rng.below(10);
        if (r < 2) g.setWall(g.cellAt(i), true);
        else if (r < 4) g.setCost(g.cellAt(i), 3);
        else if (r < 5) g.setCost(g.cellAt(i), 6);
    }
    return g;
}

int main() {
    // 二叉堆：随机 push / decreaseKey / pop，出堆顺序与排序结果一致
    {
        Pcg32 rng(1);
        IndexedMinHeap<int> heap;
        heap.reset(500);
        std::vector<int> keys(500);
        for (int id = 0; id < 500; ++id) {
            keys[id] = static_cast<int>(rng.below(1000)) + 1000;
            heap.push(id, keys[id]);
        }
        for (int k = 0; k < 300; ++k) {  // 随机降低一些 key
            int id = static_cast<int>(rng.below(500));
            keys[id] -= static_cast<int>(rng.below(900));
            heap.decreaseKey(id, keys[id]);
        }
        std::vector<int> popped;
        while (!heap.empty()) {
            popped.push_back(heap.topKey());
            heap.pop();
        }
        std::vector<int> sorted = keys;
        std::sort(sorted.begin(), sorted.end());
        CHECK(popped == sorted);
        CHECK(!heap.contains(0));
    }
    // 权重 1 的 Pathfinder 与 1.3 的 astarPath 代价一致；同一个 Pathfinder 反复使用结果正确
    // 权重 w 时，路径代价不超过最优代价的 w 倍
    {
        Pcg32 rng(99);
        for (int round = 0; round < 60; ++round) {
            CostGrid g = randomGrid(rng, 24, 16);
            Pathfinder pf(g);
            for (int q = 0; q < 10; ++q) {
                Cell s{static_cast<int>(rng.below(24)), static_cast<int>(rng.below(16))};
                Cell t{static_cast<int>(rng.below(24)), static_cast<int>(rng.below(16))};
                if (!g.passable(s) || !g.passable(t)) continue;
                auto truth = astarPath(g, s, t);
                auto p1 = pf.find(s, t, 1.0f);
                CHECK(p1.empty() == truth.empty());
                if (truth.empty()) continue;
                int best = pathCost(g, truth);
                CHECK(pathCost(g, p1) == best);
                int e1 = pf.expandedCount();
                for (float w : {1.5f, 2.0f, 3.0f}) {
                    auto pw = pf.find(s, t, w);
                    CHECK(!pw.empty());
                    CHECK(pw.front() == s && pw.back() == t);
                    CHECK(pathCost(g, pw) <= w * best + 1e-3f);
                }
                pf.find(s, t, 1.0f);
                CHECK(pf.expandedCount() == e1);  // 复用缓冲区不影响结果
            }
        }
    }
    // 超覆盖直线：从 a 到 b，4 邻接连续，经过角点时两侧都访问
    {
        std::vector<Cell> cells;
        forEachCellOnLine({0, 0}, {7, 3}, [&](Cell c) { cells.push_back(c); return true; });
        CHECK(cells.front() == (Cell{0, 0}));
        CHECK(cells.back() == (Cell{7, 3}));
        cells.clear();
        forEachCellOnLine({0, 0}, {2, 2}, [&](Cell c) { cells.push_back(c); return true; });
        CHECK(cells.size() == 7);  // 3 个对角格 + 2 个角点 × 两侧
        CostGrid g(3, 3);
        g.setWall({1, 0}, true);  // 只堵住角点一侧，也不能斜穿
        CHECK(!lineOfSight(g, {0, 0}, {2, 2}, 1));
    }
    // 路径平滑
    {
        CostGrid g(10, 6);
        Pathfinder pf(g);
        auto path = pf.find({0, 0}, {9, 5});
        auto smooth = smoothPath(g, path);
        CHECK(smooth.size() == 2);  // 空地图：起点直连终点
        CHECK(polylineLength(smooth) < polylineLength(path));

        for (int y = 0; y < 5; ++y) g.setWall({5, y}, true);  // 竖墙，下方留口
        path = pf.find({0, 0}, {9, 0});
        smooth = smoothPath(g, path);
        CHECK(smooth.size() >= 3);
        for (size_t i = 1; i < smooth.size(); ++i) CHECK(lineOfSight(g, smooth[i - 1], smooth[i], 1));

        // 原路径绕开沼泽，捷径不能穿过沼泽
        CostGrid s(9, 3);
        for (int y = 0; y < 2; ++y) s.setCost({4, y}, 6);
        Pathfinder around(s);
        auto path2 = around.find({0, 0}, {8, 0});
        CHECK(pathCost(s, path2) == 12);  // 走最下面一行绕过去，比直穿（13）便宜
        auto sm = smoothPath(s, path2);
        for (size_t i = 1; i < sm.size(); ++i) CHECK(lineOfSight(s, sm[i - 1], sm[i], 1));
    }
    return report("astar_opt_test");
}
