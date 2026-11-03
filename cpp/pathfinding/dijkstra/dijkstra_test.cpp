#include "dijkstra.h"

#include <cstdlib>

#include "../../foundations/random/random.h"
#include "check.h"

using namespace ga;

// 暴力对照：反复松弛所有边直到不再变化（Bellman-Ford），慢但显然正确
static std::vector<int> bruteForce(const CostGrid& g, Cell start) {
    std::vector<int> d(g.size(), kUnreached);
    if (!g.passable(start)) return d;
    d[g.index(start)] = 0;
    for (bool changed = true; changed;) {
        changed = false;
        for (int i = 0; i < g.size(); ++i) {
            if (d[i] == kUnreached) continue;
            g.forEachNeighbor(g.cellAt(i), [&](Cell n) {
                int ni = g.index(n);
                if (d[i] + g.cost(n) < d[ni]) {
                    d[ni] = d[i] + g.cost(n);
                    changed = true;
                }
            });
        }
    }
    return d;
}

static bool isValidPath(const CostGrid& g, const std::vector<Cell>& path, Cell start, Cell goal) {
    if (path.empty() || !(path.front() == start) || !(path.back() == goal)) return false;
    for (size_t i = 0; i < path.size(); ++i) {
        if (!g.passable(path[i])) return false;
        if (i > 0 && std::abs(path[i].x - path[i - 1].x) + std::abs(path[i].y - path[i - 1].y) != 1) return false;
    }
    return true;
}

static std::vector<Cell> runStepper(const CostGrid& g, Cell start, Cell goal) {
    DijkstraStepper s(g, start, goal);
    while (s.step()) {}
    return s.path();
}

int main() {
    // 代价全为 1 时，Dijkstra 与 BFS 的路径长度相同
    {
        CostGrid g(10, 6);
        auto a = dijkstraPath(g, {0, 0}, {9, 5});
        auto b = bfsPath(g, {0, 0}, {9, 5});
        CHECK(a.size() == b.size());
        CHECK(pathCost(g, a) == 14);
    }
    // 宁可绕路也不进沼泽
    {
        CostGrid g(7, 3);
        for (int y = 0; y < 2; ++y) g.setCost({3, y}, 10);  // 中间一列沼泽，最下面一格是平地
        auto path = dijkstraPath(g, {0, 0}, {6, 0});
        CHECK(isValidPath(g, path, {0, 0}, {6, 0}));
        CHECK(pathCost(g, path) == 10);  // 下 2、右 6、上 2
        auto bfs = bfsPath(g, {0, 0}, {6, 0});
        CHECK(pathCost(g, bfs) == 15);   // BFS 直穿沼泽：步数少，代价高
    }
    // 起点等于终点；终点在墙里；被墙隔开
    {
        CostGrid g(5, 5);
        CHECK(dijkstraPath(g, {2, 2}, {2, 2}).size() == 1);
        g.setWall({4, 4}, true);
        CHECK(dijkstraPath(g, {0, 0}, {4, 4}).empty());
        for (int y = 0; y < 5; ++y) g.setWall({2, y}, true);
        CHECK(dijkstraPath(g, {0, 0}, {4, 0}).empty());
        CHECK(runStepper(g, {0, 0}, {4, 0}).empty());
    }
    // 随机地图：与暴力解法对拍，单步版本与一次性版本一致
    {
        Pcg32 rng(2024);
        for (int round = 0; round < 200; ++round) {
            CostGrid g(14, 9);
            for (int i = 0; i < g.size(); ++i) {
                Cell c = g.cellAt(i);
                uint32_t r = rng.below(10);
                if (r < 2) g.setWall(c, true);
                else if (r < 4) g.setCost(c, 3);
                else if (r < 5) g.setCost(c, 6);
            }
            Cell start{0, 0}, goal{13, 8};
            g.setWall(start, false);
            g.setWall(goal, false);

            auto truth = bruteForce(g, start);
            auto path = dijkstraPath(g, start, goal);
            auto stepped = runStepper(g, start, goal);
            int expect = truth[g.index(goal)];
            if (expect == kUnreached) {
                CHECK(path.empty());
                CHECK(stepped.empty());
            } else {
                CHECK(isValidPath(g, path, start, goal));
                CHECK(pathCost(g, path) == expect);
                CHECK(stepped == path);
            }
            // 单源 Dijkstra 地图与暴力解法逐格一致
            CHECK(dijkstraMap(g, {start}) == truth);
        }
    }
    // 多源 Dijkstra 地图：每格取到最近源点的代价；沿"下坡"一定能走到某个源点
    {
        CostGrid g(9, 1);
        auto map = dijkstraMap(g, {{0, 0}, {8, 0}});
        CHECK(map[g.index({4, 0})] == 4);
        CHECK(map[g.index({6, 0})] == 2);
        Cell c{5, 0};
        for (int i = 0; i < 20 && map[g.index(c)] > 0; ++i) c = rollDownhill(g, map, c);
        CHECK(c == (Cell{8, 0}));
    }

    return report("dijkstra_test");
}
