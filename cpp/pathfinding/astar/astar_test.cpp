#include "astar.h"

#include "../../foundations/random/random.h"
#include "check.h"

using namespace ga;

static bool isValidPath(const CostGrid& g, const std::vector<Cell>& path, Cell start, Cell goal) {
    if (path.empty() || !(path.front() == start) || !(path.back() == goal)) return false;
    for (size_t i = 0; i < path.size(); ++i) {
        if (!g.passable(path[i])) return false;
        if (i > 0 && manhattan(path[i], path[i - 1]) != 1) return false;
    }
    return true;
}

struct Run {
    std::vector<Cell> path;
    int expanded;
};

static Run run(const CostGrid& g, Cell start, Cell goal, SearchMode mode) {
    BestFirstStepper s(g, start, goal, mode);
    while (s.step()) {}
    return {s.path(), s.expandedCount()};
}

int main() {
    // 空地图：A* 只展开路径上的格子（得益于 f 相同时优先 g 大的）
    {
        CostGrid g(20, 10);
        auto a = run(g, {0, 0}, {19, 9}, SearchMode::AStar);
        auto d = run(g, {0, 0}, {19, 9}, SearchMode::Dijkstra);
        CHECK(pathCost(g, a.path) == 28);
        CHECK(a.expanded == 29);  // 恰好是路径长度
        CHECK(d.expanded == g.size());  // Dijkstra 展开了整张图
    }
    // 起点等于终点、不可达
    {
        CostGrid g(5, 5);
        CHECK(astarPath(g, {2, 2}, {2, 2}).size() == 1);
        for (int y = 0; y < 5; ++y) g.setWall({2, y}, true);
        CHECK(astarPath(g, {0, 0}, {4, 0}).empty());
        CHECK(run(g, {0, 0}, {4, 0}, SearchMode::Greedy).path.empty());
    }
    // 凹形障碍：Greedy 一头扎进去，路径更贵；A* 与 Dijkstra 代价相同、展开更少
    {
        CostGrid g(25, 14);
        for (int y = 3; y <= 11; ++y) g.setWall({14, y}, true);
        for (int x = 10; x <= 14; ++x) {
            g.setWall({x, 3}, true);
            g.setWall({x, 11}, true);
        }
        Cell s{3, 7}, t{21, 7};
        auto a = run(g, s, t, SearchMode::AStar);
        auto d = run(g, s, t, SearchMode::Dijkstra);
        auto gr = run(g, s, t, SearchMode::Greedy);
        CHECK(pathCost(g, a.path) == pathCost(g, d.path));
        CHECK(a.expanded < d.expanded);
        CHECK(isValidPath(g, gr.path, s, t));
        CHECK(pathCost(g, gr.path) >= pathCost(g, a.path));
    }
    // 随机地图：A* 代价与 Dijkstra 一致，展开数不超过 Dijkstra；单步版与一次性版一致
    {
        Pcg32 rng(7);
        for (int round = 0; round < 300; ++round) {
            CostGrid g(16, 10);
            for (int i = 0; i < g.size(); ++i) {
                uint32_t r = rng.below(10);
                if (r < 2) g.setWall(g.cellAt(i), true);
                else if (r < 4) g.setCost(g.cellAt(i), 3);
                else if (r < 5) g.setCost(g.cellAt(i), 6);
            }
            Cell s{static_cast<int>(rng.below(16)), static_cast<int>(rng.below(10))};
            Cell t{static_cast<int>(rng.below(16)), static_cast<int>(rng.below(10))};
            g.setWall(s, false);
            g.setWall(t, false);

            auto truth = dijkstraPath(g, s, t);
            auto fast = astarPath(g, s, t);
            auto a = run(g, s, t, SearchMode::AStar);
            auto d = run(g, s, t, SearchMode::Dijkstra);
            auto gr = run(g, s, t, SearchMode::Greedy);
            if (truth.empty()) {
                CHECK(fast.empty());
                CHECK(a.path.empty());
                CHECK(gr.path.empty());
                continue;
            }
            CHECK(isValidPath(g, fast, s, t));
            CHECK(pathCost(g, fast) == pathCost(g, truth));
            CHECK(pathCost(g, a.path) == pathCost(g, fast));
            CHECK(pathCost(g, d.path) == pathCost(g, truth));
            CHECK(a.expanded <= d.expanded);
            CHECK(isValidPath(g, gr.path, s, t));
            CHECK(pathCost(g, gr.path) >= pathCost(g, truth));
        }
    }
    return report("astar_test");
}
