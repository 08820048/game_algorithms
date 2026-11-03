#include "jps.h"

#include "../../foundations/random/random.h"
#include "check.h"

using namespace ga;

struct Result {
    std::vector<Cell> path;  // 逐格路径
    int cost = -1;
    int expanded = 0;
};

static Result run(const Grid& g, Cell s, Cell t, Grid8Mode mode) {
    Grid8Search search(g, s, t, mode);
    while (search.step()) {}
    Result r;
    r.expanded = search.expandedCount();
    if (search.found()) {
        r.path = expandJumpPath(search.nodes());
        r.cost = search.g(t);
    }
    return r;
}

// 逐格检查：每一步都是合法的 8 方向移动，且总代价与搜索报告的一致
static bool validPath(const Grid& g, const Result& r, Cell s, Cell t) {
    if (r.path.empty() || !(r.path.front() == s) || !(r.path.back() == t)) return false;
    int cost = 0;
    for (size_t i = 1; i < r.path.size(); ++i) {
        int dx = r.path[i].x - r.path[i - 1].x, dy = r.path[i].y - r.path[i - 1].y;
        if (std::abs(dx) > 1 || std::abs(dy) > 1 || (dx == 0 && dy == 0)) return false;
        if (!canMove(g, r.path[i - 1], dx, dy)) return false;
        cost += octile(r.path[i - 1], r.path[i]);
    }
    return cost == r.cost;
}

int main() {
    // 空地图：先斜走再直走
    {
        Grid g(20, 10);
        auto a = run(g, {0, 0}, {19, 5}, Grid8Mode::AStar);
        auto j = run(g, {0, 0}, {19, 5}, Grid8Mode::Jps);
        CHECK(a.cost == 5 * kDiagonal + 14 * kStraight);
        CHECK(j.cost == a.cost);
        CHECK(validPath(g, j, {0, 0}, {19, 5}));
        CHECK(j.expanded < a.expanded);
    }
    // 不能切墙角：两堵墙之间只有一个对角缝隙
    {
        Grid g(3, 3);
        g.setWall({1, 0}, true);
        g.setWall({0, 1}, true);
        CHECK(run(g, {0, 0}, {2, 2}, Grid8Mode::AStar).cost < 0);
        CHECK(run(g, {0, 0}, {2, 2}, Grid8Mode::Jps).cost < 0);
    }
    // 随机地图（墙密度 0%~40%）：JPS 与 A* 的最优代价完全一致，路径逐格合法
    {
        Pcg32 rng(2025);
        int jpsExpanded = 0, astarExpanded = 0;
        for (int round = 0; round < 3000; ++round) {
            int w = 4 + static_cast<int>(rng.below(28)), h = 4 + static_cast<int>(rng.below(20));
            Grid g(w, h);
            uint32_t density = rng.below(41);
            for (int i = 0; i < g.size(); ++i) {
                if (rng.below(100) < density) g.setWall(g.cellAt(i), true);
            }
            Cell s{static_cast<int>(rng.below(w)), static_cast<int>(rng.below(h))};
            Cell t{static_cast<int>(rng.below(w)), static_cast<int>(rng.below(h))};
            g.setWall(s, false);
            g.setWall(t, false);
            auto a = run(g, s, t, Grid8Mode::AStar);
            auto j = run(g, s, t, Grid8Mode::Jps);
            CHECK(a.cost == j.cost);
            if (a.cost >= 0) {
                CHECK(validPath(g, a, s, t));
                CHECK(validPath(g, j, s, t));
            }
            jpsExpanded += j.expanded;
            astarExpanded += a.expanded;
        }
        CHECK(jpsExpanded < astarExpanded);
    }
    return report("jps_test");
}
