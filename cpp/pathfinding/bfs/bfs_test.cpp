#include "bfs.h"

#include <cstdio>
#include <cstdlib>

using namespace ga;

static int failures = 0;

#define CHECK(expr)                                                         \
    do {                                                                    \
        if (!(expr)) {                                                      \
            std::fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #expr); \
            ++failures;                                                     \
        }                                                                   \
    } while (0)

static std::vector<Cell> runStepper(const Grid& grid, Cell start, Cell goal) {
    BfsStepper s(grid, start, goal);
    while (s.step()) {}
    return s.path();
}

int main() {
    // 空地图：最短路径长度 = 曼哈顿距离 + 1（包含起点）
    {
        Grid g(10, 6);
        auto path = bfsPath(g, {0, 0}, {9, 5});
        CHECK(path.size() == 9 + 5 + 1);
        CHECK(path.front() == (Cell{0, 0}));
        CHECK(path.back() == (Cell{9, 5}));
        // 相邻两格必须是 4 邻接
        for (size_t i = 1; i < path.size(); ++i) {
            int d = std::abs(path[i].x - path[i - 1].x) + std::abs(path[i].y - path[i - 1].y);
            CHECK(d == 1);
        }
    }
    // 起点等于终点
    {
        Grid g(3, 3);
        auto path = bfsPath(g, {1, 1}, {1, 1});
        CHECK(path.size() == 1);
    }
    // 被墙完全隔开：不可达
    {
        Grid g(5, 5);
        for (int y = 0; y < 5; ++y) g.setWall({2, y}, true);
        CHECK(bfsPath(g, {0, 0}, {4, 4}).empty());
        CHECK(runStepper(g, {0, 0}, {4, 4}).empty());
    }
    // 需要绕路：墙上留一个缺口
    {
        Grid g(5, 5);
        for (int y = 0; y < 4; ++y) g.setWall({2, y}, true);  // 缺口在 (2,4)
        auto path = bfsPath(g, {0, 0}, {4, 0});
        CHECK(path.size() == 13);  // 下 4、右 4、上 4，再加起点
    }
    // 起点或终点在墙里
    {
        Grid g(4, 4);
        g.setWall({3, 3}, true);
        CHECK(bfsPath(g, {0, 0}, {3, 3}).empty());
    }
    // 单步版本与一次性版本结果一致
    {
        Grid g(12, 8);
        for (int x = 1; x < 11; ++x) g.setWall({x, 4}, true);
        g.setWall({6, 1}, true);
        auto a = bfsPath(g, {0, 0}, {11, 7});
        auto b = runStepper(g, {0, 0}, {11, 7});
        CHECK(a == b);
        CHECK(!a.empty());
    }

    if (failures == 0) std::puts("bfs_test: all checks passed");
    return failures == 0 ? 0 : 1;
}
