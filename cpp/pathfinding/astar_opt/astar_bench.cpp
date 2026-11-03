// A* 性能对比：256×256 随机地图（20% 墙、15% 森林、7% 沼泽）
//   场景 long ：随机起终点，平均要展开几千个格子
//   场景 short：起终点曼哈顿距离不超过 16，模拟"走到附近"这类大量的短寻路
// 对比：
//   astarPath (1.3)       每次寻路分配并清零数组，priority_queue + 惰性删除
//   reuse + indexed heap  复用数组，带索引的二叉堆，松弛时就地降低优先级
//   Pathfinder            复用数组，vector 堆 + 惰性删除（本章推荐的写法）
// 最后测试不同启发权重的展开数与路径代价。
//
// 用法（在 cpp/ 目录下）：
//   c++ -O2 -std=c++20 -I common pathfinding/astar_opt/astar_bench.cpp -o astar_bench && ./astar_bench

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "../../foundations/random/random.h"
#include "binary_heap.h"
#include "pathfinder.h"

using namespace ga;
using Clock = std::chrono::steady_clock;

// 对照组：复用数组 + 带索引的二叉堆（decrease-key）
class IndexedPathfinder {
public:
    explicit IndexedPathfinder(const CostGrid& g)
        : grid_(&g), seen_(g.size(), 0), closed_(g.size(), 0), g_(g.size()), from_(g.size()) {
        open_.reset(g.size());
    }
    std::vector<Cell> find(Cell start, Cell goal) {
        ++gen_;
        open_.clear();
        int s = grid_->index(start);
        seen_[s] = gen_;
        g_[s] = 0;
        from_[s] = s;
        open_.push(s, {heuristic(start, goal), 0});
        bool found = false;
        while (!open_.empty()) {
            int ci = open_.pop();
            closed_[ci] = gen_;
            Cell c = grid_->cellAt(ci);
            if (c == goal) {
                found = true;
                break;
            }
            grid_->forEachNeighbor(c, [&](Cell n) {
                int ni = grid_->index(n);
                if (closed_[ni] == gen_) return;
                int ng = g_[ci] + grid_->cost(n);
                Key k{ng + heuristic(n, goal), ng};
                if (seen_[ni] != gen_) {
                    seen_[ni] = gen_;
                    g_[ni] = ng;
                    from_[ni] = ci;
                    open_.push(ni, k);
                } else if (ng < g_[ni]) {
                    g_[ni] = ng;
                    from_[ni] = ci;
                    open_.decreaseKey(ni, k);
                }
            });
        }
        std::vector<Cell> p;
        if (!found) return p;
        for (int i = grid_->index(goal);; i = from_[i]) {
            p.push_back(grid_->cellAt(i));
            if (i == s) break;
        }
        return {p.rbegin(), p.rend()};
    }

private:
    struct Key {
        int f, g;
        bool operator<(const Key& o) const { return f != o.f ? f < o.f : g > o.g; }
    };
    const CostGrid* grid_;
    std::vector<uint32_t> seen_, closed_;
    std::vector<int> g_, from_;
    IndexedMinHeap<Key> open_;
    uint32_t gen_ = 0;
};

// 预热一次，再取 7 次中最快的一次，减少系统噪声的影响。返回每次寻路的平均微秒数
template <class F>
double microsPerQuery(size_t queries, F&& f) {
    f();
    double best = 1e30;
    for (int run = 0; run < 7; ++run) {
        auto t0 = Clock::now();
        f();
        best = std::min(best, std::chrono::duration<double, std::micro>(Clock::now() - t0).count());
    }
    return best / static_cast<double>(queries);
}

using Queries = std::vector<std::pair<Cell, Cell>>;

static Queries makeQueries(const CostGrid& grid, Pcg32& rng, int count, int maxDistance) {
    Queries q;
    const int n = grid.width();
    while (static_cast<int>(q.size()) < count) {
        Cell a{static_cast<int>(rng.below(n)), static_cast<int>(rng.below(n))};
        Cell b{static_cast<int>(rng.below(n)), static_cast<int>(rng.below(n))};
        if (maxDistance > 0) {
            b = {a.x + static_cast<int>(rng.below(2 * maxDistance + 1)) - maxDistance,
                 a.y + static_cast<int>(rng.below(2 * maxDistance + 1)) - maxDistance};
            if (manhattan(a, b) > maxDistance || !grid.inBounds(b)) continue;
        }
        if (grid.passable(a) && grid.passable(b) && !astarPath(grid, a, b).empty()) q.push_back({a, b});
    }
    return q;
}

int main() {
    constexpr int kSize = 256;
    Pcg32 rng(42);
    CostGrid grid(kSize, kSize);
    for (int i = 0; i < grid.size(); ++i) {
        uint32_t r = rng.below(100);
        if (r < 20) grid.setWall(grid.cellAt(i), true);
        else if (r < 35) grid.setCost(grid.cellAt(i), 3);
        else if (r < 42) grid.setCost(grid.cellAt(i), 6);
    }

    long sink = 0;
    IndexedPathfinder indexed(grid);
    Pathfinder pf(grid);
    const std::pair<const char*, Queries> scenarios[] = {{"long", makeQueries(grid, rng, 400, 0)},
                                                         {"short", makeQueries(grid, rng, 4000, 16)}};
    for (const auto& [name, queries] : scenarios) {
        long expanded = 0;
        for (auto [a, b] : queries) {
            pf.find(a, b);
            expanded += pf.expandedCount();
        }
        std::printf("[%s] %zu queries, avg expanded %.0f\n", name, queries.size(),
                    static_cast<double>(expanded) / static_cast<double>(queries.size()));
        double t1 = microsPerQuery(queries.size(), [&] {
            for (auto [a, b] : queries) sink += static_cast<long>(astarPath(grid, a, b).size());
        });
        double t2 = microsPerQuery(queries.size(), [&] {
            for (auto [a, b] : queries) sink += static_cast<long>(indexed.find(a, b).size());
        });
        double t3 = microsPerQuery(queries.size(), [&] {
            for (auto [a, b] : queries) sink += static_cast<long>(pf.find(a, b).size());
        });
        std::printf("  %-22s %9.1f us/query\n", "astarPath (1.3)", t1);
        std::printf("  %-22s %9.1f us/query\n", "reuse + indexed heap", t2);
        std::printf("  %-22s %9.1f us/query\n", "Pathfinder", t3);
    }

    // 启发权重：长距离寻路
    const Queries& queries = scenarios[0].second;
    std::vector<int> optimal;
    for (auto [a, b] : queries) optimal.push_back(pathCost(grid, astarPath(grid, a, b)));
    std::printf("[weight] %zu long queries\n", queries.size());
    for (float w : {1.0f, 1.2f, 1.5f, 2.0f, 3.0f}) {
        long expanded = 0;
        double ratio = 0, worst = 1;
        for (size_t i = 0; i < queries.size(); ++i) {
            auto p = pf.find(queries[i].first, queries[i].second, w);
            expanded += pf.expandedCount();
            double r = static_cast<double>(pathCost(grid, p)) / optimal[i];
            ratio += r;
            worst = std::max(worst, r);
        }
        double t = microsPerQuery(queries.size(), [&] {
            for (auto [a, b] : queries) sink += static_cast<long>(pf.find(a, b, w).size());
        });
        std::printf("  w=%.1f  %9.1f us/query  expanded %6.0f  cost x%.3f avg, x%.3f worst\n", w, t,
                    static_cast<double>(expanded) / static_cast<double>(queries.size()),
                    ratio / static_cast<double>(queries.size()), worst);
    }
    return sink == 0;
}
