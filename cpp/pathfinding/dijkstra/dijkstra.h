#pragma once

#include <algorithm>
#include <functional>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

#include "../bfs/bfs.h"  // reconstructPath
#include "cost_grid.h"

namespace ga {

constexpr int kUnreached = std::numeric_limits<int>::max();

// #region path-cost
// 一条路径的总代价：起点不计，之后每走进一个格子加上它的代价
inline int pathCost(const CostGrid& grid, const std::vector<Cell>& path) {
    int total = 0;
    for (size_t i = 1; i < path.size(); ++i) total += grid.cost(path[i]);
    return total;
}
// #endregion

// #region dijkstra-path
// Dijkstra：返回从 start 到 goal 总代价最小的路径，不可达时返回空
inline std::vector<Cell> dijkstraPath(const CostGrid& grid, Cell start, Cell goal) {
    if (!grid.passable(start) || !grid.passable(goal)) return {};

    std::vector<int> costSoFar(grid.size(), kUnreached);  // 目前已知的最小代价
    std::vector<int> cameFrom(grid.size(), -1);

    // 小顶堆：每次取出"已知代价最小"的格子。条目是 (代价, 格子下标)
    using Entry = std::pair<int, int>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> frontier;

    int s = grid.index(start);
    costSoFar[s] = 0;
    cameFrom[s] = s;
    frontier.push({0, s});

    while (!frontier.empty()) {
        auto [cost, ci] = frontier.top();
        frontier.pop();
        if (cost > costSoFar[ci]) continue;  // 过期条目：这个格子后来找到了更便宜的路

        Cell current = grid.cellAt(ci);
        if (current == goal) break;  // 终点出堆时，它的代价就是最终答案

        grid.forEachNeighbor(current, [&](Cell next) {
            int ni = grid.index(next);
            int newCost = cost + grid.cost(next);
            if (newCost < costSoFar[ni]) {  // 松弛：找到了一条更便宜的路
                costSoFar[ni] = newCost;
                cameFrom[ni] = ci;
                frontier.push({newCost, ni});
            }
        });
    }
    return reconstructPath(grid, cameFrom, start, goal);
}
// #endregion

// #region dijkstra-map
// Dijkstra 地图：从一个或多个源点出发跑完整张图，得到每个格子到"最近源点"的代价。
// 不可达的格子为 kUnreached。
inline std::vector<int> dijkstraMap(const CostGrid& grid, const std::vector<Cell>& sources) {
    std::vector<int> costSoFar(grid.size(), kUnreached);
    using Entry = std::pair<int, int>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> frontier;

    for (Cell c : sources) {
        if (!grid.passable(c)) continue;
        costSoFar[grid.index(c)] = 0;
        frontier.push({0, grid.index(c)});
    }
    while (!frontier.empty()) {
        auto [cost, ci] = frontier.top();
        frontier.pop();
        if (cost > costSoFar[ci]) continue;
        grid.forEachNeighbor(grid.cellAt(ci), [&](Cell next) {
            int ni = grid.index(next);
            int newCost = cost + grid.cost(next);
            if (newCost < costSoFar[ni]) {
                costSoFar[ni] = newCost;
                frontier.push({newCost, ni});
            }
        });
    }
    return costSoFar;
}

// 沿着 Dijkstra 地图"下坡"走一步：选择代价最小的邻居。已经在源点或无路可走时返回 from
inline Cell rollDownhill(const CostGrid& grid, const std::vector<int>& map, Cell from) {
    Cell best = from;
    grid.forEachNeighbor(from, [&](Cell next) {
        if (map[grid.index(next)] < map[grid.index(best)]) best = next;
    });
    return best;
}
// #endregion

// #region dijkstra-stepper
// 可单步执行的 Dijkstra，逻辑与 dijkstraPath 一致。
// 为了能遍历边界画出来，用 vector + std::push_heap / pop_heap 代替 priority_queue（二者等价）。
class DijkstraStepper {
public:
    DijkstraStepper(const CostGrid& grid, Cell start, Cell goal)
        : grid_(&grid), start_(start), goal_(goal),
          costSoFar_(grid.size(), kUnreached), cameFrom_(grid.size(), -1), closed_(grid.size(), 0) {
        if (grid.passable(start) && grid.passable(goal)) {
            int s = grid.index(start);
            costSoFar_[s] = 0;
            cameFrom_[s] = s;
            push({0, s});
        }
    }

    // 执行一步：取出代价最小的格子并松弛它的邻居。搜索结束时返回 false
    bool step() {
        while (!heap_.empty()) {
            Entry top = pop();
            if (top.first > costSoFar_[top.second]) continue;  // 跳过过期条目，不算一步

            int ci = top.second;
            current_ = grid_->cellAt(ci);
            hasCurrent_ = true;
            closed_[ci] = 1;
            ++expanded_;
            if (current_ == goal_) {
                found_ = true;
                return false;
            }
            grid_->forEachNeighbor(current_, [&](Cell next) {
                int ni = grid_->index(next);
                int newCost = top.first + grid_->cost(next);
                if (newCost < costSoFar_[ni]) {
                    if (costSoFar_[ni] != kUnreached) ++relaxed_;  // 已在边界上的格子被改得更便宜
                    costSoFar_[ni] = newCost;
                    cameFrom_[ni] = ci;
                    push({newCost, ni});
                }
            });
            return !done();
        }
        return false;
    }

    bool done() const { return found_ || liveFrontierSize() == 0; }
    bool found() const { return found_; }
    bool hasCurrent() const { return hasCurrent_; }
    Cell current() const { return current_; }
    int expandedCount() const { return expanded_; }
    int relaxedCount() const { return relaxed_; }
    int cost(Cell c) const { return costSoFar_[grid_->index(c)]; }  // kUnreached = 未发现
    bool closed(Cell c) const { return closed_[grid_->index(c)] != 0; }

    // 边界上仍然有效的格子（过滤掉过期条目）
    template <class F>
    void forEachFrontier(F&& visit) const {
        for (const Entry& e : heap_) {
            if (e.first == costSoFar_[e.second] && !closed_[e.second]) visit(grid_->cellAt(e.second));
        }
    }
    int liveFrontierSize() const {
        int n = 0;
        forEachFrontier([&](Cell) { ++n; });
        return n;
    }

    std::vector<Cell> path() const {
        return found_ ? reconstructPath(*grid_, cameFrom_, start_, goal_) : std::vector<Cell>{};
    }

private:
    using Entry = std::pair<int, int>;
    void push(Entry e) {
        heap_.push_back(e);
        std::push_heap(heap_.begin(), heap_.end(), std::greater<Entry>{});
    }
    Entry pop() {
        std::pop_heap(heap_.begin(), heap_.end(), std::greater<Entry>{});
        Entry e = heap_.back();
        heap_.pop_back();
        return e;
    }

    const CostGrid* grid_;
    Cell start_, goal_;
    std::vector<int> costSoFar_;
    std::vector<int> cameFrom_;
    std::vector<char> closed_;
    std::vector<Entry> heap_;
    Cell current_{};
    bool hasCurrent_ = false;
    bool found_ = false;
    int expanded_ = 0;
    int relaxed_ = 0;
};
// #endregion

}  // namespace ga
