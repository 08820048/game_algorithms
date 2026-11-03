#pragma once

#include <algorithm>
#include <cstdlib>
#include <queue>
#include <vector>

#include "../dijkstra/dijkstra.h"  // reconstructPath, pathCost, kUnreached

namespace ga {

// #region heuristic
// 曼哈顿距离：4 邻域网格上，从 a 走到 b 至少需要的步数
inline int manhattan(Cell a, Cell b) { return std::abs(a.x - b.x) + std::abs(a.y - b.y); }

// 启发函数：对"从 c 到终点还要花多少"的估计。
// 每一步的代价至少是 1（平地），所以"至少的步数 × 1"永远不会高估真实代价。
inline int heuristic(Cell c, Cell goal) { return manhattan(c, goal) * 1; }
// #endregion

// #region astar-path
// A*：在 Dijkstra 的基础上，按 f = g + h 排序。g 是已经花掉的代价，h 是对剩余代价的估计
inline std::vector<Cell> astarPath(const CostGrid& grid, Cell start, Cell goal) {
    if (!grid.passable(start) || !grid.passable(goal)) return {};

    std::vector<int> costSoFar(grid.size(), kUnreached);  // g
    std::vector<int> cameFrom(grid.size(), -1);

    struct Entry {
        int f;
        int g;
        int index;
    };
    // 小顶堆按 f 排序；f 相同时优先 g 大的，也就是离终点更近的
    auto worse = [](const Entry& a, const Entry& b) { return a.f != b.f ? a.f > b.f : a.g < b.g; };
    std::priority_queue<Entry, std::vector<Entry>, decltype(worse)> frontier(worse);

    int s = grid.index(start);
    costSoFar[s] = 0;
    cameFrom[s] = s;
    frontier.push({heuristic(start, goal), 0, s});

    while (!frontier.empty()) {
        Entry top = frontier.top();
        frontier.pop();
        if (top.g > costSoFar[top.index]) continue;  // 过期条目

        Cell current = grid.cellAt(top.index);
        if (current == goal) break;

        grid.forEachNeighbor(current, [&](Cell next) {
            int ni = grid.index(next);
            int g = top.g + grid.cost(next);
            if (g < costSoFar[ni]) {
                costSoFar[ni] = g;
                cameFrom[ni] = top.index;
                frontier.push({g + heuristic(next, goal), g, ni});  // 唯一的区别：排序时加上 h
            }
        });
    }
    return reconstructPath(grid, cameFrom, start, goal);
}
// #endregion

// #region best-first-stepper
// 三种"最佳优先"搜索共用一份代码，区别只在于堆按什么排序：
//   Dijkstra：g        只看已经花掉的
//   Greedy  ：h        只看离终点还有多远
//   A*      ：g + h    两者都看
enum class SearchMode { Dijkstra, Greedy, AStar };

class BestFirstStepper {
public:
    BestFirstStepper(const CostGrid& grid, Cell start, Cell goal, SearchMode mode)
        : grid_(&grid), start_(start), goal_(goal), mode_(mode),
          costSoFar_(grid.size(), kUnreached), cameFrom_(grid.size(), -1), closed_(grid.size(), 0) {
        if (grid.passable(start) && grid.passable(goal)) {
            int s = grid.index(start);
            costSoFar_[s] = 0;
            cameFrom_[s] = s;
            push(s, 0);
        }
    }

    int priority(Cell c, int g) const {
        switch (mode_) {
            case SearchMode::Dijkstra: return g;
            case SearchMode::Greedy: return heuristic(c, goal_);
            case SearchMode::AStar: return g + heuristic(c, goal_);
        }
        return g;
    }

    bool step() {
        while (!heap_.empty()) {
            std::pop_heap(heap_.begin(), heap_.end(), Worse{});
            Entry top = heap_.back();
            heap_.pop_back();
            if (top.g > costSoFar_[top.index]) continue;

            current_ = grid_->cellAt(top.index);
            hasCurrent_ = true;
            closed_[top.index] = 1;
            ++expanded_;
            if (current_ == goal_) {
                found_ = true;
                return false;
            }
            grid_->forEachNeighbor(current_, [&](Cell next) {
                int ni = grid_->index(next);
                int g = top.g + grid_->cost(next);
                if (g < costSoFar_[ni]) {
                    costSoFar_[ni] = g;
                    cameFrom_[ni] = top.index;
                    closed_[ni] = 0;  // Greedy 可能找到已展开格子的更短路，需要重新打开
                    push(ni, g);
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
    int cost(Cell c) const { return costSoFar_[grid_->index(c)]; }
    bool closed(Cell c) const { return closed_[grid_->index(c)] != 0; }

    template <class F>
    void forEachFrontier(F&& visit) const {
        for (const Entry& e : heap_) {
            if (e.g == costSoFar_[e.index] && !closed_[e.index]) visit(grid_->cellAt(e.index));
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
    struct Entry {
        int f;
        int g;
        int index;
    };
    struct Worse {
        bool operator()(const Entry& a, const Entry& b) const { return a.f != b.f ? a.f > b.f : a.g < b.g; }
    };
    void push(int index, int g) {
        heap_.push_back({priority(grid_->cellAt(index), g), g, index});
        std::push_heap(heap_.begin(), heap_.end(), Worse{});
    }

    const CostGrid* grid_;
    Cell start_, goal_;
    SearchMode mode_;
    std::vector<int> costSoFar_;
    std::vector<int> cameFrom_;
    std::vector<char> closed_;
    std::vector<Entry> heap_;
    Cell current_{};
    bool hasCurrent_ = false;
    bool found_ = false;
    int expanded_ = 0;
};
// #endregion

}  // namespace ga
