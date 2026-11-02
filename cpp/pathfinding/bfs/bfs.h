#pragma once

#include <algorithm>
#include <deque>
#include <vector>

#include "grid.h"

namespace ga {

// #region reconstruct-path
// 从终点沿着 cameFrom 一路回溯到起点，再反转得到正向路径
inline std::vector<Cell> reconstructPath(const Grid& grid, const std::vector<int>& cameFrom,
                                         Cell start, Cell goal) {
    if (cameFrom[grid.index(goal)] == -1) return {};  // 终点从未被访问：不可达

    std::vector<Cell> path;
    for (Cell c = goal; !(c == start); c = grid.cellAt(cameFrom[grid.index(c)])) {
        path.push_back(c);
    }
    path.push_back(start);
    std::reverse(path.begin(), path.end());
    return path;
}
// #endregion

// #region bfs-path
// 广度优先搜索：返回从 start 到 goal 的最短路径（按步数计），不可达时返回空
inline std::vector<Cell> bfsPath(const Grid& grid, Cell start, Cell goal) {
    if (!grid.passable(start) || !grid.passable(goal)) return {};

    // cameFrom[i] 记录"我是从哪个格子走过来的"，-1 表示还没访问过
    std::vector<int> cameFrom(grid.size(), -1);
    cameFrom[grid.index(start)] = grid.index(start);

    std::deque<Cell> frontier{start};  // 先进先出队列：边界
    while (!frontier.empty()) {
        Cell current = frontier.front();
        frontier.pop_front();
        if (current == goal) break;  // 提前退出：找到终点就不必继续扩散

        grid.forEachNeighbor(current, [&](Cell next) {
            int ni = grid.index(next);
            if (cameFrom[ni] == -1) {  // 每个格子只入队一次
                cameFrom[ni] = grid.index(current);
                frontier.push_back(next);
            }
        });
    }
    return reconstructPath(grid, cameFrom, start, goal);
}
// #endregion

// #region bfs-stepper
// 可单步执行的 BFS：逻辑与 bfsPath 完全一致，只是把 while 循环拆成一次一次的 step()，
// 并额外记录距离，方便可视化。
class BfsStepper {
public:
    BfsStepper(const Grid& grid, Cell start, Cell goal)
        : grid_(&grid), start_(start), goal_(goal),
          cameFrom_(grid.size(), -1), distance_(grid.size(), -1) {
        if (grid.passable(start) && grid.passable(goal)) {
            cameFrom_[grid.index(start)] = grid.index(start);
            distance_[grid.index(start)] = 0;
            frontier_.push_back(start);
        }
    }

    // 执行一步：从队首取出一个格子并展开它的邻居。搜索结束时返回 false
    bool step() {
        if (done()) return false;
        current_ = frontier_.front();
        frontier_.pop_front();
        hasCurrent_ = true;
        ++expanded_;

        if (current_ == goal_) {
            found_ = true;
            return false;
        }
        int ci = grid_->index(current_);
        grid_->forEachNeighbor(current_, [&](Cell next) {
            int ni = grid_->index(next);
            if (cameFrom_[ni] == -1) {
                cameFrom_[ni] = ci;
                distance_[ni] = distance_[ci] + 1;
                frontier_.push_back(next);
            }
        });
        return !done();
    }

    bool done() const { return found_ || frontier_.empty(); }
    bool found() const { return found_; }
    bool hasCurrent() const { return hasCurrent_; }
    Cell current() const { return current_; }
    int expandedCount() const { return expanded_; }
    const std::deque<Cell>& frontier() const { return frontier_; }
    int distance(Cell c) const { return distance_[grid_->index(c)]; }  // -1 = 未访问
    std::vector<Cell> path() const {
        return found_ ? reconstructPath(*grid_, cameFrom_, start_, goal_) : std::vector<Cell>{};
    }

private:
    const Grid* grid_;
    Cell start_, goal_;
    std::vector<int> cameFrom_;
    std::vector<int> distance_;
    std::deque<Cell> frontier_;
    Cell current_{};
    bool hasCurrent_ = false;
    bool found_ = false;
    int expanded_ = 0;
};
// #endregion

}  // namespace ga
