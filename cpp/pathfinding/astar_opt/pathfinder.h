#pragma once

#include <algorithm>
#include <cstdint>
#include <vector>

#include "../astar/astar.h"  // heuristic, reconstructPath, pathCost

namespace ga {

// #region pathfinder
// 可以反复使用的 A* 寻路器：
//   - 所有数组只在构造时分配一次，之后每次寻路复用
//   - 用"代数"（generation）标记数组内容是否属于本次搜索，省去每次 O(V) 的清零
//   - 堆用 vector + std::push_heap / pop_heap，惰性删除；vector 的容量也跨搜索复用
//   - 支持启发权重 weight：f = g + weight × h（用 1/16 精度的定点数，堆里全是整数比较）
class Pathfinder {
public:
    explicit Pathfinder(const CostGrid& grid)
        : grid_(&grid), seen_(grid.size(), 0), closed_(grid.size(), 0),
          g_(grid.size(), 0), cameFrom_(grid.size(), -1) {}

    std::vector<Cell> find(Cell start, Cell goal, float weight = 1.0f) {
        begin(start, goal, weight);
        while (step()) {}
        return path();
    }
    // #endregion

    // #region begin
    void begin(Cell start, Cell goal, float weight = 1.0f) {
        start_ = start;
        goal_ = goal;
        weight16_ = static_cast<int>(weight * 16.0f + 0.5f);
        found_ = false;
        expanded_ = 0;
        open_.clear();  // 只清空元素，保留已分配的容量
        if (++generation_ == 0) {  // 32 位计数器回绕（约 40 亿次搜索后）：真正清零一次
            std::fill(seen_.begin(), seen_.end(), 0);
            std::fill(closed_.begin(), closed_.end(), 0);
            generation_ = 1;
        }
        if (!grid_->passable(start) || !grid_->passable(goal)) return;
        int s = grid_->index(start);
        visit(s, 0, s);
        push(start, s, 0);
    }
    // #endregion

    // #region step
    // 展开一个格子。搜索结束（找到或不可达）时返回 false
    bool step() {
        while (!found_ && !open_.empty()) {
            std::pop_heap(open_.begin(), open_.end(), Worse{});
            Entry top = open_.back();
            open_.pop_back();
            if (top.g > g_[top.index]) continue;  // 过期条目：这个格子后来找到了更便宜的路

            closed_[top.index] = generation_;  // 只用于演示显示，搜索本身不读它
            ++expanded_;
            Cell current = grid_->cellAt(top.index);
            if (current == goal_) {
                found_ = true;
                return false;
            }
            grid_->forEachNeighbor(current, [&](Cell next) {
                int ni = grid_->index(next);
                int g = top.g + grid_->cost(next);
                if (seen_[ni] != generation_ || g < g_[ni]) {
                    visit(ni, g, top.index);
                    push(next, ni, g);
                }
            });
            return true;
        }
        return false;
    }
    // #endregion

    bool found() const { return found_; }
    bool done() const { return found_ || open_.empty(); }
    int expandedCount() const { return expanded_; }
    bool closed(Cell c) const { return closed_[grid_->index(c)] == generation_; }
    bool seen(Cell c) const { return seen_[grid_->index(c)] == generation_; }
    int g(Cell c) const { return g_[grid_->index(c)]; }

    // 边界上仍然有效的格子（每个格子只访问一次）
    template <class F>
    void forEachOpen(F&& visit) const {
        for (const Entry& e : open_) {
            if (closed_[e.index] != generation_ && e.g == g_[e.index]) visit(grid_->cellAt(e.index));
        }
    }

    std::vector<Cell> path() const {
        if (!found_) return {};
        std::vector<Cell> p;
        for (int i = grid_->index(goal_);; i = cameFrom_[i]) {
            p.push_back(grid_->cellAt(i));
            if (i == grid_->index(start_)) break;
        }
        return {p.rbegin(), p.rend()};
    }

private:
    struct Entry {
        int f;  // 16 × (g + weight × h)
        int g;
        int index;
    };
    // 堆的排序：f 小的优先；f 相同时 g 大的优先（离终点更近）
    struct Worse {
        bool operator()(const Entry& a, const Entry& b) const { return a.f != b.f ? a.f > b.f : a.g < b.g; }
    };
    void push(Cell c, int index, int g) {
        open_.push_back({16 * g + weight16_ * heuristic(c, goal_), g, index});
        std::push_heap(open_.begin(), open_.end(), Worse{});
    }
    void visit(int i, int g, int from) {
        seen_[i] = generation_;
        g_[i] = g;
        cameFrom_[i] = from;
    }

    const CostGrid* grid_;
    std::vector<uint32_t> seen_;    // seen_[i] == generation_：本次搜索中已发现
    std::vector<uint32_t> closed_;  // closed_[i] == generation_：本次搜索中已展开（演示用）
    std::vector<int> g_;
    std::vector<int> cameFrom_;
    std::vector<Entry> open_;
    uint32_t generation_ = 0;
    Cell start_{}, goal_{};
    int weight16_ = 16;  // weight × 16
    bool found_ = false;
    int expanded_ = 0;
};

}  // namespace ga
