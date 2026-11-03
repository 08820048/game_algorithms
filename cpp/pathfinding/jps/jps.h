#pragma once

#include <algorithm>
#include <cstdlib>
#include <vector>

#include "grid.h"

namespace ga {

// #region grid8
// 8 方向移动：直走代价 10，斜走代价 14（≈ 10√2，用整数避免浮点误差）
constexpr int kStraight = 10;
constexpr int kDiagonal = 14;

// 八方向距离（octile）：先斜走 min(dx, dy) 步，再直走剩下的
inline int octile(Cell a, Cell b) {
    int dx = std::abs(a.x - b.x), dy = std::abs(a.y - b.y);
    return kStraight * (dx + dy) + (kDiagonal - 2 * kStraight) * std::min(dx, dy);
}

// 从 c 朝 (dx, dy) 走一步是否合法。斜走时不允许切墙角：两侧的直走格子都必须可通行
inline bool canMove(const Grid& grid, Cell c, int dx, int dy) {
    if (!grid.passable({c.x + dx, c.y + dy})) return false;
    if (dx != 0 && dy != 0) return grid.passable({c.x + dx, c.y}) && grid.passable({c.x, c.y + dy});
    return true;
}

constexpr Cell kDirs8[8] = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}, {1, 1}, {-1, 1}, {-1, -1}, {1, -1}};
// #endregion

// 在 8 方向网格上搜索。A* 与 JPS 共用同一个主循环，唯一的区别是"后继节点"从哪里来：
//   A*  ：8 个相邻格子
//   JPS ：沿着每个有意义的方向"跳"，跳到的跳点才是后继
enum class Grid8Mode { AStar, Jps };

class Grid8Search {
public:
    Grid8Search(const Grid& grid, Cell start, Cell goal, Grid8Mode mode)
        : grid_(&grid), start_(start), goal_(goal), mode_(mode),
          g_(grid.size(), -1), cameFrom_(grid.size(), -1), closed_(grid.size(), 0), scanned_(grid.size(), 0) {
        if (grid.passable(start) && grid.passable(goal)) {
            int s = grid.index(start);
            g_[s] = 0;
            cameFrom_[s] = s;
            push(start, 0);
        }
    }

    // #region search
    // 展开一个节点。搜索结束时返回 false
    bool step() {
        while (!found_ && !open_.empty()) {
            std::pop_heap(open_.begin(), open_.end(), Worse{});
            Entry top = open_.back();
            open_.pop_back();
            int ci = grid_->index(top.cell);
            if (closed_[ci]) continue;  // 过期条目

            closed_[ci] = 1;
            current_ = top.cell;
            hasCurrent_ = true;
            ++expanded_;
            if (top.cell == goal_) {
                found_ = true;
                return false;
            }
            auto relax = [&](Cell next) {
                int ni = grid_->index(next);
                if (closed_[ni]) return;
                int g = top.g + octile(top.cell, next);  // 跳点可能很远，代价按八方向距离算
                if (g_[ni] < 0 || g < g_[ni]) {
                    g_[ni] = g;
                    cameFrom_[ni] = ci;
                    push(next, g);
                }
            };
            if (mode_ == Grid8Mode::AStar) {
                for (Cell d : kDirs8) {
                    if (canMove(*grid_, top.cell, d.x, d.y)) relax({top.cell.x + d.x, top.cell.y + d.y});
                }
            } else {
                forEachJumpPoint(top.cell, relax);
            }
            return true;
        }
        return false;
    }
    // #endregion

    // #region successors
    // JPS 的后继：先按"从哪个方向来"剪枝出值得探索的方向，再沿每个方向跳到下一个跳点
    template <class F>
    void forEachJumpPoint(Cell c, F&& visit) {
        Cell dirs[8];
        int n = prunedDirections(c, dirs);
        for (int i = 0; i < n; ++i) {
            Cell jp;
            if (jump({c.x + dirs[i].x, c.y + dirs[i].y}, dirs[i].x, dirs[i].y, &jp)) visit(jp);
        }
    }
    // #endregion

    // #region prune
    // 剪枝：只保留"经过 c 才更划算"的方向。起点没有来向，保留全部 8 个方向
    int prunedDirections(Cell c, Cell* out) const {
        int n = 0;
        auto add = [&](int dx, int dy) {
            if (canMove(*grid_, c, dx, dy)) out[n++] = {dx, dy};
        };
        int ci = grid_->index(c);
        if (cameFrom_[ci] == ci) {
            for (Cell d : kDirs8) add(d.x, d.y);
            return n;
        }
        Cell p = grid_->cellAt(cameFrom_[ci]);
        int dx = (c.x > p.x) - (c.x < p.x);  // 来向，取符号
        int dy = (c.y > p.y) - (c.y < p.y);
        if (dx != 0 && dy != 0) {
            // 斜着来：继续斜走，以及它的两个直走分量
            add(dx, 0);
            add(0, dy);
            add(dx, dy);
        } else if (dx != 0) {
            // 横着来：继续向前，以及两侧（墙角消失后可能出现的"强迫邻居"）
            add(dx, 0);
            add(0, 1);
            add(0, -1);
            add(dx, 1);
            add(dx, -1);
        } else {
            add(0, dy);
            add(1, 0);
            add(-1, 0);
            add(1, dy);
            add(-1, dy);
        }
        return n;
    }
    // #endregion

    // #region jump
    // 从 c 开始沿 (dx, dy) 一直走，直到遇到一个"必须停下来考虑"的格子（跳点）。
    // 找到时写入 *out 并返回 true；撞墙或走出地图返回 false。
    bool jump(Cell c, int dx, int dy, Cell* out) {
        for (;;) {
            if (!grid_->passable(c)) return false;
            scanned_[grid_->index(c)] = 1;  // 仅用于统计和可视化
            ++scanCount_;
            if (c == goal_) {
                *out = c;
                return true;
            }
            if (dx != 0 && dy != 0) {
                // 斜走：每走一步，都先沿两个直走分量各跳一次，任何一个能找到跳点，c 就是跳点
                Cell unused;
                if (jump({c.x + dx, c.y}, dx, 0, &unused) || jump({c.x, c.y + dy}, 0, dy, &unused)) {
                    *out = c;
                    return true;
                }
                if (!canMove(*grid_, c, dx, dy)) return false;
            } else if (dx != 0) {
                // 横走：上方或下方的墙刚好在身后结束，说明从这里拐弯可能更短 —— 强迫邻居
                if ((grid_->passable({c.x, c.y - 1}) && !grid_->passable({c.x - dx, c.y - 1})) ||
                    (grid_->passable({c.x, c.y + 1}) && !grid_->passable({c.x - dx, c.y + 1}))) {
                    *out = c;
                    return true;
                }
            } else {
                if ((grid_->passable({c.x - 1, c.y}) && !grid_->passable({c.x - 1, c.y - dy})) ||
                    (grid_->passable({c.x + 1, c.y}) && !grid_->passable({c.x + 1, c.y - dy}))) {
                    *out = c;
                    return true;
                }
            }
            c = {c.x + dx, c.y + dy};
        }
    }
    // #endregion

    bool found() const { return found_; }
    bool done() const { return found_ || liveOpenCount() == 0; }
    bool hasCurrent() const { return hasCurrent_; }
    Cell current() const { return current_; }
    int expandedCount() const { return expanded_; }
    int scanCount() const { return scanCount_; }
    bool closed(Cell c) const { return closed_[grid_->index(c)] != 0; }
    bool scanned(Cell c) const { return scanned_[grid_->index(c)] != 0; }
    int g(Cell c) const { return g_[grid_->index(c)]; }
    // 父节点（跳点之间的连线）；没有父节点时返回 false
    bool parent(Cell c, Cell* out) const {
        int i = grid_->index(c), p = cameFrom_[i];
        if (p < 0 || p == i) return false;
        *out = grid_->cellAt(p);
        return true;
    }

    template <class F>
    void forEachOpen(F&& visit) const {
        for (const Entry& e : open_) {
            int i = grid_->index(e.cell);
            if (!closed_[i] && e.g == g_[i]) visit(e.cell);
        }
    }
    int liveOpenCount() const {
        int n = 0;
        forEachOpen([&](Cell) { ++n; });
        return n;
    }

    // 节点序列：A* 中是每一个格子，JPS 中只有跳点
    std::vector<Cell> nodes() const {
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
        int f;
        int g;
        Cell cell;
    };
    struct Worse {
        bool operator()(const Entry& a, const Entry& b) const { return a.f != b.f ? a.f > b.f : a.g < b.g; }
    };
    void push(Cell c, int g) {
        open_.push_back({g + octile(c, goal_), g, c});
        std::push_heap(open_.begin(), open_.end(), Worse{});
    }

    const Grid* grid_;
    Cell start_, goal_;
    Grid8Mode mode_;
    std::vector<int> g_;  // -1 表示未发现
    std::vector<int> cameFrom_;
    std::vector<char> closed_;
    std::vector<char> scanned_;
    std::vector<Entry> open_;
    Cell current_{};
    bool hasCurrent_ = false;
    bool found_ = false;
    int expanded_ = 0;
    int scanCount_ = 0;
};

// #region expand-path
// 把跳点序列展开成逐格的路径：相邻两个跳点之间一定是一段纯直线或纯斜线
inline std::vector<Cell> expandJumpPath(const std::vector<Cell>& jumpPoints) {
    std::vector<Cell> path;
    if (jumpPoints.empty()) return path;
    path.push_back(jumpPoints.front());
    for (size_t i = 1; i < jumpPoints.size(); ++i) {
        Cell a = jumpPoints[i - 1], b = jumpPoints[i];
        int dx = (b.x > a.x) - (b.x < a.x), dy = (b.y > a.y) - (b.y < a.y);
        while (!(a == b)) {
            a = {a.x + dx, a.y + dy};
            path.push_back(a);
        }
    }
    return path;
}
// #endregion

}  // namespace ga
