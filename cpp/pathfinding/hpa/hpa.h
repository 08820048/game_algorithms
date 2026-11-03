#pragma once

#include <algorithm>
#include <cstdlib>
#include <functional>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

#include "cost_grid.h"

namespace ga {

constexpr int kHpaInf = std::numeric_limits<int>::max();

// #region bounded
// 限定在一个矩形区域（簇）内的 Dijkstra。
// reverse = true 时沿反方向搜索：得到的是"从各个格子走到 source"的代价（地形代价不对称，进入格子才付费）
struct Rect {
    int x0, y0, x1, y1;  // 包含 [x0, x1) × [y0, y1)
    bool contains(Cell c) const { return c.x >= x0 && c.x < x1 && c.y >= y0 && c.y < y1; }
    int width() const { return x1 - x0; }
    int index(Cell c) const { return (c.y - y0) * width() + (c.x - x0); }
    Cell cellAt(int i) const { return {x0 + i % width(), y0 + i / width()}; }
    int size() const { return width() * (y1 - y0); }
};

struct LocalSearch {
    std::vector<int> cost;    // 按 Rect::index 存储
    std::vector<int> parent;  // 正向搜索时的来路（局部下标），-1 表示无
};

inline LocalSearch boundedDijkstra(const CostGrid& grid, Rect r, Cell source, bool reverse = false) {
    LocalSearch s{std::vector<int>(r.size(), kHpaInf), std::vector<int>(r.size(), -1)};
    if (!r.contains(source) || !grid.passable(source)) return s;
    using Entry = std::pair<int, int>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
    s.cost[r.index(source)] = 0;
    open.push({0, r.index(source)});
    while (!open.empty()) {
        auto [c, i] = open.top();
        open.pop();
        if (c > s.cost[i]) continue;
        Cell cur = r.cellAt(i);
        grid.forEachNeighbor(cur, [&](Cell n) {
            if (!r.contains(n)) return;
            int ni = r.index(n);
            int nc = c + (reverse ? grid.cost(cur) : grid.cost(n));
            if (nc < s.cost[ni]) {
                s.cost[ni] = nc;
                s.parent[ni] = i;
                open.push({nc, ni});
            }
        });
    }
    return s;
}
// #endregion

struct HpaStats {
    int abstractExpanded = 0;  // 抽象图上展开的节点数
    int cellsSearched = 0;     // 插入起终点 + 细化路径时，簇内搜索访问的格子数
};

class HpaGraph {
public:
    struct Edge {
        int to;
        int cost;
        int path;  // 簇内边：预先算好的格子路径（paths_ 的下标）；簇间边为 -1
    };
    struct Node {
        Cell cell;
        int cluster;
        std::vector<Edge> edges;
    };

    // #region build
    // 预处理：找出所有入口，放置过渡点，再计算每个簇内过渡点之间的代价，并缓存对应的格子路径
    void build(const CostGrid& grid, int clusterSize) {
        grid_ = &grid;
        k_ = clusterSize;
        cols_ = (grid.width() + k_ - 1) / k_;
        rows_ = (grid.height() + k_ - 1) / k_;
        nodes_.clear();
        paths_.clear();
        nodeAt_.assign(grid.size(), -1);
        byCluster_.assign(cols_ * rows_, {});

        // 1. 入口：相邻两个簇的公共边界上，两侧都能走的连续区间
        for (int cy = 0; cy < rows_; ++cy) {
            for (int cx = 0; cx < cols_; ++cx) {
                Rect r = clusterRect(cx, cy);
                if (r.x1 < grid.width()) scanBorder(r.x1 - 1, r.y0, 0, 1, r.y1 - r.y0, 1, 0);   // 右边界
                if (r.y1 < grid.height()) scanBorder(r.x0, r.y1 - 1, 1, 0, r.x1 - r.x0, 0, 1);  // 下边界
            }
        }
        // 2. 簇内边：同一个簇里的过渡点两两之间，只在簇内搜索的最短代价和路径
        for (int id = 0; id < static_cast<int>(nodes_.size()); ++id) {
            Rect r = clusterRectOf(nodes_[id].cluster);
            LocalSearch s = boundedDijkstra(grid, r, nodes_[id].cell);
            for (int other : byCluster_[nodes_[id].cluster]) {  // 只和同一个簇里的节点比较
                if (other == id) continue;
                int c = s.cost[r.index(nodes_[other].cell)];
                if (c == kHpaInf) continue;
                paths_.push_back(tracePath(r, s, nodes_[other].cell));
                nodes_[id].edges.push_back({other, c, static_cast<int>(paths_.size()) - 1});
            }
        }
    }
    // #endregion

    // #region query
    // 查询：把起点和终点临时接入抽象图，在抽象图上跑 A*，再把每一段换成格子路径
    std::vector<Cell> findPath(Cell start, Cell goal, HpaStats* stats = nullptr,
                               std::vector<Cell>* abstractPath = nullptr) const {
        HpaStats st;
        if (!grid_->passable(start) || !grid_->passable(goal)) return {};

        const int n = static_cast<int>(nodes_.size());
        const int S = n, G = n + 1;  // 起点、终点的临时编号
        auto cellOf = [&](int id) { return id == S ? start : id == G ? goal : nodes_[id].cell; };

        // 1. 插入：起点到所在簇各过渡点的代价（正向搜索），各过渡点到终点的代价（反向搜索）
        const int cs = clusterOf(start), cg = clusterOf(goal);
        Rect rs = clusterRectOf(cs), rg = clusterRectOf(cg);
        LocalSearch fromStart = boundedDijkstra(*grid_, rs, start);
        LocalSearch toGoal = boundedDijkstra(*grid_, rg, goal, /*reverse=*/true);
        st.cellsSearched += rs.size() + rg.size();

        // 2. 抽象图上的 A*，启发函数为曼哈顿距离
        std::vector<int> g(n + 2, kHpaInf), from(n + 2, -1);
        std::vector<char> closed(n + 2, 0);
        using Entry = std::pair<int, int>;
        std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
        auto h = [&](int id) { Cell c = cellOf(id); return std::abs(c.x - goal.x) + std::abs(c.y - goal.y); };
        g[S] = 0;
        open.push({h(S), S});
        while (!open.empty()) {
            int u = open.top().second;
            open.pop();
            if (closed[u]) continue;
            closed[u] = 1;
            ++st.abstractExpanded;
            if (u == G) break;
            auto relax = [&](int v, int c) {
                if (c != kHpaInf && !closed[v] && g[u] + c < g[v]) {
                    g[v] = g[u] + c;
                    from[v] = u;
                    open.push({g[v] + h(v), v});
                }
            };
            if (u == S) {
                for (int id : clusterNodes(cs)) relax(id, fromStart.cost[rs.index(nodes_[id].cell)]);
                if (cs == cg) relax(G, fromStart.cost[rs.index(goal)]);  // 同一个簇：也可以直接走过去
            } else {
                for (const Edge& e : nodes_[u].edges) relax(e.to, e.cost);
                if (nodes_[u].cluster == cg) relax(G, toGoal.cost[rg.index(nodes_[u].cell)]);
            }
        }
        if (stats) *stats = st;
        if (g[G] == kHpaInf) return {};

        std::vector<int> ids;
        for (int v = G; v != -1; v = from[v]) ids.push_back(v);
        std::reverse(ids.begin(), ids.end());
        if (abstractPath) {
            abstractPath->clear();
            for (int id : ids) abstractPath->push_back(cellOf(id));
        }

        // 3. 拼接：每一段都已经有现成的格子路径，不需要再搜索
        std::vector<Cell> result{start};
        for (size_t i = 1; i < ids.size(); ++i) {
            int u = ids[i - 1], v = ids[i];
            std::vector<Cell> seg;
            if (u == S) {
                seg = tracePath(rs, fromStart, cellOf(v));  // 起点所在簇的正向搜索树
            } else if (v == G) {
                // 反向搜索树里，每个格子的 parent 是"离终点更近的下一步"
                for (int k = rg.index(cellOf(u)); k != -1; k = toGoal.parent[k]) seg.push_back(rg.cellAt(k));
            } else {
                const Edge& e = edgeBetween(u, v);
                seg = e.path >= 0 ? paths_[e.path] : std::vector<Cell>{cellOf(u), cellOf(v)};
            }
            result.insert(result.end(), seg.begin() + 1, seg.end());
        }
        return result;
    }
    // #endregion

    int clusterOf(Cell c) const { return (c.y / k_) * cols_ + (c.x / k_); }
    Rect clusterRect(int cx, int cy) const {
        return {cx * k_, cy * k_, std::min((cx + 1) * k_, grid_->width()), std::min((cy + 1) * k_, grid_->height())};
    }
    Rect clusterRectOf(int cluster) const { return clusterRect(cluster % cols_, cluster / cols_); }
    const std::vector<Node>& nodes() const { return nodes_; }
    const std::vector<Cell>& edgePath(const Edge& e) const { return paths_[e.path]; }
    int clusterSize() const { return k_; }
    int clusterCount() const { return cols_ * rows_; }

private:
    // #region entrances
    // 扫描一条簇边界：(x, y) 是边界内侧的第一个格子，(dx, dy) 沿边界前进，(ox, oy) 指向相邻簇
    void scanBorder(int x, int y, int dx, int dy, int length, int ox, int oy) {
        constexpr int kWide = 6;  // 入口宽度达到它时，在两端各放一个过渡点，否则只在中间放一个
        int runStart = -1;
        for (int i = 0; i <= length; ++i) {
            Cell a{x + dx * i, y + dy * i}, b{a.x + ox, a.y + oy};
            bool open = i < length && grid_->passable(a) && grid_->passable(b);
            if (open && runStart < 0) runStart = i;
            if (!open && runStart >= 0) {
                int runEnd = i - 1;
                if (runEnd - runStart + 1 >= kWide) {
                    addTransition(x + dx * runStart, y + dy * runStart, ox, oy);
                    addTransition(x + dx * runEnd, y + dy * runEnd, ox, oy);
                } else {
                    int mid = (runStart + runEnd) / 2;
                    addTransition(x + dx * mid, y + dy * mid, ox, oy);
                }
                runStart = -1;
            }
        }
    }
    // 一个过渡点 = 边界两侧各一个节点 + 两条方向相反的簇间边（代价 = 进入对方格子的代价）
    void addTransition(int x, int y, int ox, int oy) {
        Cell a{x, y}, b{x + ox, y + oy};
        int na = nodeFor(a), nb = nodeFor(b);
        nodes_[na].edges.push_back({nb, grid_->cost(b), -1});
        nodes_[nb].edges.push_back({na, grid_->cost(a), -1});
    }
    // #endregion

    int nodeFor(Cell c) {
        int& id = nodeAt_[grid_->index(c)];
        if (id < 0) {
            id = static_cast<int>(nodes_.size());
            nodes_.push_back({c, clusterOf(c), {}});
            byCluster_[nodes_.back().cluster].push_back(id);
        }
        return id;
    }
    const std::vector<int>& clusterNodes(int cluster) const { return byCluster_[cluster]; }
    const Edge& edgeBetween(int u, int v) const {
        const Edge* best = nullptr;
        for (const Edge& e : nodes_[u].edges) {
            if (e.to == v && (!best || e.cost < best->cost)) best = &e;
        }
        return *best;
    }
    // 沿正向搜索树从 target 回溯到源点，返回从源点到 target 的路径
    static std::vector<Cell> tracePath(const Rect& r, const LocalSearch& s, Cell target) {
        std::vector<Cell> p;
        for (int k = r.index(target); k != -1; k = s.parent[k]) p.push_back(r.cellAt(k));
        std::reverse(p.begin(), p.end());
        return p;
    }

    const CostGrid* grid_ = nullptr;
    int k_ = 1, cols_ = 0, rows_ = 0;
    std::vector<Node> nodes_;
    std::vector<std::vector<Cell>> paths_;      // 簇内边的格子路径
    std::vector<int> nodeAt_;                   // 格子 -> 节点编号
    std::vector<std::vector<int>> byCluster_;   // 簇 -> 其中的节点
};

}  // namespace ga
