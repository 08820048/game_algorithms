#pragma once

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <vector>

#include "cost_grid.h"

namespace ga {

// #region supercover
// 从格子 a 的中心到格子 b 的中心画一条线段，按顺序访问它经过的每一个格子。
// "超覆盖"：线段恰好穿过格子角点时，角点两侧的格子都算经过，这样线段不会从两堵墙的缝隙里斜穿过去。
// visit 返回 false 时提前停止，函数也返回 false。
template <class F>
bool forEachCellOnLine(Cell a, Cell b, F&& visit) {
    const int nx = std::abs(b.x - a.x), ny = std::abs(b.y - a.y);
    const int sx = b.x > a.x ? 1 : -1, sy = b.y > a.y ? 1 : -1;
    Cell p = a;
    if (!visit(p)) return false;
    for (int ix = 0, iy = 0; ix < nx || iy < ny;) {
        // 比较线段下一次穿过竖直格线和水平格线的"时刻"，先穿过哪条就往哪个方向走一格
        int decision = (1 + 2 * ix) * ny - (1 + 2 * iy) * nx;
        if (decision == 0) {  // 同时穿过：正好经过角点
            if (!visit(Cell{p.x + sx, p.y}) || !visit(Cell{p.x, p.y + sy})) return false;
            p.x += sx;
            p.y += sy;
            ++ix;
            ++iy;
        } else if (decision < 0) {
            p.x += sx;
            ++ix;
        } else {
            p.y += sy;
            ++iy;
        }
        if (!visit(p)) return false;
    }
    return true;
}

// 视线检查：线段经过的格子都可通行，且代价都不超过 maxCost
inline bool lineOfSight(const CostGrid& grid, Cell a, Cell b, int maxCost) {
    return forEachCellOnLine(a, b, [&](Cell c) { return grid.passable(c) && grid.cost(c) <= maxCost; });
}
// #endregion

// #region smooth-path
// 拉直路径：从当前锚点出发，找到路径上"能直接看到"的最远一点，连成一条直线，再以它为新锚点继续。
// 返回的是拐点（路标）序列，单位沿着相邻路标之间的直线移动。
//
// 只要求"没有墙"会让捷径穿过原路径特意绕开的沼泽，所以额外要求：
// 捷径经过的格子，代价不能超过被它替换掉的那段原路径上的最高代价。
inline std::vector<Cell> smoothPath(const CostGrid& grid, const std::vector<Cell>& path) {
    if (path.size() < 3) return path;
    std::vector<Cell> result{path.front()};
    size_t anchor = 0;
    while (anchor + 1 < path.size()) {
        size_t best = anchor + 1;
        int segmentMax = std::max(grid.cost(path[anchor]), grid.cost(path[anchor + 1]));
        for (size_t j = anchor + 2; j < path.size(); ++j) {
            segmentMax = std::max(segmentMax, grid.cost(path[j]));
            if (lineOfSight(grid, path[anchor], path[j], segmentMax)) best = j;
        }
        result.push_back(path[best]);
        anchor = best;
    }
    return result;
}

// 路标序列的几何长度（以格子边长为单位）
inline float polylineLength(const std::vector<Cell>& points) {
    float total = 0;
    for (size_t i = 1; i < points.size(); ++i) {
        total += std::hypot(static_cast<float>(points[i].x - points[i - 1].x),
                            static_cast<float>(points[i].y - points[i - 1].y));
    }
    return total;
}
// #endregion

}  // namespace ga
