#pragma once

#include <vector>

#include "grid.h"

namespace ga {

// #region cost-grid
// 带地形代价的网格：在 Grid（墙 / 空地）的基础上，给每个格子加一个"走进来要花多少"。
// 继承 Grid，所以 BFS 可以直接跑在它上面（只是会无视代价）。
class CostGrid : public Grid {
public:
    CostGrid(int width, int height) : Grid(width, height), costs_(static_cast<size_t>(width * height), 1) {}

    // 进入格子 c 的代价，约定 >= 1
    int cost(Cell c) const { return costs_[index(c)]; }
    void setCost(Cell c, int cost) { costs_[index(c)] = cost; }

private:
    std::vector<int> costs_;
};
// #endregion

}  // namespace ga
