#pragma once

#include <array>
#include <cstdint>
#include <vector>

namespace ga {

// #region grid
// 网格上的一个格子坐标
struct Cell {
    int x = 0;
    int y = 0;

    friend bool operator==(Cell a, Cell b) = default;
};

// 最简单的二维网格地图：每个格子要么可通行，要么是墙
class Grid {
public:
    Grid(int width, int height)
        : width_(width), height_(height), walls_(static_cast<size_t>(width * height), 0) {}

    int width() const { return width_; }
    int height() const { return height_; }
    int size() const { return width_ * height_; }

    bool inBounds(Cell c) const { return c.x >= 0 && c.x < width_ && c.y >= 0 && c.y < height_; }
    bool isWall(Cell c) const { return walls_[index(c)] != 0; }
    void setWall(Cell c, bool wall) { walls_[index(c)] = wall ? 1 : 0; }
    bool passable(Cell c) const { return inBounds(c) && !isWall(c); }

    // 二维坐标 <-> 一维下标，算法里用一维数组存储状态更紧凑
    int index(Cell c) const { return c.y * width_ + c.x; }
    Cell cellAt(int i) const { return {i % width_, i / width_}; }

    // 遍历 4 邻域中可通行的格子，顺序固定为：右、下、左、上
    template <class F>
    void forEachNeighbor(Cell c, F&& visit) const {
        static constexpr std::array<Cell, 4> kDirs{{{1, 0}, {0, 1}, {-1, 0}, {0, -1}}};
        for (Cell d : kDirs) {
            Cell next{c.x + d.x, c.y + d.y};
            if (passable(next)) visit(next);
        }
    }

private:
    int width_;
    int height_;
    std::vector<uint8_t> walls_;
};
// #endregion

}  // namespace ga
