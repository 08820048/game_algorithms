#pragma once

// 带地形的网格寻路演示共用的部分（Dijkstra、A* 等）：
//   - 坐标换算：网格画在顶栏（kTop）下方
//   - 地图编辑：画笔绘制墙 / 森林 / 沼泽，拖动起点终点
//   - 绘制：地形纹理、网格线、墙、路径、起点终点
// 只负责"画地图"，搜索状态（已访问、边界、数字）由各个演示自己画。

#include <vector>

#include "cost_grid.h"
#include "demo.h"
#include "ui.h"

namespace ga::terrain {

constexpr int kCols = 25;
constexpr int kRows = 14;
constexpr int kCell = 32;
constexpr float kTop = 52;  // 顶栏高度
constexpr int kWidth = kCols * kCell;
constexpr int kHeight = static_cast<int>(kTop) + kRows * kCell;
constexpr int kForestCost = 3;
constexpr int kSwampCost = 6;

inline Rectangle cellRect(Cell c) {
    return {static_cast<float>(c.x * kCell), kTop + static_cast<float>(c.y * kCell), kCell, kCell};
}
inline Rectangle inset(Rectangle r, float d) { return {r.x + d, r.y + d, r.width - 2 * d, r.height - 2 * d}; }
inline Vector2 center(Cell c) { return {c.x * kCell + kCell * 0.5f, kTop + c.y * kCell + kCell * 0.5f}; }

inline void fillRect(CostGrid& g, int x0, int y0, int x1, int y1, int cost) {
    for (int y = y0; y <= y1; ++y)
        for (int x = x0; x <= x1; ++x) g.setCost({x, y}, cost);
}
inline void wallRect(CostGrid& g, int x0, int y0, int x1, int y1) {
    for (int y = y0; y <= y1; ++y)
        for (int x = x0; x <= x1; ++x) g.setWall({x, y}, true);
}

// 地图编辑器：画笔、悬停格子、拖动状态
class Editor {
public:
    enum class Brush { Wall, Forest, Swamp, Erase };

    // 每帧调用；地图、起点或终点发生变化时返回 true
    bool update(CostGrid& grid, Cell& start, Cell& goal) {
        Vector2 m = GetMousePosition();
        bool inside = m.x >= 0 && m.y >= kTop && !ui::wantsMouse();
        hover_ = inside ? Cell{static_cast<int>(m.x) / kCell, static_cast<int>(m.y - kTop) / kCell} : Cell{-1, -1};
        if (!grid.inBounds(hover_)) {
            hover_ = {-1, -1};
            inside = false;
        }
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && inside) {
            drag_ = hover_ == start ? Drag::Start : hover_ == goal ? Drag::Goal : Drag::Paint;
        }
        if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) drag_ = Drag::None;
        if (!IsMouseButtonDown(MOUSE_BUTTON_LEFT) || !inside || hover_ == start || hover_ == goal) return false;

        switch (drag_) {
            case Drag::Paint: return paint(grid, hover_);
            case Drag::Start:
                if (!grid.passable(hover_)) return false;
                start = hover_;
                return true;
            case Drag::Goal:
                if (!grid.passable(hover_)) return false;
                goal = hover_;
                return true;
            case Drag::None: return false;
        }
        return false;
    }

    // 在顶栏 x 处画出画笔按钮，返回按钮组右端的 x
    float drawBrushes(float x) {
        struct Item { Brush brush; const char* label; float w; };
        static constexpr Item kItems[] = {
            {Brush::Wall, "wall", 58}, {Brush::Forest, "forest ×3", 94},
            {Brush::Swamp, "swamp ×6", 88}, {Brush::Erase, "erase", 62},
        };
        for (const Item& it : kItems) {
            if (ui::button({x, 13, it.w, 26}, it.label, brush_ == it.brush)) brush_ = it.brush;
            x += it.w + 4;
        }
        return x - 4;
    }

    Cell hover() const { return hover_; }
    void setBrush(Brush b) { brush_ = b; }

private:
    enum class Drag { None, Paint, Start, Goal };

    bool paint(CostGrid& grid, Cell c) const {
        bool wall = brush_ == Brush::Wall;
        int cost = brush_ == Brush::Forest ? kForestCost : brush_ == Brush::Swamp ? kSwampCost : 1;
        if (grid.isWall(c) == wall && (wall || grid.cost(c) == cost)) return false;
        grid.setWall(c, wall);
        grid.setCost(c, cost);
        return true;
    }

    Brush brush_ = Brush::Swamp;
    Drag drag_ = Drag::None;
    Cell hover_{-1, -1};
};

// 地形纹理：森林是四角的点，沼泽是上下两组横线，格子中间留给数字
inline void drawTerrain(const CostGrid& grid) {
    Color ink = Fade(palette().text, isDarkTheme() ? 0.32f : 0.26f);
    for (int i = 0; i < grid.size(); ++i) {
        Cell c = grid.cellAt(i);
        if (grid.isWall(c)) continue;
        Rectangle r = cellRect(c);
        if (grid.cost(c) == kForestCost) {
            for (int k = 0; k < 4; ++k) {
                float dx = (k % 2 ? 0.8f : 0.2f) * kCell, dy = (k / 2 ? 0.8f : 0.2f) * kCell;
                DrawCircleV({r.x + dx, r.y + dy}, 2.2f, ink);
            }
        } else if (grid.cost(c) == kSwampCost) {
            for (float f : {0.16f, 0.27f, 0.73f, 0.84f}) {
                float y = r.y + f * kCell;
                DrawLineEx({r.x + 5, y}, {r.x + kCell - 5, y}, 1.4f, ink);
            }
        }
    }
}

// 网格细线 + 墙 + 顶栏分隔线
inline void drawGridAndWalls(const CostGrid& grid) {
    const Palette& p = palette();
    const float hl = hairline();
    for (int x = 0; x <= kCols; ++x) {
        DrawLineEx({x * float(kCell), kTop}, {x * float(kCell), float(kHeight)}, hl, p.gridLine);
    }
    for (int y = 0; y <= kRows; ++y) {
        DrawLineEx({0, kTop + y * float(kCell)}, {float(kWidth), kTop + y * float(kCell)}, hl, p.gridLine);
    }
    for (int i = 0; i < grid.size(); ++i) {
        Cell c = grid.cellAt(i);
        if (grid.isWall(c)) DrawRectangleRec(cellRect(c), p.wall);
    }
}

inline void drawPath(const std::vector<Cell>& path, Color color) {
    for (size_t i = 1; i < path.size(); ++i) DrawLineEx(center(path[i - 1]), center(path[i]), 3, color);
    for (size_t i = 1; i + 1 < path.size(); ++i) DrawCircleV(center(path[i]), 3.5f, color);
}

inline void drawDashedPath(const std::vector<Cell>& path, Color color) {
    for (size_t i = 1; i < path.size(); ++i) drawDashedLine(center(path[i - 1]), center(path[i]), 2, color, 5, 4);
}

// 悬停高亮 + 起点 S / 终点 G
inline void drawOverlay(Cell hover, Cell start, Cell goal) {
    const Palette& p = palette();
    if (hover.x >= 0) DrawRectangleRec(cellRect(hover), p.hover);
    DrawCircleV(center(start), kCell * 0.36f, p.start);
    drawTextCentered("S", center(start), 14, p.startLabel);
    DrawCircleV(center(goal), kCell * 0.36f, p.goal);
    drawTextCentered("G", center(goal), 14, p.goalLabel);
}

// 顶栏右侧从右往左排列的文字，返回下一段文字的右边界
inline float drawRightLabel(float right, const char* text, Color color) {
    Vector2 sz = measureText(text, 13);
    drawText(text, {right - sz.x, 20}, 13, color);
    return right - sz.x - 12;
}

}  // namespace ga::terrain
