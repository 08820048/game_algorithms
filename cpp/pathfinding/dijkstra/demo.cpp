// Dijkstra 演示
//   - 顶部选择画笔：wall / forest ×3 / swamp ×6 / erase，在格子上拖动绘制
//   - 拖动 S / G：移动起点 / 终点
//   - 格子里的数字：到起点的最小代价（只在格子"出堆"、代价确定后显示）
//   - 虚线：同一张地图上 BFS 找到的路径（步数最少，但无视地形代价）

#include <algorithm>
#include <cstdio>
#include <memory>
#include <string>

#include "demo.h"
#include "dijkstra.h"
#include "raymath.h"
#include "ui.h"

namespace {

using namespace ga;

constexpr int kCols = 25;
constexpr int kRows = 14;
constexpr int kCell = 32;
constexpr float kTop = 52;
constexpr int kForestCost = 3;
constexpr int kSwampCost = 6;

enum class Brush { Wall, Forest, Swamp, Erase };
enum class DragMode { None, Paint, MoveStart, MoveGoal };

class DijkstraDemo final : public Demo {
public:
    DijkstraDemo() : grid_(kCols, kRows) {
        // 一条竖着的沼泽带挡在 S、G 之间，下方留出平地：
        // BFS 会直穿沼泽（步数最少），Dijkstra 会绕到下面（代价最小）
        for (int x = 11; x <= 13; ++x)
            for (int y = 0; y <= 9; ++y) grid_.setCost({x, y}, kSwampCost);
        for (int x = 4; x <= 7; ++x)
            for (int y = 9; y <= 12; ++y) grid_.setCost({x, y}, kForestCost);
        for (int x = 16; x <= 20; ++x)
            for (int y = 2; y <= 4; ++y) grid_.setCost({x, y}, kForestCost);
        for (int y = 1; y <= 5; ++y) grid_.setWall({7, y}, true);
        for (int x = 17; x <= 22; ++x) grid_.setWall({x, 12}, true);
        reset();
    }

    void reset() override {
        search_ = std::make_unique<DijkstraStepper>(grid_, start_, goal_);
        bfsPath_ = bfsPath(grid_, start_, goal_);
    }

    bool step() override { return search_->step(); }

    void update() override {
        Vector2 m = GetMousePosition();
        bool inside = m.x >= 0 && m.y >= kTop && !ui::wantsMouse();
        hover_ = inside ? Cell{static_cast<int>(m.x) / kCell, static_cast<int>(m.y - kTop) / kCell} : Cell{-1, -1};
        if (!grid_.inBounds(hover_)) {
            hover_ = {-1, -1};
            inside = false;
        }

        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && inside) {
            if (hover_ == start_) drag_ = DragMode::MoveStart;
            else if (hover_ == goal_) drag_ = DragMode::MoveGoal;
            else drag_ = DragMode::Paint;
        }
        if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) drag_ = DragMode::None;
        if (!IsMouseButtonDown(MOUSE_BUTTON_LEFT) || !inside) return;

        bool changed = false;
        switch (drag_) {
            case DragMode::Paint:
                if (!(hover_ == start_) && !(hover_ == goal_)) changed = paint(hover_);
                break;
            case DragMode::MoveStart:
                if (grid_.passable(hover_) && !(hover_ == goal_) && !(hover_ == start_)) {
                    start_ = hover_;
                    changed = true;
                }
                break;
            case DragMode::MoveGoal:
                if (grid_.passable(hover_) && !(hover_ == start_) && !(hover_ == goal_)) {
                    goal_ = hover_;
                    changed = true;
                }
                break;
            case DragMode::None:
                break;
        }
        if (changed) notifyProblemChanged();
    }

    void draw() const override {
        const Palette& p = palette();
        const DijkstraStepper& s = *search_;
        const float hl = hairline();

        int maxCost = 1;
        for (int i = 0; i < grid_.size(); ++i) {
            Cell c = grid_.cellAt(i);
            if (s.closed(c)) maxCost = std::max(maxCost, s.cost(c));
        }

        // 1. 已确定代价的格子：越便宜颜色越深
        for (int i = 0; i < grid_.size(); ++i) {
            Cell c = grid_.cellAt(i);
            if (s.closed(c) && !grid_.isWall(c)) {
                float t = static_cast<float>(s.cost(c)) / static_cast<float>(maxCost);
                DrawRectangleRec(cellRect(c), ColorLerp(p.visitedNear, p.visitedFar, t));
            }
        }

        // 2. 地形纹理：森林是四角的点，沼泽是上下两组横线，中间留给代价数字
        Color ink = Fade(p.text, isDarkTheme() ? 0.32f : 0.26f);
        for (int i = 0; i < grid_.size(); ++i) {
            Cell c = grid_.cellAt(i);
            if (grid_.isWall(c)) continue;
            Rectangle r = cellRect(c);
            if (grid_.cost(c) == kForestCost) {
                for (int k = 0; k < 4; ++k) {
                    float dx = (k % 2 ? 0.8f : 0.2f) * kCell, dy = (k / 2 ? 0.8f : 0.2f) * kCell;
                    DrawCircleV({r.x + dx, r.y + dy}, 2.2f, ink);
                }
            } else if (grid_.cost(c) == kSwampCost) {
                for (float f : {0.16f, 0.27f, 0.73f, 0.84f}) {
                    float y = r.y + f * kCell;
                    DrawLineEx({r.x + 5, y}, {r.x + kCell - 5, y}, 1.4f, ink);
                }
            }
        }

        // 3. 边界（堆中仍然有效的格子）
        s.forEachFrontier([&](Cell c) { DrawRectangleRec(inset(cellRect(c), 5), p.frontier); });

        // 4. 网格细线与墙
        for (int x = 0; x <= kCols; ++x) {
            DrawLineEx({x * float(kCell), kTop}, {x * float(kCell), kTop + kRows * float(kCell)}, hl, p.gridLine);
        }
        for (int y = 0; y <= kRows; ++y) {
            DrawLineEx({0, kTop + y * float(kCell)}, {kCols * float(kCell), kTop + y * float(kCell)}, hl, p.gridLine);
        }
        for (int i = 0; i < grid_.size(); ++i) {
            Cell c = grid_.cellAt(i);
            if (grid_.isWall(c)) DrawRectangleRec(cellRect(c), p.wall);
        }

        // 5. 代价数字：已确定的用灰色，边界上（暂定值）用深色
        std::vector<char> onFrontier(grid_.size(), 0);
        s.forEachFrontier([&](Cell c) { onFrontier[grid_.index(c)] = 1; });
        for (int i = 0; i < grid_.size(); ++i) {
            Cell c = grid_.cellAt(i);
            if (c == start_ || c == goal_ || (!s.closed(c) && !onFrontier[i])) continue;
            char buf[8];
            std::snprintf(buf, sizeof buf, "%d", s.cost(c));
            drawTextCentered(buf, center(c), 10, onFrontier[i] ? p.startLabel : p.textMuted);
        }

        // 6. 当前展开的格子
        if (s.hasCurrent() && !s.found()) DrawRectangleLinesEx(inset(cellRect(s.current()), 2), 2, p.current);

        // 7. BFS 路径（虚线）与 Dijkstra 路径（朱红实线）
        if (showBfs_) {
            for (size_t i = 1; i < bfsPath_.size(); ++i) {
                drawDashedLine(center(bfsPath_[i - 1]), center(bfsPath_[i]), 2, p.text, 5, 4);
            }
        }
        auto path = s.path();
        for (size_t i = 1; i < path.size(); ++i) DrawLineEx(center(path[i - 1]), center(path[i]), 3, p.path);
        for (size_t i = 1; i + 1 < path.size(); ++i) DrawCircleV(center(path[i]), 3.5f, p.path);

        if (hover_.x >= 0) DrawRectangleRec(cellRect(hover_), p.hover);
        drawEndpoint(start_, p.start, p.startLabel, "S");
        drawEndpoint(goal_, p.goal, p.goalLabel, "G");

        // 顶栏
        DrawLineEx({0, kTop}, {float(kCols * kCell), kTop}, hl, p.gridLine);
        char buf[64];
        float x = kCols * kCell - 16.0f;
        if (search_->found()) {
            std::snprintf(buf, sizeof buf, "dijkstra %d", pathCost(grid_, path));
            Vector2 sz = measureText(buf, 13);
            drawText(buf, {x - sz.x, 20}, 13, p.path);
            x -= sz.x + 16;
        }
        if (showBfs_ && !bfsPath_.empty()) {
            std::snprintf(buf, sizeof buf, "bfs %d", pathCost(grid_, bfsPath_));
            Vector2 sz = measureText(buf, 13);
            drawText(buf, {x - sz.x, 20}, 13, p.text);
            x -= sz.x + 12;
            drawText("cost", {x - measureText("cost", 13).x, 20}, 13, p.textMuted);
        }
    }

    void drawUi() override {
        struct Item { Brush brush; const char* label; float w; };
        static constexpr Item kItems[] = {
            {Brush::Wall, "wall", 64}, {Brush::Forest, "forest ×3", 98},
            {Brush::Swamp, "swamp ×6", 92}, {Brush::Erase, "erase", 68},
        };
        float x = 16;
        for (const Item& it : kItems) {
            if (ui::button({x, 13, it.w, 26}, it.label, brush_ == it.brush)) brush_ = it.brush;
            x += it.w + 4;
        }
        ui::toggle({x + 16, 13, 64, 26}, "bfs", &showBfs_);
    }

    std::string status() const override {
        const DijkstraStepper& s = *search_;
        char buf[96];
        if (s.found()) {
            std::snprintf(buf, sizeof buf, "expanded %d · cost %d · found", s.expandedCount(),
                          pathCost(grid_, s.path()));
        } else {
            std::snprintf(buf, sizeof buf, "expanded %d · frontier %d · %s", s.expandedCount(),
                          s.liveFrontierSize(), s.done() ? "unreachable" : "searching");
        }
        return buf;
    }

private:
    // 用当前画笔修改一个格子，返回地图是否真的变了
    bool paint(Cell c) {
        bool wall = brush_ == Brush::Wall;
        int cost = brush_ == Brush::Forest ? kForestCost : brush_ == Brush::Swamp ? kSwampCost : 1;
        if (grid_.isWall(c) == wall && (wall || grid_.cost(c) == cost)) return false;
        grid_.setWall(c, wall);
        grid_.setCost(c, cost);
        return true;
    }

    static Rectangle cellRect(Cell c) {
        return {static_cast<float>(c.x * kCell), kTop + static_cast<float>(c.y * kCell), kCell, kCell};
    }
    static Rectangle inset(Rectangle r, float d) { return {r.x + d, r.y + d, r.width - 2 * d, r.height - 2 * d}; }
    static Vector2 center(Cell c) { return {c.x * kCell + kCell * 0.5f, kTop + c.y * kCell + kCell * 0.5f}; }
    static void drawEndpoint(Cell c, Color fill, Color label, const char* text) {
        DrawCircleV(center(c), kCell * 0.36f, fill);
        drawTextCentered(text, center(c), 14, label);
    }

    CostGrid grid_;
    Cell start_{2, 5};
    Cell goal_{22, 5};
    Cell hover_{-1, -1};
    Brush brush_ = Brush::Swamp;
    DragMode drag_ = DragMode::None;
    bool showBfs_ = true;
    std::vector<Cell> bfsPath_;
    std::unique_ptr<DijkstraStepper> search_;
};

}  // namespace

int main() {
    static DijkstraDemo demo;
    ga::DemoConfig config;
    config.title = "Dijkstra - Weighted Grid";
    config.width = kCols * kCell;
    config.height = static_cast<int>(kTop) + kRows * kCell;
    config.stepsPerSecond = 30.0f;
    return ga::runDemo(demo, config);
}
