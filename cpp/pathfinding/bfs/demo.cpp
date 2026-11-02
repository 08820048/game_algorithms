// BFS 洪水填充演示
//   - 鼠标左键在空地上拖动：画墙；在墙上拖动：擦墙
//   - 拖动绿色 S / 红色 G：移动起点 / 终点
//   - 格子里的数字：该格到起点的步数（BFS 访问的"波次"）

#include <cstdio>
#include <memory>

#include "bfs.h"
#include "demo.h"
#include "raymath.h"

namespace {

using namespace ga;

constexpr int kCols = 25;
constexpr int kRows = 15;
constexpr int kCell = 32;

enum class DragMode { None, DrawWall, EraseWall, MoveStart, MoveGoal };

class BfsDemo final : public Demo {
public:
    BfsDemo() : grid_(kCols, kRows) {
        // 预置几段墙，让第一次打开时就能看到"绕路"
        for (int y = 2; y <= 11; ++y) grid_.setWall({8, y}, true);
        for (int y = 4; y <= 14; ++y) grid_.setWall({15, y}, true);
        for (int x = 15; x <= 19; ++x) grid_.setWall({x, 4}, true);
        reset();
    }

    void reset() override { search_ = std::make_unique<BfsStepper>(grid_, start_, goal_); }

    bool step() override { return search_->step(); }

    void update() override {
        Vector2 m = GetMousePosition();
        hover_ = {static_cast<int>(m.x) / kCell, static_cast<int>(m.y) / kCell};
        bool inside = m.x >= 0 && m.y >= 0 && grid_.inBounds(hover_);
        if (!inside) hover_ = {-1, -1};

        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && inside) {
            if (hover_ == start_) {
                drag_ = DragMode::MoveStart;
            } else if (hover_ == goal_) {
                drag_ = DragMode::MoveGoal;
            } else {
                drag_ = grid_.isWall(hover_) ? DragMode::EraseWall : DragMode::DrawWall;
            }
        }
        if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) drag_ = DragMode::None;
        if (!IsMouseButtonDown(MOUSE_BUTTON_LEFT) || !inside) return;

        bool changed = false;
        switch (drag_) {
            case DragMode::DrawWall:
            case DragMode::EraseWall: {
                bool wall = drag_ == DragMode::DrawWall;
                if (!(hover_ == start_) && !(hover_ == goal_) && grid_.isWall(hover_) != wall) {
                    grid_.setWall(hover_, wall);
                    changed = true;
                }
                break;
            }
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
        const BfsStepper& s = *search_;

        // 计算最大距离，用于已访问格子的颜色渐变
        int maxDist = 1;
        for (int i = 0; i < grid_.size(); ++i) maxDist = std::max(maxDist, s.distance(grid_.cellAt(i)));

        // 1. 格子底色：墙 / 已访问（按距离渐变）
        for (int i = 0; i < grid_.size(); ++i) {
            Cell c = grid_.cellAt(i);
            Rectangle r = cellRect(c);
            if (grid_.isWall(c)) {
                DrawRectangleRec(r, p.wall);
                continue;
            }
            int d = s.distance(c);
            if (d >= 0) {
                float t = static_cast<float>(d) / static_cast<float>(maxDist);
                DrawRectangleRec(r, ColorLerp(p.visitedNear, p.visitedFar, t));
            }
        }

        // 2. 边界（队列中的格子）
        for (Cell c : s.frontier()) DrawRectangleRec(inset(cellRect(c), 3), p.frontier);

        // 3. 网格线
        for (int x = 0; x <= kCols; ++x) DrawLine(x * kCell, 0, x * kCell, kRows * kCell, p.gridLine);
        for (int y = 0; y <= kRows; ++y) DrawLine(0, y * kCell, kCols * kCell, y * kCell, p.gridLine);

        // 4. 距离数字
        for (int i = 0; i < grid_.size(); ++i) {
            Cell c = grid_.cellAt(i);
            int d = s.distance(c);
            if (d <= 0 || c == goal_) continue;
            char buf[8];
            std::snprintf(buf, sizeof buf, "%d", d);
            int w = MeasureText(buf, 10);
            DrawText(buf, c.x * kCell + (kCell - w) / 2, c.y * kCell + 11, 10, p.textMuted);
        }

        // 5. 当前正在展开的格子
        if (s.hasCurrent() && !s.found()) {
            DrawRectangleLinesEx(inset(cellRect(s.current()), 1), 3, p.current);
        }

        // 6. 找到的路径
        auto path = s.path();
        for (size_t i = 1; i < path.size(); ++i) {
            DrawLineEx(center(path[i - 1]), center(path[i]), 5, p.path);
        }
        for (Cell c : path) DrawCircleV(center(c), 4, p.path);

        // 7. 鼠标悬停
        if (hover_.x >= 0) DrawRectangleRec(cellRect(hover_), p.hover);

        // 8. 起点 / 终点
        drawEndpoint(start_, p.start, "S");
        drawEndpoint(goal_, p.goal, "G");

        // 9. 统计面板
        char info[96];
        const char* state = s.found() ? "found" : (s.done() ? "unreachable" : "searching");
        std::snprintf(info, sizeof info, "expanded %d   frontier %d   %s", s.expandedCount(),
                      static_cast<int>(s.frontier().size()), state);
        int w = MeasureText(info, 10);
        Rectangle panel{static_cast<float>(kCols * kCell - w - 20), 6, static_cast<float>(w + 14), 20};
        DrawRectangleRounded(panel, 0.4f, 6, p.panel);
        DrawText(info, static_cast<int>(panel.x) + 7, 11, 10, p.text);
    }

private:
    static Rectangle cellRect(Cell c) {
        return {static_cast<float>(c.x * kCell), static_cast<float>(c.y * kCell), kCell, kCell};
    }
    static Rectangle inset(Rectangle r, float d) { return {r.x + d, r.y + d, r.width - 2 * d, r.height - 2 * d}; }
    static Vector2 center(Cell c) {
        return {c.x * kCell + kCell * 0.5f, c.y * kCell + kCell * 0.5f};
    }
    static void drawEndpoint(Cell c, Color color, const char* label) {
        DrawCircleV(center(c), kCell * 0.4f, color);
        int w = MeasureText(label, 20);
        DrawText(label, c.x * kCell + (kCell - w) / 2, c.y * kCell + 7, 20, RAYWHITE);
    }

    Grid grid_;
    Cell start_{3, 7};
    Cell goal_{21, 9};
    Cell hover_{-1, -1};
    DragMode drag_ = DragMode::None;
    std::unique_ptr<BfsStepper> search_;
};

}  // namespace

int main() {
    static BfsDemo demo;
    ga::DemoConfig config;
    config.title = "BFS - Flood Fill";
    config.width = kCols * kCell;
    config.height = kRows * kCell;
    config.stepsPerSecond = 30.0f;
    return ga::runDemo(demo, config);
}
