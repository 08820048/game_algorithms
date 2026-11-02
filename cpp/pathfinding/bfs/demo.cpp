// BFS 洪水填充演示
//   - 鼠标左键在空地上拖动：画墙；在墙上拖动：擦墙
//   - 拖动绿色 S / 红色 G：移动起点 / 终点
//   - 格子里的数字：该格到起点的步数（BFS 访问的"波次"）

#include <cstdio>
#include <memory>
#include <string>

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
        const float hl = hairline();

        // 计算最大距离，用于已访问格子的颜色渐变
        int maxDist = 1;
        for (int i = 0; i < grid_.size(); ++i) maxDist = std::max(maxDist, s.distance(grid_.cellAt(i)));

        // 1. 已访问格子：离起点越近颜色越深，形成"等高线"
        for (int i = 0; i < grid_.size(); ++i) {
            Cell c = grid_.cellAt(i);
            int d = s.distance(c);
            if (d >= 0 && !grid_.isWall(c)) {
                float t = static_cast<float>(d) / static_cast<float>(maxDist);
                DrawRectangleRec(cellRect(c), ColorLerp(p.visitedNear, p.visitedFar, t));
            }
        }

        // 2. 边界（队列中的格子）：内缩的实心方块
        for (Cell c : s.frontier()) DrawRectangleRec(inset(cellRect(c), 5), p.frontier);

        // 3. 网格细线（一个物理像素）
        for (int x = 0; x <= kCols; ++x) {
            DrawLineEx({x * float(kCell), 0}, {x * float(kCell), kRows * float(kCell)}, hl, p.gridLine);
        }
        for (int y = 0; y <= kRows; ++y) {
            DrawLineEx({0, y * float(kCell)}, {kCols * float(kCell), y * float(kCell)}, hl, p.gridLine);
        }

        // 4. 墙：画在网格线之上，保持整块墨色
        for (int i = 0; i < grid_.size(); ++i) {
            Cell c = grid_.cellAt(i);
            if (grid_.isWall(c)) DrawRectangleRec(cellRect(c), p.wall);
        }

        // 5. 距离数字（边界格子上用深色，保证在赭黄底上可读）
        std::vector<char> onFrontier(grid_.size(), 0);
        for (Cell c : s.frontier()) onFrontier[grid_.index(c)] = 1;
        for (int i = 0; i < grid_.size(); ++i) {
            Cell c = grid_.cellAt(i);
            int d = s.distance(c);
            if (d <= 0 || c == goal_) continue;
            char buf[8];
            std::snprintf(buf, sizeof buf, "%d", d);
            drawTextCentered(buf, center(c), 10, onFrontier[i] ? p.startLabel : p.textMuted);
        }

        // 6. 当前正在展开的格子：墨色细框
        if (s.hasCurrent() && !s.found()) {
            DrawRectangleLinesEx(inset(cellRect(s.current()), 2), 2, p.current);
        }

        // 7. 找到的路径：朱红线
        auto path = s.path();
        for (size_t i = 1; i < path.size(); ++i) {
            DrawLineEx(center(path[i - 1]), center(path[i]), 3, p.path);
        }
        for (size_t i = 1; i + 1 < path.size(); ++i) DrawCircleV(center(path[i]), 3.5f, p.path);

        // 8. 鼠标悬停
        if (hover_.x >= 0) DrawRectangleRec(cellRect(hover_), p.hover);

        // 9. 起点（墨）/ 终点（朱红）
        drawEndpoint(start_, p.start, p.startLabel, "S");
        drawEndpoint(goal_, p.goal, p.goalLabel, "G");
    }

    std::string status() const override {
        const BfsStepper& s = *search_;
        const char* state = s.found() ? "found" : (s.done() ? "unreachable" : "searching");
        char buf[96];
        if (s.found()) {
            std::snprintf(buf, sizeof buf, "expanded %d \u00b7 path %d \u00b7 %s", s.expandedCount(),
                          static_cast<int>(s.path().size()) - 1, state);
        } else {
            std::snprintf(buf, sizeof buf, "expanded %d \u00b7 frontier %d \u00b7 %s", s.expandedCount(),
                          static_cast<int>(s.frontier().size()), state);
        }
        return buf;
    }

private:
    static Rectangle cellRect(Cell c) {
        return {static_cast<float>(c.x * kCell), static_cast<float>(c.y * kCell), kCell, kCell};
    }
    static Rectangle inset(Rectangle r, float d) { return {r.x + d, r.y + d, r.width - 2 * d, r.height - 2 * d}; }
    static Vector2 center(Cell c) {
        return {c.x * kCell + kCell * 0.5f, c.y * kCell + kCell * 0.5f};
    }
    static void drawEndpoint(Cell c, Color fill, Color label, const char* text) {
        DrawCircleV(center(c), kCell * 0.36f, fill);
        drawTextCentered(text, center(c), 14, label);
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
