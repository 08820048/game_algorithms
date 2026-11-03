// A* 演示：同一张地图上切换 Dijkstra / Greedy / A*
//   - 格子里的数字是堆排序用的优先级：Dijkstra 显示 g，Greedy 显示 h，A* 显示 f = g + h
//     边界上数字最小的格子，就是下一个要展开的
//   - 顶部画笔编辑地形，拖动 S / G 移动起点终点

#include <algorithm>
#include <cstdio>
#include <memory>
#include <string>

#include "../terrain_map.h"
#include "astar.h"
#include "raymath.h"

namespace {

using namespace ga;
using namespace ga::terrain;

class AStarDemo final : public Demo {
public:
    AStarDemo() : grid_(kCols, kRows) {
        // 开口朝向起点的 U 形墙：Greedy 会一头扎进去
        wallRect(grid_, 14, 3, 14, 11);
        wallRect(grid_, 10, 3, 13, 3);
        wallRect(grid_, 10, 11, 13, 11);
        fillRect(grid_, 15, 0, 18, 2, kSwampCost);
        fillRect(grid_, 3, 10, 6, 13, kForestCost);
        reset();
    }

    void reset() override { search_ = std::make_unique<BestFirstStepper>(grid_, start_, goal_, mode_); }
    bool step() override { return search_->step(); }

    void update() override {
        if (editor_.update(grid_, start_, goal_)) notifyProblemChanged();
    }

    void draw() const override {
        const Palette& p = palette();
        const BestFirstStepper& s = *search_;

        // 1. 已展开的格子：按展开时的 g 着色
        int maxCost = 1;
        for (int i = 0; i < grid_.size(); ++i) {
            if (s.closed(grid_.cellAt(i))) maxCost = std::max(maxCost, s.cost(grid_.cellAt(i)));
        }
        for (int i = 0; i < grid_.size(); ++i) {
            Cell c = grid_.cellAt(i);
            if (s.closed(c) && !grid_.isWall(c)) {
                float t = static_cast<float>(s.cost(c)) / static_cast<float>(maxCost);
                DrawRectangleRec(cellRect(c), ColorLerp(p.visitedNear, p.visitedFar, t));
            }
        }

        drawTerrain(grid_);
        std::vector<char> onFrontier(grid_.size(), 0);
        s.forEachFrontier([&](Cell c) {
            onFrontier[grid_.index(c)] = 1;
            DrawRectangleRec(inset(cellRect(c), 5), p.frontier);
        });
        drawGridAndWalls(grid_);

        // 2. 优先级数字
        for (int i = 0; i < grid_.size(); ++i) {
            Cell c = grid_.cellAt(i);
            if (c == start_ || c == goal_ || (!s.closed(c) && !onFrontier[i])) continue;
            char buf[8];
            std::snprintf(buf, sizeof buf, "%d", s.priority(c, s.cost(c)));
            drawTextCentered(buf, center(c), 10, onFrontier[i] ? p.startLabel : p.textMuted);
        }

        if (s.hasCurrent() && !s.found()) DrawRectangleLinesEx(inset(cellRect(s.current()), 2), 2, p.current);
        auto path = s.path();
        drawPath(path, p.path);
        drawOverlay(editor_.hover(), start_, goal_);

        // 3. 顶栏右侧：数字含义与路径代价
        char buf[32];
        float x = kWidth - 16.0f;
        if (s.found()) {
            std::snprintf(buf, sizeof buf, "cost %d", pathCost(grid_, path));
            x = drawRightLabel(x, buf, p.path);
        }
        const char* legend = mode_ == SearchMode::Dijkstra ? "# = g" : mode_ == SearchMode::Greedy ? "# = h" : "# = g+h";
        drawRightLabel(x, legend, p.textMuted);
    }

    void drawUi() override {
        float x = editor_.drawBrushes(16) + 20;
        struct Item { SearchMode mode; const char* label; float w; };
        static constexpr Item kModes[] = {
            {SearchMode::Dijkstra, "dijkstra", 84}, {SearchMode::Greedy, "greedy", 70}, {SearchMode::AStar, "a*", 44},
        };
        for (const Item& it : kModes) {
            if (ui::button({x, 13, it.w, 26}, it.label, mode_ == it.mode) && mode_ != it.mode) {
                mode_ = it.mode;
                notifyProblemChanged();
            }
            x += it.w + 4;
        }
    }

    std::string status() const override {
        const BestFirstStepper& s = *search_;
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
    CostGrid grid_;
    Cell start_{3, 7};
    Cell goal_{21, 7};
    SearchMode mode_ = SearchMode::AStar;
    Editor editor_;
    std::unique_ptr<BestFirstStepper> search_;
};

}  // namespace

int main() {
    static AStarDemo demo;
    ga::DemoConfig config;
    config.title = "A* - Dijkstra vs Greedy vs A*";
    config.width = kWidth;
    config.height = kHeight;
    config.stepsPerSecond = 20.0f;
    return ga::runDemo(demo, config);
}
