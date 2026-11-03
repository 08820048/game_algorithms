// Dijkstra 演示
//   - 顶部选择画笔：wall / forest ×3 / swamp ×6 / erase，在格子上拖动绘制
//   - 拖动 S / G：移动起点 / 终点
//   - 格子里的数字：到起点的最小代价（灰色 = 已出堆、确定；深色 = 边界上的暂定值）
//   - 虚线：同一张地图上 BFS 找到的路径（步数最少，但无视地形代价）

#include <algorithm>
#include <cstdio>
#include <memory>
#include <string>

#include "../terrain_map.h"
#include "dijkstra.h"
#include "raymath.h"

namespace {

using namespace ga;
using namespace ga::terrain;

class DijkstraDemo final : public Demo {
public:
    DijkstraDemo() : grid_(kCols, kRows) {
        // 一条竖着的沼泽带挡在 S、G 之间，下方留出平地：
        // BFS 会直穿沼泽（步数最少），Dijkstra 会绕到下面（代价最小）
        fillRect(grid_, 11, 0, 13, 9, kSwampCost);
        fillRect(grid_, 4, 9, 7, 12, kForestCost);
        fillRect(grid_, 16, 2, 20, 4, kForestCost);
        wallRect(grid_, 7, 1, 7, 5);
        wallRect(grid_, 17, 12, 22, 12);
        reset();
    }

    void reset() override {
        search_ = std::make_unique<DijkstraStepper>(grid_, start_, goal_);
        bfsPath_ = bfsPath(grid_, start_, goal_);
    }

    bool step() override { return search_->step(); }

    void update() override {
        if (editor_.update(grid_, start_, goal_)) notifyProblemChanged();
    }

    void draw() const override {
        const Palette& p = palette();
        const DijkstraStepper& s = *search_;

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

        // 2. 地形、边界、网格与墙
        drawTerrain(grid_);
        s.forEachFrontier([&](Cell c) { DrawRectangleRec(inset(cellRect(c), 5), p.frontier); });
        drawGridAndWalls(grid_);

        // 3. 代价数字：已确定的用灰色，边界上（暂定值）用深色
        std::vector<char> onFrontier(grid_.size(), 0);
        s.forEachFrontier([&](Cell c) { onFrontier[grid_.index(c)] = 1; });
        for (int i = 0; i < grid_.size(); ++i) {
            Cell c = grid_.cellAt(i);
            if (c == start_ || c == goal_ || (!s.closed(c) && !onFrontier[i])) continue;
            char buf[8];
            std::snprintf(buf, sizeof buf, "%d", s.cost(c));
            drawTextCentered(buf, center(c), 10, onFrontier[i] ? p.startLabel : p.textMuted);
        }

        // 4. 当前展开的格子、BFS 路径（虚线）、Dijkstra 路径（朱红）
        if (s.hasCurrent() && !s.found()) DrawRectangleLinesEx(inset(cellRect(s.current()), 2), 2, p.current);
        if (showBfs_) drawDashedPath(bfsPath_, p.text);
        auto path = s.path();
        drawPath(path, p.path);
        drawOverlay(editor_.hover(), start_, goal_);

        // 5. 顶栏右侧：两条路径的总代价
        char buf[32];
        float x = kWidth - 16.0f;
        if (s.found()) {
            std::snprintf(buf, sizeof buf, "dijkstra %d", pathCost(grid_, path));
            x = drawRightLabel(x, buf, p.path);
        }
        if (showBfs_ && !bfsPath_.empty()) {
            std::snprintf(buf, sizeof buf, "bfs %d", pathCost(grid_, bfsPath_));
            x = drawRightLabel(x, buf, p.text);
            drawRightLabel(x, "cost", p.textMuted);
        }
    }

    void drawUi() override {
        float x = editor_.drawBrushes(16);
        ui::toggle({x + 20, 13, 60, 26}, "bfs", &showBfs_);
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
    CostGrid grid_;
    Cell start_{2, 5};
    Cell goal_{22, 5};
    Editor editor_;
    bool showBfs_ = true;
    std::vector<Cell> bfsPath_;
    std::unique_ptr<DijkstraStepper> search_;
};

}  // namespace

int main() {
    static DijkstraDemo demo;
    ga::DemoConfig config;
    config.title = "Dijkstra - Weighted Grid";
    config.width = kWidth;
    config.height = kHeight;
    config.stepsPerSecond = 30.0f;
    return ga::runDemo(demo, config);
}
