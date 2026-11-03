// A* 优化演示：启发权重与路径平滑
//   - weight 滑块：f = g + w × h。w 越大，搜索越"急于"奔向终点，展开越少，路径可能变差
//   - smooth 开关：把网格路径拉直成若干段直线（虚线为原始网格路径）
//   - 格子里的数字是 g：从起点走到这里已经花掉的代价

#include <algorithm>
#include <cstdio>
#include <memory>
#include <string>

#include "../terrain_map.h"
#include "pathfinder.h"
#include "raymath.h"
#include "smoothing.h"

namespace {

using namespace ga;
using namespace ga::terrain;

class AStarOptDemo final : public Demo {
public:
    AStarOptDemo() : grid_(kCols, kRows), finder_(grid_) {
        // 两堵交错的墙让路径蛇形穿过，网格路径的台阶感很明显，适合观察平滑；
        // 终点前方是一大片沼泽：最优路线从下方绕过去，权重够大时搜索会直接蹚过去
        wallRect(grid_, 6, 0, 7, 8);
        wallRect(grid_, 12, 5, 13, 13);
        fillRect(grid_, 16, 0, 20, 8, kSwampCost);
        fillRect(grid_, 8, 10, 11, 13, kSwampCost);
        fillRect(grid_, 14, 0, 15, 2, kForestCost);
        reset();
    }

    void reset() override {
        finder_.begin(start_, goal_, weight_);
        optimalCost_ = pathCost(grid_, Pathfinder(grid_).find(start_, goal_, 1.0f));
    }
    bool step() override { return finder_.step(); }

    void update() override {
        if (editor_.update(grid_, start_, goal_)) notifyProblemChanged();
    }

    void draw() const override {
        const Palette& p = palette();
        const Pathfinder& f = finder_;

        int maxCost = 1;
        for (int i = 0; i < grid_.size(); ++i) {
            if (f.closed(grid_.cellAt(i))) maxCost = std::max(maxCost, f.g(grid_.cellAt(i)));
        }
        for (int i = 0; i < grid_.size(); ++i) {
            Cell c = grid_.cellAt(i);
            if (f.closed(c) && !grid_.isWall(c)) {
                float t = static_cast<float>(f.g(c)) / static_cast<float>(maxCost);
                DrawRectangleRec(cellRect(c), ColorLerp(p.visitedNear, p.visitedFar, t));
            }
        }
        drawTerrain(grid_);
        std::vector<char> open(grid_.size(), 0);
        f.forEachOpen([&](Cell c) {
            open[grid_.index(c)] = 1;
            DrawRectangleRec(inset(cellRect(c), 5), p.frontier);
        });
        drawGridAndWalls(grid_);

        for (int i = 0; i < grid_.size(); ++i) {
            Cell c = grid_.cellAt(i);
            if (c == start_ || c == goal_ || (!f.closed(c) && !open[i])) continue;
            char buf[8];
            std::snprintf(buf, sizeof buf, "%d", f.g(c));
            drawTextCentered(buf, center(c), 10, open[i] ? p.startLabel : p.textMuted);
        }

        auto path = f.path();
        char buf[48];
        float x = kWidth - 16.0f;
        if (smooth_ && !path.empty()) {
            auto waypoints = smoothPath(grid_, path);
            drawDashedPath(path, p.text);
            for (size_t i = 1; i < waypoints.size(); ++i) {
                DrawLineEx(center(waypoints[i - 1]), center(waypoints[i]), 3, p.path);
            }
            for (size_t i = 1; i + 1 < waypoints.size(); ++i) {
                DrawCircleV(center(waypoints[i]), 5, p.path);
                DrawCircleV(center(waypoints[i]), 2.5f, p.background);
            }
            std::snprintf(buf, sizeof buf, "%.1f", polylineLength(waypoints));
            x = drawRightLabel(x, buf, p.path);
            std::snprintf(buf, sizeof buf, "length %.1f →", polylineLength(path));
            drawRightLabel(x, buf, p.textMuted);
        } else {
            drawPath(path, p.path);
            if (!path.empty()) {
                int cost = pathCost(grid_, path);
                std::snprintf(buf, sizeof buf, "+%.0f%%", 100.0 * (cost - optimalCost_) / optimalCost_);
                x = drawRightLabel(x, buf, cost > optimalCost_ ? p.path : p.textMuted);
                std::snprintf(buf, sizeof buf, "cost %d", cost);
                drawRightLabel(x, buf, p.text);
            }
        }
        drawOverlay(editor_.hover(), start_, goal_);
    }

    void drawUi() override {
        float x = editor_.drawBrushes(16) + 20;
        if (ui::slider({x, 13, 170, 26}, "w", &weight_, 1.0f, 4.0f, "%.1f", 0.1f)) notifyProblemChanged();
        ui::toggle({x + 182, 13, 82, 26}, "smooth", &smooth_);
    }

    std::string status() const override {
        char buf[96];
        if (finder_.found()) {
            std::snprintf(buf, sizeof buf, "expanded %d · cost %d · optimal %d", finder_.expandedCount(),
                          pathCost(grid_, finder_.path()), optimalCost_);
        } else {
            std::snprintf(buf, sizeof buf, "expanded %d · %s", finder_.expandedCount(),
                          finder_.done() ? "unreachable" : "searching");
        }
        return buf;
    }

private:
    CostGrid grid_;
    Pathfinder finder_;
    Cell start_{2, 11};
    Cell goal_{23, 4};
    float weight_ = 1.0f;
    bool smooth_ = false;
    int optimalCost_ = 0;
    Editor editor_;
};

}  // namespace

int main() {
    static AStarOptDemo demo;
    ga::DemoConfig config;
    config.title = "A* - Weight & Smoothing";
    config.width = kWidth;
    config.height = kHeight;
    config.stepsPerSecond = 30.0f;
    return ga::runDemo(demo, config);
}
