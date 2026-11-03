// JPS 演示：8 方向网格上切换 A* / JPS
//   - 浅色格子：被"扫描"过的格子（JPS 的 jump 沿直线走过的格子 / A* 中被发现的格子）
//   - 深色格子：被展开的节点（A* 中是每个格子，JPS 中只有跳点）
//   - 细线：每个节点指向它的父节点。JPS 中这些线很长，因为跳点之间隔着整段直线
//   - 顶部画笔 wall / erase 编辑地图，拖动 S / G 移动起点终点

#include <cstdio>
#include <memory>
#include <string>

#include "../terrain_map.h"
#include "jps.h"
#include "raymath.h"

namespace {

using namespace ga;
using namespace ga::terrain;

class JpsDemo final : public Demo {
public:
    JpsDemo() : grid_(kCols, kRows), editor_(false) {
        // 几个房间和柱子：大片空地正是 JPS 擅长的场景
        wallRect(grid_, 8, 0, 8, 9);
        wallRect(grid_, 12, 5, 20, 5);
        wallRect(grid_, 16, 8, 16, 13);
        wallRect(grid_, 3, 9, 5, 10);
        wallRect(grid_, 20, 1, 21, 2);
        wallRect(grid_, 11, 9, 12, 10);
        reset();
    }

    void reset() override {
        search_ = std::make_unique<Grid8Search>(grid_, start_, goal_, mode_);
        // JPS 的一步要跳过整段直线，放慢一些才看得清；A* 的步数多得多，放快一些
        setStepsPerSecond(mode_ == Grid8Mode::Jps ? 5.0f : 40.0f);
    }
    bool step() override { return search_->step(); }

    void update() override {
        if (editor_.update(grid_, start_, goal_)) notifyProblemChanged();
    }

    void draw() const override {
        const Palette& p = palette();
        const Grid8Search& s = *search_;

        for (int i = 0; i < grid_.size(); ++i) {
            Cell c = grid_.cellAt(i);
            if (grid_.isWall(c)) continue;
            bool touched = mode_ == Grid8Mode::Jps ? s.scanned(c) : s.g(c) >= 0;
            if (s.closed(c)) DrawRectangleRec(cellRect(c), p.visitedNear);
            else if (touched) DrawRectangleRec(cellRect(c), p.visitedFar);
        }
        s.forEachOpen([&](Cell c) { DrawRectangleRec(inset(cellRect(c), 5), p.frontier); });
        drawGridAndWalls(grid_);

        // 搜索树：每个节点连向父节点
        Color tree = Fade(p.text, 0.45f);
        for (int i = 0; i < grid_.size(); ++i) {
            Cell c = grid_.cellAt(i), parent;
            if (s.g(c) >= 0 && s.parent(c, &parent)) DrawLineEx(center(parent), center(c), 1.2f, tree);
        }
        if (s.hasCurrent() && !s.found()) DrawRectangleLinesEx(inset(cellRect(s.current()), 2), 2, p.current);

        auto nodes = s.nodes();
        auto path = expandJumpPath(nodes);
        drawPath(path, p.path);
        if (mode_ == Grid8Mode::Jps) {
            for (size_t i = 1; i + 1 < nodes.size(); ++i) {
                DrawCircleV(center(nodes[i]), 6, p.path);
                DrawCircleV(center(nodes[i]), 3, p.background);
            }
        }
        drawOverlay(editor_.hover(), start_, goal_);

        if (s.found()) {
            char buf[32];
            std::snprintf(buf, sizeof buf, "cost %.1f", s.g(goal_) / static_cast<float>(kStraight));
            drawRightLabel(kWidth - 16.0f, buf, p.path);
        }
    }

    void drawUi() override {
        float x = editor_.drawBrushes(16) + 20;
        if (ui::button({x, 13, 50, 26}, "a*", mode_ == Grid8Mode::AStar) && mode_ != Grid8Mode::AStar) {
            mode_ = Grid8Mode::AStar;
            notifyProblemChanged();
        }
        if (ui::button({x + 54, 13, 50, 26}, "jps", mode_ == Grid8Mode::Jps) && mode_ != Grid8Mode::Jps) {
            mode_ = Grid8Mode::Jps;
            notifyProblemChanged();
        }
    }

    std::string status() const override {
        const Grid8Search& s = *search_;
        char buf[96];
        const char* state = s.found() ? "found" : s.done() ? "unreachable" : "searching";
        if (mode_ == Grid8Mode::Jps) {
            std::snprintf(buf, sizeof buf, "expanded %d · scanned %d · %s", s.expandedCount(), s.scanCount(), state);
        } else {
            std::snprintf(buf, sizeof buf, "expanded %d · %s", s.expandedCount(), state);
        }
        return buf;
    }

private:
    CostGrid grid_;  // 代价全为 1，只用到墙
    Cell start_{2, 2};
    Cell goal_{22, 11};
    Grid8Mode mode_ = Grid8Mode::Jps;
    Editor editor_;
    std::unique_ptr<Grid8Search> search_;
};

}  // namespace

int main() {
    static JpsDemo demo;
    ga::DemoConfig config;
    config.title = "JPS - Jump Point Search";
    config.width = kWidth;
    config.height = kHeight;
    config.stepsPerSecond = 5.0f;
    return ga::runDemo(demo, config);
}
