// 流场寻路演示：几百个单位共享同一张流场
//   - 底色：积分场，越深离目标越近（代价越小）
//   - 箭头：流场，每个格子指向代价最小的邻居
//   - 拖动 G 改变目标，或用画笔修改地形：流场立即重算，所有单位马上改道
//   - spawn：在左侧重新生成一群单位

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

#include "../../foundations/random/random.h"
#include "../terrain_map.h"
#include "flowfield.h"
#include "raymath.h"

namespace {

using namespace ga;
using namespace ga::terrain;

constexpr int kUnits = 240;

class FlowFieldDemo final : public Demo {
public:
    FlowFieldDemo() : grid_(kCols, kRows) {
        wallRect(grid_, 8, 0, 8, 9);
        wallRect(grid_, 16, 4, 16, 13);
        wallRect(grid_, 3, 11, 5, 11);
        fillRect(grid_, 10, 0, 14, 3, kSwampCost);
        fillRect(grid_, 10, 9, 13, 13, kForestCost);
        fillRect(grid_, 18, 0, 22, 2, kForestCost);
        reset();
    }

    void reset() override {
        rebuild();
        Pcg32 rng(7);
        agents_.clear();
        while (static_cast<int>(agents_.size()) < kUnits) {
            Vec2 p{0.3f + rng.nextFloat() * 5.4f, 0.3f + rng.nextFloat() * (kRows - 0.6f)};
            if (grid_.passable({static_cast<int>(p.x), static_cast<int>(p.y)})) agents_.push_back({p, {0, 0}});
        }
    }

    bool step() override {
        stepCrowd(grid_, flow_, goal_, agents_, params_, 1.0f / 60.0f);
        return true;  // 模拟没有终点
    }

    void update() override {
        Cell dummyStart{-9, -9};  // 流场只有目标，没有起点
        if (editor_.update(grid_, dummyStart, goal_)) rebuild();  // 只重算流场，单位保持原位
    }

    void draw() const override {
        const Palette& p = palette();

        // 积分场：越接近目标颜色越深
        int maxCost = 1;
        for (int c : field_) {
            if (c != kFlowUnreached) maxCost = std::max(maxCost, c);
        }
        for (int i = 0; i < grid_.size(); ++i) {
            if (field_[i] == kFlowUnreached || grid_.isWall(grid_.cellAt(i))) continue;
            float t = std::sqrt(static_cast<float>(field_[i]) / static_cast<float>(maxCost));
            DrawRectangleRec(cellRect(grid_.cellAt(i)), ColorLerp(p.visitedNear, p.visitedFar, t));
        }
        drawTerrain(grid_);
        drawGridAndWalls(grid_);

        if (showCost_) {
            for (int i = 0; i < grid_.size(); ++i) {
                if (field_[i] == kFlowUnreached || field_[i] == 0) continue;
                char buf[8];
                std::snprintf(buf, sizeof buf, "%d", (field_[i] + 5) / 10);
                drawTextCentered(buf, center(grid_.cellAt(i)), 10, p.textMuted);
            }
        } else if (showArrows_) {
            Color ink = Fade(p.text, 0.42f);
            for (int i = 0; i < grid_.size(); ++i) {
                Vec2 d = flow_[i];
                if (d == Vec2{0, 0}) continue;
                Vector2 c = center(grid_.cellAt(i));
                Vector2 from{c.x - d.x * 8, c.y - d.y * 8}, to{c.x + d.x * 8, c.y + d.y * 8};
                drawArrow(from, to, 1.3f, ink, 5);
            }
        }

        // 单位
        float r = params_.radius * kCell;
        for (const Agent& a : agents_) {
            DrawCircleV({a.pos.x * kCell, kTop + a.pos.y * kCell}, r, p.text);
        }
        drawOverlay(editor_.hover(), {-9, -9}, goal_);

        char buf[32];
        std::snprintf(buf, sizeof buf, "units %d", static_cast<int>(agents_.size()));
        drawRightLabel(kWidth - 16.0f, buf, p.text);
    }

    void drawUi() override {
        float x = editor_.drawBrushes(16) + 20;
        ui::toggle({x, 13, 78, 26}, "arrows", &showArrows_);
        ui::toggle({x + 82, 13, 64, 26}, "cost", &showCost_);
        if (ui::button({x + 156, 13, 64, 26}, "spawn")) reset();
    }

    std::string status() const override {
        int arrived = 0;
        Vec2 g{goal_.x + 0.5f, goal_.y + 0.5f};
        for (const Agent& a : agents_) arrived += distance(a.pos, g) < 2.0f;
        char buf[64];
        std::snprintf(buf, sizeof buf, "units %d · near goal %d", static_cast<int>(agents_.size()), arrived);
        return buf;
    }

private:
    void rebuild() {
        field_ = integrationField(grid_, {goal_});
        flow_ = flowField(grid_, field_);
    }

    CostGrid grid_;
    Cell goal_{22, 10};
    Editor editor_;
    std::vector<int> field_;
    std::vector<Vec2> flow_;
    std::vector<Agent> agents_;
    CrowdParams params_;
    bool showArrows_ = true;
    bool showCost_ = false;
};

}  // namespace

int main() {
    static FlowFieldDemo demo;
    ga::DemoConfig config;
    config.title = "Flow Field Pathfinding";
    config.width = kWidth;
    config.height = kHeight;
    config.stepsPerSecond = 60.0f;
    return ga::runDemo(demo, config);
}
