// 导航网格与漏斗算法演示
//   - 拖动 S / G 移动起点终点（只能放在空地上）
//   - 浅色区域：A* 在多边形之间找到的通道；赭黄线段：通道依次穿过的门
//   - 每一步处理一扇门：墨色两条线是漏斗的左右边，圆圈是漏斗尖，朱红线是已经确定的路径
//   - mesh：显示导航网格的多边形；midpoints：对照"直接连接门中点"的路径

#include <cstdio>
#include <memory>
#include <string>

#include "demo.h"
#include "funnel.h"
#include "navmesh.h"
#include "ui.h"

namespace {

using namespace ga;

constexpr int kWidth = 800;
constexpr int kHeight = 500;
constexpr float kTop = 52;

inline Vector2 rl(Vec2 v) { return {v.x, v.y}; }

const std::vector<Polygon>& obstacles() {
    static const std::vector<Polygon> kObstacles{
        {{140, 120}, {230, 100}, {250, 260}, {160, 280}},
        {{330, 340}, {450, 470}, {300, 480}},
        {{360, 95}, {420, 95}, {420, 280}, {360, 280}},
        {{635, 215}, {600, 276}, {530, 276}, {495, 215}, {530, 154}, {600, 154}},
        {{650, 330}, {760, 360}, {740, 440}, {630, 420}},
        {{60, 370}, {180, 340}, {150, 460}},
        {{500, 330}, {560, 330}, {560, 470}, {500, 470}},
    };
    return kObstacles;
}

class NavMeshDemo final : public Demo {
public:
    NavMeshDemo() : mesh_(buildNavMesh({0, kTop}, {kWidth, kHeight}, obstacles())) { reset(); }

    void reset() override {
        channel_ = findChannel(mesh_, start_, goal_);
        portals_ = channelPortals(mesh_, channel_, start_, goal_);
        funnel_ = std::make_unique<Funnel>(portals_);
        finalLength_ = pathLength(stringPull(portals_));
    }
    bool step() override { return funnel_->step(); }

    void update() override {
        Vector2 m = GetMousePosition();
        Vec2 p{m.x, m.y};
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && !ui::wantsMouse()) {
            if (distance(p, start_) < 16) dragging_ = &start_;
            else if (distance(p, goal_) < 16) dragging_ = &goal_;
        }
        if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) dragging_ = nullptr;
        if (dragging_ && !(p == *dragging_) && mesh_.findPoly(p) >= 0) {
            *dragging_ = p;
            notifyProblemChanged();
        }
    }

    void draw() const override {
        const Palette& p = palette();
        const float hl = hairline();

        // 通道里的多边形
        for (int idx : channel_) {
            const auto& v = mesh_.polys[idx].verts;
            for (size_t k = 1; k + 1 < v.size(); ++k) DrawTriangle(rl(v[0]), rl(v[k + 1]), rl(v[k]), p.visitedNear);
        }
        // 导航网格的多边形轮廓
        if (showMesh_) {
            for (const NavPoly& poly : mesh_.polys) {
                for (size_t k = 0; k < poly.verts.size(); ++k) {
                    DrawLineEx(rl(poly.verts[k]), rl(poly.verts[(k + 1) % poly.verts.size()]), 1.0f, Fade(p.text, 0.18f));
                }
            }
        }
        // 障碍物
        for (const Polygon& o : obstacles()) {
            for (size_t k = 1; k + 1 < o.size(); ++k) {
                DrawTriangle(rl(o[0]), rl(o[k + 1]), rl(o[k]), p.wall);
                DrawTriangle(rl(o[0]), rl(o[k]), rl(o[k + 1]), p.wall);  // 两种绕序都画，避免被背面剔除
            }
        }
        DrawLineEx({0, kTop}, {float(kWidth), kTop}, hl, p.gridLine);

        // 门：已处理的用浅色，当前这扇用朱红
        const Funnel& f = *funnel_;
        for (size_t i = 1; i + 1 < portals_.size(); ++i) {
            Color c = i == f.index() ? p.path : (i < f.index() ? Fade(p.frontier, 0.55f) : p.frontier);
            DrawLineEx(rl(portals_[i].left), rl(portals_[i].right), 3, c);
            DrawCircleV(rl(portals_[i].left), 3, p.text);  // 左端点：实心
            DrawCircleLinesV(rl(portals_[i].right), 3, p.text);  // 右端点：空心
        }

        if (showMidpoints_) {
            auto mid = midpointPath(portals_);
            for (size_t i = 1; i < mid.size(); ++i) drawDashedLine(rl(mid[i - 1]), rl(mid[i]), 1.6f, p.text, 5, 4);
        }

        // 漏斗：从漏斗尖出发的左右两条边
        bool done = f.index() >= portals_.size();
        if (!done) {
            drawArrow(rl(f.apex()), rl(f.left()), 1.6f, p.text, 7);
            drawDashedLine(rl(f.apex()), rl(f.right()), 1.6f, p.text, 6, 3);
            DrawCircleLinesV(rl(f.apex()), 9, p.text);
        }
        const auto& path = f.path();
        for (size_t i = 1; i < path.size(); ++i) DrawLineEx(rl(path[i - 1]), rl(path[i]), 3, p.path);
        for (size_t i = 1; i + 1 < path.size(); ++i) DrawCircleV(rl(path[i]), 4.5f, p.path);

        drawEndpoint(start_, p.start, p.startLabel, "S");
        drawEndpoint(goal_, p.goal, p.goalLabel, "G");

        // 顶栏右侧：路径长度对比
        char buf[48];
        float x = kWidth - 16.0f;
        if (done) {
            std::snprintf(buf, sizeof buf, "funnel %.0f", pathLength(path));
            Vector2 sz = measureText(buf, 13);
            drawText(buf, {x - sz.x, 20}, 13, p.path);
            x -= sz.x + 12;
        }
        if (showMidpoints_) {
            std::snprintf(buf, sizeof buf, "midpoints %.0f", pathLength(midpointPath(portals_)));
            Vector2 sz = measureText(buf, 13);
            drawText(buf, {x - sz.x, 20}, 13, p.text);
        }
    }

    void drawUi() override {
        ui::toggle({16, 13, 70, 26}, "mesh", &showMesh_);
        ui::toggle({92, 13, 110, 26}, "midpoints", &showMidpoints_);
    }

    std::string status() const override {
        char buf[96];
        size_t i = funnel_->index();
        if (i >= portals_.size()) {
            std::snprintf(buf, sizeof buf, "polys %zu · channel %zu · length %.0f", mesh_.polys.size(), channel_.size(),
                          finalLength_);
        } else {
            std::snprintf(buf, sizeof buf, "portal %zu / %zu · corners %zu", i, portals_.size() - 1,
                          funnel_->path().size() - 1);
        }
        return buf;
    }

private:
    static void drawEndpoint(Vec2 c, Color fill, Color label, const char* text) {
        DrawCircleV(rl(c), 12, fill);
        drawTextCentered(text, rl(c), 14, label);
    }

    NavMesh mesh_;
    Vec2 start_{60, 300};
    Vec2 goal_{740, 120};
    Vec2* dragging_ = nullptr;
    std::vector<int> channel_;
    std::vector<Portal> portals_;
    std::unique_ptr<Funnel> funnel_;
    float finalLength_ = 0;
    bool showMesh_ = true;
    bool showMidpoints_ = false;
};

}  // namespace

int main() {
    static NavMeshDemo demo;
    ga::DemoConfig config;
    config.title = "NavMesh & Funnel";
    config.width = kWidth;
    config.height = kHeight;
    config.stepsPerSecond = 2.5f;
    return ga::runDemo(demo, config);
}
