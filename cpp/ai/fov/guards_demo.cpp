// 感知演示：守卫的视野锥 + 视线 + 警觉度
//   - 按住鼠标，红点（你）朝鼠标移动；黑色三角是守卫，来回扫视
//   - 黄色区域是守卫看得见的格子：在视野锥内、距离不超过 range、视线不被墙挡住
//   - 被看见时警觉度上涨（越近涨得越快），看不见时慢慢回落；涨满即"发现"，守卫会一直盯着你

#include <cmath>
#include <cstdio>
#include <string>

#include "demo.h"
#include "fov.h"
#include "maps.h"
#include "ui.h"

namespace {

using namespace ga;
using namespace ga::fov;

constexpr int kWidth = 800, kHeight = 480;
constexpr float kTop = 52;
constexpr int kCols = 48, kRows = 23;
constexpr float kCell = 16;
constexpr float kOx = 16, kOy = kTop + 10;
constexpr float kDt = 1.0f / 60;

struct Guard {
    Viewer v;
    float base, sweep, phase, speed;  // 朝向在 base ± sweep 之间来回摆动
    float awareness = 0;
    bool spotted = false;
};

class GuardsDemo final : public Demo {
public:
    GuardsDemo() {
        Pcg32 rng(11);
        grid_ = makeDungeon(rng, kCols, kRows, 9);
        reset();
    }

    void reset() override {
        for (int i = 0; i < kCols * kRows; ++i)
            if (!grid_.wall[i]) {
                px_ = i % kCols + 0.5f, py_ = i / kCols + 0.5f;
                break;
            }
        // 守卫放在离起点较远、并且看不到起点的地面格子上
        Pcg32 rng(5);
        guards_.clear();
        int tries = 0;
        while (guards_.size() < 4 && tries++ < 5000) {
            int x = rng.below(kCols), y = rng.below(kRows);
            if (grid_.wall[grid_.index(x, y)] || std::hypot(x - px_, y - py_) < 12 || canSee(grid_, x, y, int(px_), int(py_))) continue;
            bool far = true;
            for (const Guard& g : guards_) far &= std::hypot(g.v.x - x, g.v.y - y) > 9;
            if (!far) continue;
            float base = float(rng.nextDouble() * 6.2831853);
            guards_.push_back({{x + 0.5f, y + 0.5f, base, 0, 0}, base, 0.9f, float(rng.nextDouble() * 6.28), 0.5f + float(rng.nextDouble()) * 0.4f});
        }
        time_ = 0;
    }

    bool step() override {
        time_ += kDt;
        if (moving_) {  // 朝鼠标移动，每帧最多 0.09 格，按坐标轴分别检查碰撞（可以贴着墙滑动）
            float dx = mx_ - px_, dy = my_ - py_, d = std::hypot(dx, dy);
            if (d > 0.1f) {
                float s = std::min(d, 0.09f) / d;
                float nx = px_ + dx * s, ny = py_ + dy * s;
                if (!grid_.blocked(int(nx), int(py_))) px_ = nx;
                if (!grid_.blocked(int(px_), int(ny))) py_ = ny;
            }
        }
        int cx = int(px_), cy = int(py_);
        for (Guard& g : guards_) {
            g.v.halfAngle = halfDeg_ * 3.14159265f / 180, g.v.range = range_;
            bool see = sees(grid_, g.v, cx, cy);
            float dist = std::hypot(px_ - g.v.x, py_ - g.v.y);
            g.awareness = updateAwareness(g.awareness, see, dist, g.v.range, kDt);
            if (g.awareness >= 1) g.spotted = true;
            if (g.awareness <= 0) g.spotted = false;
            if (g.spotted && see) g.v.facing = std::atan2(py_ - g.v.y, px_ - g.v.x);  // 盯住
            else if (!g.spotted) g.v.facing = g.base + g.sweep * std::sin(time_ * g.speed + g.phase);
        }
        return true;
    }

    void update() override {
        Vector2 m = GetMousePosition();
        moving_ = IsMouseButtonDown(MOUSE_BUTTON_LEFT) && !ui::wantsMouse() && m.y > kTop;
        mx_ = (m.x - kOx) / kCell, my_ = (m.y - kOy) / kCell;
    }

    void draw() const override {
        const Palette& pal = palette();
        const float hl = hairline();
        // 每个守卫看得见的格子：先用阴影投射求出视野，再筛掉锥外的格子
        std::vector<uint8_t> seen(grid_.wall.size(), 0), fovOut;
        for (const Guard& g : guards_) {
            shadowcast(grid_, int(g.v.x), int(g.v.y), int(std::ceil(g.v.range)), fovOut);
            for (int i = 0; i < kCols * kRows; ++i)
                if (fovOut[i] && !grid_.wall[i] && inCone(g.v, i % kCols + 0.5f, i / kCols + 0.5f)) seen[i] = 1;
        }
        for (int y = 0; y < kRows; ++y)
            for (int x = 0; x < kCols; ++x) {
                int i = grid_.index(x, y);
                Rectangle r{kOx + x * kCell, kOy + y * kCell, kCell, kCell};
                if (grid_.wall[i]) DrawRectangleRec(r, pal.wall);
                else if (showCones_ && seen[i]) DrawRectangleRec(r, Fade(pal.frontier, 0.45f));
            }
        for (int x = 0; x <= kCols; ++x) DrawLineEx({kOx + x * kCell, kOy}, {kOx + x * kCell, kOy + kRows * kCell}, hl, Fade(pal.gridLine, 0.5f));
        for (int y = 0; y <= kRows; ++y) DrawLineEx({kOx, kOy + y * kCell}, {kOx + kCols * kCell, kOy + y * kCell}, hl, Fade(pal.gridLine, 0.5f));

        int cx = int(px_), cy = int(py_);
        for (const Guard& g : guards_) {
            Vector2 c = pos(g.v.x, g.v.y);
            bool see = sees(grid_, g.v, cx, cy);
            if (see) DrawLineEx(c, pos(px_, py_), 1.5f, g.spotted ? pal.path : pal.text);
            float f = g.v.facing;
            DrawLineEx(c, {c.x + std::cos(f) * 13, c.y + std::sin(f) * 13}, 3, pal.text);  // 朝向
            DrawCircleV(c, 6, pal.text);
            DrawCircleLinesV(c, 6, pal.background);
            // 警觉度条
            Rectangle bar{c.x - 12, c.y - 18, 24, 4};
            DrawRectangleRec(bar, pal.background);
            DrawRectangleRec({bar.x, bar.y, bar.width * g.awareness, bar.height}, g.spotted ? pal.path : pal.text);
            DrawRectangleLinesEx(bar, 1, pal.textMuted);
            if (g.spotted) drawTextCentered("!", {c.x, c.y - 28}, 14, pal.path);
        }
        DrawCircleV(pos(px_, py_), kCell * 0.36f, pal.path);
        DrawRectangleLinesEx({kOx, kOy, kCols * kCell, kRows * kCell}, hl * 1.5f, pal.textMuted);

        int spotted = 0;
        for (const Guard& g : guards_) spotted += g.spotted;
        const char* msg = spotted ? "spotted! break line of sight and wait for the bars to drain"
                                  : "hold the mouse button to move the red dot. stay out of the yellow";
        drawText(msg, {kOx, kOy + kRows * kCell + 10}, 12, spotted ? pal.path : pal.text);
        drawText("guard sees you = within range + inside the cone + line of sight. awareness rises faster up close", {kOx, kOy + kRows * kCell + 30},
                 11, pal.textMuted);
        DrawLineEx({0, kTop}, {float(kWidth), kTop}, hl, pal.gridLine);
    }

    void drawUi() override {
        ui::slider({12, 13, 250, 26}, "cone", &halfDeg_, 10, 90, "%.0f deg", 5);
        ui::slider({272, 13, 220, 26}, "range", &range_, 3, 16, "%.0f", 1);
        ui::toggle({504, 13, 110, 26}, "show cones", &showCones_);
        if (ui::button({624, 13, 80, 26}, "reset")) reset();
    }

    std::string status() const override {
        char buf[64];
        float maxA = 0;
        for (const Guard& g : guards_) maxA = std::max(maxA, g.awareness);
        std::snprintf(buf, sizeof buf, "max awareness %.0f%%", maxA * 100);
        return buf;
    }

private:
    Vector2 pos(float x, float y) const { return {kOx + x * kCell, kOy + y * kCell}; }

    Grid grid_;
    std::vector<Guard> guards_;
    float px_ = 1.5f, py_ = 1.5f, mx_ = 0, my_ = 0, time_ = 0;
    float halfDeg_ = 35, range_ = 9;
    bool moving_ = false, showCones_ = true;
};

}  // namespace

int main() {
    static GuardsDemo demo;
    ga::DemoConfig config;
    config.title = "Guards: perception";
    config.width = kWidth;
    config.height = kHeight;
    config.stepsPerSecond = 60.0f;
    return ga::runDemo(demo, config);
}
