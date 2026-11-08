// 视野演示
//   - 拖动红点移动观察者；在别处点击或拖动可以画墙 / 擦墙
//   - exact：精确的阴影投射（地面看中心，墙看是否有视线到达）；classic：经典递归阴影投射；rays：Bresenham 射线
//   - compare：与 exact 比较，黄色是多点亮的格子，描边的是漏掉的格子
//   - 鼠标悬停在一个格子上：画出视线，并显示"我看得见它 / 它看得见我"，检查对称性

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

class FovDemo final : public Demo {
public:
    FovDemo() { newMap(); }

    void reset() override {}
    bool step() override { return true; }

    void update() override {
        Vector2 m = GetMousePosition();
        int cx = int(std::floor((m.x - kOx) / kCell)), cy = int(std::floor((m.y - kOy) / kCell));
        bool in = cx >= 0 && cy >= 0 && cx < kCols && cy < kRows && !ui::wantsMouse();
        hover_ = in ? grid_.index(cx, cy) : -1;
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && in) {
            dragging_ = cx == px_ && cy == py_;
            paint_ = dragging_ ? -1 : !grid_.wall[grid_.index(cx, cy)];
        }
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT) && in) {
            if (dragging_ && !grid_.wall[grid_.index(cx, cy)]) px_ = cx, py_ = cy, dirty_ = true;
            if (paint_ >= 0 && !(cx == px_ && cy == py_) && grid_.wall[grid_.index(cx, cy)] != paint_)
                grid_.wall[grid_.index(cx, cy)] = uint8_t(paint_), dirty_ = true;
        }
        if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) dragging_ = false, paint_ = -1;
        if (dirty_) recompute();
    }

    void draw() const override {
        const Palette& pal = palette();
        const float hl = hairline();
        for (int y = 0; y < kRows; ++y)
            for (int x = 0; x < kCols; ++x) {
                int i = grid_.index(x, y);
                Rectangle r{kOx + x * kCell, kOy + y * kCell, kCell, kCell};
                bool vis = shown_[i], wall = grid_.wall[i];
                if (wall) DrawRectangleRec(r, vis ? pal.wall : Fade(pal.wall, 0.22f));
                else if (!vis) DrawRectangleRec(r, Fade(pal.textMuted, 0.16f));
                if (compare_ && method_ != 0) {
                    if (shown_[i] && !exact_[i]) DrawRectangleRec({r.x + 3, r.y + 3, r.width - 6, r.height - 6}, Fade(pal.frontier, 0.9f));
                    if (!shown_[i] && exact_[i]) DrawRectangleLinesEx({r.x + 2, r.y + 2, r.width - 4, r.height - 4}, 2, pal.path);
                }
            }
        for (int x = 0; x <= kCols; ++x) DrawLineEx({kOx + x * kCell, kOy}, {kOx + x * kCell, kOy + kRows * kCell}, hl, Fade(pal.gridLine, 0.5f));
        for (int y = 0; y <= kRows; ++y) DrawLineEx({kOx, kOy + y * kCell}, {kOx + kCols * kCell, kOy + y * kCell}, hl, Fade(pal.gridLine, 0.5f));
        // 半径
        DrawCircleLinesV(center(px_, py_), radius_ * kCell, Fade(pal.textMuted, 0.5f));
        // 悬停：视线与对称性
        std::string info;
        if (hover_ >= 0 && !grid_.wall[hover_] && hover_ != grid_.index(px_, py_)) {
            int hx = hover_ % kCols, hy = hover_ / kCols;
            bool iSee = shown_[hover_];
            std::vector<uint8_t> back;
            runMethod(hx, hy, back);
            bool itSees = back[grid_.index(px_, py_)];
            DrawLineEx(center(px_, py_), center(hx, hy), 1.5f, iSee ? pal.text : pal.textMuted);
            DrawRectangleLinesEx({kOx + hx * kCell, kOy + hy * kCell, kCell, kCell}, 2, pal.text);
            char buf[96];
            std::snprintf(buf, sizeof buf, "hovered cell: I %s it, it %s me%s", iSee ? "see" : "don't see", itSees ? "sees" : "doesn't see",
                          iSee != itSees ? "   <- one-way!" : "");
            info = buf;
        }
        DrawCircleV(center(px_, py_), kCell * 0.36f, pal.path);
        DrawRectangleLinesEx({kOx, kOy, kCols * kCell, kRows * kCell}, hl * 1.5f, pal.textMuted);

        int visF = 0, extra = 0, missing = 0;
        for (size_t i = 0; i < shown_.size(); ++i) {
            visF += shown_[i] && !grid_.wall[i];
            extra += shown_[i] && !exact_[i] && !grid_.wall[i];
            missing += !shown_[i] && exact_[i] && !grid_.wall[i];
        }
        char buf[160];
        std::snprintf(buf, sizeof buf, "visible floor %d   vs exact: +%d / -%d", visF, extra, missing);
        drawText(buf, {kOx, kOy + kRows * kCell + 10}, 12, pal.text);
        drawText(info.empty() ? "drag the red dot · click elsewhere to draw or erase walls · hover a cell to test symmetry" : info.c_str(),
                 {kOx, kOy + kRows * kCell + 30}, 12, info.find("one-way") != std::string::npos ? pal.path : pal.textMuted);
        DrawLineEx({0, kTop}, {float(kWidth), kTop}, hl, pal.gridLine);
    }

    void drawUi() override {
        const char* names[] = {"exact", "classic", "rays"};
        for (int k = 0; k < 3; ++k)
            if (ui::button({12.0f + k * 74, 13, 70, 26}, names[k], method_ == k)) method_ = k, recompute();
        ui::toggle({240, 13, 92, 26}, "compare", &compare_);
        if (ui::slider({344, 13, 250, 26}, "radius", &radius_, 2, 30, "%.0f", 1)) recompute();
        if (ui::button({606, 13, 88, 26}, "new map")) newMap();
        if (ui::button({698, 13, 90, 26}, "pillars")) pillars();
    }

    std::string status() const override {
        char buf[64];
        std::snprintf(buf, sizeof buf, "%s · radius %d", method_ == 0 ? "exact shadowcast" : method_ == 1 ? "classic" : "rays", int(radius_));
        return buf;
    }

private:
    void runMethod(int x, int y, std::vector<uint8_t>& out) const {
        if (method_ == 0) shadowcast(grid_, x, y, int(radius_), out);
        else if (method_ == 1) classicShadowcast(grid_, x, y, int(radius_), out);
        else rayCast(grid_, x, y, int(radius_), out);
    }
    void recompute() {
        runMethod(px_, py_, shown_);
        shadowcast(grid_, px_, py_, int(radius_), exact_);
        dirty_ = false;
    }
    void newMap() {
        Pcg32 rng(seed_++);
        grid_ = makeDungeon(rng, kCols, kRows, 9);
        // 起点放在看得最远的格子上
        int best = -1;
        std::vector<uint8_t> out;
        for (int i = 0; i < kCols * kRows; ++i)
            if (!grid_.wall[i]) {
                shadowcast(grid_, i % kCols, i / kCols, int(radius_), out);
                int n = 0;
                for (uint8_t v : out) n += v;
                if (n > best) best = n, px_ = i % kCols, py_ = i / kCols;
            }
        recompute();
    }
    // 一个空房间，中间是规则排列的柱子：最容易看出各算法的差别
    void pillars() {
        grid_ = Grid(kCols, kRows);
        for (int x = 0; x < kCols; ++x) grid_.wall[grid_.index(x, 0)] = grid_.wall[grid_.index(x, kRows - 1)] = 1;
        for (int y = 0; y < kRows; ++y) grid_.wall[grid_.index(0, y)] = grid_.wall[grid_.index(kCols - 1, y)] = 1;
        for (int y = 4; y < kRows - 2; y += 4)
            for (int x = 5; x < kCols - 2; x += 5) grid_.wall[grid_.index(x, y)] = 1;
        px_ = 22, py_ = 10;
        recompute();
    }
    Vector2 center(int x, int y) const { return {kOx + (x + 0.5f) * kCell, kOy + (y + 0.5f) * kCell}; }

    Grid grid_;
    std::vector<uint8_t> shown_, exact_;
    int px_ = 1, py_ = 1, method_ = 0, hover_ = -1, paint_ = -1;
    float radius_ = 12;
    bool compare_ = false, dragging_ = false, dirty_ = false;
    uint64_t seed_ = 4;
};

}  // namespace

int main() {
    static FovDemo demo;
    ga::DemoConfig config;
    config.title = "Field of view";
    config.width = kWidth;
    config.height = kHeight;
    config.stepsPerSecond = 30.0f;
    return ga::runDemo(demo, config);
}
