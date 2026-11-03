// 向量演示：守卫的视野判断
//   - 拖动红点（玩家）、黑点（守卫）、守卫前方的箭头（改变朝向）
//   - 播放时守卫左右扫视
//   - 左下角实时显示点积、叉积、夹角和局部坐标

#include <cmath>
#include <cstdio>
#include <string>

#include "demo.h"
#include "transform.h"
#include "ui.h"
#include "vision.h"

namespace {

using namespace ga;

constexpr int kWidth = 800;
constexpr int kHeight = 500;
constexpr float kTop = 52;
constexpr float kHandleLen = 52;

enum class Drag { None, Player, Guard, Handle };

Vector2 rl(Vec2 v) { return {v.x, v.y}; }

class VectorsDemo final : public Demo {
public:
    VectorsDemo() { reset(); }

    void reset() override {
        sweep_ = 0;
        guard_.rotation = baseAngle_;
    }

    bool step() override {
        sweep_ += 1.0f / 60.0f;
        guard_.rotation = baseAngle_ + 0.55f * std::sin(sweep_ * 1.1f);
        return true;
    }

    void update() override {
        Vector2 m = GetMousePosition();
        Vec2 mouse{m.x, std::max(m.y, kTop + 8)};
        if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) drag_ = Drag::None;

        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && !ui::wantsMouse() && m.y > kTop) {
            if (distance(mouse, handleTip()) < 14) {
                drag_ = Drag::Handle;
            } else if (distance(mouse, player_) < 16) {
                drag_ = Drag::Player;
            } else if (distance(mouse, guard_.position) < 20) {
                drag_ = Drag::Guard;
            } else {
                drag_ = Drag::Player;  // 点空白处：把玩家移过去
                player_ = mouse;
            }
        }
        if (!IsMouseButtonDown(MOUSE_BUTTON_LEFT)) return;
        switch (drag_) {
            case Drag::Player: player_ = mouse; break;
            case Drag::Guard: guard_.position = mouse; break;
            case Drag::Handle:
                baseAngle_ = angleOf(mouse - guard_.position);
                guard_.rotation = baseAngle_;
                sweep_ = 0;
                break;
            case Drag::None: break;
        }
    }

    void draw() const override {
        const Palette& p = palette();
        const float hl = hairline();

        // 背景网格
        for (int x = 0; x <= kWidth; x += 40) DrawLineEx({float(x), kTop}, {float(x), float(kHeight)}, hl, p.gridLine);
        for (int y = int(kTop) + 8; y <= kHeight; y += 40) DrawLineEx({0, float(y)}, {float(kWidth), float(y)}, hl, p.gridLine);
        DrawLineEx({0, kTop}, {float(kWidth), kTop}, hl, p.gridLine);

        const Vec2 f = guard_.forward();
        const Vec2 r = guard_.right();
        const Vec2 g = guard_.position;
        const Vec2 to = player_ - g;
        const VisionCone cone = visionCone();
        const bool seen = canSee(cone, player_);
        const float d = dot(f, to);

        // 1. 视野锥
        float a0 = (guard_.rotation - cone.halfAngle) * RAD2DEG;
        float a1 = (guard_.rotation + cone.halfAngle) * RAD2DEG;
        DrawCircleSector(rl(g), cone.range, a0, a1, 48, Fade(seen ? p.goal : p.wall, seen ? 0.10f : 0.05f));
        DrawCircleSectorLines(rl(g), cone.range, a0, a1, 48, Fade(seen ? p.goal : p.textMuted, 0.6f));

        // 2. 前方轴所在的直线，以及 to 在前方轴上的投影（= 点积）
        drawDashedLine(rl(g - f * 900), rl(g + f * 900), hl, p.gridLine, 3, 5);
        Vec2 foot = g + f * d;
        DrawLineEx(rl(g), rl(foot), 3, d >= 0 ? p.frontier : p.textMuted);
        drawDashedLine(rl(foot), rl(player_), hl * 1.5f, p.textMuted, 4, 3);
        // 直角标记
        if (std::abs(d) > 14 && length(player_ - foot) > 14) {
            Vec2 back = f * (d > 0 ? -8.0f : 8.0f);
            Vec2 side = normalized(player_ - foot) * 8.0f;
            DrawLineEx(rl(foot + back), rl(foot + back + side), hl, p.textMuted);
            DrawLineEx(rl(foot + back + side), rl(foot + side), hl, p.textMuted);
        }

        // 3. 指向玩家的向量
        drawArrow(rl(g), rl(player_ - normalized(to) * 11), 1.5f, p.text, 9);

        // 4. 守卫的局部坐标轴：forward（可拖动）和 right
        drawArrow(rl(g), rl(handleTip()), 2.5f, p.text, 11);
        drawArrow(rl(g), rl(g + r * 34), 1.5f, p.textMuted, 8);
        drawTextCentered("f", rl(handleTip() + f * 12), 11, p.text);
        drawTextCentered("r", rl(g + r * 46), 11, p.textMuted);

        // 5. 守卫和玩家
        DrawCircleV(rl(g), 13, p.start);
        DrawCircleV(rl(handleTip()), 5, p.start);
        if (seen) {
            DrawCircleV(rl(player_), 10, p.goal);
        } else {
            DrawCircleV(rl(player_), 10, p.background);
            DrawRing(rl(player_), 8.5f, 10, 0, 360, 32, p.goal);
        }

        drawReadout(seen);

        // 视野锥可能延伸到顶部控件区，用底色盖住
        DrawRectangleRec({0, 0, float(kWidth), kTop}, p.background);
        DrawLineEx({0, kTop}, {float(kWidth), kTop}, hl, p.gridLine);
    }

    void drawUi() override {
        ui::slider({16, 13, 240, 26}, "fov", &fovDeg_, 20, 170, "%.0f°", 1);
        ui::slider({280, 13, 240, 26}, "range", &range_, 80, 420, "%.0f", 1);
        const Palette& p = palette();
        drawText("drag the dots and the arrow", {kWidth - 216.0f, 20}, 11, p.textMuted);
    }

    std::string status() const override {
        Vec2 to = player_ - guard_.position;
        char buf[96];
        std::snprintf(buf, sizeof buf, "dot %.0f · cross %.0f · %s", dot(guard_.forward(), to),
                      cross(guard_.forward(), to), canSee(visionCone(), player_) ? "visible" : "hidden");
        return buf;
    }

private:
    VisionCone visionCone() const {
        return {guard_.position, guard_.forward(), fovDeg_ * 0.5f * DEG2RAD, range_};
    }
    Vec2 handleTip() const { return guard_.position + guard_.forward() * kHandleLen; }

    void drawReadout(bool seen) const {
        const Palette& p = palette();
        Vec2 f = guard_.forward();
        Vec2 to = player_ - guard_.position;
        float d = dot(f, to);
        float c = cross(f, to);
        Vec2 local = guard_.toLocal(player_);

        Rectangle box{16, kHeight - 136.0f, 268, 120};
        DrawRectangleRec(box, Fade(p.background, 0.92f));
        DrawRectangleLinesEx(box, hairline(), p.gridLine);

        char line[64];
        float x = box.x + 14, y = box.y + 12;
        auto row = [&](const char* k, const char* v, const char* note, Color noteColor) {
            drawText(k, {x, y}, 11, p.textMuted);
            drawText(v, {x + 104, y}, 11, p.text);
            if (note) drawText(note, {x + 186, y}, 11, noteColor);
            y += 20;
        };
        std::snprintf(line, sizeof line, "%8.1f", d);
        row("dot(f, to)", line, d > 0 ? "front" : "behind", d > 0 ? p.text : p.textMuted);
        std::snprintf(line, sizeof line, "%8.1f", c);
        row("cross(f, to)", line, c > 0 ? "right" : "left", p.text);
        std::snprintf(line, sizeof line, "%7.1f°", signedAngle(f, to) * RAD2DEG);
        row("angle", line, nullptr, p.text);
        std::snprintf(line, sizeof line, "(%.0f, %.0f)", local.x, local.y);
        row("local", line, nullptr, p.text);
        row("can see", seen ? "yes" : "no", nullptr, p.text);
    }

    Transform2D guard_{{300, 300}, -0.35f};
    float baseAngle_ = -0.35f;
    float sweep_ = 0;
    Vec2 player_{560, 210};
    float fovDeg_ = 90;
    float range_ = 300;
    Drag drag_ = Drag::None;
};

}  // namespace

int main() {
    static VectorsDemo demo;
    ga::DemoConfig config;
    config.title = "Vectors - Dot & Cross";
    config.width = kWidth;
    config.height = kHeight;
    config.stepsPerSecond = 60;
    return ga::runDemo(demo, config);
}
