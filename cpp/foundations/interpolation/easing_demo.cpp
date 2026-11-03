// 缓动函数演示：左侧选择缓动函数，中间是曲线，右侧是三种常见用法（位移、缩放、进度条）

#include <cmath>
#include <cstdio>
#include <string>

#include "demo.h"
#include "interpolation.h"
#include "ui.h"

namespace {

using namespace ga;

constexpr int kWidth = 800;
constexpr int kHeight = 400;

struct Easing {
    const char* name;
    EaseFn fn;
};

constexpr Easing kEasings[] = {
    {"linear", ease::linear},       {"inQuad", ease::inQuad},         {"outQuad", ease::outQuad},
    {"inOutQuad", ease::inOutQuad}, {"inCubic", ease::inCubic},       {"outCubic", ease::outCubic},
    {"inOutCubic", ease::inOutCubic}, {"inOutSine", ease::inOutSine}, {"smoothstep", ease::smoothstep},
    {"outBack", ease::outBack},     {"outElastic", ease::outElastic}, {"outBounce", ease::outBounce},
};
constexpr int kCount = sizeof kEasings / sizeof kEasings[0];
constexpr float kHold = 0.6f;  // 每轮结束后停顿的秒数

// 曲线图区域与数值范围（留出过冲空间）
constexpr Rectangle kPlot{196, 46, 270, 270};
constexpr float kVMin = -0.35f, kVMax = 1.35f;

class EasingDemo final : public Demo {
public:
    void reset() override { time_ = 0; }

    bool step() override {
        time_ += 1.0f / 60.0f;
        if (time_ > duration_ + kHold) time_ = 0;
        return true;
    }

    void draw() const override {
        const Palette& p = palette();
        const float hl = hairline();
        const EaseFn f = kEasings[selected_].fn;
        const float t = clamp01(time_ / duration_);
        const float v = f(t);

        // ---- 曲线图 ----
        auto plot = [&](float tt, float vv) {
            return Vector2{kPlot.x + tt * kPlot.width,
                           kPlot.y + kPlot.height * (1 - (vv - kVMin) / (kVMax - kVMin))};
        };
        DrawRectangleLinesEx(kPlot, hl, p.gridLine);
        for (float g : {0.0f, 1.0f}) DrawLineEx(plot(0, g), plot(1, g), hl, p.textMuted);
        drawDashedLine(plot(0, 0.5f), plot(1, 0.5f), hl, p.gridLine, 3, 4);
        drawText("0", {kPlot.x - 14, plot(0, 0).y - 6}, 10, p.textMuted);
        drawText("1", {kPlot.x - 14, plot(0, 1).y - 6}, 10, p.textMuted);
        drawText("t", {kPlot.x + kPlot.width - 6, kPlot.y + kPlot.height + 6}, 10, p.textMuted);

        // 线性参考线 + 当前缓动曲线
        drawDashedLine(plot(0, 0), plot(1, 1), 1, p.textMuted, 4, 4);
        Vector2 prev = plot(0, f(0));
        for (int i = 1; i <= 120; ++i) {
            float tt = i / 120.0f;
            Vector2 cur = plot(tt, f(tt));
            DrawLineEx(prev, cur, 2.5f, p.text);
            prev = cur;
        }
        // 当前时刻
        DrawLineEx(plot(t, kVMin), plot(t, kVMax), hl, p.goal);
        DrawCircleV(plot(t, v), 6, p.goal);

        char buf[64];
        std::snprintf(buf, sizeof buf, "t %.2f  ->  %.3f", t, v);
        drawText(buf, {kPlot.x, kPlot.y + kPlot.height + 24}, 11, p.text);

        // ---- 右侧：三种用法 ----
        const float x0 = 520, x1 = kWidth - 30;
        // 1. 位移：缓动小球 vs 线性小球
        float y = 74;
        drawText("move", {x0, y - 30}, 10, p.textMuted);
        DrawLineEx({x0, y}, {x1, y}, hl, p.gridLine);
        DrawCircleV({lerp(x0, x1, t), y + 20}, 6, Fade(p.textMuted, 0.5f));
        DrawCircleV({lerp(x0, x1, v), y}, 10, p.start);
        drawText("linear", {x0, y + 30}, 9, p.textMuted);

        // 2. 缩放：弹出的面板
        y = 196;
        drawText("scale", {x0, y - 64}, 10, p.textMuted);
        float size = std::max(0.0f, v) * 90;
        Rectangle panel{(x0 + x1) * 0.5f - size * 0.5f, y - size * 0.5f, size, size};
        DrawRectangleRec(panel, p.goal);
        DrawRectangleLinesEx({(x0 + x1) * 0.5f - 45, y - 45, 90, 90}, hl, p.gridLine);

        // 3. 进度条
        y = 300;
        drawText("fill", {x0, y - 26}, 10, p.textMuted);
        DrawRectangleLinesEx({x0, y, x1 - x0, 16}, hl, p.textMuted);
        DrawRectangleRec({x0, y, (x1 - x0) * clamp01(v), 16}, p.frontier);
        if (v > 1) DrawRectangleRec({x1, y, (x1 - x0) * (v - 1), 16}, Fade(p.goal, 0.6f));  // 过冲部分

        DrawLineEx({180, 16}, {180, float(kHeight) - 16}, hl, p.gridLine);
        DrawLineEx({494, 16}, {494, float(kHeight) - 16}, hl, p.gridLine);
    }

    void drawUi() override {
        for (int i = 0; i < kCount; ++i) {
            if (ui::button({16, 16.0f + i * 28, 148, 24}, kEasings[i].name, i == selected_)) {
                selected_ = i;
                time_ = 0;
            }
        }
        ui::slider({196, kHeight - 40.0f, 270, 24}, "duration", &duration_, 0.3f, 3.0f, "%.1fs", 0.1f);
    }

    std::string status() const override {
        float t = clamp01(time_ / duration_);
        char buf[64];
        std::snprintf(buf, sizeof buf, "%s · t %.2f → %.2f", kEasings[selected_].name, t, kEasings[selected_].fn(t));
        return buf;
    }

private:
    int selected_ = 5;  // outCubic
    float time_ = 0;
    float duration_ = 1.2f;
};

}  // namespace

int main() {
    static EasingDemo demo;
    ga::DemoConfig config;
    config.title = "Easing Functions";
    config.width = kWidth;
    config.height = kHeight;
    config.stepsPerSecond = 60;
    return ga::runDemo(demo, config);
}
