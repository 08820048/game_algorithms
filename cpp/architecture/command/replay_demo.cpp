// 录像回放与混沌
//   - 墨色实心：原始的一局（推杆由自动驾驶控制，每帧的输入被录下来）
//   - 朱红空心：回放（从同一个种子出发，逐帧喂入录下的输入）
//   - exact：逐位相同，空心圆始终套在实心圆上，状态哈希一致
//   - 1 ulp / 1e-3 px：回放开始时把一个球的 x 改动一点点。右侧是两局里最远的一对球的距离（对数坐标），
//     差距大约每 20 帧放大 10 倍，两秒后两局完全不同
//   - 每 20 秒重新开始

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "command.h"
#include "demo.h"
#include "ui.h"

namespace {

using namespace ga;
using namespace ga::command;

constexpr int kWidth = 800, kHeight = 480;
constexpr float kTop = 80, kX0 = 12, kY0 = 98;
constexpr int kBalls = 36, kRound = 1200;

class ReplayDemo final : public Demo {
public:
    ReplayDemo() { reset(); }
    void reset() override {
        rec_ = Recording{11, kBalls, {}};
        a_ = Sim::make(11, kBalls);
        b_ = a_;
        if (mode_ == 1) b_.balls[0].p.x = std::nextafter(b_.balls[0].p.x, 1e9f);
        if (mode_ == 2) b_.balls[0].p.x += 1e-3f;
        dist_.clear();
        diverged_ = -1;
    }
    bool step() override {
        if (a_.frame >= kRound) reset();
        uint32_t f = a_.frame;
        Input in{int8_t(std::sin(f * 0.013f) > 0 ? 1 : -1), int8_t(std::sin(f * 0.021f + 1) > 0.3f ? 1 : -1), (f / 90) % 3 == 0};
        rec_.inputs.push_back(in);
        a_.step(in);
        b_.step(rec_.inputs[b_.frame]);
        float d = a_.maxDistance(b_);
        dist_.push_back(d);
        if (diverged_ < 0 && d > 1) diverged_ = int(a_.frame);
        return true;
    }

    void draw() const override {
        const Palette& pal = palette();
        const float hl = hairline();
        DrawRectangleLinesEx({kX0, kY0, Sim::kW, Sim::kH}, hl, pal.gridLine);
        auto at = [](Vec2 p) { return Vector2{kX0 + p.x, kY0 + p.y}; };
        DrawCircleV(at(a_.pusher), a_.pusherRadius(rec_.inputs.empty() ? false : rec_.inputs.back().push), Fade(pal.textMuted, 0.35f));
        for (const Ball& b : a_.balls) DrawCircleV(at(b.p), Sim::kR, pal.text);
        for (const Ball& b : b_.balls) DrawCircleLinesV(at(b.p), Sim::kR + 2, pal.path);

        // 距离曲线（对数）
        const float px = kX0 + Sim::kW + 22, pw = kWidth - px - 14, py = kY0 + 40, ph = 230;
        DrawRectangleLinesEx({px, py, pw, ph}, hl, pal.gridLine);
        auto yOf = [&](float d) {
            float l = std::log10(std::max(d, 1e-7f));
            return py + ph - (l + 7) / 10 * ph;  // 1e-7 … 1e3
        };
        for (int e = -6; e <= 2; e += 2) {
            float y = yOf(std::pow(10.0f, float(e)));
            DrawLineEx({px, y}, {px + pw, y}, hl, Fade(pal.gridLine, 0.6f));
            char b[16];
            std::snprintf(b, sizeof b, "1e%d", e);
            drawText(b, {px + 2, y - 12}, 9, pal.textMuted);
        }
        float y1 = yOf(1);
        drawDashedLine({px, y1}, {px + pw, y1}, 1, pal.path);
        for (size_t i = 1; i < dist_.size(); ++i) {
            float x0 = px + pw * float(i - 1) / kRound, x1 = px + pw * float(i) / kRound;
            DrawLineEx({x0, yOf(dist_[i - 1])}, {x1, yOf(dist_[i])}, 1.4f, pal.text);
        }
        drawText("max distance (px)", {px, py - 16}, 10, pal.textMuted);
        drawText("frame 0 ... 1200", {px, py + ph + 4}, 10, pal.textMuted);

        char buf[160];
        bool same = a_.hash() == b_.hash();
        std::snprintf(buf, sizeof buf, "frame %u   hash %s   max distance %.3g px", a_.frame, same ? "identical" : "different",
                      dist_.empty() ? 0.0 : double(dist_.back()));
        drawText(buf, {12, kTop + 6}, 11, same ? pal.textMuted : pal.path);
        if (diverged_ >= 0) {
            std::snprintf(buf, sizeof buf, "off by > 1 px at frame %d", diverged_);
            drawText(buf, {px, py + ph + 22}, 11, pal.path);
        }
        char inBuf[64];
        std::snprintf(inBuf, sizeof inBuf, "recorded %zu inputs, %zu runs", rec_.inputs.size(), rec_.compress().size());
        drawText(inBuf, {px, py + ph + 40}, 10, pal.textMuted);
        DrawLineEx({0, kTop}, {float(kWidth), kTop}, hl, pal.gridLine);
    }

    void drawUi() override {
        const char* names[3] = {"exact", "1 ulp", "1e-3 px"};
        for (int i = 0; i < 3; ++i)
            if (ui::button({12.0f + i * 84, 10, 80, 26}, names[i], mode_ == i) && mode_ != i) mode_ = i, reset(), notifyProblemChanged();
    }

    std::string status() const override {
        char buf[64];
        std::snprintf(buf, sizeof buf, "frame %u · %s", a_.frame, a_.hash() == b_.hash() ? "in sync" : "diverged");
        return buf;
    }

private:
    Recording rec_;
    Sim a_, b_;
    std::vector<float> dist_;
    int mode_ = 1, diverged_ = -1;
};

}  // namespace

int main() {
    static ReplayDemo demo;
    ga::DemoConfig config;
    config.title = "Replay and chaos";
    config.width = kWidth;
    config.height = kHeight;
    config.stepsPerSecond = 60.0f;
    return ga::runDemo(demo, config);
}
