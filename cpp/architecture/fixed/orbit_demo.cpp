// 定点数的范围与精度：同一个轨道积分器，只换数值类型
//   - 三颗行星，半径 30、60、100。灰色细线是 double 版本的轨迹，墨色是所选类型的轨迹，朱红点是行星当前位置
//   - Q23.8：分辨率 1/256，加速度只有 0.01 左右，只剩一两位有效数字，轨道明显走样
//   - Q15.16：半径 100 以内都能算，但加速度这样的小数值有效位不多，误差比 float 大
//   - Q11.20 / Q7.24：更精细，但范围只有 ±2048 / ±128。r² = 3600、10000 放不下，乘法溢出回绕，行星乱飞
//     ——r 本身放得下，溢出的是中间结果
//   - 右侧：每颗行星和 double 版本的距离，以及 r² 是否超出这个格式的范围

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <deque>
#include <memory>
#include <string>

#include "demo.h"
#include "fixed.h"
#include "ui.h"

namespace {

using namespace ga;
using namespace ga::fixed;

constexpr int kWidth = 800, kHeight = 480;
constexpr float kTop = 80, kCx = 200, kCy = 98 + 186, kScale = 1.65f;
constexpr double kRadii[3] = {30, 60, 100};
constexpr int kTrail = 700, kRound = 8000;

struct Runner {
    virtual ~Runner() = default;
    virtual void step() = 0;
    virtual void pos(int i, double& x, double& y) const = 0;
};
template <class T>
struct RunnerT final : Runner {
    Orbit<T> o[3] = {Orbit<T>::circular(kRadii[0]), Orbit<T>::circular(kRadii[1]), Orbit<T>::circular(kRadii[2])};
    void step() override {
        for (auto& p : o) p.step();
    }
    void pos(int i, double& x, double& y) const override { x = toD(o[i].x), y = toD(o[i].y); }
};

struct Format {
    const char* name;
    int frac;  // 0 = float
};
constexpr Format kFormats[6] = {{"Q23.8", 8}, {"Q19.12", 12}, {"Q15.16", 16}, {"Q11.20", 20}, {"Q7.24", 24}, {"float", 0}};

class OrbitDemo final : public Demo {
public:
    OrbitDemo() { reset(); }
    void reset() override {
        switch (kFormats[format_].frac) {
            case 8: run_ = std::make_unique<RunnerT<Fixed<8>>>(); break;
            case 12: run_ = std::make_unique<RunnerT<Fixed<12>>>(); break;
            case 16: run_ = std::make_unique<RunnerT<Fixed<16>>>(); break;
            case 20: run_ = std::make_unique<RunnerT<Fixed<20>>>(); break;
            case 24: run_ = std::make_unique<RunnerT<Fixed<24>>>(); break;
            default: run_ = std::make_unique<RunnerT<float>>(); break;
        }
        ref_ = RunnerT<double>{};
        for (int i = 0; i < 3; ++i) trail_[i].clear(), refTrail_[i].clear(), worst_[i] = 0;
        steps_ = 0;
    }
    bool step() override {
        if (steps_ >= kRound) reset();
        run_->step(), ref_.step();
        ++steps_;
        for (int i = 0; i < 3; ++i) {
            double x, y, rx, ry;
            run_->pos(i, x, y), ref_.pos(i, rx, ry);
            worst_[i] = std::max(worst_[i], std::hypot(x - rx, y - ry));
            trail_[i].push_back(at(x, y)), refTrail_[i].push_back(at(rx, ry));
            if (trail_[i].size() > kTrail) trail_[i].pop_front(), refTrail_[i].pop_front();
        }
        return true;
    }

    void draw() const override {
        const Palette& pal = palette();
        const float hl = hairline();
        const Rectangle view{12, 98, 376, 364};
        DrawRectangleLinesEx(view, hl, pal.gridLine);
        BeginScissorMode(int(view.x), int(view.y), int(view.width), int(view.height));
        DrawCircleV({kCx, kCy}, 7, pal.text);  // 恒星
        for (int i = 0; i < 3; ++i) {
            for (size_t k = 1; k < refTrail_[i].size(); ++k) DrawLineEx(refTrail_[i][k - 1], refTrail_[i][k], 1, Fade(pal.textMuted, 0.45f));
            for (size_t k = 1; k < trail_[i].size(); ++k) {
                Vector2 a = trail_[i][k - 1], b = trail_[i][k];
                if (std::fabs(a.x - b.x) + std::fabs(a.y - b.y) < 60) DrawLineEx(a, b, 1.4f, pal.text);  // 溢出时的跳变不连线
            }
            if (!trail_[i].empty()) DrawCircleV(trail_[i].back(), 5, pal.path);
        }
        EndScissorMode();
        // 飞出视野的行星：在边框上画一个空心圆
        for (int i = 0; i < 3; ++i) {
            if (trail_[i].empty()) continue;
            Vector2 p = trail_[i].back();
            if (CheckCollisionPointRec(p, view)) continue;
            Vector2 q{std::clamp(p.x, view.x, view.x + view.width), std::clamp(p.y, view.y, view.y + view.height)};
            DrawCircleLinesV(q, 6, pal.path);
        }

        // 右侧面板
        const Format& f = kFormats[format_];
        const float x0 = 410;
        float y = 104;
        char buf[160];
        if (f.frac) {
            double res = std::ldexp(1.0, -f.frac), mx = std::ldexp(1.0, 31 - f.frac);
            std::snprintf(buf, sizeof buf, "%s: 32-bit integer, %d fraction bits", f.name, f.frac);
            drawText(buf, {x0, y}, 12, pal.text), y += 20;
            std::snprintf(buf, sizeof buf, "resolution %.3g   range +/-%.0f", res, mx);
            drawText(buf, {x0, y}, 11, pal.textMuted), y += 16;
            std::snprintf(buf, sizeof buf, "acceleration at r = 100: GM/r^2 = 0.01 = %.0f x resolution", 0.01 / res);
            drawText(buf, {x0, y}, 11, pal.textMuted), y += 30;
        } else {
            drawText("float: 24-bit significand, exponent moves the point", {x0, y}, 12, pal.text), y += 20;
            drawText("spacing grows with the value: 7.6e-6 at 100, 9.3e-10 at 0.01", {x0, y}, 11, pal.textMuted), y += 46;
        }
        drawText("planet    r^2      off from double (max so far)", {x0, y}, 11, pal.textMuted), y += 18;
        for (int i = 0; i < 3; ++i) {
            double r2 = kRadii[i] * kRadii[i];
            bool overflow = f.frac && r2 >= std::ldexp(1.0, 31 - f.frac);
            std::snprintf(buf, sizeof buf, "r = %-4.0f %6.0f%s   %10.3g", kRadii[i], r2, overflow ? " !" : "  ", worst_[i]);
            drawText(buf, {x0, y}, 12, overflow ? pal.path : pal.text), y += 18;
        }
        y += 8;
        bool anyOverflow = false;
        for (double r : kRadii) anyOverflow |= f.frac && r * r >= std::ldexp(1.0, 31 - f.frac);
        if (anyOverflow) drawText("! r^2 does not fit: the multiply wraps around", {x0, y}, 11, pal.path), y += 16;
        std::snprintf(buf, sizeof buf, "step %d", steps_);
        drawText(buf, {12, kTop + 6}, 11, pal.textMuted);
        DrawLineEx({0, kTop}, {float(kWidth), kTop}, hl, pal.gridLine);
    }

    void drawUi() override {
        for (int i = 0; i < 6; ++i)
            if (ui::button({12.0f + i * 72, 10, 68, 26}, kFormats[i].name, format_ == i) && format_ != i)
                format_ = i, reset(), notifyProblemChanged();
    }

    std::string status() const override {
        char buf[64];
        std::snprintf(buf, sizeof buf, "%s · step %d", kFormats[format_].name, steps_);
        return buf;
    }

private:
    static Vector2 at(double x, double y) { return {kCx + float(x) * kScale, kCy - float(y) * kScale}; }
    std::unique_ptr<Runner> run_;
    RunnerT<double> ref_;
    std::deque<Vector2> trail_[3], refTrail_[3];
    double worst_[3] = {};
    int format_ = 2, steps_ = 0;
};

}  // namespace

int main() {
    static OrbitDemo demo;
    ga::DemoConfig config;
    config.title = "Fixed-point range and precision";
    config.width = kWidth;
    config.height = kHeight;
    config.stepsPerSecond = 120.0f;
    return ga::runDemo(demo, config);
}
