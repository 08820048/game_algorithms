// 三次贝塞尔曲线
//   - 拖动四个控制点。de Casteljau 构造：相邻点按 t 插值，三层之后得到曲线上的点（红点）
//   - equal t：t 等分的 16 个点；equal length：弧长等分的 16 个点。两组点差得越多，t 匀速时速度越不均匀
//   - flatten：自适应细分得到的折线，tolerance 是允许的最大偏差（像素）；右边对比 Wang 公式给出的均匀段数
//   - comb：曲率梳，每个采样点沿法向画一根长度正比于曲率的线

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "demo.h"
#include "spline.h"
#include "ui.h"

namespace {

using namespace ga;
using namespace ga::spline;

constexpr int kWidth = 800, kHeight = 480;
constexpr float kTop = 80;
constexpr float kPanel = 600;

Vector2 rv(Vec2 v) { return {v.x, v.y}; }

class BezierDemo final : public Demo {
public:
    BezierDemo() { rebuild(); }
    void reset() override { t_ = 0, dir_ = 1; }
    bool step() override {
        t_ += dir_ / 150.0f;
        if (t_ >= 1) t_ = 1, dir_ = -1;
        if (t_ <= 0) t_ = 0, dir_ = 1;
        return true;
    }
    void update() override {
        Vector2 m = GetMousePosition();
        Vec2 mv{m.x, m.y};
        if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) drag_ = -1;
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && !ui::wantsMouse() && m.y > kTop) {
            for (int i = 0; i < 4; ++i)
                if (length(c_.p[i] - mv) < 12) drag_ = i;
        }
        if (drag_ >= 0 && IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
            c_.p[drag_] = {std::clamp(m.x, 10.0f, kPanel - 20), std::clamp(m.y, kTop + 10, kHeight - 10.0f)};
            rebuild();
        }
    }

    void draw() const override {
        const Palette& pal = palette();
        const float hl = hairline();
        // 控制多边形
        for (int i = 0; i < 3; ++i) drawDashedLine(rv(c_.p[i]), rv(c_.p[i + 1]), hl, pal.textMuted);
        // 曲率梳
        if (comb_) {
            Vec2 prev{};
            for (int i = 0; i <= 120; ++i) {
                float t = i / 120.0f;
                Vec2 p = c_.at(t), n = perp(normalized(c_.d1(t)));
                Vec2 q = p - n * std::clamp(c_.curvature(t) * 3000, -150.0f, 150.0f);  // 尖点附近曲率趋于无穷，截断
                DrawLineEx(rv(p), rv(q), hl, Fade(pal.frontier, 0.7f));
                if (i) DrawLineEx(rv(prev), rv(q), hl, pal.frontier);
                prev = q;
            }
        }
        // 曲线
        for (size_t i = 0; i + 1 < fine_.size(); ++i) DrawLineEx(rv(fine_[i]), rv(fine_[i + 1]), 2.0f, pal.text);
        if (flatten_) {
            for (size_t i = 0; i + 1 < flat_.size(); ++i) DrawLineEx(rv(flat_[i]), rv(flat_[i + 1]), 1.0f, pal.path);
            for (Vec2 v : flat_) DrawCircleV(rv(v), 2.5f, pal.path);
        }
        if (equalT_)
            for (int i = 0; i <= 16; ++i) DrawCircleV(rv(c_.at(i / 16.0f)), 4, pal.text);
        if (equalS_)
            for (int i = 0; i <= 16; ++i) {
                Vec2 p = c_.at(table_.paramAt(table_.length() * i / 16, 3));
                DrawCircleLinesV(rv(p), 6, pal.path);
            }
        // de Casteljau 构造
        if (construct_) {
            Vec2 a[3], b[2];
            for (int i = 0; i < 3; ++i) a[i] = lerp(c_.p[i], c_.p[i + 1], t_);
            for (int i = 0; i < 2; ++i) b[i] = lerp(a[i], a[i + 1], t_);
            Vec2 f = lerp(b[0], b[1], t_);
            Color l1 = Fade(pal.frontier, 0.9f);
            DrawLineEx(rv(a[0]), rv(a[1]), 1.2f, l1), DrawLineEx(rv(a[1]), rv(a[2]), 1.2f, l1);
            DrawLineEx(rv(b[0]), rv(b[1]), 1.2f, pal.current);
            for (Vec2 v : a) DrawCircleV(rv(v), 3.5f, l1);
            for (Vec2 v : b) DrawCircleV(rv(v), 3.5f, pal.current);
            DrawCircleV(rv(f), 6, pal.path);
        }
        for (int i = 0; i < 4; ++i) {
            DrawCircleV(rv(c_.p[i]), 7, pal.background);
            DrawCircleLinesV(rv(c_.p[i]), 7, pal.text);
            char lb[4];
            std::snprintf(lb, sizeof lb, "P%d", i);
            drawText(lb, {c_.p[i].x + 10, c_.p[i].y - 18}, 12, pal.text);
        }

        DrawLineEx({kPanel, kTop}, {kPanel, float(kHeight)}, hl, pal.gridLine);
        char buf[96];
        float y = kTop + 14;
        auto line = [&](const char* s, Color col, float size = 12) {
            drawText(s, {kPanel + 14, y}, size, col);
            y += size + 7;
        };
        std::snprintf(buf, sizeof buf, "t = %.2f", t_);
        line(buf, pal.text, 13);
        std::snprintf(buf, sizeof buf, "arc length %.1f px", table_.length());
        line(buf, pal.text);
        std::snprintf(buf, sizeof buf, "speed max/min %.1f", speedRatio_);
        line(buf, pal.text);
        y += 8;
        std::snprintf(buf, sizeof buf, "tolerance %.2f px", tol_);
        line(buf, pal.textMuted);
        std::snprintf(buf, sizeof buf, "adaptive  %zu segs", flat_.size() - 1);
        line(buf, pal.text);
        std::snprintf(buf, sizeof buf, "Wang      %d segs", wang_);
        line(buf, pal.text);
        std::snprintf(buf, sizeof buf, "best even %d segs", bestUniform_);
        line(buf, pal.text);
        y += 8;
        line("drag P0..P3", pal.textMuted, 11);
        DrawLineEx({0, kTop}, {float(kWidth), kTop}, hl, pal.gridLine);
    }

    void drawUi() override {
        ui::toggle({12, 10, 118, 26}, "construction", &construct_);
        ui::toggle({136, 10, 90, 26}, "equal t", &equalT_);
        ui::toggle({232, 10, 116, 26}, "equal length", &equalS_);
        ui::toggle({354, 10, 80, 26}, "flatten", &flatten_);
        ui::toggle({440, 10, 70, 26}, "comb", &comb_);
        if (ui::slider({12, 44, 260, 26}, "tolerance", &tol_, 0.1f, 8, "%.1f", 0.1f)) rebuild();
        if (ui::button({284, 44, 80, 26}, "S curve")) c_ = {{{90, 400}, {250, 100}, {330, 440}, {520, 140}}}, rebuild();
        if (ui::button({370, 44, 70, 26}, "loop")) c_ = {{{120, 420}, {560, 120}, {60, 120}, {500, 420}}}, rebuild();
        if (ui::button({446, 44, 70, 26}, "cusp")) c_ = {{{120, 420}, {480, 120}, {120, 120}, {480, 420}}}, rebuild();
    }

    std::string status() const override {
        char buf[96];
        std::snprintf(buf, sizeof buf, "length %.0f px · adaptive %zu segs · Wang %d", table_.length(), flat_.size() - 1, wang_);
        return buf;
    }

private:
    void rebuild() {
        fine_.clear(), flat_.clear();
        flattenUniform(c_, 200, &fine_);
        flattenAdaptive(c_, tol_, &flat_);
        wang_ = wangSegments(c_, tol_);
        // 均匀切分最少要几段：逐个试
        bestUniform_ = wang_;
        for (int n = 1; n <= wang_; ++n) {
            std::vector<Vec2> u;
            flattenUniform(c_, n, &u);
            float worst = 0;
            for (int i = 0; i <= 400 && worst <= tol_; ++i) {
                float t = i / 400.0f;
                int k = std::min(n - 1, int(t * n));
                worst = std::max(worst, distToSegment(c_.at(t), u[size_t(k)], u[size_t(k) + 1]));
            }
            if (worst <= tol_) {
                bestUniform_ = n;
                break;
            }
        }
        table_ = ArcLengthTable(c_, 32);
        float lo = 1e30f, hi = 0;
        for (int i = 0; i <= 200; ++i) {
            float v = length(c_.d1(i / 200.0f));
            lo = std::min(lo, v), hi = std::max(hi, v);
        }
        speedRatio_ = hi / std::max(lo, 1e-6f);
    }

    Cubic c_{{{90, 400}, {250, 100}, {330, 440}, {520, 140}}};
    float t_ = 0, dir_ = 1, tol_ = 2, speedRatio_ = 1;
    int drag_ = -1, wang_ = 1, bestUniform_ = 1;
    bool construct_ = true, equalT_ = false, equalS_ = false, flatten_ = false, comb_ = false;
    std::vector<Vec2> fine_, flat_;
    ArcLengthTable table_;
};

}  // namespace

int main() {
    static BezierDemo demo;
    ga::DemoConfig config;
    config.title = "Cubic Bezier curve";
    config.width = kWidth;
    config.height = kHeight;
    config.stepsPerSecond = 60.0f;
    return ga::runDemo(demo, config);
}
