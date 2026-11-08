// 一维格点噪声：值、一阶导数、二阶导数
//   - value：格点上是随机的值（圆点），拖动圆点改值
//   - gradient：格点上是随机的斜率（短线），噪声在格点处为 0；拖动短线末端改斜率
//   - linear / smooth / quintic：插值用的缓和函数。下面两行画 f' 与 f''，朱红色竖线标出格点处的跳变
//   - parts：鼠标所在的格子里，画出两端各自的贡献（value 是两条水平线，gradient 是两条直线）

#include <cmath>
#include <cstdio>
#include <string>

#include "demo.h"
#include "noise.h"
#include "ui.h"

namespace {

using namespace ga;
using namespace ga::noise;

constexpr int kWidth = 800, kHeight = 480;
constexpr float kTop = 52;
constexpr int kCells = 10;
constexpr float kX0 = 56, kX1 = 776;
constexpr float kCellW = (kX1 - kX0) / kCells;

struct Row {
    float top, bottom;
    float mid() const { return (top + bottom) / 2; }
};
constexpr Row kRowF{70, 262}, kRowD1{286, 370}, kRowD2{386, 470};

class LatticeDemo final : public Demo {
public:
    LatticeDemo() { randomize(); }

    void reset() override {}
    bool step() override { return true; }

    void update() override {
        Vector2 m = GetMousePosition();
        hoverX_ = m.y > kTop && !ui::wantsMouse() ? (m.x - kX0) / kCellW : -1;
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && !ui::wantsMouse()) {
            drag_ = -1;
            float best = 14;
            for (int i = 0; i <= kCells; ++i) {
                float d = std::hypot(m.x - handle(i).x, m.y - handle(i).y);
                if (d < best) best = d, drag_ = i;
            }
        }
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT) && drag_ >= 0) {
            if (gradient_) {
                float dy = (kRowF.mid() - m.y) / yScale(), dx = kHandleLen;
                slope_[drag_] = std::clamp(dy / dx, -1.5f, 1.5f);
            } else {
                value_[drag_] = std::clamp((kRowF.mid() - m.y) / yScale(), -1.0f, 1.0f);
            }
        }
        if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) drag_ = -1;
    }

    void draw() const override {
        const Palette& pal = palette();
        const float hl = hairline();
        // 格线
        for (int i = 0; i <= kCells; ++i) {
            float x = kX0 + i * kCellW;
            DrawLineEx({x, kRowF.top}, {x, kRowD2.bottom}, hl, pal.gridLine);
        }
        for (const Row* r : {&kRowF, &kRowD1, &kRowD2}) DrawLineEx({kX0, r->mid()}, {kX1, r->mid()}, hl, Fade(pal.textMuted, 0.6f));
        drawText("f", {16, kRowF.mid() - 8}, 14, pal.text);
        drawText("f'", {16, kRowD1.mid() - 8}, 14, pal.text);
        drawText("f''", {16, kRowD2.mid() - 8}, 14, pal.text);

        // 导数两行各自按最大值缩放
        float m1 = 1e-3f, m2 = 1e-3f;
        for (int i = 0; i < kCells; ++i)
            for (int s = 0; s <= 64; ++s) {
                float t = s / 64.0f;
                m1 = std::max(m1, std::abs(eval(i, t, 1))), m2 = std::max(m2, std::abs(eval(i, t, 2)));
            }
        float sc1 = (kRowD1.bottom - kRowD1.top) * 0.45f / m1, sc2 = (kRowD2.bottom - kRowD2.top) * 0.45f / m2;
        char buf[64];
        std::snprintf(buf, sizeof buf, "max %.2f", m1);
        drawText(buf, {kX1 - 64, kRowD1.top - 2}, 10, pal.textMuted);
        std::snprintf(buf, sizeof buf, "max %.2f", m2);
        drawText(buf, {kX1 - 64, kRowD2.top - 2}, 10, pal.textMuted);

        // 鼠标所在格子的两端贡献
        int hc = int(std::floor(hoverX_));
        if (showParts_ && hc >= 0 && hc < kCells) {
            float xa = kX0 + hc * kCellW, xb = xa + kCellW;
            DrawRectangleRec({xa, kRowF.top, kCellW, kRowF.bottom - kRowF.top}, pal.hover);
            if (gradient_) {  // 左端的直线 g0·t，右端的直线 g1·(t-1)，各自延伸到整个格子（裁剪到这一行之内）
                dashedClipped({xa, ry(0)}, {xb, ry(slope_[hc])}, pal.textMuted);
                dashedClipped({xa, ry(-slope_[hc + 1])}, {xb, ry(0)}, pal.textMuted);
            } else {
                drawDashedLine({xa, ry(value_[hc])}, {xb, ry(value_[hc])}, 1.5f, pal.textMuted);
                drawDashedLine({xa, ry(value_[hc + 1])}, {xb, ry(value_[hc + 1])}, 1.5f, pal.textMuted);
            }
        }

        // 三条曲线
        for (int k = 0; k < 3; ++k) {
            const Row& row = k == 0 ? kRowF : k == 1 ? kRowD1 : kRowD2;
            float sc = k == 0 ? yScale() : k == 1 ? sc1 : sc2;
            for (int i = 0; i < kCells; ++i) {
                Vector2 prev{};
                for (int s = 0; s <= 48; ++s) {
                    float t = s / 48.0f;
                    Vector2 p{kX0 + (i + t) * kCellW, row.mid() - eval(i, t, k) * sc};
                    p.y = std::clamp(p.y, row.top - 6, row.bottom + 6);
                    if (s > 0) DrawLineEx(prev, p, k == 0 ? 2.5f : 1.8f, pal.text);
                    prev = p;
                }
            }
            // 格点处的跳变：左极限与右极限不同
            if (k > 0)
                for (int i = 1; i < kCells; ++i) {
                    float l = eval(i - 1, 1, k), r = eval(i, 0, k);
                    if (std::abs(l - r) < 1e-4f) continue;
                    float x = kX0 + i * kCellW;
                    float yl = std::clamp(row.mid() - l * sc, row.top, row.bottom), yr = std::clamp(row.mid() - r * sc, row.top, row.bottom);
                    DrawLineEx({x, yl}, {x, yr}, 2.5f, pal.path);
                    DrawCircleV({x, yl}, 2.5f, pal.path);
                    DrawCircleV({x, yr}, 2.5f, pal.path);
                }
        }

        // 格点上的控制柄
        for (int i = 0; i <= kCells; ++i) {
            Vector2 h = handle(i);
            if (gradient_) {
                Vector2 o{kX0 + i * kCellW, kRowF.mid()};
                Vector2 a{o.x - kHandleLen * kCellW, o.y + slope_[i] * kHandleLen * yScale()};
                DrawLineEx(a, h, 2.5f, pal.path);
                DrawCircleV(o, 4, pal.text);
                DrawCircleV(h, 5, pal.path);
            } else {
                DrawCircleV(h, 6, pal.path);
                DrawCircleLinesV(h, 6, pal.background);
            }
        }

        // 跳变计数
        int j1 = 0, j2 = 0;
        for (int i = 1; i < kCells; ++i) {
            j1 += std::abs(eval(i - 1, 1, 1) - eval(i, 0, 1)) > 1e-4f;
            j2 += std::abs(eval(i - 1, 1, 2) - eval(i, 0, 2)) > 1e-4f;
        }
        std::snprintf(buf, sizeof buf, "jumps at interior lattice points:  f' %d   f'' %d", j1, j2);
        drawText(buf, {kX0, kRowF.top - 14}, 11, j1 + j2 ? pal.path : pal.textMuted);
        DrawLineEx({0, kTop}, {float(kWidth), kTop}, hl, pal.gridLine);
    }

    void drawUi() override {
        if (ui::button({12, 13, 70, 26}, "value", !gradient_)) gradient_ = false;
        if (ui::button({86, 13, 80, 26}, "gradient", gradient_)) gradient_ = true;
        const char* names[] = {"linear", "smooth", "quintic"};
        for (int k = 0; k < 3; ++k)
            if (ui::button({186.0f + k * 78, 13, 74, 26}, names[k], int(fade_) == k)) fade_ = FadeCurve(k);
        ui::toggle({430, 13, 80, 26}, "parts", &showParts_);
        if (ui::button({522, 13, 96, 26}, "randomize")) randomize();
        if (ui::button({622, 13, 70, 26}, "flat")) {  // 全部清零后只留一个格点，方便看单个格点的影响
            for (int i = 0; i <= kCells; ++i) value_[i] = slope_[i] = 0;
            value_[5] = 0.8f, slope_[5] = 1;
        }
    }

    std::string status() const override {
        const char* f = fade_ == FadeCurve::Linear ? "linear" : fade_ == FadeCurve::Smooth ? "smoothstep" : "quintic";
        return std::string(gradient_ ? "gradient noise · " : "value noise · ") + f;
    }

private:
    static constexpr float kHandleLen = 0.32f;  // 斜率控制柄的长度（以格子为单位）

    static void dashedClipped(Vector2 a, Vector2 b, Color c) {
        auto clip = [&](Vector2& p, Vector2 q) {  // 把 p 沿 pq 方向移进 [top, bottom]
            for (float lim : {kRowF.top, kRowF.bottom})
                if ((lim == kRowF.top && p.y < lim) || (lim == kRowF.bottom && p.y > lim)) {
                    float t = (lim - p.y) / (q.y - p.y);
                    p = {p.x + (q.x - p.x) * t, lim};
                }
        };
        clip(a, b), clip(b, a);
        drawDashedLine(a, b, 1.5f, c);
    }
    float yScale() const { return gradient_ ? 150.0f : 88.0f; }  // 每单位值多少像素
    float ry(float v) const { return kRowF.mid() - v * yScale(); }
    Vector2 handle(int i) const {
        float x = kX0 + i * kCellW;
        if (gradient_) return {x + kHandleLen * kCellW, kRowF.mid() - slope_[i] * kHandleLen * yScale()};
        return {x, ry(value_[i])};
    }
    // 第 i 个格子内 t 处的第 order 阶导数（相对 x，格子宽度为 1）
    float eval(int i, float t, int order) const {
        float F = fade(fade_, t), dF = fadeD(fade_, t), ddF = fadeDD(fade_, t);
        if (!gradient_) {
            float a = value_[i], b = value_[i + 1];
            return order == 0 ? valueSegment(a, b, t, fade_) : order == 1 ? (b - a) * dF : (b - a) * ddF;
        }
        float g0 = slope_[i], g1 = slope_[i + 1], n0 = g0 * t, n1 = g1 * (t - 1);
        if (order == 0) return gradientSegment(g0, g1, t, fade_);
        if (order == 1) return g0 + (g1 - g0) * F + (n1 - n0) * dF;
        return 2 * (g1 - g0) * dF + (n1 - n0) * ddF;
    }
    void randomize() {
        Pcg32 rng(seed_++);
        for (int i = 0; i <= kCells; ++i) value_[i] = rng.nextFloat() * 2 - 1, slope_[i] = rng.nextFloat() * 2 - 1;
    }

    float value_[kCells + 1]{}, slope_[kCells + 1]{};
    bool gradient_ = false, showParts_ = true;
    FadeCurve fade_ = FadeCurve::Linear;
    int drag_ = -1;
    float hoverX_ = -1;
    uint64_t seed_ = 3;
};

}  // namespace

int main() {
    static LatticeDemo demo;
    ga::DemoConfig config;
    config.title = "Lattice noise in 1D";
    config.width = kWidth;
    config.height = kHeight;
    config.stepsPerSecond = 30.0f;
    return ga::runDemo(demo, config);
}
