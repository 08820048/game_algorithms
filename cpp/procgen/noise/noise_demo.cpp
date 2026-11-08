// 二维噪声
//   - value / perlin，linear / smooth / quintic；cells：画面宽度对应多少个格子
//   - gray：灰度（-1 黑，+1 白，不做拉伸，所以 Perlin 显得灰）；bands：按值分成 12 条带，看形状；
//     shade：把噪声当成高度图打光，用解析梯度求法线——导数的折角在光照下一眼就能看到
//   - lattice：画出格线与格点上的随机量；拖动画面平移（噪声可以在任意位置求值，没有边界）
//   - 右侧：当前画面的直方图（Perlin 的理论上限 ±√2/2 用虚线标出），鼠标所在行的剖面

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "demo.h"
#include "noise.h"
#include "ui.h"

namespace {

using namespace ga;
using namespace ga::noise;

constexpr int kWidth = 800, kHeight = 480;
constexpr float kTop = 52;
constexpr int kImgW = 480, kImgH = 392;
constexpr float kIx = 16, kIy = kTop + 10;
constexpr float kPx = 512, kPw = 272;  // 右侧面板
constexpr int kBins = 40;

class NoiseDemo final : public Demo {
public:
    void reset() override {}
    bool step() override { return true; }

    void update() override {
        if (!texReady_) {
            Image img = GenImageColor(kImgW, kImgH, BLACK);
            tex_ = LoadTextureFromImage(img);
            UnloadImage(img);
            SetTextureFilter(tex_, TEXTURE_FILTER_BILINEAR);
            pixels_.resize(kImgW * kImgH);
            values_.resize(kImgW * kImgH);
            texReady_ = true, dirty_ = true;
        }
        Vector2 m = GetMousePosition();
        bool in = m.x >= kIx && m.x < kIx + kImgW && m.y >= kIy && m.y < kIy + kImgH && !ui::wantsMouse();
        hoverRow_ = in ? int(m.y - kIy) : -1;
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && in) panning_ = true, last_ = m;
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT) && panning_) {
            if (m.x != last_.x || m.y != last_.y) {
                float k = cells_ / kImgW;
                ox_ -= (m.x - last_.x) * k, oy_ -= (m.y - last_.y) * k;
                last_ = m, dirty_ = true;
            }
        }
        if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) panning_ = false;
        if (dirty_) render();
    }

    void draw() const override {
        const Palette& pal = palette();
        const float hl = hairline();
        if (texReady_) DrawTexture(tex_, int(kIx), int(kIy), WHITE);

        float k = kImgW / cells_;  // 每格多少像素
        if (showLattice_) {
            float x0 = std::ceil(ox_), y0 = std::ceil(oy_);
            Color line = Fade(pal.path, 0.55f);
            for (float x = x0; x <= ox_ + cells_ + 1e-4f; x += 1) DrawLineEx({sx(x), kIy}, {sx(x), kIy + kImgH}, hl, line);
            for (float y = y0; y <= oy_ + kImgH / k + 1e-4f; y += 1) DrawLineEx({kIx, sy(y)}, {kIx + kImgW, sy(y)}, hl, line);
            if (k >= 22)
                for (float y = y0; y <= oy_ + kImgH / k + 1e-4f; y += 1)
                    for (float x = x0; x <= ox_ + cells_ + 1e-4f; x += 1) {
                        Vector2 c{sx(x), sy(y)};
                        if (c.x < kIx + 6 || c.x > kIx + kImgW - 6 || c.y < kIy + 6 || c.y > kIy + kImgH - 6) continue;
                        if (perlin_) {
                            float gx, gy;
                            gradientVector(lat_.gradient(int(x), int(y)), gx, gy);
                            drawArrow(c, {c.x + gx * k * 0.4f, c.y + gy * k * 0.4f}, 1.8f, pal.path, 6);
                        } else {
                            unsigned char g = (unsigned char)std::lround((lat_.value(int(x), int(y)) * 0.5f + 0.5f) * 255);
                            DrawCircleV(c, 4.5f, {g, g, g, 255});
                            DrawCircleLinesV(c, 4.5f, pal.path);
                        }
                    }
        }
        if (hoverRow_ >= 0) DrawLineEx({kIx, kIy + hoverRow_ + 0.5f}, {kIx + kImgW, kIy + hoverRow_ + 0.5f}, 1, pal.path);
        DrawRectangleLinesEx({kIx, kIy, float(kImgW), float(kImgH)}, hl * 1.5f, pal.textMuted);

        // 直方图：横轴 [-1, 1]
        const float hx = kPx, hw = kPw, hy = 118, hh = 110;
        drawText("histogram of this view", {hx, hy - 18}, 11, pal.textMuted);
        int maxBin = 1;
        for (int b : hist_) maxBin = std::max(maxBin, b);
        for (int b = 0; b < kBins; ++b) {
            float h = hh * hist_[b] / maxBin, w = hw / kBins;
            DrawRectangleRec({hx + b * w, hy + hh - h, w - 1, h}, pal.text);
        }
        DrawLineEx({hx, hy + hh}, {hx + hw, hy + hh}, hl, pal.textMuted);
        auto hxOf = [&](float v) { return hx + (v + 1) / 2 * hw; };
        drawText("-1", {hx - 4, hy + hh + 3}, 10, pal.textMuted);
        drawText("0", {hxOf(0) - 3, hy + hh + 3}, 10, pal.textMuted);
        drawText("1", {hx + hw - 4, hy + hh + 3}, 10, pal.textMuted);
        if (perlin_)
            for (float s : {-0.7071f, 0.7071f}) drawDashedLine({hxOf(s), hy - 4}, {hxOf(s), hy + hh}, 1.2f, pal.path, 4, 3);
        char buf[96];
        std::snprintf(buf, sizeof buf, "min %.3f   max %.3f   std %.3f", vmin_, vmax_, vstd_);
        drawText(buf, {hx, hy + hh + 18}, 11, pal.text);
        if (perlin_) drawText("dashed: perlin bound +-0.707", {hx, hy + hh + 34}, 10, pal.path);

        // 剖面
        const float py = 318, ph = 110;
        int row = hoverRow_ >= 0 ? hoverRow_ : kImgH / 2;
        drawText(hoverRow_ >= 0 ? "profile of the marked row" : "profile of the middle row (hover to pick)", {kPx, py - 18}, 11, pal.textMuted);
        DrawLineEx({kPx, py + ph / 2}, {kPx + kPw, py + ph / 2}, hl, pal.gridLine);
        for (float x = std::ceil(ox_); x <= ox_ + cells_; x += 1) {
            float px = kPx + (x - ox_) / cells_ * kPw;
            DrawLineEx({px, py}, {px, py + ph}, hl, Fade(pal.path, 0.5f));
        }
        Vector2 prev{};
        for (int i = 0; i < kImgW; i += 2) {
            Vector2 p{kPx + float(i) / kImgW * kPw, py + ph / 2 - values_[row * kImgW + i] * ph / 2};
            if (i > 0) DrawLineEx(prev, p, 1.5f, pal.text);
            prev = p;
        }
        DrawRectangleLinesEx({kPx, py, kPw, ph}, hl, pal.textMuted);
        drawText("+1", {kPx + kPw + 3, py - 4}, 9, pal.textMuted);
        drawText("-1", {kPx + kPw + 3, py + ph - 8}, 9, pal.textMuted);
        drawText("drag the image to pan: noise is defined everywhere", {kIx, kIy + kImgH + 5}, 11, pal.textMuted);
        DrawLineEx({0, kTop}, {float(kWidth), kTop}, hl, pal.gridLine);
    }

    void drawUi() override {
        if (ui::button({12, 13, 64, 26}, "value", !perlin_)) perlin_ = false, dirty_ = true;
        if (ui::button({80, 13, 64, 26}, "perlin", perlin_)) perlin_ = true, dirty_ = true;
        const char* names[] = {"linear", "smooth", "quintic"};
        for (int k = 0; k < 3; ++k)
            if (ui::button({160.0f + k * 72, 13, 68, 26}, names[k], int(fade_) == k)) fade_ = FadeCurve(k), dirty_ = true;
        if (ui::slider({384, 13, 220, 26}, "cells", &cells_, 2, 48, "%.0f", 1)) dirty_ = true;
        if (ui::button({614, 13, 80, 26}, "new seed")) lat_.seed += 1, dirty_ = true;
        const char* views[] = {"gray", "bands", "shade"};
        for (int k = 0; k < 3; ++k)
            if (ui::button({kPx + k * 66, 64, 62, 24}, views[k], view_ == k)) view_ = k, dirty_ = true;
        ui::toggle({kPx + 200, 64, 72, 24}, "lattice", &showLattice_);
    }

    std::string status() const override {
        char buf[96];
        std::snprintf(buf, sizeof buf, "%s · %s · seed %u", perlin_ ? "perlin" : "value",
                      fade_ == FadeCurve::Linear ? "linear" : fade_ == FadeCurve::Smooth ? "smoothstep" : "quintic", lat_.seed);
        return buf;
    }

private:
    float sx(float x) const { return kIx + (x - ox_) / cells_ * kImgW; }
    float sy(float y) const { return kIy + (y - oy_) / cells_ * kImgW; }

    void render() {
        dirty_ = false;
        const float k = cells_ / kImgW;
        // 光照：光从左上方来；高度放大 relief 倍后求法线
        const float lx = -0.55f, ly = -0.55f, lz = 0.63f, relief = 0.5f;
        double s = 0, s2 = 0;
        vmin_ = 1e9f, vmax_ = -1e9f;
        for (int& b : hist_) b = 0;
        for (int j = 0; j < kImgH; ++j)
            for (int i = 0; i < kImgW; ++i) {
                float x = ox_ + (i + 0.5f) * k, y = oy_ + (j + 0.5f) * k;
                Sample<float> smp = perlin_ ? perlinNoiseD(lat_, x, y, fade_) : valueNoiseD(lat_, x, y, fade_);
                float v = smp.value;
                values_[j * kImgW + i] = v;
                s += v, s2 += double(v) * v, vmin_ = std::min(vmin_, v), vmax_ = std::max(vmax_, v);
                hist_[std::clamp(int((v + 1) / 2 * kBins), 0, kBins - 1)]++;
                float g;
                if (view_ == 0) {
                    g = v * 0.5f + 0.5f;
                } else if (view_ == 1) {  // 12 条带，相邻条带深浅交替，边界画深色线
                    float t = (v + 1) * 6;
                    int band = int(std::floor(t));
                    float frac = t - band;
                    g = band % 2 ? 0.78f : 0.92f;
                    if (frac < 0.06f) g = 0.35f;
                } else {
                    float nx = -smp.dx * relief, ny = -smp.dy * relief, nz = 1;
                    float len = std::sqrt(nx * nx + ny * ny + nz * nz);
                    g = std::max(0.0f, (nx * lx + ny * ly + nz * lz) / len);
                    g = 0.12f + 0.88f * g;
                }
                unsigned char c = (unsigned char)std::lround(std::clamp(g, 0.0f, 1.0f) * 255);
                pixels_[j * kImgW + i] = {c, c, c, 255};
            }
        double n = double(kImgW) * kImgH;
        vstd_ = float(std::sqrt(std::max(0.0, s2 / n - (s / n) * (s / n))));
        UpdateTexture(tex_, pixels_.data());
    }

    HashLattice lat_{7};
    bool perlin_ = true, showLattice_ = false, panning_ = false, dirty_ = true, texReady_ = false;
    FadeCurve fade_ = FadeCurve::Quintic;
    int view_ = 0, hoverRow_ = -1;
    float cells_ = 8, ox_ = 0.0f, oy_ = 0.0f;
    Vector2 last_{};
    Texture2D tex_{};
    std::vector<Color> pixels_;
    std::vector<float> values_;
    int hist_[kBins]{};
    float vmin_ = 0, vmax_ = 0, vstd_ = 0;
};

}  // namespace

int main() {
    static NoiseDemo demo;
    ga::DemoConfig config;
    config.title = "Value and Perlin noise";
    config.width = kWidth;
    config.height = kHeight;
    config.stepsPerSecond = 30.0f;
    return ga::runDemo(demo, config);
}
