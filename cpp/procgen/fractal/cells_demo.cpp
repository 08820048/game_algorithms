// Simplex 与 Worley：两种不用正方形插值的噪声
//   - simplex：三角形网格。鼠标所在的三角形和它的 3 个角会标出来，圆圈是每个角的影响范围（半径 √0.5），箭头是梯度
//   - F1 / F2 / F2-F1：到最近、第二近特征点的距离；cells：按最近特征点所在的格子着色（Voronoi 图）
//   - 鼠标悬停：画出到最近、第二近特征点的连线，以及常见的 3×3 搜索范围
//   - 3x3 errors：逐像素比较 3×3 搜索与精确搜索，答案不同的像素标成朱红色
//   - 拖动画面平移

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "demo.h"
#include "fractal.h"
#include "ui.h"

namespace {

using namespace ga;
using namespace ga::noise;

constexpr int kWidth = 800, kHeight = 480;
constexpr float kTop = 52;
constexpr int kImgW = 480, kImgH = 392;
constexpr float kIx = 16, kIy = kTop + 10;
constexpr float kPx = 512, kPw = 272;

class CellsDemo final : public Demo {
public:
    void reset() override {}
    bool step() override { return true; }

    void update() override {
        if (!ready_) {
            Image img = GenImageColor(kImgW, kImgH, BLACK);
            tex_ = LoadTextureFromImage(img);
            UnloadImage(img);
            SetTextureFilter(tex_, TEXTURE_FILTER_BILINEAR);
            ready_ = true, dirty_ = true;
        }
        Vector2 m = GetMousePosition();
        bool in = m.x >= kIx && m.x < kIx + kImgW && m.y >= kIy && m.y < kIy + kImgH && !ui::wantsMouse();
        hover_ = in;
        mx_ = ox_ + (m.x - kIx) * cells_ / kImgW, my_ = oy_ + (m.y - kIy) * cells_ / kImgW;
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && in) panning_ = true, last_ = m;
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT) && panning_ && (m.x != last_.x || m.y != last_.y)) {
            ox_ -= (m.x - last_.x) * cells_ / kImgW, oy_ -= (m.y - last_.y) * cells_ / kImgW;
            last_ = m, dirty_ = true;
        }
        if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) panning_ = false;
        if (dirty_) render();
        if (hover_) updateHover();
    }

    void draw() const override {
        const Palette& pal = palette();
        const float hl = hairline();
        if (ready_) DrawTexture(tex_, int(kIx), int(kIy), WHITE);
        BeginScissor();
        if (mode_ == 0) drawSimplexOverlay(pal, hl);
        else drawWorleyOverlay(pal);
        EndScissorMode();
        DrawRectangleLinesEx({kIx, kIy, float(kImgW), float(kImgH)}, hl * 1.5f, pal.textMuted);

        // 右侧说明
        float y = 100;
        auto line = [&](const char* s, Color c) {
            drawText(s, {kPx, y}, 11, c);
            y += 17;
        };
        char buf[128];
        if (mode_ == 0) {
            line("each point is affected only by the 3 corners", pal.text);
            line("of its triangle. each corner adds", pal.text);
            line("  (0.5 - |d|^2)^4 * (g . d)", pal.text);
            line("which fades to 0 at |d| = 0.707: exactly", pal.text);
            line("the height of a triangle, so the corner", pal.text);
            line("beyond an edge never reaches inside.", pal.text);
            y += 10;
            if (hover_) {
                line("corner contributions at the mouse:", pal.textMuted);
                for (int k = 0; k < 3; ++k) {
                    std::snprintf(buf, sizeof buf, "  corner %d: %+.3f", k + 1, contrib_[k]);
                    line(buf, pal.text);
                }
                std::snprintf(buf, sizeof buf, "  sum:      %+.3f", contrib_[0] + contrib_[1] + contrib_[2]);
                line(buf, pal.path);
            }
        } else {
            line("one random feature point per cell.", pal.text);
            line("F1 / F2: distance to the nearest /", pal.text);
            line("second nearest feature point.", pal.text);
            line("the usual shortcut only searches the", pal.text);
            line("3x3 cells around the point (dashed box).", pal.text);
            y += 10;
            if (hover_) {
                std::snprintf(buf, sizeof buf, "F1 %.3f   F2 %.3f   cells searched %d", hov_.f1, hov_.f2, hov_.searched);
                line(buf, pal.text);
                bool wrong = hov3_.f1 != hov_.f1 || hov3_.f2 != hov_.f2;
                line(wrong ? "3x3 search would be WRONG here" : "3x3 search agrees here", wrong ? pal.path : pal.textMuted);
            }
            if (showErrors_) {
                y += 6;
                std::snprintf(buf, sizeof buf, "pixels where 3x3 is wrong: %d of %d", errors_, kImgW * kImgH);
                line(buf, errors_ ? pal.path : pal.textMuted);
                if (mode_ == 1 || mode_ == 4) line("(F1 errors are ~1 in 10^6: try F2 / F2-F1)", pal.textMuted);
            }
        }
        drawText("drag the image to pan", {kIx, kIy + kImgH + 5}, 11, pal.textMuted);
        DrawLineEx({0, kTop}, {float(kWidth), kTop}, hl, pal.gridLine);
    }

    void drawUi() override {
        const char* names[] = {"simplex", "F1", "F2", "F2-F1", "cells"};
        const float w[] = {74, 44, 44, 64, 60};
        float x = 12;
        for (int k = 0; k < 5; ++k) {
            if (ui::button({x, 13, w[k], 26}, names[k], mode_ == k)) mode_ = k, dirty_ = true;
            x += w[k] + 4;
        }
        if (ui::slider({316, 13, 190, 26}, "cells", &cells_, 2, 32, "%.0f", 1)) dirty_ = true;
        if (ui::toggle({516, 13, 104, 26}, "3x3 errors", &showErrors_)) dirty_ = true;
        if (ui::button({628, 13, 80, 26}, "new seed")) lat_.seed += 1, dirty_ = true;
    }

    std::string status() const override {
        const char* names[] = {"simplex", "worley F1", "worley F2", "worley F2-F1", "voronoi cells"};
        char buf[96];
        std::snprintf(buf, sizeof buf, "%s · seed %u", names[mode_], lat_.seed);
        return buf;
    }

private:
    static void BeginScissor() {
        float s = 1.0f / hairline();
        BeginScissorMode(int(kIx * s), int(kIy * s), int(kImgW * s), int(kImgH * s));
    }
    Vector2 scr(float x, float y) const { return {kIx + (x - ox_) / cells_ * kImgW, kIy + (y - oy_) / cells_ * kImgW}; }

    void drawSimplexOverlay(const Palette& pal, float hl) const {
        // 三角形网格：斜切坐标里的三组直线 u = 常数、v = 常数、u - v = 常数
        Color lc = Fade(pal.path, 0.35f);
        auto vert = [&](float i, float j) {
            float t = (i + j) * float(kUnskew);
            return scr(i - t, j - t);
        };
        float s0 = (ox_ + oy_) * float(kSkew);
        int lo = int(std::floor(std::min(ox_, oy_) + s0)) - 2, hi = int(std::ceil(std::max(ox_, oy_) + cells_ * 1.6f + s0)) + 2;
        for (int a = lo; a <= hi; ++a) {
            DrawLineEx(vert(float(a), float(lo)), vert(float(a), float(hi)), hl, lc);
            DrawLineEx(vert(float(lo), float(a)), vert(float(hi), float(a)), hl, lc);
            DrawLineEx(vert(float(a + lo), float(lo)), vert(float(a + hi), float(hi)), hl, lc);
        }
        if (!hover_) return;
        float s = (mx_ + my_) * float(kSkew);
        float fi = std::floor(mx_ + s), fj = std::floor(my_ + s);
        float t = (fi + fj) * float(kUnskew);
        float x0 = mx_ - (fi - t), y0 = my_ - (fj - t);
        int i1 = x0 > y0 ? 1 : 0, j1 = 1 - i1;
        float ci[3] = {fi, fi + i1, fi + 1}, cj[3] = {fj, fj + j1, fj + 1};
        Vector2 c[3];
        for (int k = 0; k < 3; ++k) c[k] = vert(ci[k], cj[k]);
        DrawTriangleLines(c[0], c[1], c[2], pal.path);
        float rpx = std::sqrt(0.5f) / cells_ * kImgW;
        for (int k = 0; k < 3; ++k) {
            DrawCircleLinesV(c[k], rpx, Fade(pal.path, 0.8f));
            float gx, gy;
            gradientVector(lat_.gradient(int(ci[k]), int(cj[k])), gx, gy);
            drawArrow(c[k], {c[k].x + gx * rpx * 0.6f, c[k].y + gy * rpx * 0.6f}, 2, pal.path, 7);
            DrawLineEx(c[k], scr(mx_, my_), 1, Fade(pal.text, 0.6f));
        }
        DrawCircleV(scr(mx_, my_), 3.5f, pal.text);
    }

    void drawWorleyOverlay(const Palette& pal) const {
        int i0 = int(std::floor(ox_)) - 1, j0 = int(std::floor(oy_)) - 1;
        Color grid = Fade(pal.textMuted, 0.45f);
        for (int i = i0; i <= i0 + int(cells_) + 3; ++i) DrawLineEx(scr(float(i), oy_), scr(float(i), oy_ + cells_), 1, grid);
        for (int j = j0; j <= j0 + int(cells_) + 3; ++j) DrawLineEx(scr(ox_, float(j)), scr(ox_ + cells_, float(j)), 1, grid);
        // 3×3 搜索出错的像素：画圈标出（单个像素太小，看不见）
        for (Vector2 e : errorPos_) {
            DrawCircleLinesV(e, 9, pal.path);
            DrawCircleLinesV(e, 10, pal.path);
            DrawCircleV(e, 2, pal.path);
        }
        for (int j = j0; j <= j0 + int(cells_) + 3; ++j)
            for (int i = i0; i <= i0 + int(cells_) + 3; ++i) {
                float px, py;
                featurePoint(lat_, i, j, px, py);
                DrawCircleV(scr(px, py), 2.5f, pal.path);
            }
        if (!hover_) return;
        int ix = int(std::floor(mx_)), iy = int(std::floor(my_));
        Vector2 a = scr(float(ix - 1), float(iy - 1)), b = scr(float(ix + 2), float(iy + 2));
        Color box = pal.textMuted;
        drawDashedLine(a, {b.x, a.y}, 1.5f, box), drawDashedLine({b.x, a.y}, b, 1.5f, box);
        drawDashedLine(b, {a.x, b.y}, 1.5f, box), drawDashedLine({a.x, b.y}, a, 1.5f, box);
        // 最近与第二近的特征点（暴力找出位置，用于画线）
        float best[2] = {1e30f, 1e30f};
        Vector2 bp[2]{};
        for (int j = iy - 3; j <= iy + 3; ++j)
            for (int i = ix - 3; i <= ix + 3; ++i) {
                float px, py;
                featurePoint(lat_, i, j, px, py);
                float d = std::hypot(px - mx_, py - my_);
                if (d < best[0]) best[1] = best[0], bp[1] = bp[0], best[0] = d, bp[0] = scr(px, py);
                else if (d < best[1]) best[1] = d, bp[1] = scr(px, py);
            }
        Vector2 m = scr(mx_, my_);
        DrawLineEx(m, bp[1], 1.5f, pal.textMuted);
        DrawLineEx(m, bp[0], 2.5f, pal.path);
        DrawCircleV(m, 3.5f, pal.text);
    }

    void render() {
        dirty_ = false;
        const float k = cells_ / kImgW;
        std::vector<Color> px(kImgW * kImgH);
        errors_ = 0;
        errorPos_.clear();
        for (int j = 0; j < kImgH; ++j)
            for (int i = 0; i < kImgW; ++i) {
                float x = ox_ + (i + 0.5f) * k, y = oy_ + (j + 0.5f) * k;
                float g = 0;
                bool err = false;
                if (mode_ == 0) {
                    g = simplexNoise(lat_, x, y) * 0.5f + 0.5f;
                } else {
                    CellSample<float> c = worley(lat_, x, y);
                    if (showErrors_) {
                        CellSample<float> c3 = worley(lat_, x, y, 1);
                        err = mode_ == 1 || mode_ == 4 ? c3.f1 != c.f1 : mode_ == 2 ? c3.f2 != c.f2 : (c3.f2 - c3.f1) != (c.f2 - c.f1);
                    }
                    if (mode_ == 1) g = c.f1 / 1.1f;
                    else if (mode_ == 2) g = c.f2 / 1.4f;
                    else if (mode_ == 3) g = (c.f2 - c.f1) / 1.0f;
                    else {  // Voronoi：按格子编号取一个灰度，边界（F2 - F1 很小）画深色
                        uint32_t h = mix32(lat_.hash(c.cellX, c.cellY) ^ 0x5bd1e995u);
                        g = 0.45f + 0.5f * float(h >> 8) / 16777216.0f;
                        // 到边界（两个特征点的垂直平分线）的距离 = (F2² - F1²) / (2 |p1 p2|)
                        float ax, ay, bx, by;
                        featurePoint(lat_, c.cellX, c.cellY, ax, ay);
                        featurePoint(lat_, c.cell2X, c.cell2Y, bx, by);
                        float border = (c.f2 * c.f2 - c.f1 * c.f1) / (2 * std::hypot(bx - ax, by - ay));
                        if (border < 1.2f * k) g = 0.15f;  // 约 1.2 像素宽
                    }
                }
                unsigned char cc = (unsigned char)std::lround(std::clamp(g, 0.0f, 1.0f) * 255);
                px[j * kImgW + i] = Color{cc, cc, cc, 255};
                if (err) ++errors_, errorPos_.push_back({kIx + i + 0.5f, kIy + j + 0.5f});
            }
        UpdateTexture(tex_, px.data());
    }
    void updateHover() {
        hov_ = worley(lat_, mx_, my_), hov3_ = worley(lat_, mx_, my_, 1);
        float s = (mx_ + my_) * float(kSkew);
        float fi = std::floor(mx_ + s), fj = std::floor(my_ + s), t = (fi + fj) * float(kUnskew);
        float x0 = mx_ - (fi - t), y0 = my_ - (fj - t);
        int i1 = x0 > y0 ? 1 : 0, j1 = 1 - i1;
        float dx[3] = {x0, x0 - i1 + float(kUnskew), x0 - 1 + 2 * float(kUnskew)};
        float dy[3] = {y0, y0 - j1 + float(kUnskew), y0 - 1 + 2 * float(kUnskew)};
        int ci[3] = {int(fi), int(fi) + i1, int(fi) + 1}, cj[3] = {int(fj), int(fj) + j1, int(fj) + 1};
        for (int k = 0; k < 3; ++k) {
            float a = std::max(0.0f, 0.5f - dx[k] * dx[k] - dy[k] * dy[k]), gx, gy;
            gradientVector(lat_.gradient(ci[k], cj[k]), gx, gy);
            contrib_[k] = a * a * a * a * (gx * dx[k] + gy * dy[k]) * float(kSimplexScale);
        }
    }

    HashLattice lat_{5};
    int mode_ = 0, errors_ = 0;
    bool showErrors_ = false, ready_ = false, dirty_ = true, panning_ = false, hover_ = false;
    float cells_ = 8, ox_ = 0.3f, oy_ = 0.2f, mx_ = 0, my_ = 0;
    float contrib_[3]{};
    std::vector<Vector2> errorPos_;
    CellSample<float> hov_, hov3_;
    Vector2 last_{};
    Texture2D tex_{};
};

}  // namespace

int main() {
    static CellsDemo demo;
    ga::DemoConfig config;
    config.title = "Simplex and Worley noise";
    config.width = kWidth;
    config.height = kHeight;
    config.stepsPerSecond = 30.0f;
    return ga::runDemo(demo, config);
}
