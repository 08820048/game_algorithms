// 分形布朗运动
//   - base：每一层用哪种噪声；fbm / ridged：普通叠加，或每层取 (1 - |n|)² 形成山脊
//   - octaves：层数；gain：每层振幅乘以多少（频率每层翻倍）
//   - offsets：每层平移一个非整数偏移。关掉后各层的格点重合，lattice 标出的点上所有层都为 0（Perlin、Simplex）
//   - 右侧上方：每一层单独的样子（按各自的振幅缩放）；下方：画面左上 256×256 的径向功率谱（双对数），
//     虚线是理论斜率 -(2 - 2·log2 gain)

#include <cmath>
#include <complex>
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
constexpr int kImg = 384;
constexpr float kIx = 16, kIy = kTop + 10;
constexpr float kPx = 420, kPw = 366;
constexpr int kThumb = 40, kMaxOct = 8;
constexpr int kFft = 256;
constexpr float kBaseCells = 3;  // 第 0 层：画面宽度对应 3 个格子

void fft(std::vector<std::complex<float>>& a) {
    size_t n = a.size();
    for (size_t i = 1, j = 0; i < n; ++i) {
        size_t bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) std::swap(a[i], a[j]);
    }
    for (size_t len = 2; len <= n; len <<= 1) {
        std::complex<float> w = std::polar(1.0f, -2 * 3.14159265f / float(len));
        for (size_t i = 0; i < n; i += len) {
            std::complex<float> cur = 1;
            for (size_t k = 0; k < len / 2; ++k, cur *= w) {
                auto u = a[i + k], v = a[i + k + len / 2] * cur;
                a[i + k] = u + v, a[i + k + len / 2] = u - v;
            }
        }
    }
}

class FbmDemo final : public Demo {
public:
    void reset() override {}
    bool step() override { return true; }

    void update() override {
        if (!ready_) {
            Image img = GenImageColor(kImg, kImg, BLACK);
            tex_ = LoadTextureFromImage(img);
            UnloadImage(img);
            SetTextureFilter(tex_, TEXTURE_FILTER_BILINEAR);
            Image t = GenImageColor(kThumb, kThumb * kMaxOct, BLACK);
            thumbs_ = LoadTextureFromImage(t);
            UnloadImage(t);
            ready_ = true, dirty_ = true;
        }
        if (dirty_) render();
    }

    void draw() const override {
        const Palette& pal = palette();
        const float hl = hairline();
        if (ready_) DrawTexture(tex_, int(kIx), int(kIy), WHITE);
        if (showLattice_) {  // 第 0 层的格点（Simplex 是三角形网格的顶点）
            float k = kImg / kBaseCells;
            for (int j = -2; j <= 8; ++j)
                for (int i = -2; i <= 8; ++i) {
                    float x = float(i), y = float(j);
                    if (base_ == 2) {
                        float t = (x + y) * float(kUnskew);
                        x -= t, y -= t;
                    }
                    Vector2 c{kIx + x * k, kIy + y * k};
                    if (c.x < kIx + 4 || c.y < kIy + 4 || c.x > kIx + kImg - 4 || c.y > kIy + kImg - 4) continue;
                    DrawCircleLinesV(c, 5, pal.path);
                    DrawCircleV(c, 1.5f, pal.path);
                }
        }
        DrawRectangleLinesEx({kIx, kIy, float(kImg), float(kImg)}, hl * 1.5f, pal.textMuted);
        char buf[128];
        std::snprintf(buf, sizeof buf, "min %.3f  max %.3f  std %.3f", vmin_, vmax_, vstd_);
        drawText(buf, {kIx, kIy + kImg + 6}, 11, pal.textMuted);

        // 各层
        drawText("each octave on its own (scaled by its amplitude)", {kPx, 66}, 11, pal.textMuted);
        for (int o = 0; o < octaves(); ++o) {
            Rectangle src{0, float(o * kThumb), float(kThumb), float(kThumb)};
            Rectangle dst{kPx + o * (kThumb + 6.0f), 84, float(kThumb), float(kThumb)};
            if (ready_) DrawTexturePro(thumbs_, src, dst, {0, 0}, 0, WHITE);
            DrawRectangleLinesEx(dst, hl, pal.textMuted);
            std::snprintf(buf, sizeof buf, "%g", std::pow(gain_, o));
            drawText(buf, {dst.x, dst.y + kThumb + 3}, 9, pal.textMuted);
        }

        // 频谱：横轴 log10 r（r = 1..128，单位 1/256 周期每像素），纵轴 log10 功率
        const float sx = kPx + 30, sy = 168, sw = kPw - 40, sh = 260;
        DrawRectangleLinesEx({sx, sy, sw, sh}, hl, pal.textMuted);
        drawText("radial power spectrum of the top-left 256x256 (log-log)", {kPx, sy - 18}, 11, pal.textMuted);
        float lmax = -1e9f, lmin = 1e9f;
        for (int r = 1; r < kFft / 2; ++r) lmax = std::max(lmax, logPsd_[r]), lmin = std::min(lmin, logPsd_[r]);
        lmin = std::max(lmin, lmax - 12);
        auto px = [&](float r) { return sx + std::log10(r) / std::log10(128.0f) * sw; };
        auto py = [&](float l) { return sy + (lmax - l) / (lmax - lmin) * sh; };
        // 每层噪声的峰值频率附近画一条竖线：第 o 层每像素 (3·2^o / 384) × 0.5 个周期
        for (int o = 0; o < octaves(); ++o) {
            float r = kBaseCells * std::pow(2.0f, float(o)) / kImg * 0.5f * kFft;
            if (r >= 1 && r < 128) DrawLineEx({px(r), sy}, {px(r), sy + sh}, hl, Fade(pal.gridLine, 0.9f));
        }
        Vector2 prev{};
        for (int r = 1; r < kFft / 2; ++r) {
            Vector2 p{px(float(r)), std::clamp(py(logPsd_[r]), sy, sy + sh)};
            if (r > 1) DrawLineEx(prev, p, 1.6f, pal.text);
            prev = p;
        }
        if (!ridged_) {  // 理论斜率，过拟合区间的中点
            float theory = -(2 - 2 * std::log2(gain_));
            float rm = 16, lm = fitMid_;
            Vector2 a{px(2), py(lm + theory * (std::log10(2.0f) - std::log10(rm)))};
            Vector2 b{px(127), py(lm + theory * (std::log10(127.0f) - std::log10(rm)))};
            drawDashedLine(a, b, 1.5f, pal.path, 6, 4);
            std::snprintf(buf, sizeof buf, "fitted slope %.2f   theory %.2f", slope_, theory);
        } else {
            std::snprintf(buf, sizeof buf, "fitted slope %.2f   (ridged: no simple theory)", slope_);
        }
        drawText(buf, {sx, sy + sh + 8}, 11, pal.text);
        drawText("frequency ->", {sx + sw - 80, sy + sh - 16}, 10, pal.textMuted);
        drawText("power", {sx + 4, sy + 4}, 10, pal.textMuted);
        DrawLineEx({0, kTop}, {float(kWidth), kTop}, hl, pal.gridLine);
    }

    void drawUi() override {
        const char* names[] = {"value", "perlin", "simplex"};
        for (int k = 0; k < 3; ++k)
            if (ui::button({12.0f + k * 66, 13, 62, 26}, names[k], base_ == k)) base_ = k, dirty_ = true;
        if (ui::button({214, 13, 52, 26}, "fbm", !ridged_)) ridged_ = false, dirty_ = true;
        if (ui::button({268, 13, 64, 26}, "ridged", ridged_)) ridged_ = true, dirty_ = true;
        if (ui::slider({342, 13, 150, 26}, "octaves", &octaves_, 1, kMaxOct, "%.0f", 1)) dirty_ = true;
        if (ui::slider({500, 13, 140, 26}, "gain", &gain_, 0.2f, 0.8f, "%.2f", 0.05f)) dirty_ = true;
        if (ui::toggle({648, 13, 76, 26}, "offsets", &offsets_)) dirty_ = true;
        ui::toggle({728, 13, 64, 26}, "lattice", &showLattice_);
    }

    std::string status() const override {
        const char* names[] = {"value", "perlin", "simplex"};
        char buf[96];
        std::snprintf(buf, sizeof buf, "%s %s · %d octaves · gain %.2f%s", ridged_ ? "ridged" : "fbm", names[base_], octaves(), gain_,
                      offsets_ ? "" : " · no offsets");
        return buf;
    }

private:
    int octaves() const { return int(std::lround(octaves_)); }
    float baseNoise(float x, float y) const {
        if (base_ == 0) return valueNoise(lat_, x, y);
        if (base_ == 1) return perlinNoise(lat_, x, y);
        return simplexNoise(lat_, x, y);
    }
    float sample(float x, float y) const {
        FbmParams p;
        p.octaves = octaves(), p.gain = gain_, p.offsets = offsets_;
        auto b = [&](float u, float v) { return baseNoise(u, v); };
        return ridged_ ? ridged(b, x, y, p) : fbm(b, x, y, p);
    }

    void render() {
        dirty_ = false;
        const float k = kBaseCells / kImg;
        std::vector<float> v(kImg * kImg);
        double s = 0, s2 = 0;
        vmin_ = 1e9f, vmax_ = -1e9f;
        for (int j = 0; j < kImg; ++j)
            for (int i = 0; i < kImg; ++i) {
                float x = (i + 0.5f) * k, y = (j + 0.5f) * k, val = sample(x, y);
                v[j * kImg + i] = val;
                s += val, s2 += double(val) * val, vmin_ = std::min(vmin_, val), vmax_ = std::max(vmax_, val);
            }
        double n = double(kImg) * kImg, mean = s / n;
        vstd_ = float(std::sqrt(std::max(0.0, s2 / n - mean * mean)));
        // 灰度：按单层噪声的典型范围映射（不按本画面拉伸，层数变化时亮度范围的变化也看得到）
        float range = base_ == 1 ? 0.7f : 1.0f;
        std::vector<Color> px(kImg * kImg);
        for (int i = 0; i < kImg * kImg; ++i) {
            float g = ridged_ ? v[i] : v[i] / range * 0.5f + 0.5f;
            unsigned char c = (unsigned char)std::lround(std::clamp(g, 0.0f, 1.0f) * 255);
            px[i] = {c, c, c, 255};
        }
        UpdateTexture(tex_, px.data());

        // 各层缩略图：第 o 层 = 振幅 gain^o × base(频率 2^o)，显示同一块区域
        std::vector<Color> th(kThumb * kThumb * kMaxOct, {0, 0, 0, 255});
        for (int o = 0; o < octaves(); ++o) {
            double ox = 0, oy = 0;
            if (offsets_) octaveOffset(o, ox, oy);
            float amp = std::pow(gain_, float(o)), freq = std::pow(2.0f, float(o));
            for (int j = 0; j < kThumb; ++j)
                for (int i = 0; i < kThumb; ++i) {
                    float x = (i + 0.5f) * kImg / kThumb * k, y = (j + 0.5f) * kImg / kThumb * k;
                    float b = baseNoise(x * freq + float(ox), y * freq + float(oy));
                    if (ridged_) b = (1 - std::abs(b)) * (1 - std::abs(b)) * 2 - 1;
                    float g = 0.5f + 0.5f * amp * b / range;
                    unsigned char c = (unsigned char)std::lround(std::clamp(g, 0.0f, 1.0f) * 255);
                    th[(o * kThumb + j) * kThumb + i] = {c, c, c, 255};
                }
        }
        UpdateTexture(thumbs_, th.data());

        // 径向功率谱
        std::vector<std::vector<std::complex<float>>> a(kFft, std::vector<std::complex<float>>(kFft));
        for (int j = 0; j < kFft; ++j)
            for (int i = 0; i < kFft; ++i) {
                float w = (0.5f - 0.5f * std::cos(2 * 3.14159265f * i / kFft)) * (0.5f - 0.5f * std::cos(2 * 3.14159265f * j / kFft));
                a[j][i] = (v[j * kImg + i] - float(mean)) * w;
            }
        for (auto& row : a) fft(row);
        std::vector<std::complex<float>> col(kFft);
        for (int i = 0; i < kFft; ++i) {
            for (int j = 0; j < kFft; ++j) col[j] = a[j][i];
            fft(col);
            for (int j = 0; j < kFft; ++j) a[j][i] = col[j];
        }
        std::vector<double> psd(kFft / 2, 0), cnt(kFft / 2, 0);
        for (int j = 0; j < kFft; ++j)
            for (int i = 0; i < kFft; ++i) {
                int fi = i <= kFft / 2 ? i : i - kFft, fj = j <= kFft / 2 ? j : j - kFft;
                int r = int(std::lround(std::hypot(fi, fj)));
                if (r < kFft / 2) psd[r] += std::norm(a[j][i]), cnt[r] += 1;
            }
        for (int r = 0; r < kFft / 2; ++r) logPsd_[r] = float(std::log10(std::max(1e-20, psd[r] / std::max(1.0, cnt[r]))));
        // 在 r = 4..64 上拟合直线
        double mx = 0, my = 0, sxy = 0, sxx = 0;
        int m = 0;
        for (int r = 4; r <= 64; ++r) mx += std::log10(r), my += logPsd_[r], ++m;
        mx /= m, my /= m;
        for (int r = 4; r <= 64; ++r) sxy += (std::log10(r) - mx) * (logPsd_[r] - my), sxx += (std::log10(r) - mx) * (std::log10(r) - mx);
        slope_ = float(sxy / sxx);
        fitMid_ = float(my + slope_ * (std::log10(16.0) - mx));
    }

    HashLattice lat_{3};
    int base_ = 1;
    bool ridged_ = false, offsets_ = true, showLattice_ = false, ready_ = false, dirty_ = true;
    float octaves_ = 6, gain_ = 0.5f;
    Texture2D tex_{}, thumbs_{};
    float logPsd_[kFft / 2]{};
    float slope_ = 0, fitMid_ = 0, vmin_ = 0, vmax_ = 0, vstd_ = 0;
};

}  // namespace

int main() {
    static FbmDemo demo;
    ga::DemoConfig config;
    config.title = "Fractal Brownian motion";
    config.width = kWidth;
    config.height = kHeight;
    config.stepsPerSecond = 30.0f;
    return ga::runDemo(demo, config);
}
