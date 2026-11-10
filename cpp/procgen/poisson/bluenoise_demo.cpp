// 五种点集与它们的功率谱
//   - white：均匀随机；jitter：抖动网格（每格一个随机点）；dart：投飞镖到连续 10 万次失败；bridson：k = 30；circle：候选放在圆上的变体
//   - 白噪声和抖动网格的点数与 Bridson 相同
//   - 中间是周期图（亮 = 能量高，中心是频率 0），下面是径向平均：1 是白噪声的水平，低频凹下去就是"蓝噪声"
//   - gaps：标出还能再放一个点的地方（离所有点都至少 r）

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "demo.h"
#include "poisson.h"
#include "ui.h"

namespace {

using namespace ga;
using namespace ga::poisson;

constexpr int kWidth = 800, kHeight = 480;
constexpr float kTop = 50;
constexpr float kS = 24;                    // 区域边长（r = 1）
constexpr float kScale = 16;                // 像素 / 单位
constexpr float kMx = 12, kMy = kTop + 22;  // 点集视图 384×384
constexpr int kM = 36;                      // 频率范围 ±kM / kS = ±1.5 个周期 / r
constexpr float kIx = 420, kIy = kTop + 22, kIw = 190;
constexpr float kCx = 420, kCy = 310, kCw = 368, kCh = 130;

class BlueNoiseDemo final : public Demo {
public:
    void reset() override {}
    bool step() override { return false; }
    void update() override {
        if (!tex_.id) {
            Image img = GenImageColor(2 * kM + 1, 2 * kM + 1, BLACK);
            tex_ = LoadTextureFromImage(img);
            UnloadImage(img);
        }
        if ((palette().background.r < 128) != dark_) dirty_ = true;  // 换了主题，重画周期图
        if (dirty_) generate();
    }

    void draw() const override {
        const Palette& pal = palette();
        const float hl = hairline();
        DrawRectangleLinesEx({kMx - 1, kMy - 1, kS * kScale + 2, kS * kScale + 2}, hl, pal.gridLine);
        if (showGaps_)
            for (const Vec2& g : gaps_) DrawRectangleRec({kMx + g.x * kScale - 1, kMy + g.y * kScale - 1, 2, 2}, pal.path);
        for (const Vec2& p : pts_) DrawCircleV({kMx + p.x * kScale, kMy + p.y * kScale}, 2.2f, pal.text);
        drawText("points (r = 16 px)", {kMx, kTop + 4}, 11, pal.textMuted);

        // 周期图
        drawText("periodogram", {kIx, kTop + 4}, 11, pal.textMuted);
        DrawTexturePro(tex_, {0, 0, float(tex_.width), float(tex_.height)}, {kIx, kIy, kIw, kIw}, {0, 0}, 0, WHITE);
        DrawRectangleLinesEx({kIx, kIy, kIw, kIw}, hl, pal.gridLine);

        // 数字
        char buf[96];
        float y = kIy;
        float tx = kIx + kIw + 14;
        auto line = [&](const char* s, Color c, float size = 12) {
            drawText(s, {tx, y}, size, c);
            y += size + 7;
        };
        std::snprintf(buf, sizeof buf, "points %d", int(pts_.size()));
        line(buf, pal.text);
        std::snprintf(buf, sizeof buf, "min distance %.3f r", minNN_);
        line(buf, minNN_ < 1 ? pal.path : pal.text);
        std::snprintf(buf, sizeof buf, "coverage %.3f", packingDensity(pts_.size(), kS, kS, 1));
        line(buf, pal.text);
        std::snprintf(buf, sizeof buf, "room for more %.2f%%", 100 * uncovered_);
        line(buf, pal.text);
        std::snprintf(buf, sizeof buf, "low-freq power %.2f", lowFreq_);
        line(buf, pal.text);
        line("(mean P for f < 0.5/r)", pal.textMuted, 10);

        // 径向平均
        drawText("radial average  (1 = white noise)", {kCx, kCy - 18}, 11, pal.textMuted);
        DrawRectangleLinesEx({kCx, kCy, kCw, kCh}, hl, pal.gridLine);
        const float pmax = 3;
        float y1 = kCy + kCh * (1 - 1 / pmax);
        drawDashedLine({kCx, y1}, {kCx + kCw, y1}, hl * 1.2f, pal.textMuted);
        for (float f : {0.5f, 1.0f}) {
            float x = kCx + kCw * f / (kM / kS);
            DrawLineEx({x, kCy}, {x, kCy + kCh}, hl, Fade(pal.gridLine, 0.7f));
            std::snprintf(buf, sizeof buf, "%.1f/r", f);
            drawText(buf, {x - 12, kCy + kCh + 3}, 9, pal.textMuted);
        }
        for (int b = 1; b + 1 <= kM; ++b) {
            auto pt = [&](int k) { return Vector2{kCx + kCw * k / kM, kCy + kCh * (1 - std::min(1.0f, float(radial_[k]) / pmax))}; };
            DrawLineEx(pt(b), pt(b + 1), 1.8f, pal.path);
        }
        DrawLineEx({0, kTop}, {float(kWidth), kTop}, hl, pal.gridLine);
    }

    void drawUi() override {
        bool ch = false;
        const char* names[] = {"white", "jitter", "dart", "bridson", "circle"};
        for (int k = 0; k < 5; ++k)
            if (ui::button({12.0f + k * 76, 12, 72, 26}, names[k], method_ == k)) method_ = k, ch = true;
        ch |= ui::slider({400, 12, 150, 26}, "seed", &seed_, 1, 30, "%.0f", 1);
        ui::toggle({562, 12, 80, 26}, "gaps", &showGaps_);
        if (ch) dirty_ = true;
    }

    std::string status() const override {
        const char* names[] = {"white", "jitter", "dart", "bridson", "circle"};
        char buf[96];
        std::snprintf(buf, sizeof buf, "%s · %d points · low-freq power %.2f", names[method_], int(pts_.size()), lowFreq_);
        return buf;
    }

private:
    void generate() {
        dirty_ = false;
        Pcg32 rng{uint64_t(seed_)};
        BridsonParams prm;
        std::vector<Vec2> ref = bridson(kS, kS, 1, rng, prm);
        if (method_ == 0) pts_ = whiteNoise(kS, kS, int(ref.size()), rng);
        else if (method_ == 1) pts_ = jitteredGrid(kS, kS, std::sqrt(kS * kS / ref.size()), rng);
        else if (method_ == 2) pts_ = dartThrowing(kS, kS, 1, rng, 100000);
        else if (method_ == 3) pts_ = ref;
        else {
            prm.annulus = Annulus::Circle;
            pts_ = bridson(kS, kS, 1, rng, prm);
        }
        std::vector<float> nn = nearestNeighbor(pts_, kS, kS, 1);
        minNN_ = INFINITY;
        for (float d : nn) minNN_ = std::min(minNN_, d);
        // 还能放点的地方（只对最小距离 ≥ r 的点集有意义）
        gaps_.clear();
        uncovered_ = 0;
        if (minNN_ >= 1) {
            const int res = 192;
            Grid g(kS, kS, 1);
            for (int i = 0; i < int(pts_.size()); ++i) g.insert(pts_[i], i);
            for (int yy = 0; yy < res; ++yy)
                for (int xx = 0; xx < res; ++xx) {
                    Vec2 p{(xx + 0.5f) * kS / res, (yy + 0.5f) * kS / res};
                    if (g.fits(p, pts_)) gaps_.push_back(p);
                }
            uncovered_ = double(gaps_.size()) / (res * res);
        }
        std::vector<float> P = periodogram(pts_, kS, kS, kM);
        radial_ = radialAverage(P, kM);
        double s = 0, c = 0;
        for (int b = 1; b <= kM; ++b)
            if (b / kS < 0.5f) s += radial_[b], c += 1;
        lowFreq_ = float(s / c);
        // 图像：log 刻度，P = 0.05 .. 5 映射到 0 .. 1；中心（P = n）钳到最亮
        const Palette& pal = palette();
        int side = 2 * kM + 1;
        std::vector<Color> px(size_t(side) * side);
        for (size_t i = 0; i < px.size(); ++i) {
            float v = std::clamp((std::log10(std::max(P[i], 1e-3f)) + 1.3f) / 2.0f, 0.0f, 1.0f);
            px[i] = ColorLerp(pal.background, pal.text, v);
        }
        UpdateTexture(tex_, px.data());
        dark_ = pal.background.r < 128;
    }

    int method_ = 3;
    float seed_ = 1;
    bool dirty_ = true, showGaps_ = false, dark_ = false;
    Texture2D tex_{};
    std::vector<Vec2> pts_, gaps_;
    std::vector<double> radial_;
    float minNN_ = 0, lowFreq_ = 0;
    double uncovered_ = 0;
};

}  // namespace

int main() {
    static BlueNoiseDemo demo;
    ga::DemoConfig config;
    config.title = "Blue noise spectra";
    config.width = kWidth;
    config.height = kHeight;
    config.autoPlay = false;
    return ga::runDemo(demo, config);
}
