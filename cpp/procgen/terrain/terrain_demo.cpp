// 用噪声生成地形
//   - fbm / ridged / damped / warped：高度用哪种噪声。damped 是"坡度大的地方细节弱"的 fBm，strength 是抑制强度 k；
//     warped 是域扭曲，strength 是扭曲距离（单位：第 0 层的格子）
//   - exponent：高度重映射的幂次；island：把高度往"中间高、边缘低"的形状拉；sea：海平面
//   - biomes：按高度和湿度（另一个独立的 fBm）查表着色
//   - 右侧：同一张地图的斜视 3D 视图（从南往北看），以及高度直方图（虚线是海平面）

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "demo.h"
#include "terrain.h"
#include "terrain_view.h"
#include "ui.h"

namespace {

using namespace ga;
using namespace ga::noise;
using namespace ga::terrain;

constexpr int kWidth = 800, kHeight = 480;
constexpr float kTop = 80;
constexpr int kMap = 256, kImg = 384;
constexpr float kIx = 16, kIy = kTop + 8;
constexpr float kPx = 416;
constexpr int kViewW = 368, kViewH = 236;
constexpr float kViewY = kTop + 8;
constexpr float kHistY = 356, kHistH = 84;
constexpr int kBins = 64;
constexpr float kCells = 3;  // 地图宽度 = 第 0 层的 3 个格子

Color biomeColor(float t, float m) {  // t：陆地高度（0 海岸 .. 1 最高）；m：湿度 0..1
    if (t < 0.04f) return {214, 200, 150, 255};                       // 海滩
    if (t > 0.85f) return m > 0.35f ? Color{245, 245, 245, 255} : Color{160, 155, 150, 255};  // 雪 / 裸岩
    if (t > 0.6f) return m < 0.33f ? Color{150, 140, 120, 255} : m < 0.66f ? Color{125, 140, 110, 255} : Color{90, 120, 95, 255};
    if (t > 0.3f) return m < 0.33f ? Color{190, 180, 120, 255} : m < 0.66f ? Color{100, 150, 70, 255} : Color{50, 110, 60, 255};
    return m < 0.33f ? Color{220, 200, 130, 255} : m < 0.66f ? Color{140, 180, 80, 255} : Color{60, 130, 70, 255};
}

class TerrainDemo final : public Demo {
public:
    void reset() override {}
    bool step() override { return true; }

    void update() override {
        if (!ready_) {
            Image img = GenImageColor(kMap, kMap, BLACK);
            map_ = LoadTextureFromImage(img);
            UnloadImage(img);
            SetTextureFilter(map_, TEXTURE_FILTER_BILINEAR);
            Image v = GenImageColor(kViewW, kViewH, BLANK);
            view_ = LoadTextureFromImage(v);
            UnloadImage(v);
            ready_ = true, dirty_ = true;
        }
        if (dirty_) render();
    }

    void draw() const override {
        const Palette& pal = palette();
        const float hl = hairline();
        if (ready_) {
            DrawTexturePro(map_, {0, 0, float(kMap), float(kMap)}, {kIx, kIy, float(kImg), float(kImg)}, {0, 0}, 0, WHITE);
            DrawTexture(view_, int(kPx), int(kViewY), WHITE);
        }
        DrawRectangleLinesEx({kIx, kIy, float(kImg), float(kImg)}, hl * 1.5f, pal.textMuted);
        drawText("oblique view from the south", {kPx, kViewY - 2}, 10, pal.textMuted);

        // 高度直方图：横轴高度 0..1，纵轴面积占比；虚线是海平面
        const float hx = kPx, hw = float(kViewW);
        DrawRectangleLinesEx({hx, kHistY, hw, kHistH}, hl, pal.textMuted);
        float peak = 1e-9f;
        for (float c : hist_) peak = std::max(peak, c);
        for (int b = 0; b < kBins; ++b) {
            float bh = hist_[b] / peak * (kHistH - 6), bx = hx + b * hw / kBins;
            bool water = (b + 0.5f) / kBins < sea_;
            DrawRectangleRec({bx + 0.5f, kHistY + kHistH - bh, hw / kBins - 1, bh}, water ? Color{90, 150, 200, 255} : pal.textMuted);
        }
        float sx = hx + sea_ * hw;
        drawDashedLine({sx, kHistY}, {sx, kHistY + kHistH}, 1.5f, pal.path, 4, 3);
        drawText("height histogram  0 -> 1", {hx, kHistY - 15}, 10, pal.textMuted);
        char buf[128];
        std::snprintf(buf, sizeof buf, "land %.0f%%   above 0.7: %.1f%%   %.1f ms", land_ * 100, high_ * 100, ms_);
        drawText(buf, {hx, kHistY + kHistH + 6}, 11, pal.text);
        DrawLineEx({0, kTop}, {float(kWidth), kTop}, hl, pal.gridLine);
    }

    void drawUi() override {
        const char* names[] = {"fbm", "ridged", "damped", "warped"};
        for (int k = 0; k < 4; ++k)
            if (ui::button({12.0f + k * 70, 10, 66, 26}, names[k], type_ == k)) {
                type_ = k, dirty_ = true;
                strength_ = k == 2 ? 1.0f : 1.5f;
            }
        if (type_ >= 2 && ui::slider({300, 10, 200, 26}, type_ == 2 ? "k" : "warp", &strength_, 0, type_ == 2 ? 4.0f : 3.0f, "%.2f", 0.05f))
            dirty_ = true;
        if (ui::slider({512, 10, 150, 26}, "seed", &seed_, 1, 20, "%.0f", 1)) dirty_ = true;
        if (ui::toggle({672, 10, 116, 26}, "biomes", &biomes_)) dirty_ = true;
        if (ui::slider({12, 44, 220, 26}, "exponent", &exponent_, 0.5f, 3.0f, "%.2f", 0.05f)) dirty_ = true;
        if (ui::slider({244, 44, 200, 26}, "sea", &sea_, 0.0f, 0.7f, "%.2f", 0.01f)) dirty_ = true;
        if (ui::toggle({456, 44, 76, 26}, "island", &island_)) dirty_ = true;
        if (island_ && ui::slider({540, 44, 160, 26}, "mix", &mix_, 0.0f, 1.0f, "%.2f", 0.05f)) dirty_ = true;
    }

    std::string status() const override {
        const char* names[] = {"fbm", "ridged", "damped", "warped"};
        char buf[128];
        std::snprintf(buf, sizeof buf, "%s · exponent %.2f · sea %.2f%s · land %.0f%%", names[type_], exponent_, sea_, island_ ? " · island" : "",
                      land_ * 100);
        return buf;
    }

private:
    void render() {
        dirty_ = false;
        double t0 = GetTime();
        HashLattice lat{uint32_t(seed_)}, mlat{uint32_t(seed_) + 1000};
        FbmParams p;
        p.octaves = 7;
        auto base = [&](float x, float y) { return perlinNoise(lat, x, y); };
        auto baseD = [&](float x, float y) { return perlinNoiseD(lat, x, y); };
        auto f = [&](float x, float y) { return fbm(base, x, y, p); };
        Heightmap hm(kMap, kMap);
        for (int y = 0; y < kMap; ++y)
            for (int x = 0; x < kMap; ++x) {
                float u = (x + 0.5f) * kCells / kMap, v = (y + 0.5f) * kCells / kMap, e = 0;
                if (type_ == 0) e = 0.5f + f(u, v);
                if (type_ == 1) e = ridged(base, u, v, p);
                if (type_ == 2) e = 0.5f + dampedFbm(baseD, u, v, p, strength_);
                if (type_ == 3) e = 0.5f + warped(f, u, v, strength_);
                if (island_) e = island(e, (x + 0.5f) / kMap * 2 - 1, (y + 0.5f) / kMap * 2 - 1, mix_);
                hm.at(x, y) = redistribute(e, exponent_);
            }
        // 直方图与统计
        for (float& c : hist_) c = 0;
        int land = 0, high = 0;
        for (float e : hm.v) {
            hist_[std::clamp(int(e * kBins), 0, kBins - 1)] += 1;
            land += e >= sea_, high += e > 0.7f;
        }
        land_ = float(land) / hm.v.size(), high_ = float(high) / hm.v.size();
        // 着色
        std::vector<float> shade = view::hillshade(hm, 60);
        std::vector<Color> px(hm.v.size());
        FbmParams mp;
        mp.octaves = 4;
        for (int y = 0; y < kMap; ++y)
            for (int x = 0; x < kMap; ++x) {
                size_t i = size_t(y) * kMap + x;
                float e = hm.v[i];
                if (e < sea_) {
                    px[i] = view::waterColor(sea_ > 0 ? (sea_ - e) / sea_ : 0);
                    continue;
                }
                float t = sea_ < 1 ? (e - sea_) / (1 - sea_) : 0;
                Color c;
                if (biomes_) {
                    float u = (x + 0.5f) * kCells / kMap, v = (y + 0.5f) * kCells / kMap;
                    float m = std::clamp(0.5f + 1.2f * fbm([&](float a, float b) { return perlinNoise(mlat, a, b); }, u, v, mp), 0.0f, 1.0f);
                    c = biomeColor(t, m);
                } else {
                    c = view::landColor(t);
                }
                px[i] = view::scale(c, shade[i]);
            }
        UpdateTexture(map_, px.data());
        std::vector<Color> ob;
        view::renderOblique(hm, px, sea_, ob, kViewW, kViewH, 90);
        UpdateTexture(view_, ob.data());
        ms_ = float((GetTime() - t0) * 1000);
    }

    int type_ = 0;
    float exponent_ = 1.0f, sea_ = 0.4f, seed_ = 3, strength_ = 1.0f, mix_ = 0.5f;
    bool island_ = false, biomes_ = false, ready_ = false, dirty_ = true;
    Texture2D map_{}, view_{};
    float hist_[kBins]{};
    float land_ = 0, high_ = 0, ms_ = 0;
};

}  // namespace

int main() {
    static TerrainDemo demo;
    ga::DemoConfig config;
    config.title = "Terrain from noise";
    config.width = kWidth;
    config.height = kHeight;
    config.stepsPerSecond = 30.0f;
    return ga::runDemo(demo, config);
}
