// 侵蚀
//   - hydraulic：每一步放下 drops/step 个水滴（位置随机）。floor 关掉后，冲刷可以把格子挖到比水滴下一步更低——
//     在地图边缘很快会挖出无底的沟
//   - thermal：每一步做 iters/step 轮热侵蚀，talus 是允许的最大坡度（高度 / 格）。4 / 8 / 8√2 是邻域：
//     只看上下左右、加上对角但对角距离按 1 算、对角距离按 √2 算
//   - pile：平地中间一根柱子。用 thermal 塌成沙堆，虚线圆的半径等于沙堆沿 x 轴的半径——沙堆不圆，说明有方向性
//   - shaded：高度着色 + 光照；change：红色是被冲走的、蓝色是堆积的；rivers：D8 汇流的上游面积（河网）

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
constexpr double kMaxDropsPerCell = 4;
constexpr int kMaxThermal = 6000;

class ErosionDemo final : public Demo {
public:
    ErosionDemo() { build(); }

    void reset() override {
        hm_ = h0_;
        drops_ = 0, iters_ = 0, lastMoved_ = 0, stepCount_ = 0;
        stats_ = DropletStats{};
        rng_ = Pcg32(11);
        flowDirty_ = true, dirty_ = true;
    }

    // 到达上限后什么都不做（仍返回 true）：这样切换模式时框架只需重置，而不会"立即重新跑完"
    bool step() override {
        if (hydraulic_) {
            if (drops_ >= kMaxDropsPerCell * kMap * kMap) return true;
            DropletParams p;
            p.floorAtNext = floor_;
            int n = int(std::lround(dropsPerStep_));
            for (int k = 0; k < n; ++k)
                simulateDroplet(hm_, float(rng_.nextDouble()) * (kMap - 1.001f), float(rng_.nextDouble()) * (kMap - 1.001f), p, stats_);
            drops_ += n;
        } else {
            if (iters_ >= kMaxThermal || (iters_ > 0 && lastMoved_ < 1e-5)) return true;
            for (int k = 0; k < int(std::lround(itersPerStep_)); ++k, ++iters_)
                lastMoved_ = thermalStep(hm_, delta_, talus_, Neighborhood(nb_));
        }
        ++stepCount_;
        dirty_ = true;
        if (view_ == 2 || stepCount_ % 8 == 0) flowDirty_ = true;
        return true;
    }

    void update() override {
        if (!ready_) {
            Image img = GenImageColor(kMap, kMap, BLACK);
            map_ = LoadTextureFromImage(img);
            UnloadImage(img);
            SetTextureFilter(map_, TEXTURE_FILTER_BILINEAR);
            Image v = GenImageColor(kViewW, kViewH, BLANK);
            obl_ = LoadTextureFromImage(v);
            UnloadImage(v);
            ready_ = true, dirty_ = true;
        }
        if (flowDirty_) {
            flow_ = flow(hm_);
            flowDirty_ = false, dirty_ = true;
        }
        if (dirty_) render();
    }

    void draw() const override {
        const Palette& pal = palette();
        const float hl = hairline();
        if (ready_) {
            DrawTexturePro(map_, {0, 0, float(kMap), float(kMap)}, {kIx, kIy, float(kImg), float(kImg)}, {0, 0}, 0, WHITE);
            DrawTexture(obl_, int(kPx), int(kViewY), WHITE);
        }
        DrawRectangleLinesEx({kIx, kIy, float(kImg), float(kImg)}, hl * 1.5f, pal.textMuted);
        if (pile_) {  // 以沙堆沿 +x 方向的半径画圆
            int c = kMap / 2, r = 0;
            for (int k = 1; k < c; ++k)
                if (hm_.at(c + k, c) > kPileBase + 0.004f) r = k;
            float s = float(kImg) / kMap;
            Vector2 ctr{kIx + (c + 0.5f) * s, kIy + (c + 0.5f) * s};
            const int seg = 96;
            for (int i = 0; i < seg; i += 2) {
                float a0 = 2 * PI * i / seg, a1 = 2 * PI * (i + 1) / seg;
                DrawLineEx({ctr.x + std::cos(a0) * r * s, ctr.y + std::sin(a0) * r * s}, {ctr.x + std::cos(a1) * r * s, ctr.y + std::sin(a1) * r * s},
                           1.5f, pal.path);
            }
        }
        drawText("oblique view from the south", {kPx, kViewY - 2}, 10, pal.textMuted);

        char buf[160];
        float y = kViewY + kViewH + 14;
        auto line = [&](const char* s, Color c) {
            drawText(s, {kPx, y}, 12, c);
            y += 19;
        };
        if (hydraulic_) {
            std::snprintf(buf, sizeof buf, "droplets %d  (%.2f per cell)", int(drops_), drops_ / (kMap * kMap));
            line(buf, pal.text);
            std::snprintf(buf, sizeof buf, "eroded %.1f  deposited %.1f  carried off %.1f", stats_.eroded, stats_.deposited, stats_.carriedOff);
            line(buf, pal.textMuted);
        } else {
            std::snprintf(buf, sizeof buf, "iterations %d  moved last iteration %.4f", iters_, lastMoved_);
            line(buf, pal.text);
            line(lastMoved_ < 1e-5 && iters_ > 0 ? "converged: no slope above talus" : " ", pal.textMuted);
        }
        std::snprintf(buf, sizeof buf, "pits %d   drains off the map %.0f%%", flow_.pits, flow_.drainedOut * 100);
        line(buf, pal.text);
        std::snprintf(buf, sizeof buf, "lowest %.3f  highest %.3f", lo_, hi_);
        line(buf, lo_ < -0.5f ? pal.path : pal.textMuted);
        if (view_ == 1) line("red: removed   blue: deposited", pal.textMuted);
        if (view_ == 2) line("blue: upstream area > 40 cells (log scale)", pal.textMuted);
        DrawLineEx({0, kTop}, {float(kWidth), kTop}, hl, pal.gridLine);
    }

    void drawUi() override {
        if (ui::button({12, 10, 90, 26}, "hydraulic", hydraulic_)) hydraulic_ = true, changed();
        if (ui::button({106, 10, 80, 26}, "thermal", !hydraulic_)) hydraulic_ = false, changed();
        if (ui::button({200, 10, 76, 26}, "terrain", !pile_)) pile_ = false, build(), changed();
        if (ui::button({280, 10, 56, 26}, "pile", pile_)) pile_ = true, build(), changed();
        if (hydraulic_) {
            if (ui::toggle({350, 10, 70, 26}, "floor", &floor_)) changed();
        } else {
            const char* nbs[] = {"4", "8", "8 scaled"};
            const float wd[] = {34, 34, 84};
            float x = 350;
            for (int k = 0; k < 3; ++k) {
                if (ui::button({x, 10, wd[k], 26}, nbs[k], nb_ == k)) nb_ = k, changed();
                x += wd[k] + 4;
            }
        }
        if (ui::slider({536, 10, 180, 26}, "seed", &seed_, 1, 20, "%.0f", 1)) build(), changed();
        if (hydraulic_) {
            ui::slider({12, 44, 260, 26}, "drops/step", &dropsPerStep_, 256, 8192, "%.0f", 256);
        } else {
            ui::slider({12, 44, 200, 26}, "iters/step", &itersPerStep_, 1, 40, "%.0f", 1);
            if (ui::slider({222, 44, 200, 26}, "talus", &talus_, 0.004f, 0.03f, "%.3f", 0.001f)) changed();
        }
        const char* views[] = {"shaded", "change", "rivers"};
        for (int k = 0; k < 3; ++k)
            if (ui::button({536.0f + k * 84, 44, 80, 26}, views[k], view_ == k)) view_ = k, flowDirty_ = true, dirty_ = true;
    }

    std::string status() const override {
        char buf[128];
        if (hydraulic_)
            std::snprintf(buf, sizeof buf, "hydraulic · %.2f drops/cell · pits %d%s", drops_ / (kMap * kMap), flow_.pits, floor_ ? "" : " · no floor");
        else
            std::snprintf(buf, sizeof buf, "thermal (%s) · talus %.3f · %d iterations", nb_ == 0 ? "4" : nb_ == 1 ? "8" : "8 scaled", talus_, iters_);
        return buf;
    }

private:
    static constexpr float kPileBase = 0.1f;

    void changed() { notifyProblemChanged(); }

    void build() {
        h0_ = Heightmap(kMap, kMap);
        if (pile_) {  // 平地 + 中间一根半径 4、高 20 的柱子
            for (float& v : h0_.v) v = kPileBase;
            for (int y = -4; y <= 4; ++y)
                for (int x = -4; x <= 4; ++x)
                    if (x * x + y * y <= 16) h0_.at(kMap / 2 + x, kMap / 2 + y) = 20;
        } else {
            HashLattice lat{uint32_t(seed_)};
            FbmParams p;
            p.octaves = 6;
            for (int y = 0; y < kMap; ++y)
                for (int x = 0; x < kMap; ++x)
                    h0_.at(x, y) = 0.5f + fbm([&](float u, float v) { return perlinNoise(lat, u, v); }, x * 3.0f / kMap, y * 3.0f / kMap, p);
        }
        reset();
    }

    void render() {
        dirty_ = false;
        lo_ = 1e30f, hi_ = -1e30f;
        for (float v : hm_.v) lo_ = std::min(lo_, v), hi_ = std::max(hi_, v);
        std::vector<float> shade = view::hillshade(hm_, 60);
        std::vector<Color> px(hm_.v.size());
        float amax = 1;
        if (view_ == 2)
            for (float a : flow_.area) amax = std::max(amax, a);
        for (size_t i = 0; i < px.size(); ++i) {
            float e = hm_.v[i];
            if (view_ == 0) {
                px[i] = view::scale(view::landColor(std::clamp((e - 0.1f) / 0.8f, 0.0f, 1.0f)), shade[i]);
            } else if (view_ == 1) {
                float d = e - h0_.v[i], g = std::clamp(0.55f * shade[i], 0.0f, 1.0f);
                Color base{(unsigned char)(g * 235), (unsigned char)(g * 235), (unsigned char)(g * 235), 255};
                float t = std::sqrt(std::min(1.0f, std::abs(d) / 0.2f));  // 变化 0.2 以上饱和；开方让小的变化也看得见
                px[i] = view::lerpColor(base, d < 0 ? Color{220, 60, 40, 255} : Color{50, 110, 220, 255}, t);
            } else {
                Color c = view::scale(view::landColor(std::clamp((e - 0.1f) / 0.8f, 0.0f, 1.0f)), shade[i]);
                float a = flow_.area.empty() ? 0 : flow_.area[i];
                if (a > 40) c = view::lerpColor(c, Color{30, 90, 210, 255}, 0.35f + 0.65f * std::log(a / 40) / std::log(amax / 40 + 1e-6f));
                px[i] = c;
            }
        }
        UpdateTexture(map_, px.data());
        std::vector<Color> ob;
        view::renderOblique(hm_, px, -1e30f, ob, kViewW, kViewH, 90);
        UpdateTexture(obl_, ob.data());
    }

    bool hydraulic_ = true, pile_ = false, floor_ = true, ready_ = false, dirty_ = true, flowDirty_ = true;
    int nb_ = 2, view_ = 0, iters_ = 0, stepCount_ = 0;
    float seed_ = 3, dropsPerStep_ = 512, itersPerStep_ = 10, talus_ = 0.012f;
    double drops_ = 0, lastMoved_ = 0;
    float lo_ = 0, hi_ = 0;
    Heightmap h0_, hm_;
    std::vector<float> delta_;
    DropletStats stats_;
    FlowResult flow_;
    Pcg32 rng_{11};
    Texture2D map_{}, obl_{};
};

}  // namespace

int main() {
    static ErosionDemo demo;
    ga::DemoConfig config;
    config.title = "Erosion";
    config.width = kWidth;
    config.height = kHeight;
    config.stepsPerSecond = 30.0f;
    return ga::runDemo(demo, config);
}
