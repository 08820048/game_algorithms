// 洞穴的连通处理与填充率的"相变"
//   - fill：随机填充时墙的概率，之后做 5 轮 4-5 平滑
//   - raw：不处理，最大的一块是纸色，其余每块用赭黄标出
//   - largest：只保留最大的一块；connect：用最短的隧道（红色）把每一块连到主区域上
//   - radius：颜色表示离最近的墙的距离（张开半径），越深越开阔
//   - 右下：fill 从 0.35 到 0.60，最大一块占全部地面的比例（每个点 24 个种子），竖线是当前的 fill

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "cave.h"
#include "demo.h"
#include "ui.h"

namespace {

using namespace ga;
using namespace ga::cave;

constexpr int kWidth = 800, kHeight = 480;
constexpr float kTop = 80;
constexpr int kW = 96, kH = 60;
constexpr float kCell = 6;
constexpr float kMx = 12, kMy = kTop + 12;
constexpr float kPx = 606, kPw = 182;

class CavesDemo final : public Demo {
public:
    void reset() override {}
    void update() override {
        if (curve_.empty()) buildCurve();
        if (dirty_) generate();
    }
    bool step() override { return false; }

    void draw() const override {
        const Palette& pal = palette();
        const float hl = hairline();
        DrawRectangleRec({kMx, kMy, kW * kCell, kH * kCell}, pal.wall);
        int maxR = 1, big = regions_.size.empty() ? -1 : regions_.largest();
        for (int d : radius_) maxR = std::max(maxR, d);
        for (int y = 0; y < kH; ++y)
            for (int x = 0; x < kW; ++x) {
                int i = y * kW + x;
                if (map_.t[i] == Wall) continue;
                Color c = pal.background;
                if (map_.t[i] == Corridor) c = pal.path;
                else if (showRadius_) c = ColorLerp(pal.background, pal.frontier, std::min(1.0f, (radius_[i] - 1) / 6.0f));
                else if (regions_.label[i] != big) c = pal.frontier;
                DrawRectangleRec({kMx + x * kCell, kMy + y * kCell, kCell, kCell}, c);
            }

        char buf[128];
        float y = kMy;
        auto line = [&](const char* s, Color c, float size = 12) {
            drawText(s, {kPx, y}, size, c);
            y += size + 6;
        };
        std::snprintf(buf, sizeof buf, "regions before %d", regionsBefore_);
        line(buf, pal.text);
        std::snprintf(buf, sizeof buf, "largest %.1f%% of floor", largestPct_);
        line(buf, pal.text);
        if (mode_ == 1) std::snprintf(buf, sizeof buf, "floor removed %d cells", removed_);
        else if (mode_ == 2) std::snprintf(buf, sizeof buf, "tunnel cells %d", tunnel_);
        else std::snprintf(buf, sizeof buf, "floor %.1f%%", floorPct_);
        line(buf, pal.text);
        std::snprintf(buf, sizeof buf, "dead ends %d", dead_);
        line(buf, pal.text);
        if (mode_ > 0) {
            std::snprintf(buf, sizeof buf, "detour ratio %.2f", detour_);
            line(buf, pal.text);
        }
        std::snprintf(buf, sizeof buf, "max open radius %d", maxR);
        line(buf, pal.text);

        // 曲线：最大一块占地面的比例
        float cx = kPx, cy = kMy + 190, cw = kPw - 6, ch = 130;
        drawText("largest region / floor", {cx, cy - 18}, 11, pal.textMuted);
        DrawRectangleLinesEx({cx, cy, cw, ch}, hl, pal.gridLine);
        for (float f : {0.25f, 0.5f, 0.75f}) DrawLineEx({cx, cy + ch * (1 - f)}, {cx + cw, cy + ch * (1 - f)}, hl, Fade(pal.gridLine, 0.6f));
        auto px = [&](int k) { return Vector2{cx + cw * k / float(curve_.size() - 1), cy + ch * (1 - curve_[k])}; };
        for (int k = 0; k + 1 < int(curve_.size()); ++k) DrawLineEx(px(k), px(k + 1), 1.8f, pal.text);
        float fx = cx + cw * (fill_ - 0.35f) / 0.25f;
        DrawLineEx({fx, cy}, {fx, cy + ch}, 1.5f, pal.path);
        drawText("0.35", {cx, cy + ch + 3}, 9, pal.textMuted);
        drawText("0.60", {cx + cw - 20, cy + ch + 3}, 9, pal.textMuted);
        drawText("fill", {cx + cw / 2 - 8, cy + ch + 3}, 9, pal.textMuted);
        drawText("100%", {cx + 3, cy + 2}, 9, pal.textMuted);
        DrawLineEx({0, kTop}, {float(kWidth), kTop}, hl, pal.gridLine);
    }

    void drawUi() override {
        bool ch = false;
        ch |= ui::slider({12, 10, 150, 26}, "seed", &seed_, 1, 50, "%.0f", 1);
        ch |= ui::slider({172, 10, 220, 26}, "fill", &fill_, 0.35f, 0.60f, "%.2f", 0.01f);
        const char* names[] = {"raw", "largest", "connect"};
        for (int k = 0; k < 3; ++k)
            if (ui::button({12.0f + k * 82, 44, 78, 26}, names[k], mode_ == k)) mode_ = k, ch = true;
        ui::toggle({262, 44, 92, 26}, "radius", &showRadius_);
        if (ch) dirty_ = true;
    }

    std::string status() const override {
        const char* names[] = {"raw", "largest", "connect"};
        char buf[128];
        std::snprintf(buf, sizeof buf, "fill %.2f · %d regions · largest %.0f%% · %s", fill_, regionsBefore_, largestPct_, names[mode_]);
        return buf;
    }

private:
    static Map makeCave(int w, int h, uint64_t seed, float p) {
        Map m(w, h);
        Pcg32 rng{seed};
        randomFill(m, rng, p);
        for (int i = 0; i < 5; ++i) cave::step(m, Rule{});
        return m;
    }

    void buildCurve() {
        for (int k = 0; k <= 25; ++k) {
            float p = 0.35f + 0.01f * k, sum = 0;
            for (int s = 1; s <= 24; ++s) {
                Map m = makeCave(kW, kH, s, p);
                Regions r = labelRegions(m);
                sum += r.size.empty() ? 0 : float(r.size[r.largest()]) / m.floorCount();
            }
            curve_.push_back(sum / 24);
        }
    }

    void generate() {
        dirty_ = false;
        map_ = makeCave(kW, kH, uint64_t(seed_), fill_);
        Regions before = labelRegions(map_);
        regionsBefore_ = int(before.size.size());
        largestPct_ = before.size.empty() ? 0 : 100.0f * before.size[before.largest()] / std::max(1, map_.floorCount());
        removed_ = tunnel_ = 0;
        if (mode_ == 1) removed_ = keepLargest(map_);
        if (mode_ == 2) tunnel_ = connectRegions(map_);
        regions_ = labelRegions(map_);
        radius_ = openRadius(map_);
        floorPct_ = 100.0f * map_.floorCount() / ((kW - 2) * (kH - 2));
        dead_ = dungeon::deadEnds(map_);
        Pcg32 r2(1);
        detour_ = map_.floorCount() > 20 ? float(dungeon::detourRatio(map_, r2, 30, 30, 10)) : 0;
    }

    float seed_ = 5, fill_ = 0.48f;
    int mode_ = 0;
    bool dirty_ = true, showRadius_ = false;
    Map map_{kW, kH};
    Regions regions_;
    std::vector<int> radius_;
    std::vector<float> curve_;
    int regionsBefore_ = 0, removed_ = 0, tunnel_ = 0, dead_ = 0;
    float largestPct_ = 0, floorPct_ = 0, detour_ = 0;
};

}  // namespace

int main() {
    static CavesDemo demo;
    ga::DemoConfig config;
    config.title = "Cave regions";
    config.width = kWidth;
    config.height = kHeight;
    config.autoPlay = false;
    return ga::runDemo(demo, config);
}
