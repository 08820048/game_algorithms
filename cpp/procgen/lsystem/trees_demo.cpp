// L-System：随机的树
//   - 五棵树用同一套随机规则、不同的种子。每个芽 X 长出一段树干，再分出一到两根更细更短的枝
//   - 按"从根部沿树枝走过的路程"显示，看起来像在生长；叶子（树枝末端）长到时出现
//   - jitter：每次转向加一点随机角度；gravity：树枝朝下弯（负值朝上）

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "demo.h"
#include "lsystem.h"
#include "presets.h"
#include "ui.h"

namespace {

using namespace ga;
using namespace ga::lsys;

constexpr int kWidth = 800, kHeight = 480;
constexpr float kTop = 80;
constexpr int kTrees = 5;
constexpr float kGround = kHeight - 28;

class TreesDemo final : public Demo {
public:
    TreesDemo() : sys_(treeSystem()) { rebuild(); }
    void reset() override { t_ = 0; }
    bool step() override {
        t_ += maxDist_ / 120;
        return t_ < maxDist_;
    }

    void draw() const override {
        const Palette& pal = palette();
        const float hl = hairline();
        DrawLineEx({0, kGround}, {float(kWidth), kGround}, hl * 2, pal.gridLine);
        Color bark = ColorLerp(pal.text, Color{120, 80, 50, 255}, 0.45f);
        Color leaf = ColorLerp(pal.background, Color{80, 160, 70, 255}, 0.85f);
        for (int k = 0; k < kTrees; ++k) {
            const Tree& tr = trees_[size_t(k)];
            for (const Segment& s : tr.segs) {
                if (s.dist >= t_) continue;
                float len = std::hypot(s.x1 - s.x0, s.y1 - s.y0);
                float f = len > 0 ? std::min(1.0f, (t_ - s.dist) / len) : 1;
                Vector2 a = map(k, s.x0, s.y0), b = map(k, s.x0 + (s.x1 - s.x0) * f, s.y0 + (s.y1 - s.y0) * f);
                float w = std::max(hl, s.width * trunk_);
                DrawLineEx(a, b, w, bark);
                if (w > 2.5f) DrawCircleV(b, w / 2, bark);
            }
            for (const Leaf& l : tr.leaves)
                if (l.dist <= t_) DrawCircleV(map(k, l.x, l.y), leafR_, Fade(leaf, 0.8f));
        }
        char buf[96];
        long long segs = 0, leaves = 0;
        for (const Tree& tr : trees_) segs += (long long)tr.segs.size(), leaves += (long long)tr.leaves.size();
        std::snprintf(buf, sizeof buf, "segments %lld  leaves %lld", segs, leaves);
        drawText(buf, {12, kTop + 8}, 11, pal.textMuted);
        DrawLineEx({0, kTop}, {float(kWidth), kTop}, hl, pal.gridLine);
    }

    void drawUi() override {
        bool ch = false;
        ch |= ui::slider({12, 10, 180, 26}, "generation", &gen_, 2, 8, "%.0f", 1);
        ch |= ui::slider({204, 10, 180, 26}, "seed", &seed_, 1, 50, "%.0f", 1);
        ch |= ui::slider({396, 10, 180, 26}, "angle", &angle_, 5, 60, "%.0f", 1);
        ch |= ui::slider({12, 44, 180, 26}, "jitter", &jitter_, 0, 30, "%.0f", 1);
        ch |= ui::slider({204, 44, 180, 26}, "gravity", &gravity_, -0.2f, 0.2f, "%.2f", 0.01f);
        ch |= ui::slider({396, 44, 180, 26}, "shrink", &shrink_, 0.5f, 1.0f, "%.2f", 0.01f);
        if (ch) rebuild(), notifyProblemChanged();
    }

    std::string status() const override {
        char buf[96];
        std::snprintf(buf, sizeof buf, "n %d · seeds %d-%d", int(gen_), int(seed_) * kTrees, int(seed_) * kTrees + kTrees - 1);
        return buf;
    }

private:
    struct Tree {
        std::vector<Segment> segs;
        std::vector<Leaf> leaves;
        float minX = 0, maxX = 0;
    };
    void rebuild() {
        float maxH = 1e-3f;
        maxDist_ = 1e-3f;
        for (int k = 0; k < kTrees; ++k) {
            Tree& tr = trees_[size_t(k)];
            tr.segs.clear(), tr.leaves.clear();
            TurtleOptions o;
            o.angle = angle_, o.heading = 90, o.lengthScale = shrink_, o.widthScale = 0.7;
            o.jitter = jitter_, o.tropism = gravity_;
            uint64_t seed = uint64_t(seed_) * kTrees + uint64_t(k);
            o.seed = seed;
            Turtle<double> t(o, &tr.segs, &tr.leaves);
            stream(sys_, int(gen_), seed, [&](char c) { t.feed(c); });
            t.finish();
            tr.minX = t.minX(), tr.maxX = t.maxX();
            maxH = std::max(maxH, t.maxY());
            for (const Segment& s : tr.segs) maxDist_ = std::max(maxDist_, s.dist + 1.5f);
        }
        // 所有树用同一个比例：最高的那棵占满画布高度
        scale_ = (kGround - kTop - 30) / maxH;
        trunk_ = std::clamp(scale_ * 0.5f, 1.5f, 10.0f);
        leafR_ = std::clamp(scale_ * 0.35f, 1.2f, 4.0f);
        reset();
    }
    Vector2 map(int k, float x, float y) const {
        float cx = kWidth * (k + 0.5f) / kTrees;
        return {cx + x * scale_, kGround - y * scale_};
    }

    LSystem sys_;
    Tree trees_[kTrees];
    float gen_ = 6, seed_ = 1, angle_ = 24, jitter_ = 6, gravity_ = 0.03f, shrink_ = 0.75f;
    float scale_ = 1, trunk_ = 4, leafR_ = 2, maxDist_ = 1, t_ = 0;
};

}  // namespace

int main() {
    static TreesDemo demo;
    ga::DemoConfig config;
    config.title = "L-system: stochastic trees";
    config.width = kWidth;
    config.height = kHeight;
    config.stepsPerSecond = 40.0f;
    return ga::runDemo(demo, config);
}
