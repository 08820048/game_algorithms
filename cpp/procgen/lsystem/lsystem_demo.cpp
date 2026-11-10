// L-System：改写与海龟
//   - 选一个系统、代数和转角。右边是公理、规则和展开后的串的开头
//   - 海龟按串的顺序画线：遇到 [ 记住位置，遇到 ] 跳回去。红点是海龟当前的位置
//   - 曲线类（koch dragon hilbert sierpinski）的转角是 60° 或 90°，改动转角后曲线不再闭合、开始自交

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
constexpr Rectangle kArea{12, kTop + 10, 540, kHeight - kTop - 22};
constexpr float kPx = 572;

class LSystemDemo final : public Demo {
public:
    LSystemDemo() : presets_(presets()) { select(4); }
    void reset() override { shown_ = 0; }
    bool step() override {
        size_t chunk = std::max<size_t>(1, segs_.size() / 150);
        shown_ = std::min(segs_.size(), shown_ + chunk);
        return shown_ < segs_.size();
    }

    void draw() const override {
        const Palette& pal = palette();
        const float hl = hairline();
        const Preset& p = presets_[size_t(which_)];
        bool plant = p.lattice == 0;
        float w = segs_.size() > 20000 ? hl : segs_.size() > 3000 ? 1.0f : plant ? 1.6f : 1.5f;
        for (size_t i = 0; i < shown_; ++i) {
            const Segment& s = segs_[i];
            Color c = plant ? ColorLerp(pal.text, Color{90, 150, 70, 255}, std::min(1.0f, s.depth / 5.0f))
                            : ColorLerp(pal.text, pal.path, float(i) / float(segs_.size()));
            DrawLineEx(map(s.x0, s.y0), map(s.x1, s.y1), w, c);
        }
        if (shown_ > 0 && shown_ < segs_.size()) DrawCircleV(map(segs_[shown_ - 1].x1, segs_[shown_ - 1].y1), 4, pal.path);
        DrawLineEx({kPx - 10, kTop}, {kPx - 10, float(kHeight)}, hl, pal.gridLine);

        char buf[160];
        float y = kTop + 12;
        auto line = [&](const char* str, Color col, float size = 12) {
            drawText(str, {kPx, y}, size, col);
            y += size + 6;
        };
        std::snprintf(buf, sizeof buf, "axiom  %s", p.sys.axiom.c_str());
        line(buf, pal.text);
        for (int c = 0; c < 128; ++c)
            for (const Production& r : p.sys.rules[size_t(c)]) {
                if (p.sys.rules[size_t(c)].size() > 1) std::snprintf(buf, sizeof buf, "%c -> %s  (1/%zu)", char(c), r.rhs.c_str(), p.sys.rules[size_t(c)].size());
                else std::snprintf(buf, sizeof buf, "%c -> %s", char(c), r.rhs.c_str());
                line(buf, pal.text);
            }
        y += 6;
        std::snprintf(buf, sizeof buf, "angle %.1f", angle_);
        line(buf, pal.textMuted);
        y += 6;
        std::snprintf(buf, sizeof buf, "generation %d", int(gen_));
        line(buf, pal.text, 13);
        std::snprintf(buf, sizeof buf, "length %zu", length_);
        line(buf, pal.text);
        std::snprintf(buf, sizeof buf, "segments %zu", segs_.size());
        line(buf, pal.text);
        std::snprintf(buf, sizeof buf, "max stack %d", maxStack_);
        line(buf, pal.text);
        y += 6;
        line("string:", pal.textMuted, 11);
        for (size_t k = 0; k < 4 && k * 30 < prefix_.size(); ++k) line(prefix_.substr(k * 30, 30).c_str(), pal.textMuted, 11);
        if (length_ > prefix_.size()) line("...", pal.textMuted, 11);
        DrawLineEx({0, kTop}, {float(kWidth), kTop}, hl, pal.gridLine);
    }

    void drawUi() override {
        float x = 12;
        for (int i = 0; i < int(presets_.size()); ++i) {
            float w = std::string(presets_[size_t(i)].name).size() * 8.0f + 18;
            if (ui::button({x, 10, w, 26}, presets_[size_t(i)].name, which_ == i)) select(i), notifyProblemChanged();
            x += w + 4;
        }
        const Preset& p = presets_[size_t(which_)];
        bool ch = false;
        ch |= ui::slider({12, 44, 200, 26}, "generation", &gen_, 0, float(p.maxGen), "%.0f", 1);
        ch |= ui::slider({224, 44, 220, 26}, "angle", &angle_, 5, 120, "%.1f", 0.5f);
        if (ui::button({456, 44, 70, 26}, "default")) angle_ = float(p.turtle.angle), ch = true;
        if (p.sys.stochastic()) ch |= ui::slider({538, 44, 150, 26}, "seed", &seed_, 1, 50, "%.0f", 1);
        if (ch) rebuild(), notifyProblemChanged();
    }

    std::string status() const override {
        char buf[96];
        std::snprintf(buf, sizeof buf, "%s · n %d · %zu symbols · %zu segments", presets_[size_t(which_)].name, int(gen_), length_, segs_.size());
        return buf;
    }

private:
    void select(int i) {
        which_ = i;
        gen_ = float(presets_[size_t(i)].gen);
        angle_ = float(presets_[size_t(i)].turtle.angle);
        rebuild();
    }
    void rebuild() {
        const Preset& p = presets_[size_t(which_)];
        TurtleOptions o = p.turtle;
        o.angle = angle_;
        segs_.clear();
        Turtle<double> t(o, &segs_);
        prefix_.clear();
        length_ = 0;
        stream(p.sys, int(gen_), uint64_t(seed_), [&](char c) {
            if (prefix_.size() < 120) prefix_ += c;
            ++length_;
            t.feed(c);
        });
        maxStack_ = t.maxStack();
        // 按包围盒缩放到画布里，y 轴朝上
        float bw = std::max(1e-3f, t.maxX() - t.minX()), bh = std::max(1e-3f, t.maxY() - t.minY());
        if (segs_.empty()) bw = bh = 1;
        scale_ = std::min(kArea.width / bw, kArea.height / bh) * 0.94f;
        ox_ = kArea.x + kArea.width / 2 - (t.minX() + bw / 2) * scale_;
        oy_ = kArea.y + kArea.height / 2 + (t.minY() + bh / 2) * scale_;
        if (segs_.empty()) ox_ = kArea.x + kArea.width / 2, oy_ = kArea.y + kArea.height / 2;
        reset();
    }
    Vector2 map(float x, float y) const { return {ox_ + x * scale_, oy_ - y * scale_}; }

    std::vector<Preset> presets_;
    int which_ = 4;
    float gen_ = 5, angle_ = 25.7f, seed_ = 1;
    std::vector<Segment> segs_;
    std::string prefix_;
    size_t length_ = 0, shown_ = 0;
    int maxStack_ = 0;
    float scale_ = 1, ox_ = 0, oy_ = 0;
};

}  // namespace

int main() {
    static LSystemDemo demo;
    ga::DemoConfig config;
    config.title = "L-system: rewriting and turtle";
    config.width = kWidth;
    config.height = kHeight;
    config.stepsPerSecond = 40.0f;
    return ga::runDemo(demo, config);
}
