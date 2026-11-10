// 元胞自动机：逐轮平滑
//   - 第 0 轮是随机填充，之后每一步是一轮平滑，直到出现不动点或周期 2（最多 80 轮）
//   - rule：4-5 是 3×3 多数表决；R2→4-5 是前 4 轮加上"5×5 里墙 ≤ 2 也变墙"再换回 4-5；R2 always 一直用 R2 规则，它永远不会停下
//   - in-place：原地更新（常见的错误写法），左边和上边的邻居已经是这一轮的新值
//   - changes：标出上一轮改变的格子（红色：变成墙，赭黄：变成地面）
//   - 右侧柱状图：每一轮改变的格子数（对数刻度）

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
constexpr int kMaxIter = 80;

class CaveDemo final : public Demo {
public:
    CaveDemo() { regenerate(); }

    void reset() override { regenerate(); }

    bool step() override {
        if (done_) return true;  // 返回 true：结束后改参数不会立刻重放所有步骤
        Rule r = ruleFor(iter_);
        prev_ = map_.t;
        int c = inPlace_ ? stepInPlace(map_, r) : cave::step(map_, r);
        ++iter_;
        changed_.push_back(c);
        if (map_.t == prev_) done_ = 1;
        else if (map_.t == prev2_ && !alwaysR2()) done_ = 2;
        else if (iter_ >= kMaxIter) done_ = 3;
        prev2_ = prev_;
        measure();
        return true;
    }

    void draw() const override {
        const Palette& pal = palette();
        const float hl = hairline();
        DrawRectangleRec({kMx, kMy, kW * kCell, kH * kCell}, pal.wall);
        for (int y = 0; y < kH; ++y)
            for (int x = 0; x < kW; ++x) {
                int i = y * kW + x;
                bool floor = map_.t[i] != Wall;
                bool was = !prev_.empty() && prev_[i] != Wall;
                Rectangle r{kMx + x * kCell, kMy + y * kCell, kCell, kCell};
                if (floor) DrawRectangleRec(r, pal.background);
                if (showChanges_ && iter_ > 0 && floor != was) DrawRectangleRec(r, floor ? pal.frontier : pal.path);
            }

        // 右侧：数字
        char buf[128];
        float y = kMy;
        auto line = [&](const char* s, Color c, float size = 12) {
            drawText(s, {kPx, y}, size, c);
            y += size + 6;
        };
        std::snprintf(buf, sizeof buf, "iteration %d", iter_);
        line(buf, pal.text, 13);
        std::snprintf(buf, sizeof buf, "changed %d", iter_ ? changed_.back() : 0);
        line(buf, pal.text);
        std::snprintf(buf, sizeof buf, "floor %.1f%%", floorPct_);
        line(buf, pal.text);
        std::snprintf(buf, sizeof buf, "regions %d  largest %.0f%%", regions_, largestPct_);
        line(buf, pal.text);
        const char* state[] = {"running", "fixed point", "2-cycle", "still changing"};
        line(state[done_], done_ == 3 ? pal.path : pal.textMuted, 11);

        // 柱状图：每轮改变的格子数，对数刻度 1 .. 10^4
        float cx = kPx + 26, cy = kMy + 150, cw = kPw - 26, ch = 150;
        DrawLineEx({cx, cy + ch}, {cx + cw, cy + ch}, hl, pal.gridLine);
        for (int e = 0; e <= 4; ++e) {
            float yy = cy + ch - ch * e / 4.0f;
            DrawLineEx({cx, yy}, {cx + cw, yy}, hl, Fade(pal.gridLine, 0.6f));
            const char* lab[] = {"1", "10", "100", "1k", "10k"};
            drawText(lab[e], {kPx, yy - 5}, 9, pal.textMuted);
        }
        drawText("changed cells per iteration", {kPx, cy - 22}, 11, pal.textMuted);
        int n = int(changed_.size());
        float bw = std::min(8.0f, cw / std::max(1, n));
        for (int i = 0; i < n; ++i) {
            if (changed_[i] <= 0) continue;
            float hgt = ch * std::min(1.0f, float(std::log10(changed_[i] + 1.0)) / 4.0f);
            DrawRectangleRec({cx + i * bw, cy + ch - hgt, std::max(1.0f, bw - 1), hgt}, i == n - 1 ? pal.path : pal.textMuted);
        }
        DrawLineEx({0, kTop}, {float(kWidth), kTop}, hl, pal.gridLine);
    }

    void drawUi() override {
        bool ch = false;
        ch |= ui::slider({12, 10, 150, 26}, "seed", &seed_, 1, 50, "%.0f", 1);
        ch |= ui::slider({172, 10, 200, 26}, "fill", &fill_, 0.35f, 0.60f, "%.2f", 0.01f);
        const char* names[] = {"4-5", "R2>4-5", "R2 always"};
        const float bx[] = {384, 440, 516}, bwid[] = {52, 72, 90};
        for (int k = 0; k < 3; ++k)
            if (ui::button({bx[k], 10, bwid[k], 26}, names[k], rule_ == k)) rule_ = k, ch = true;
        ch |= ui::toggle({12, 44, 100, 26}, "in-place", &inPlace_);
        ui::toggle({122, 44, 100, 26}, "changes", &showChanges_);
        if (ch) regenerate(), notifyProblemChanged();
    }

    std::string status() const override {
        char buf[128];
        std::snprintf(buf, sizeof buf, "iteration %d · changed %d · floor %.1f%% · regions %d", iter_, iter_ ? changed_.back() : 0, floorPct_, regions_);
        return buf;
    }

private:
    bool alwaysR2() const { return rule_ == 2; }
    Rule ruleFor(int it) const {
        if (rule_ == 2 || (rule_ == 1 && it < 4)) return Rule{5, 2};
        return Rule{};
    }

    void regenerate() {
        map_ = Map(kW, kH);
        Pcg32 rng{uint64_t(seed_)};
        randomFill(map_, rng, fill_);
        prev_.clear(), prev2_.clear(), changed_.clear();
        iter_ = 0, done_ = 0;
        measure();
    }

    void measure() {
        floorPct_ = 100.0f * map_.floorCount() / ((kW - 2) * (kH - 2));
        Regions r = labelRegions(map_);
        regions_ = int(r.size.size());
        largestPct_ = r.size.empty() ? 0 : 100.0f * r.size[r.largest()] / std::max(1, map_.floorCount());
    }

    float seed_ = 7, fill_ = 0.45f;
    int rule_ = 0;
    bool inPlace_ = false, showChanges_ = true;
    Map map_{kW, kH};
    std::vector<uint8_t> prev_, prev2_;
    std::vector<int> changed_;
    int iter_ = 0, done_ = 0, regions_ = 0;
    float floorPct_ = 0, largestPct_ = 0;
};

}  // namespace

int main() {
    static CaveDemo demo;
    ga::DemoConfig config;
    config.title = "Cellular automaton cave";
    config.width = kWidth;
    config.height = kHeight;
    config.stepsPerSecond = 4.0f;
    return ga::runDemo(demo, config);
}
