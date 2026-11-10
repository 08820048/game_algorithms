// Bridson 泊松圆盘采样的过程
//   - 每一步处理一个活跃点：在它周围的环带 [r, 2r] 里最多试 k 个候选，放下一个就结束这一步；k 个都放不下就把它移出活跃表
//   - 黄色是活跃点，墨色是已经退出的点；红色叉是被拒绝的候选，红圈是当前活跃点的环带
//   - annulus：area 按面积均匀取（Bridson 原文），radius 半径均匀取（偏向内圈），circle 全部放在半径 r 的圆上（更密，但会长出晶格）
//   - grid：边长 r/√2 的背景网格，每格最多一个点；disks：以每个点为圆心、直径为 r 的圆盘，互不重叠

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
constexpr float kTop = 80;
constexpr float kMx = 12, kMy = kTop + 10, kDw = 580, kDh = 380;
constexpr float kPx = 606;

class PoissonDemo final : public Demo {
public:
    PoissonDemo() { generate(); }
    void reset() override { round_ = 0; }
    bool step() override {
        if (round_ >= int(rounds_.size())) return false;
        ++round_;
        return round_ < int(rounds_.size());
    }

    void draw() const override {
        const Palette& pal = palette();
        const float hl = hairline();
        DrawRectangleLinesEx({kMx - 1, kMy - 1, kDw + 2, kDh + 2}, hl, pal.gridLine);
        float r = r_;
        if (showGrid_) {
            float c = r / std::sqrt(2.0f);
            for (float x = 0; x <= kDw; x += c) DrawLineEx({kMx + x, kMy}, {kMx + x, kMy + kDh}, hl, Fade(pal.gridLine, 0.8f));
            for (float y = 0; y <= kDh; y += c) DrawLineEx({kMx, kMy + y}, {kMx + kDw, kMy + y}, hl, Fade(pal.gridLine, 0.8f));
        }
        int shown = shownPoints();
        if (showDisks_)
            for (int i = 0; i < shown; ++i) DrawCircleV(sp(pts_[i]), r * 0.5f, Fade(pal.frontier, 0.22f));
        for (int i = 0; i < shown; ++i) {
            bool active = removedAt_[i] > round_;
            DrawCircleV(sp(pts_[i]), active ? 3.2f : 2.4f, active ? pal.frontier : pal.text);
        }
        if (round_ > 0 && round_ <= int(rounds_.size())) {
            const Round& rd = rounds_[round_ - 1];
            Vector2 c = sp(pts_[rd.from]);
            DrawCircleLinesV(c, r, pal.path);
            DrawCircleLinesV(c, 2 * r, Fade(pal.path, 0.6f));
            for (int t = rd.begin; t < rd.end; ++t) {
                Vector2 p = sp(log_[t].p);
                if (log_[t].accepted) DrawCircleV(p, 4, pal.path);
                else {
                    DrawLineEx({p.x - 2.5f, p.y - 2.5f}, {p.x + 2.5f, p.y + 2.5f}, 1.2f, pal.path);
                    DrawLineEx({p.x - 2.5f, p.y + 2.5f}, {p.x + 2.5f, p.y - 2.5f}, 1.2f, pal.path);
                }
            }
            DrawCircleV(c, 4, pal.text);
        }

        char buf[128];
        float y = kMy;
        auto line = [&](const char* s, Color col, float size = 12) {
            drawText(s, {kPx, y}, size, col);
            y += size + 7;
        };
        int act = 0;
        for (int i = 0; i < shown; ++i) act += removedAt_[i] > round_;
        long long tries = round_ > 0 ? rounds_[round_ - 1].end : 0;
        std::snprintf(buf, sizeof buf, "points %d", shown);
        line(buf, pal.text, 13);
        std::snprintf(buf, sizeof buf, "active %d", act);
        line(buf, pal.text);
        std::snprintf(buf, sizeof buf, "candidates tried %lld", tries);
        line(buf, pal.text);
        std::snprintf(buf, sizeof buf, "per point %.1f", shown ? double(tries) / shown : 0.0);
        line(buf, pal.text);
        std::snprintf(buf, sizeof buf, "disk coverage %.3f", packingDensity(shown, kDw, kDh, r));
        line(buf, pal.text);
        y += 10;
        line("random packing limit 0.547", pal.textMuted, 11);
        line("hexagonal packing  0.907", pal.textMuted, 11);
        if (round_ >= int(rounds_.size())) line("done", pal.textMuted, 11);
        DrawLineEx({0, kTop}, {float(kWidth), kTop}, hl, pal.gridLine);
    }

    void drawUi() override {
        bool ch = false;
        ch |= ui::slider({12, 10, 140, 26}, "seed", &seed_, 1, 50, "%.0f", 1);
        ch |= ui::slider({162, 10, 160, 26}, "r", &r_, 14, 48, "%.0f", 1);
        ch |= ui::slider({332, 10, 160, 26}, "k", &k_, 1, 60, "%.0f", 1);
        const char* names[] = {"area", "radius", "circle"};
        for (int i = 0; i < 3; ++i)
            if (ui::button({12.0f + i * 72, 44, 68, 26}, names[i], ann_ == i)) ann_ = i, ch = true;
        ui::toggle({240, 44, 80, 26}, "grid", &showGrid_);
        ui::toggle({328, 44, 86, 26}, "disks", &showDisks_);
        if (ch) generate(), notifyProblemChanged();
    }

    std::string status() const override {
        char buf[96];
        int n = shownPoints();
        std::snprintf(buf, sizeof buf, "%d points · coverage %.3f", n, packingDensity(n, kDw, kDh, r_));
        return buf;
    }

private:
    struct Round {
        int from, begin, end;
        bool placed;
    };
    Vector2 sp(Vec2 p) const { return {kMx + p.x, kMy + p.y}; }
    int shownPoints() const { return 1 + (round_ > 0 ? placedBefore_[round_ - 1] : 0); }

    void generate() {
        Pcg32 rng{uint64_t(seed_)};
        BridsonParams prm;
        prm.k = int(k_), prm.annulus = Annulus(ann_);
        log_.clear();
        pts_ = bridson(kDw, kDh, r_, rng, prm, nullptr, &log_);
        // 把日志切成"一轮"：同一个活跃点的连续尝试，到接受为止，或到 k 次失败为止
        rounds_.clear();
        placedBefore_.clear();
        removedAt_.assign(pts_.size(), 1 << 30);
        int placed = 0;
        for (int t = 0; t < int(log_.size());) {
            Round rd{log_[t].from, t, t, false};
            int n = 0;
            while (rd.end < int(log_.size()) && log_[rd.end].from == rd.from && n < prm.k) {
                ++n;
                if (log_[rd.end++].accepted) {
                    rd.placed = true;
                    break;
                }
            }
            if (!rd.placed) removedAt_[rd.from] = int(rounds_.size()) + 1;
            placed += rd.placed;
            rounds_.push_back(rd);
            placedBefore_.push_back(placed);
            t = rd.end;
        }
        round_ = 0;
    }

    float seed_ = 3, r_ = 24, k_ = 30;
    int ann_ = 0;
    bool showGrid_ = false, showDisks_ = false;
    std::vector<Vec2> pts_;
    std::vector<Trial> log_;
    std::vector<Round> rounds_;
    std::vector<int> placedBefore_, removedAt_;
    int round_ = 0;
};

}  // namespace

int main() {
    static PoissonDemo demo;
    ga::DemoConfig config;
    config.title = "Bridson Poisson disk sampling";
    config.width = kWidth;
    config.height = kHeight;
    config.stepsPerSecond = 30.0f;
    return ga::runDemo(demo, config);
}
