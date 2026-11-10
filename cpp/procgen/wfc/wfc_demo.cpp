// 波函数坍缩：图块模型
//   - 每一步观察一个格子（定成一个图块），然后把约束传播出去。没定的格子越深，剩下的候选越少
//   - 红框是刚观察的格子；红色实心是矛盾（候选被删光）的格子。不回溯时，矛盾后整张图重来
//   - roads：16 种道路图块；no dead ends：去掉断头路，有的局面无解；terrain：四个角各是水、沙、草，水和草之间必须隔着沙
//   - entropy 挑熵最小的格子；scanline 逐行扫描；random 随机挑——只有它会让"无断头路"频繁矛盾
//   - pure weight：地形里四角同色的图块的权重。超过 13 左右，整张图常被水或草一种地形占满

#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "demo.h"
#include "samples.h"
#include "ui.h"
#include "wfc.h"

namespace {

using namespace ga;
using namespace ga::wfc;

constexpr int kWidth = 800, kHeight = 480;
constexpr float kTop = 80;
constexpr int kGw = 24, kGh = 16;
constexpr float kCell = 24, kMx = 12, kMy = kTop + 6;
constexpr float kPx = kMx + kGw * kCell + 16;

class WfcDemo final : public Demo {
public:
    WfcDemo() { rebuildRules(); }
    void reset() override {
        attempt_ = 0, restarts_ = 0, hold_ = 0;
        start();
    }
    bool step() override {
        Status st = solver_->status();
        if (st == Status::Contradiction) {
            if (++hold_ < 12) return true;  // 停一下让人看清矛盾在哪，再重来
            ++attempt_, ++restarts_, hold_ = 0;
            start();
            return true;
        }
        if (st == Status::Done) return false;
        solver_->step();
        return solver_->status() != Status::Done;
    }

    void draw() const override {
        const Palette& pal = palette();
        const float hl = hairline();
        const Solver& s = *solver_;
        for (int y = 0; y < kGh; ++y)
            for (int x = 0; x < kGw; ++x) {
                int c = y * kGw + x;
                Rectangle r{kMx + x * kCell, kMy + y * kCell, kCell, kCell};
                int t = s.tileAt(c);
                if (t >= 0) drawTile(t, r);
                else {
                    float f = s.count(c) <= 0 ? 1 : 1 - std::log(float(s.count(c))) / std::log(float(rules_.T));
                    DrawRectangleRec(r, Fade(pal.text, 0.05f + 0.4f * f));
                    if (s.count(c) == 0) DrawRectangleRec(r, pal.path);
                }
            }
        for (int x = 0; x <= kGw; ++x) DrawLineEx({kMx + x * kCell, kMy}, {kMx + x * kCell, kMy + kGh * kCell}, hl, Fade(pal.gridLine, 0.6f));
        for (int y = 0; y <= kGh; ++y) DrawLineEx({kMx, kMy + y * kCell}, {kMx + kGw * kCell, kMy + y * kCell}, hl, Fade(pal.gridLine, 0.6f));
        if (s.status() == Status::Contradiction && s.contradictionCell() >= 0) {
            int c = s.contradictionCell();
            DrawRectangleRec({kMx + c % kGw * kCell, kMy + c / kGw * kCell, kCell, kCell}, pal.path);
        }
        if (s.lastObserved() >= 0) {
            int c = s.lastObserved();
            DrawRectangleLinesEx({kMx + c % kGw * kCell - 1, kMy + c / kGw * kCell - 1, kCell + 2, kCell + 2}, 2, pal.path);
        }

        char buf[128];
        float y = kMy;
        auto line = [&](const char* str, Color col, float size = 12) {
            drawText(str, {kPx, y}, size, col);
            y += size + 7;
        };
        int decided = 0;
        for (int c = 0; c < kGw * kGh; ++c) decided += s.count(c) == 1;
        std::snprintf(buf, sizeof buf, "tiles %d", rules_.T);
        line(buf, pal.text, 13);
        std::snprintf(buf, sizeof buf, "decided %d / %d", decided, kGw * kGh);
        line(buf, pal.text);
        std::snprintf(buf, sizeof buf, "observations %lld", s.stats().observations);
        line(buf, pal.text);
        std::snprintf(buf, sizeof buf, "by propagation %lld", std::max(0LL, decided - s.stats().observations));
        line(buf, pal.text);
        std::snprintf(buf, sizeof buf, "backtracks %lld", s.stats().backtracks);
        line(buf, backtrack_ ? pal.text : pal.textMuted);
        std::snprintf(buf, sizeof buf, "restarts %d", restarts_);
        line(buf, pal.text);
        y += 8;
        if (s.status() == Status::Contradiction) line("contradiction", pal.path, 13);
        else if (s.status() == Status::Done) line("done", pal.textMuted, 13);
        y += 8;
        line("darker = fewer options", pal.textMuted, 10);
        DrawLineEx({0, kTop}, {float(kWidth), kTop}, hl, pal.gridLine);
    }

    void drawUi() override {
        bool ch = false, rules = false;
        const char* sets[] = {"roads", "no dead ends", "terrain"};
        float x = 12;
        for (int i = 0; i < 3; ++i) {
            float w = i == 1 ? 112 : 76;
            if (ui::button({x, 10, w, 26}, sets[i], set_ == i)) set_ = i, rules = true;
            x += w + 4;
        }
        ch |= ui::slider({x + 10, 10, 160, 26}, "seed", &seed_, 1, 50, "%.0f", 1);
        const char* hs[] = {"entropy", "scanline", "random"};
        for (int i = 0; i < 3; ++i)
            if (ui::button({12.0f + i * 84, 44, 80, 26}, hs[i], heur_ == i)) heur_ = i, ch = true;
        ch |= ui::toggle({268, 44, 110, 26}, "backtrack", &backtrack_);
        if (set_ == 2) rules |= ui::slider({392, 44, 200, 26}, "pure weight", &pureWeight_, 1, 20, "%.0f", 1);
        if (rules) rebuildRules();
        if (ch || rules) notifyProblemChanged();
    }

    std::string status() const override {
        char buf[96];
        std::snprintf(buf, sizeof buf, "%lld observations · %d restarts · %lld backtracks", solver_->stats().observations, restarts_, solver_->stats().backtracks);
        return buf;
    }

private:
    void rebuildRules() {
        tiles_ = set_ == 2 ? terrainTiles(pureWeight_) : roadTiles(set_ == 1);
        rules_ = fromSockets(tiles_);
        corners_ = terrainCorners();
        reset();
    }
    void start() {
        Heuristic h = heur_ == 0 ? Heuristic::Entropy : heur_ == 1 ? Heuristic::Scanline : Heuristic::Random;
        solver_ = std::make_unique<Solver>(rules_, kGw, kGh, Options{h, false, backtrack_}, uint64_t(seed_) * 7919 + uint64_t(attempt_));
    }

    void drawTile(int t, Rectangle r) const {
        const Palette& pal = palette();
        if (set_ == 2) {
            const CornerTile& c = corners_[t];
            int q[4] = {c.tl, c.tr, c.bl, c.br};
            for (int k = 0; k < 4; ++k)
                DrawRectangleRec({r.x + (k % 2) * r.width / 2, r.y + (k / 2) * r.height / 2, r.width / 2, r.height / 2}, terrainColor(q[k]));
            return;
        }
        DrawRectangleRec(r, ColorLerp(pal.background, Color{110, 160, 90, 255}, 0.35f));
        int m = roadMask(tiles_[t]);
        Vector2 c{r.x + r.width / 2, r.y + r.height / 2};
        Color road = ColorLerp(pal.background, pal.text, 0.75f);
        const float w = 7;
        for (int d = 0; d < 4; ++d)
            if (m >> d & 1) DrawLineEx(c, {c.x + kDx[d] * r.width / 2, c.y + kDy[d] * r.height / 2}, w, road);
        if (m) DrawCircleV(c, w / 2, road);
    }
    Color terrainColor(int k) const {
        const Palette& pal = palette();
        Color base = k == 0 ? Color{60, 120, 200, 255} : k == 1 ? Color{220, 190, 120, 255} : Color{100, 160, 80, 255};
        return ColorLerp(pal.background, base, 0.8f);
    }

    int set_ = 0, heur_ = 0;
    float seed_ = 1, pureWeight_ = 8;
    bool backtrack_ = false;
    std::vector<SocketTile> tiles_;
    std::vector<CornerTile> corners_;
    Rules rules_;
    std::unique_ptr<Solver> solver_;
    int attempt_ = 0, restarts_ = 0, hold_ = 0;
};

}  // namespace

int main() {
    static WfcDemo demo;
    ga::DemoConfig config;
    config.title = "Wave function collapse: tiles";
    config.width = kWidth;
    config.height = kHeight;
    config.stepsPerSecond = 30.0f;
    return ga::runDemo(demo, config);
}
