// 随机数演示，三个标签页：
//   shuffle  朴素洗牌 vs Fisher–Yates：每张牌落在每个位置的频率热力图
//   crit     独立随机 vs 伪随机分布（PRD）：暴击时间线与"间隔"直方图
//   loot     加权掉落表：期望概率 vs 实际频率
// 所有随机数来自固定种子的 PCG32，按"重置"会得到完全相同的序列。

#include <algorithm>
#include <cstdio>
#include <deque>
#include <numeric>
#include <string>
#include <vector>

#include "demo.h"
#include "random.h"
#include "ui.h"

namespace {

using namespace ga;

constexpr int kWidth = 800;
constexpr int kHeight = 470;
constexpr float kTop = 52;
constexpr int kCards = 6;
constexpr int kGapBins = 16;  // 间隔直方图：1..15，16 表示 16 及以上

enum class Tab { Shuffle, Crit, Loot };

struct CritTrack {
    std::deque<bool> recent;     // 最近的攻击：是否暴击
    int gaps[kGapBins] = {};     // 两次暴击之间的攻击次数分布
    int sinceLast = 0;
    int attacks = 0;
    int crits = 0;
    int longestDrought = 0;

    void record(bool crit) {
        ++attacks;
        ++sinceLast;
        recent.push_back(crit);
        if (recent.size() > 84) recent.pop_front();
        if (crit) {
            ++crits;
            gaps[std::min(sinceLast, kGapBins) - 1]++;
            sinceLast = 0;
        } else {
            longestDrought = std::max(longestDrought, sinceLast);
        }
    }
};

struct LootTier {
    const char* name;
    double weight;
};
constexpr LootTier kLoot[] = {{"common", 60}, {"uncommon", 25}, {"rare", 10}, {"epic", 4}, {"legendary", 1}};
constexpr int kTiers = 5;

class RandomDemo final : public Demo {
public:
    RandomDemo() { reset(); }

    void reset() override {
        rng_ = Pcg32(2026);
        for (auto& row : naiveFreq_) std::fill(std::begin(row), std::end(row), 0);
        for (auto& row : fyFreq_) std::fill(std::begin(row), std::end(row), 0);
        shuffles_ = 0;
        independent_ = {};
        prdTrack_ = {};
        prd_ = PrdRoll(critChance_ / 100.0);
        lootCounts_.assign(kTiers, 0);
        lootRecent_.clear();
        lootDraws_ = 0;
    }

    bool step() override {
        // 三个实验同时推进，切换标签页时数据不会中断
        for (int i = 0; i < 100; ++i) {
            std::vector<int> a(kCards), b(kCards);
            std::iota(a.begin(), a.end(), 0);
            std::iota(b.begin(), b.end(), 0);
            naiveShuffle(a, rng_);
            shuffle(b, rng_);
            for (int pos = 0; pos < kCards; ++pos) {
                naiveFreq_[a[pos]][pos]++;
                fyFreq_[b[pos]][pos]++;
            }
            ++shuffles_;
        }

        independent_.record(rng_.chance(critChance_ / 100.0));
        prdTrack_.record(prd_.roll(rng_));

        for (int i = 0; i < 4; ++i) {
            int tier = lootTable_.pick(rng_);
            lootCounts_[tier]++;
            ++lootDraws_;
            lootRecent_.push_back(tier);
            if (lootRecent_.size() > 40) lootRecent_.pop_front();
        }
        return true;
    }

    void draw() const override {
        const Palette& p = palette();
        DrawLineEx({0, kTop}, {float(kWidth), kTop}, hairline(), p.gridLine);
        switch (tab_) {
            case Tab::Shuffle: drawShuffle(); break;
            case Tab::Crit: drawCrit(); break;
            case Tab::Loot: drawLoot(); break;
        }
    }

    void drawUi() override {
        if (ui::button({16, 13, 88, 26}, "shuffle", tab_ == Tab::Shuffle)) tab_ = Tab::Shuffle;
        if (ui::button({108, 13, 88, 26}, "crit", tab_ == Tab::Crit)) tab_ = Tab::Crit;
        if (ui::button({200, 13, 88, 26}, "loot", tab_ == Tab::Loot)) tab_ = Tab::Loot;
        if (tab_ == Tab::Crit) {
            if (ui::slider({320, 13, 250, 26}, "chance", &critChance_, 5, 60, "%.0f%%", 1)) {
                // 概率变了，两条统计都从头开始
                prd_ = PrdRoll(critChance_ / 100.0);
                independent_ = {};
                prdTrack_ = {};
            }
        }
    }

    std::string status() const override {
        char buf[96];
        switch (tab_) {
            case Tab::Shuffle:
                std::snprintf(buf, sizeof buf, "shuffles %d · bias %.1f%% vs %.1f%%", shuffles_,
                              maxBias(naiveFreq_) * 100, maxBias(fyFreq_) * 100);
                break;
            case Tab::Crit:
                std::snprintf(buf, sizeof buf, "attacks %d · drought %d vs %d", independent_.attacks,
                              independent_.longestDrought, prdTrack_.longestDrought);
                break;
            case Tab::Loot:
                std::snprintf(buf, sizeof buf, "draws %d · legendary %d", lootDraws_, lootCounts_[4]);
                break;
        }
        return buf;
    }

private:
    using Freq = int[kCards][kCards];

    double maxBias(const Freq& f) const {
        if (shuffles_ == 0) return 0;
        double expected = shuffles_ / double(kCards), worst = 0;
        for (int i = 0; i < kCards; ++i)
            for (int j = 0; j < kCards; ++j) worst = std::max(worst, std::abs(f[i][j] / expected - 1));
        return worst;
    }

    // ---------------- shuffle ----------------
    void drawHeatmap(const Freq& f, float x0, float y0, const char* title, const char* subtitle) const {
        const Palette& p = palette();
        const float cell = 42;
        drawText(title, {x0, y0 - 52}, 12, p.text);
        drawText(subtitle, {x0, y0 - 34}, 10, p.textMuted);
        double expected = std::max(1.0, shuffles_ / double(kCards));
        for (int card = 0; card < kCards; ++card) {
            char lbl[8];
            std::snprintf(lbl, sizeof lbl, "%d", card);
            drawTextCentered(lbl, {x0 - 14, y0 + card * cell + cell / 2}, 10, p.textMuted);
            drawTextCentered(lbl, {x0 + card * cell + cell / 2, y0 - 12}, 10, p.textMuted);
            for (int pos = 0; pos < kCards; ++pos) {
                double dev = shuffles_ ? f[card][pos] / expected - 1 : 0;
                float k = std::min(1.0f, static_cast<float>(std::abs(dev) / 0.3));
                Color c = dev > 0 ? ColorLerp(p.background, p.goal, k) : ColorLerp(p.background, p.wall, k * 0.7f);
                Rectangle r{x0 + pos * cell, y0 + card * cell, cell, cell};
                DrawRectangleRec(r, c);
                DrawRectangleLinesEx(r, hairline(), p.gridLine);
                char pct[16];
                std::snprintf(pct, sizeof pct, "%.1f", shuffles_ ? 100.0 * f[card][pos] / shuffles_ : 0.0);
                drawTextCentered(pct, {r.x + cell / 2, r.y + cell / 2}, 10, k > 0.55f ? p.background : p.text);
            }
        }
        char buf[48];
        std::snprintf(buf, sizeof buf, "max bias %.1f%%", maxBias(f) * 100);
        drawText(buf, {x0, y0 + kCards * cell + 12}, 11, p.text);
    }

    void drawShuffle() const {
        const Palette& p = palette();
        drawHeatmap(naiveFreq_, 80, 140, "naive", "swap(i, random[0, n))");
        drawHeatmap(fyFreq_, 460, 140, "Fisher-Yates", "swap(i, random[0, i])");
        drawText("row = card, column = final position, cell = % of shuffles (expected 16.7)", {80, kHeight - 26.0f}, 10,
                 p.textMuted);
    }

    // ---------------- crit ----------------
    void drawTimeline(const CritTrack& t, float y, const char* name, const char* detail) const {
        const Palette& p = palette();
        drawText(name, {16, y}, 12, p.text);
        drawText(detail, {16, y + 18}, 10, p.textMuted);
        float x = 190;
        for (bool crit : t.recent) {
            if (crit) {
                DrawRectangleRec({x, y - 4, 5, 34}, p.goal);
            } else {
                DrawRectangleRec({x, y + 22, 5, 8}, p.gridLine);
            }
            x += 7;
        }
        char buf[64];
        std::snprintf(buf, sizeof buf, "rate %.1f%%   longest drought %d",
                      t.attacks ? 100.0 * t.crits / t.attacks : 0.0, t.longestDrought);
        drawText(buf, {190, y + 38}, 10, p.textMuted);
    }

    void drawCrit() const {
        const Palette& p = palette();
        char c[48];
        std::snprintf(c, sizeof c, "C = %.4f", prd_.constant());
        drawTimeline(independent_, 82, "independent", "each hit: chance p");
        drawTimeline(prdTrack_, 160, "PRD", c);

        // 间隔直方图：两次暴击之间隔了几次攻击
        const float x0 = 60, y0 = kHeight - 40.0f, h = 170, bw = 40;
        drawText("attacks between crits", {x0, y0 - h - 26}, 11, p.text);
        double maxShare = 0.05;
        for (const CritTrack* t : {&independent_, &prdTrack_})
            for (int g : t->gaps) maxShare = std::max(maxShare, t->crits ? g / double(t->crits) : 0.0);
        for (int i = 0; i < kGapBins; ++i) {
            float x = x0 + i * (bw + 4);
            auto bar = [&](const CritTrack& t, float dx, Color col) {
                double share = t.crits ? t.gaps[i] / double(t.crits) : 0;
                float bh = static_cast<float>(share / maxShare) * h;
                DrawRectangleRec({x + dx, y0 - bh, bw / 2 - 1, bh}, col);
            };
            bar(independent_, 0, p.textMuted);
            bar(prdTrack_, bw / 2, p.goal);
            char lbl[8];
            std::snprintf(lbl, sizeof lbl, i == kGapBins - 1 ? "%d+" : "%d", i + 1);
            drawTextCentered(lbl, {x + bw / 2, y0 + 12}, 9, p.textMuted);
        }
        DrawLineEx({x0, y0}, {x0 + kGapBins * (bw + 4), y0}, hairline(), p.text);
        DrawRectangleRec({kWidth - 230.0f, y0 - h - 24, 10, 10}, p.textMuted);
        drawText("independent", {kWidth - 214.0f, y0 - h - 26}, 10, p.text);
        DrawRectangleRec({kWidth - 120.0f, y0 - h - 24, 10, 10}, p.goal);
        drawText("PRD", {kWidth - 104.0f, y0 - h - 26}, 10, p.text);
    }

    // ---------------- loot ----------------
    void drawLoot() const {
        const Palette& p = palette();
        const float x0 = 150, w = 440;
        float y = 92;
        for (int i = 0; i < kTiers; ++i) {
            double expected = lootTable_.probability(i);
            double observed = lootDraws_ ? lootCounts_[i] / double(lootDraws_) : 0;
            drawText(kLoot[i].name, {16, y + 4}, 12, i == 4 ? p.goal : p.text);
            // 条形长度用平方根缩放，让 1% 和 60% 都看得清
            float we = std::sqrt(static_cast<float>(expected)) * w;
            float wo = std::sqrt(static_cast<float>(observed)) * w;
            DrawRectangleRec({x0, y + 2, wo, 18}, i == 4 ? p.goal : p.frontier);
            DrawLineEx({x0 + we, y - 4}, {x0 + we, y + 26}, 2, p.text);
            char buf[64];
            std::snprintf(buf, sizeof buf, "%5.2f%%  /  %4.1f%%", observed * 100, expected * 100);
            drawText(buf, {x0 + w + 20, y + 5}, 11, p.text);
            y += 44;
        }
        drawText("bar = observed (sqrt scale), tick = expected", {x0, y}, 10, p.textMuted);

        // 最近掉落
        y += 40;
        drawText("recent drops", {16, y}, 11, p.textMuted);
        float x = 150;
        for (int tier : lootRecent_) {
            Color col = tier == 4 ? p.goal : (tier == 3 ? p.text : ColorLerp(p.gridLine, p.textMuted, tier / 3.0f));
            DrawRectangleRec({x, y - 2, 12, 16}, col);
            x += 15;
        }
    }

    Tab tab_ = Tab::Shuffle;
    Pcg32 rng_{2026};

    int naiveFreq_[kCards][kCards] = {};
    int fyFreq_[kCards][kCards] = {};
    int shuffles_ = 0;

    float critChance_ = 25;
    PrdRoll prd_{0.25};
    CritTrack independent_;
    CritTrack prdTrack_;

    WeightedTable lootTable_{{kLoot[0].weight, kLoot[1].weight, kLoot[2].weight, kLoot[3].weight, kLoot[4].weight}};
    std::vector<int> lootCounts_ = std::vector<int>(kTiers, 0);
    std::deque<int> lootRecent_;
    int lootDraws_ = 0;
};

}  // namespace

int main() {
    static RandomDemo demo;
    ga::DemoConfig config;
    config.title = "Randomness in Games";
    config.width = kWidth;
    config.height = kHeight;
    config.stepsPerSecond = 20;
    return ga::runDemo(demo, config);
}
