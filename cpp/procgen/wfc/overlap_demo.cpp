// 波函数坍缩：重叠模型
//   - 左边是样图。取出它所有 N×N 的窗口（symmetry 打开时再加上旋转和镜像）作为图案，出现次数是权重
//   - 右边每个格子放一个图案，相邻图案错开一格后重叠部分必须一致；格子显示图案左上角的颜色，没定的格子显示候选颜色的加权平均
//   - N 越大，保留的结构越多，图案也越多、越容易矛盾；rooms 生成的房间普遍比样图大——局部统计并不约束房间大小

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
constexpr int kGw = 64, kGh = 46;
constexpr float kCell = 8;
constexpr float kSx = 12, kSy = kTop + 22, kSs = 9;  // 样图
constexpr float kOx = 800 - 12 - kGw * kCell, kOy = kTop + 10;

class OverlapDemo final : public Demo {
public:
    OverlapDemo() { rebuild(); }
    void reset() override {
        attempt_ = 0, restarts_ = 0, hold_ = 0;
        start();
    }
    bool step() override {
        for (int k = 0; k < 6; ++k) {
            Status st = solver_->status();
            if (st == Status::Contradiction) {
                if (++hold_ < 40) return true;
                ++attempt_, ++restarts_, hold_ = 0;
                start();
                return true;
            }
            if (st == Status::Done) return false;
            solver_->step();
        }
        return solver_->status() != Status::Done;
    }
    void update() override {
        if ((palette().background.r < 128) != dark_) {  // 换了主题，全部重算颜色
            dark_ = !dark_;
            std::fill(cachedCount_.begin(), cachedCount_.end(), -1);
        }
        refreshColours();
    }

    void draw() const override {
        const Palette& pal = palette();
        const float hl = hairline();
        drawText("sample", {kSx, kTop + 6}, 11, pal.textMuted);
        for (int y = 0; y < sample_.h; ++y)
            for (int x = 0; x < sample_.w; ++x) DrawRectangleRec({kSx + x * kSs, kSy + y * kSs, kSs, kSs}, colourOf(sample_.at(x, y)));
        DrawRectangleLinesEx({kSx - 1, kSy - 1, sample_.w * kSs + 2, sample_.h * kSs + 2}, hl, pal.gridLine);

        for (int y = 0; y < kGh; ++y)
            for (int x = 0; x < kGw; ++x) DrawRectangleRec({kOx + x * kCell, kOy + y * kCell, kCell, kCell}, colour_[y * kGw + x]);
        if (solver_->status() == Status::Contradiction && solver_->contradictionCell() >= 0) {
            int c = solver_->contradictionCell();
            DrawRectangleLinesEx({kOx + c % kGw * kCell - 3, kOy + c / kGw * kCell - 3, kCell + 6, kCell + 6}, 2, pal.path);
        }
        DrawRectangleLinesEx({kOx - 1, kOy - 1, kGw * kCell + 2, kGh * kCell + 2}, hl, pal.gridLine);

        char buf[128];
        float y = kSy + sample_.h * kSs + 18;
        auto line = [&](const char* str, Color col, float size = 12) {
            drawText(str, {kSx, y}, size, col);
            y += size + 7;
        };
        std::snprintf(buf, sizeof buf, "patterns %d", rules_.T);
        line(buf, pal.text, 13);
        int decided = 0;
        for (int c = 0; c < kGw * kGh; ++c) decided += solver_->count(c) == 1;
        std::snprintf(buf, sizeof buf, "decided %.0f%%", 100.0 * decided / (kGw * kGh));
        line(buf, pal.text);
        std::snprintf(buf, sizeof buf, "observed %lld", solver_->stats().observations);
        line(buf, pal.text);
        std::snprintf(buf, sizeof buf, "backtracks %lld", solver_->stats().backtracks);
        line(buf, backtrack_ ? pal.text : pal.textMuted);
        std::snprintf(buf, sizeof buf, "restarts %d", restarts_);
        line(buf, pal.text);
        if (solver_->status() == Status::Contradiction) line("contradiction", pal.path, 13);
        else if (solver_->status() == Status::Done) line("done", pal.textMuted, 13);
        DrawLineEx({0, kTop}, {float(kWidth), kTop}, hl, pal.gridLine);
    }

    void drawUi() override {
        bool ch = false;
        const char* names[] = {"rooms", "lakes", "maze"};
        for (int i = 0; i < 3; ++i)
            if (ui::button({12.0f + i * 72, 10, 68, 26}, names[i], which_ == i)) which_ = i, ch = true;
        ch |= ui::slider({236, 10, 120, 26}, "N", &n_, 2, 4, "%.0f", 1);
        ch |= ui::slider({366, 10, 150, 26}, "seed", &seed_, 1, 50, "%.0f", 1);
        const char* hs[] = {"entropy", "scanline"};
        for (int i = 0; i < 2; ++i)
            if (ui::button({12.0f + i * 84, 44, 80, 26}, hs[i], heur_ == i)) heur_ = i, ch = true;
        ch |= ui::toggle({184, 44, 110, 26}, "symmetry", &symmetry_);
        ch |= ui::toggle({302, 44, 110, 26}, "backtrack", &backtrack_);
        if (ch) rebuild(), notifyProblemChanged();
    }

    std::string status() const override {
        char buf[96];
        std::snprintf(buf, sizeof buf, "%d patterns · %d restarts · %lld backtracks", rules_.T, restarts_, solver_->stats().backtracks);
        return buf;
    }

private:
    bool periodic() const { return which_ != 0; }  // 房间的四周是墙，不能首尾相接
    void rebuild() {
        alphabet_.clear();
        sample_ = sampleFromText(which_ == 0 ? roomsText() : which_ == 1 ? lakesText() : mazeText(), &alphabet_);
        rules_ = fromSample(sample_, int(n_), symmetry_, periodic(), &patterns_);
        reset();
    }
    void start() {
        Heuristic h = heur_ == 0 ? Heuristic::Entropy : Heuristic::Scanline;
        solver_ = std::make_unique<Solver>(rules_, kGw, kGh, Options{h, periodic(), backtrack_, 20000}, uint64_t(seed_) * 7919 + uint64_t(attempt_));
        colour_.assign(size_t(kGw) * kGh, BLANK);
        cachedCount_.assign(size_t(kGw) * kGh, -1);  // 颜色在下一次 update() 里算（构造时还不能读调色板）
    }
    Color colourOf(int idx) const {
        const Palette& pal = palette();
        char ch = alphabet_[idx];
        Color base = ch == '#' ? pal.text : ch == '+' ? pal.path : ch == '~' ? Color{60, 120, 200, 255} : ch == ':' ? Color{220, 190, 120, 255}
                   : which_ == 1 ? Color{100, 160, 80, 255} : ColorLerp(pal.background, pal.text, 0.06f);
        return ch == '#' || ch == '+' || (ch == '.' && which_ != 1) ? base : ColorLerp(pal.background, base, 0.8f);
    }
    // 只重算候选数变了的格子：加权平均每个候选图案左上角的颜色
    void refreshColours() {
        int ncol = int(alphabet_.size());
        std::vector<Color> cols(ncol);
        for (int k = 0; k < ncol; ++k) cols[k] = colourOf(k);
        for (int c = 0; c < kGw * kGh; ++c) {
            int cnt = solver_->count(c);
            if (cnt == cachedCount_[c]) continue;
            cachedCount_[c] = cnt;
            double r = 0, g = 0, b = 0, tw = 0;
            const uint64_t* d = solver_->domain(c);
            for (int t = 0; t < rules_.T; ++t)
                if ((d[t >> 6] >> (t & 63)) & 1) {
                    Color col = cols[patterns_[t][0]];
                    double w = rules_.weight[t];
                    r += col.r * w, g += col.g * w, b += col.b * w, tw += w;
                }
            colour_[c] = tw > 0 ? Color{(unsigned char)(r / tw), (unsigned char)(g / tw), (unsigned char)(b / tw), 255} : palette().path;
        }
    }

    int which_ = 0, heur_ = 0;
    float n_ = 3, seed_ = 1;
    bool symmetry_ = true, backtrack_ = true, dark_ = false;
    std::string alphabet_;
    Sample sample_;
    Rules rules_;
    std::vector<std::vector<uint8_t>> patterns_;
    std::unique_ptr<Solver> solver_;
    std::vector<Color> colour_;
    std::vector<int> cachedCount_;
    int attempt_ = 0, restarts_ = 0, hold_ = 0;
};

}  // namespace

int main() {
    static OverlapDemo demo;
    ga::DemoConfig config;
    config.title = "Wave function collapse: overlapping model";
    config.width = kWidth;
    config.height = kHeight;
    config.stepsPerSecond = 60.0f;
    return ga::runDemo(demo, config);
}
