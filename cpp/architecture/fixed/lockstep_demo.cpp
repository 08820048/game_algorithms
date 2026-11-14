// 同一份代码，两种算法细节：多久不同步
//   - 64 个互相排斥的粒子（混沌）。墨色实心是 A 局，朱红空心是 B 局，两局从同一个状态出发
//   - float, sum order：B 局把每个粒子受到的力按相反顺序累加（相当于换一种多线程归约方式）
//   - float, fma：B 局每一项用 fma 累加（只舍入一次），相当于换一个会合并乘加的编译器
//   - fixed, sum order：同样换顺序，但用 Q15.16 定点数：整数加法满足结合律，两局永远逐位相同
//   - 右侧：两局里相差最远的一个粒子（对数坐标）。第 3000 帧时把 A 局的哈希和本机原生编译的结果比较
//   - 跑到第 3000 帧停 3 秒，然后重新开始

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "demo.h"
#include "fixed.h"
#include "ui.h"

namespace {

using namespace ga;
using namespace ga::fixed;

constexpr int kWidth = 800, kHeight = 480;
constexpr float kTop = 80, kX0 = 12, kY0 = 98;
constexpr int kCount = 64, kRound = 3000;
// fixed_bench platform 在 Apple M3 上原生编译（clang -O2）得到的第 3000 帧哈希
constexpr uint64_t kNativeFloat = 0xe85462b6853c883full, kNativeFixed = 0xefaf785213611901ull;

class LockstepDemo final : public Demo {
public:
    LockstepDemo() { reset(); }
    void reset() override {
        fa_ = fb_ = Swarm<float>::make(1, kCount);
        xa_ = xb_ = Swarm<Fx>::make(1, kCount);
        dist_.clear();
        diverged_ = -1, firstBit_ = -1;
        hashAt3000_ = 0, hold_ = 0;
    }
    bool step() override {
        if (frame() >= kRound) {  // 停 3 秒，显示第 3000 帧的哈希
            if (++hold_ > 360) reset();
            return true;
        }
        double d;
        bool same;
        if (mode_ < 2) {
            fa_.step(false, false), fb_.step(mode_ == 0, mode_ == 1);
            d = fa_.maxDistance(fb_), same = fa_.hash() == fb_.hash();
            if (frame() == kRound) hashAt3000_ = fa_.hash();
        } else {
            xa_.step(false), xb_.step(true);
            d = xa_.maxDistance(xb_), same = xa_.hash() == xb_.hash();
            if (frame() == kRound) hashAt3000_ = xa_.hash();
        }
        dist_.push_back(float(d));
        if (firstBit_ < 0 && !same) firstBit_ = int(frame());
        if (diverged_ < 0 && d > 1) diverged_ = int(frame());
        return true;
    }

    void draw() const override {
        const Palette& pal = palette();
        const float hl = hairline();
        DrawRectangleLinesEx({kX0, kY0, float(Swarm<float>::kW), float(Swarm<float>::kH)}, hl, pal.gridLine);
        auto at = [](double x, double y) { return Vector2{kX0 + float(x), kY0 + float(y)}; };
        BeginScissorMode(int(kX0), int(kY0), Swarm<float>::kW, Swarm<float>::kH);
        if (mode_ < 2) {
            for (auto& p : fa_.ps) DrawCircleV(at(p.x, p.y), 4.5f, pal.text);
            for (auto& p : fb_.ps) DrawCircleLinesV(at(p.x, p.y), 7, pal.path);
        } else {
            for (auto& p : xa_.ps) DrawCircleV(at(p.x.toDouble(), p.y.toDouble()), 4.5f, pal.text);
            for (auto& p : xb_.ps) DrawCircleLinesV(at(p.x.toDouble(), p.y.toDouble()), 7, pal.path);
        }
        EndScissorMode();

        // 距离曲线（对数）
        const float px = kX0 + Swarm<float>::kW + 22, pw = kWidth - px - 14, py = kY0 + 40, ph = 230;
        DrawRectangleLinesEx({px, py, pw, ph}, hl, pal.gridLine);
        auto yOf = [&](float d) {
            float l = std::log10(std::max(d, 1e-7f));
            return py + ph - (l + 7) / 10 * ph;  // 1e-7 … 1e3
        };
        for (int e = -6; e <= 2; e += 2) {
            float y = yOf(std::pow(10.0f, float(e)));
            DrawLineEx({px, y}, {px + pw, y}, hl, Fade(pal.gridLine, 0.6f));
            char b[16];
            std::snprintf(b, sizeof b, "1e%d", e);
            drawText(b, {px + 2, y - 12}, 9, pal.textMuted);
        }
        float y1 = yOf(1);
        drawDashedLine({px, y1}, {px + pw, y1}, 1, pal.path);
        for (size_t i = 1; i < dist_.size(); ++i) {
            float x0 = px + pw * float(i - 1) / kRound, x1 = px + pw * float(i) / kRound;
            DrawLineEx({x0, yOf(dist_[i - 1])}, {x1, yOf(dist_[i])}, 1.4f, pal.text);
        }
        if (mode_ == 2 && !dist_.empty() && *std::max_element(dist_.begin(), dist_.end()) == 0)
            drawTextCentered("0 px on every frame", {px + pw / 2, py + ph / 2}, 11, pal.textMuted);
        drawText("max distance (px)", {px, py - 16}, 10, pal.textMuted);
        drawText("frame 0 ... 3000", {px, py + ph + 4}, 10, pal.textMuted);

        char buf[160];
        bool same = mode_ < 2 ? fa_.hash() == fb_.hash() : xa_.hash() == xb_.hash();
        std::snprintf(buf, sizeof buf, "frame %u   A and B %s   max distance %.3g px", frame(), same ? "bit-identical" : "differ",
                      dist_.empty() ? 0.0 : double(dist_.back()));
        drawText(buf, {12, kTop + 6}, 11, same ? pal.textMuted : pal.path);
        float ty = py + ph + 22;
        if (firstBit_ >= 0) {
            std::snprintf(buf, sizeof buf, "first different bit: frame %d", firstBit_);
            drawText(buf, {px, ty}, 10, pal.textMuted), ty += 16;
        }
        if (diverged_ >= 0) {
            std::snprintf(buf, sizeof buf, "off by > 1 px: frame %d", diverged_);
            drawText(buf, {px, ty}, 10, pal.path), ty += 16;
        }
        if (hashAt3000_) {
            uint64_t native = mode_ < 2 ? kNativeFloat : kNativeFixed;
            std::snprintf(buf, sizeof buf, "A at 3000: %08x...", unsigned(hashAt3000_ >> 32));
            drawText(buf, {px, ty}, 10, pal.textMuted), ty += 14;
            drawText(hashAt3000_ == native ? "= native build" : "!= native build", {px, ty}, 10,
                     hashAt3000_ == native ? pal.textMuted : pal.path);
        }
        DrawLineEx({0, kTop}, {float(kWidth), kTop}, hl, pal.gridLine);
    }

    void drawUi() override {
        const char* names[3] = {"float, sum order", "float, fma", "fixed, sum order"};
        for (int i = 0; i < 3; ++i)
            if (ui::button({12.0f + i * 134, 10, 130, 26}, names[i], mode_ == i) && mode_ != i) mode_ = i, reset(), notifyProblemChanged();
    }

    std::string status() const override {
        char buf[64];
        bool same = mode_ < 2 ? fa_.hash() == fb_.hash() : xa_.hash() == xb_.hash();
        std::snprintf(buf, sizeof buf, "frame %u · %s", frame(), same ? "in sync" : "diverged");
        return buf;
    }

private:
    uint32_t frame() const { return mode_ < 2 ? fa_.frame : xa_.frame; }
    Swarm<float> fa_, fb_;
    Swarm<Fx> xa_, xb_;
    std::vector<float> dist_;
    int mode_ = 0, diverged_ = -1, firstBit_ = -1, hold_ = 0;
    uint64_t hashAt3000_ = 0;
};

}  // namespace

int main() {
    static LockstepDemo demo;
    ga::DemoConfig config;
    config.title = "Lockstep: float vs fixed";
    config.width = kWidth;
    config.height = kHeight;
    config.stepsPerSecond = 120.0f;
    return ga::runDemo(demo, config);
}
