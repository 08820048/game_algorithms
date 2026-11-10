// 反向动力学：四种方法追同一个移动的目标
//   - 每帧每种方法只能做 budget 趟，从上一帧的姿势继续
//   - circle：目标匀速绕圈；jump：目标每隔一秒半跳到一个新位置
//   - 每个面板下方：当前误差、这一帧关节转动的最大角度（大于 20° 记一次"跳变"）

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "../../foundations/random/random.h"
#include "demo.h"
#include "ik.h"
#include "ui.h"

namespace {

using namespace ga;
using namespace ga::ik;

constexpr int kWidth = 800, kHeight = 480;
constexpr float kTop = 80;
constexpr int kPanels = 4;
constexpr float kPw = 200, kScale = 200;

Vector2 rv(Vec2 v) { return {v.x, v.y}; }

class CompareDemo final : public Demo {
public:
    CompareDemo() { reset(); }
    void reset() override {
        frame_ = 0;
        rng_ = Pcg32(7);
        jumpTarget_ = {0.2f, 0.6f};
        for (int k = 0; k < kPanels; ++k) {
            chains_[k] = straightChain({0, 0}, std::vector<float>(8, 1.0f / 8), {0, 1});
            chains_[k].limit = limits_ ? 60 * float(M_PI) / 180 : 0;
            std::vector<float> a(8, 0.12f);
            a[0] = 0;
            applyAngles(chains_[k], a);
            pops_[k] = 0, jump_[k] = 0;
            trail_[k].clear();
        }
    }
    bool step() override {
        ++frame_;
        Vec2 t = target();
        for (int k = 0; k < kPanels; ++k) {
            auto before = relativeAngles(chains_[k]);
            for (int b = 0; b < int(budget_); ++b) sweep(chains_[k], t, Method(k), 0.1f);
            auto after = relativeAngles(chains_[k]);
            float j = 0;
            for (size_t i = 0; i < before.size(); ++i) {
                float d = std::fabs(after[i] - before[i]);
                j = std::max(j, std::min(d, 6.2832f - d));
            }
            jump_[k] = j * 180 / float(M_PI);
            if (jump_[k] > 20 && frame_ > 30) ++pops_[k], flash_[k] = 20;  // 开头 30 帧是从初始姿势调整过来，不计
            if (flash_[k] > 0) --flash_[k];
            trail_[k].push_back(chains_[k].end());
            if (trail_[k].size() > 90) trail_[k].erase(trail_[k].begin());
        }
        return true;
    }

    void draw() const override {
        const Palette& pal = palette();
        const float hl = hairline();
        const char* names[4] = {"CCD", "FABRIK", "J-transpose", "DLS"};
        Vec2 t = target();
        for (int k = 0; k < kPanels; ++k) {
            float x0 = k * kPw;
            if (k) DrawLineEx({x0, kTop}, {x0, float(kHeight)}, hl, pal.gridLine);
            drawText(names[k], {x0 + 10, kTop + 8}, 13, pal.text);
            const Chain& c = chains_[k];
            for (size_t i = 1; i < trail_[k].size(); ++i)
                DrawLineEx(rv(map(k, trail_[k][i - 1])), rv(map(k, trail_[k][i])), 1.2f, Fade(pal.path, 0.15f + 0.5f * float(i) / float(trail_[k].size())));
            Color col = flash_[k] > 0 ? pal.path : pal.text;
            for (int i = 0; i < c.bones(); ++i) DrawLineEx(rv(map(k, c.p[size_t(i)])), rv(map(k, c.p[size_t(i) + 1])), 4, col);
            for (Vec2 p : c.p) DrawCircleV(rv(map(k, p)), 3.5f, col);
            Vec2 tp = map(k, t);
            DrawCircleLinesV(rv(tp), 7, pal.path);
            DrawRectangleRec({x0 + kPw / 2 - 16, kHeight - 70.0f, 32, 6}, pal.text);
            char buf[64];
            std::snprintf(buf, sizeof buf, "error %5.1f px", length(c.end() - t) * kScale);
            drawText(buf, {x0 + 10, kHeight - 50.0f}, 11, pal.textMuted);
            std::snprintf(buf, sizeof buf, "max turn %5.1f deg", jump_[k]);
            drawText(buf, {x0 + 10, kHeight - 35.0f}, 11, jump_[k] > 20 ? pal.path : pal.textMuted);
            std::snprintf(buf, sizeof buf, "jumps > 20 deg: %d", pops_[k]);
            drawText(buf, {x0 + 10, kHeight - 20.0f}, 11, pal.textMuted);
        }
        DrawLineEx({0, kTop}, {float(kWidth), kTop}, hl, pal.gridLine);
    }

    void drawUi() override {
        bool ch = false;
        if (ui::button({12, 10, 80, 26}, "circle", !jump_mode_)) jump_mode_ = false, ch = true;
        if (ui::button({96, 10, 80, 26}, "jump", jump_mode_)) jump_mode_ = true, ch = true;
        ui::slider({190, 10, 200, 26}, "budget", &budget_, 1, 10, "%.0f", 1);
        ch |= ui::toggle({404, 10, 90, 26}, "limits", &limits_);
        if (ui::button({502, 10, 80, 26}, "restart")) ch = true;
        if (ch) notifyProblemChanged();
    }

    std::string status() const override {
        char buf[96];
        std::snprintf(buf, sizeof buf, "budget %d sweeps/frame · jumps CCD %d FABRIK %d JT %d DLS %d", int(budget_), pops_[0], pops_[1], pops_[2], pops_[3]);
        return buf;
    }

private:
    // 目标（以链长为单位，基座在原点、y 朝上）
    Vec2 target() const {
        if (!jump_mode_) {
            float a = frame_ / 240.0f * 6.2832f;
            return Vec2{0.05f, 0.62f} + Vec2{std::cos(a), std::sin(a)} * 0.25f;
        }
        return jumpTarget_;
    }
    // 链长 1 对应 kScale 像素，基座在面板底部中间
    Vec2 map(int k, Vec2 p) const { return {k * kPw + kPw / 2 + p.x * kScale, kHeight - 70.0f - p.y * kScale}; }

    Chain chains_[kPanels];
    std::vector<Vec2> trail_[kPanels];
    float jump_[kPanels] = {};
    int pops_[kPanels] = {}, flash_[kPanels] = {};
    int frame_ = 0;
    float budget_ = 2;
    bool limits_ = false, jump_mode_ = false;
    Vec2 jumpTarget_{0.4f, 0.6f};
    Pcg32 rng_{7};
    int lastJumpFrame_ = -1;

public:
    void update() override {
        // jump 模式：每 90 帧换一个可达的位置（frame_ 在 step 里推进）
        if (jump_mode_ && frame_ != lastJumpFrame_ && frame_ % 90 == 0) {
            lastJumpFrame_ = frame_;
            float a = 1.0f + rng_.nextFloat() * 1.14f, r = 0.3f + rng_.nextFloat() * 0.65f;  // 不超出面板
            jumpTarget_ = Vec2{std::cos(a), std::sin(a)} * r;
        }
    }
};

}  // namespace

int main() {
    static CompareDemo demo;
    ga::DemoConfig config;
    config.title = "IK methods compared";
    config.width = kWidth;
    config.height = kHeight;
    config.stepsPerSecond = 60.0f;
    return ga::runDemo(demo, config);
}
