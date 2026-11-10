// 反向动力学：一条链追一个目标
//   - 拖动（或点击）设置红色目标。每一步执行一趟算法，浅色是上一趟的姿势
//   - CCD 从最末一个关节往回逐个转；FABRIK 先把末端贴到目标往回拉，再把基座放回往前拉；
//     J-transpose、DLS 把所有关节角一起按雅可比矩阵更新
//   - 目标放在接近链的总长处，可以看到所有方法都慢下来；limits 打开后每个关节最多转 60°

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "demo.h"
#include "ik.h"
#include "ui.h"

namespace {

using namespace ga;
using namespace ga::ik;

constexpr int kWidth = 800, kHeight = 480;
constexpr float kTop = 80;
const Vec2 kBase{400, 440};
constexpr float kReach = 330;

Vector2 rv(Vec2 v) { return {v.x, v.y}; }

class IkDemo final : public Demo {
public:
    IkDemo() { rebuild(); }
    void reset() override { sweeps_ = 0, prev_ = chain_; }
    bool step() override {
        float err = length(chain_.end() - target_);
        if (err < 0.05f || sweeps_ >= 2000) return false;
        prev_ = chain_;
        sweep(chain_, target_, method(), 0.1f * kReach);
        ++sweeps_;
        return length(chain_.end() - target_) >= 0.05f;
    }
    void update() override {
        Vector2 m = GetMousePosition();
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT) && !ui::wantsMouse() && m.y > kTop + 4) {
            Vec2 t{m.x, m.y};
            if (length(t - target_) > 0.5f) target_ = t, notifyProblemChanged();
        }
    }

    void draw() const override {
        const Palette& pal = palette();
        const float hl = hairline();
        // 可达范围
        DrawCircleLinesV(rv(kBase), kReach, Fade(pal.gridLine, 0.9f));
        drawChain(prev_, Fade(pal.textMuted, 0.35f), 3);
        drawChain(chain_, pal.text, 5);
        DrawCircleV(rv(target_), 7, pal.path);
        DrawCircleLinesV(rv(target_), 12, pal.path);
        DrawRectangleRec({kBase.x - 20, kBase.y, 40, 8}, pal.text);

        char buf[96];
        float y = kTop + 12;
        auto line = [&](const char* s, Color c) {
            drawText(s, {14, y}, 12, c);
            y += 19;
        };
        std::snprintf(buf, sizeof buf, "sweeps %d", sweeps_);
        line(buf, pal.text);
        std::snprintf(buf, sizeof buf, "error %.2f px", length(chain_.end() - target_));
        line(buf, pal.text);
        std::snprintf(buf, sizeof buf, "target %.0f%% of reach", 100 * length(target_ - kBase) / kReach);
        line(buf, pal.text);
        float bend = 0;
        auto a = relativeAngles(chain_);
        for (size_t i = 1; i < a.size(); ++i) bend += std::fabs(a[i]);
        std::snprintf(buf, sizeof buf, "total bend %.0f deg", bend * 180 / M_PI);
        line(buf, pal.text);
        line("drag to move the target", pal.textMuted);
        DrawLineEx({0, kTop}, {float(kWidth), kTop}, hl, pal.gridLine);
    }

    void drawUi() override {
        const char* names[4] = {"CCD", "FABRIK", "J-transpose", "DLS"};
        float x = 12;
        for (int i = 0; i < 4; ++i) {
            float w = i == 2 ? 106 : 72;
            if (ui::button({x, 10, w, 26}, names[i], method_ == i)) method_ = i, notifyProblemChanged();
            x += w + 4;
        }
        if (ui::slider({x + 10, 10, 200, 26}, "bones", &bones_, 2, 20, "%.0f", 1)) rebuild(), notifyProblemChanged();
        if (ui::toggle({12, 44, 100, 26}, "limits", &limits_)) chain_.limit = limits_ ? 60 * float(M_PI) / 180 : 0, rebuild(), notifyProblemChanged();
        if (ui::button({120, 44, 100, 26}, "straighten")) rebuild(), notifyProblemChanged();
        if (ui::button({228, 44, 110, 26}, "near reach")) target_ = kBase + normalized(target_ - kBase) * (kReach * 0.995f), notifyProblemChanged();
    }

    std::string status() const override {
        char buf[96];
        std::snprintf(buf, sizeof buf, "%d sweeps · error %.2f px", sweeps_, length(chain_.end() - target_));
        return buf;
    }

private:
    Method method() const { return Method(method_); }
    void rebuild() {
        int n = int(bones_);
        chain_ = straightChain(kBase, std::vector<float>(size_t(n), kReach / n), {0, -1});
        chain_.limit = limits_ ? 60 * float(M_PI) / 180 : 0;
        // 稍微弯一点，避免完全伸直时的奇异姿势
        std::vector<float> a(size_t(n), 0.15f);
        a[0] = 0;
        applyAngles(chain_, a);
        reset();
    }
    void drawChain(const Chain& c, Color col, float w) const {
        for (int i = 0; i < c.bones(); ++i) DrawLineEx(rv(c.p[size_t(i)]), rv(c.p[size_t(i) + 1]), w, col);
        for (size_t i = 0; i < c.p.size(); ++i) DrawCircleV(rv(c.p[i]), w * 0.9f, col);
    }

    Chain chain_, prev_;
    Vec2 target_{560, 220};
    float bones_ = 6;
    int method_ = 1, sweeps_ = 0;
    bool limits_ = false;
};

}  // namespace

int main() {
    static IkDemo demo;
    ga::DemoConfig config;
    config.title = "Inverse kinematics";
    config.width = kWidth;
    config.height = kHeight;
    config.stepsPerSecond = 6.0f;
    return ga::runDemo(demo, config);
}
