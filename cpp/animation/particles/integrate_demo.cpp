// 粒子的积分与帧率
//   - 同一个初速度，三种积分方法按 simulated fps 推进：显式欧拉、半隐式欧拉、精确解
//   - 灰色细线是精确的轨迹（与帧率无关），彩色点是各方法在每一帧的位置
//   - 拉低帧率：显式欧拉飞得更高，半隐式飞得更低（无阻力时误差大小相同、方向相反）；加上阻力，半隐式的误差大得多

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "demo.h"
#include "particles.h"
#include "ui.h"

namespace {

using namespace ga;
using namespace ga::particles;

constexpr int kWidth = 800, kHeight = 480;
constexpr float kTop = 80;
const Vec2 kStart{80, 440};
const Vec2 kV0{260, -560};
const Vec2 kG{0, 600};

class IntegrateDemo final : public Demo {
public:
    IntegrateDemo() { reset(); }
    void reset() override { t_ = 0; }
    bool step() override {
        t_ += 1 / 60.0f;
        if (t_ > 2.2f) t_ = 0;
        return true;
    }

    void draw() const override {
        const Palette& pal = palette();
        const float hl = hairline();
        DrawLineEx({0, kStart.y}, {float(kWidth), kStart.y}, hl, pal.gridLine);
        // 精确轨迹
        Vec2 prev = kStart;
        for (int i = 1; i <= 300; ++i) {
            Vec2 p = positionAt(kStart, kV0, kG, drag_, i * 2.2f / 300);
            DrawLineEx({prev.x, prev.y}, {p.x, p.y}, 1.0f, Fade(pal.textMuted, 0.6f));
            prev = p;
        }
        const Color cols[3] = {pal.path, Color{60, 130, 220, 255}, pal.text};
        const char* names[3] = {"explicit Euler", "semi-implicit Euler", "exact"};
        float dt = 1 / fps_;
        int frames = int(2.2f / dt);
        for (int m = 0; m < 3; ++m) {
            Vec2 p = kStart, v = kV0;
            float apex = p.y;
            Vec2 last = p;
            for (int f = 0; f < frames; ++f) {
                integrate(p, v, kG, drag_, dt, Integrator(m));
                apex = std::min(apex, p.y);
                bool done = (f + 1) * dt > t_;
                DrawCircleV({p.x, p.y}, done ? 1.6f : 3.0f, done ? Fade(cols[m], 0.35f) : cols[m]);
                if (!done) last = p;
            }
            DrawCircleV({last.x, last.y}, 6, cols[m]);
            char buf[96];
            Vec2 ex = positionAt(kStart, kV0, kG, drag_, 1.0f);
            Vec2 p1 = kStart, v1 = kV0;
            for (int f = 0; f < int(std::lround(fps_)); ++f) integrate(p1, v1, kG, drag_, dt, Integrator(m));
            std::snprintf(buf, sizeof buf, "%-20s apex %6.1f px   error after 1 s %5.1f px", names[m], kStart.y - apex, length(p1 - ex));
            drawText(buf, {12, kTop + 14 + m * 20.0f}, 12, cols[m]);
        }
        DrawLineEx({0, kTop}, {float(kWidth), kTop}, hl, pal.gridLine);
    }

    void drawUi() override {
        ui::slider({12, 10, 260, 26}, "simulated fps", &fps_, 5, 240, "%.0f", 1);
        ui::slider({284, 10, 220, 26}, "drag", &drag_, 0, 3, "%.1f", 0.1f);
    }

    std::string status() const override {
        char buf[64];
        std::snprintf(buf, sizeof buf, "%.0f fps · drag %.1f", fps_, drag_);
        return buf;
    }

private:
    float t_ = 0, fps_ = 12, drag_ = 0;
};

}  // namespace

int main() {
    static IntegrateDemo demo;
    ga::DemoConfig config;
    config.title = "Particle integration";
    config.width = kWidth;
    config.height = kHeight;
    config.stepsPerSecond = 60.0f;
    return ga::runDemo(demo, config);
}
