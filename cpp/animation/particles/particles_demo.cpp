// 粒子发射器
//   - fountain / fire / comet 三种效果；拖动画布移动发射器，orbit 让它绕圈快速移动
//   - simulated fps：模拟按这个帧率推进（画面仍按原帧率绘制）
//   - truncate：每帧发射 int(rate·dt) 个；accumulate：小数部分留到下一帧；subframe：再按真实的出生时刻补走、发射器位置插值
//     在 orbit + 低帧率下对比 accumulate 和 subframe：前者的拖尾一团一团，后者均匀

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
const Vec2 kCentre{400, 290};

struct Effect {
    float rate, lifeLo, lifeHi, speedLo, speedHi, spread, drag, radius;
    Vec2 dir, gravity;
};

class ParticlesDemo final : public Demo {
public:
    ParticlesDemo() { reset(); }
    void reset() override {
        pool_.clear();
        emitter_.reset();
        time_ = 0, acc_ = 0, simTime_ = 0, next_ = 0, emittedWindow_ = 0, windowTime_ = 0, shownRate_ = 0;
        emitterPos_ = orbit_ ? orbitAt(0) : target_;
    }
    bool step() override {
        const float frame = 1 / 60.0f;
        acc_ += frame;
        float dt = 1 / fps_;
        const Effect& e = effect();
        emitter_.rate = e.rate;
        emitter_.mode = EmitMode(mode_);
        while (acc_ >= dt) {
            acc_ -= dt;
            simTime_ += dt;
            Vec2 from = emitterPos_;
            emitterPos_ = orbit_ ? orbitAt(simTime_) : target_;
            pool_.update(dt, e.gravity, e.drag, Integrator::Exact);
            int n = emitter_.emit(dt, from, emitterPos_, [&](Vec2 p, float age) {
                Vec2 v = cone(rng_, e.dir, e.spread, e.speedLo, e.speedHi);
                Vec2 q = p + (e.radius > 0 ? uniformDisk(rng_, e.radius) : Vec2{0, 0});
                float life = e.lifeLo + (e.lifeHi - e.lifeLo) * rng_.nextFloat();
                Vec2 vv = v;
                integrate(q, vv, e.gravity, e.drag, age, Integrator::Exact);  // 补走已经存在的时间
                pool_.add(q, vv, life, next_++, age);
            });
            emittedWindow_ += n;
            windowTime_ += dt;
            if (windowTime_ >= 1) shownRate_ = float(emittedWindow_) / windowTime_, emittedWindow_ = 0, windowTime_ = 0;
        }
        return true;
    }
    void update() override {
        Vector2 m = GetMousePosition();
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT) && !ui::wantsMouse() && m.y > kTop + 4) target_ = {m.x, m.y}, orbit_ = false;
    }

    void draw() const override {
        const Palette& pal = palette();
        const float hl = hairline();
        bool dark = pal.background.r < 128;
        for (size_t i = 0; i < pool_.size(); ++i) {
            float t = std::min(1.0f, pool_.age[i] / pool_.life[i]);
            Color c;
            float size, alpha;
            if (kind_ == 0) {  // 喷泉：蓝色水滴，慢慢变透明
                c = ColorLerp(Color{70, 140, 230, 255}, pal.text, 0.15f);
                size = evaluate({{0, 2.5f}, {1, 1.5f}}, t), alpha = evaluate({{0, 0.9f}, {0.7f, 0.8f}, {1, 0}}, t);
            } else if (kind_ == 1) {  // 火：黄 → 橙红 → 灰烟
                Color a{255, 220, 90, 255}, b{230, 80, 30, 255}, smoke = dark ? Color{140, 140, 140, 255} : Color{120, 120, 120, 255};
                c = t < 0.4f ? ColorLerp(a, b, t / 0.4f) : ColorLerp(b, smoke, std::min(1.0f, (t - 0.4f) / 0.4f));
                size = evaluate({{0, 3}, {0.4f, 5}, {1, 9}}, t), alpha = evaluate({{0, 0.9f}, {0.5f, 0.6f}, {1, 0}}, t);
            } else {  // 彗星：墨色 → 朱红，变小
                c = ColorLerp(pal.text, pal.path, t);
                size = evaluate({{0, 3}, {1, 0.8f}}, t), alpha = evaluate({{0, 1}, {1, 0}}, t);
            }
            DrawCircleV({pool_.pos[i].x, pool_.pos[i].y}, size, Fade(c, alpha));
        }
        DrawCircleLinesV({emitterPos_.x, emitterPos_.y}, 6, pal.textMuted);
        char buf[128];
        std::snprintf(buf, sizeof buf, "particles %zu   emitted %.0f / s (rate %.0f)   sim %.0f fps", pool_.size(), shownRate_, effect().rate, fps_);
        drawText(buf, {12, kTop + 8}, 11, pal.textMuted);
        DrawLineEx({0, kTop}, {float(kWidth), kTop}, hl, pal.gridLine);
    }

    void drawUi() override {
        const char* kinds[3] = {"fountain", "fire", "comet"};
        bool ch = false;
        for (int i = 0; i < 3; ++i)
            if (ui::button({12.0f + i * 84, 10, 80, 26}, kinds[i], kind_ == i)) kind_ = i, ch = true;
        ch |= ui::toggle({268, 10, 80, 26}, "orbit", &orbit_);
        ui::slider({360, 10, 230, 26}, "simulated fps", &fps_, 10, 240, "%.0f", 1);
        const char* modes[4] = {"truncate", "round", "accumulate", "subframe"};
        float x = 12;
        for (int i = 0; i < 4; ++i) {
            float w = i == 2 ? 100 : 84;
            if (ui::button({x, 44, w, 26}, modes[i], mode_ == i)) mode_ = i, ch = true;
            x += w + 4;
        }
        if (ch) notifyProblemChanged();
    }

    std::string status() const override {
        char buf[96];
        std::snprintf(buf, sizeof buf, "%zu particles · %.0f emitted/s", pool_.size(), shownRate_);
        return buf;
    }

private:
    const Effect& effect() const {
        static const Effect fx[3] = {
            {150, 1.6f, 2.2f, 380, 470, 0.18f, 0.3f, 0, {0, -1}, {0, 600}},       // 喷泉
            {220, 0.7f, 1.3f, 40, 110, 0.5f, 1.2f, 10, {0, -1}, {0, -160}},       // 火：浮力向上
            {300, 0.6f, 0.9f, 5, 30, 3.14159f, 2.0f, 0, {1, 0}, {0, 0}},          // 彗星拖尾
        };
        return fx[kind_];
    }
    Vec2 orbitAt(float t) const { return kCentre + Vec2{std::cos(t * 4 * kPi), std::sin(t * 4 * kPi)} * 130; }

    Pool pool_;
    Emitter emitter_;
    Pcg32 rng_{3};
    Vec2 target_ = kCentre + Vec2{0, 120}, emitterPos_ = kCentre;
    float time_ = 0, acc_ = 0, simTime_ = 0, fps_ = 30, windowTime_ = 0, shownRate_ = 0;
    uint32_t next_ = 0;
    int emittedWindow_ = 0, kind_ = 2, mode_ = 2;
    bool orbit_ = true;
};

}  // namespace

int main() {
    static ParticlesDemo demo;
    ga::DemoConfig config;
    config.title = "Particle emitter";
    config.width = kWidth;
    config.height = kHeight;
    config.stepsPerSecond = 60.0f;
    return ga::runDemo(demo, config);
}
