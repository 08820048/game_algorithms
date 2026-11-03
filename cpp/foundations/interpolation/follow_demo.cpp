// 平滑跟随演示：三种追随方式在不同帧率下的表现
//   红  naive lerp：每帧 lerp(pos, target, 0.1)，跟随速度随帧率变化
//   黑  exp decay ：按真实时间指数衰减，与帧率无关（60fps 时与红点完全重合）
//   黄  smoothDamp：临界阻尼弹簧，起步和停下更柔和
// 按住鼠标可以让目标跟随鼠标；松开后目标沿固定轨迹运动。

#include <cmath>
#include <cstdio>
#include <deque>
#include <string>

#include "demo.h"
#include "interpolation.h"
#include "ui.h"

namespace {

using namespace ga;

constexpr int kWidth = 800;
constexpr int kHeight = 440;
constexpr float kTop = 52;
constexpr float kNaiveFactor = 0.1f;
// 选择 decay 使得 60fps 下与朴素写法完全一致：(1 - 0.1) = e^(-decay / 60)
const float kDecay = -std::log(1.0f - kNaiveFactor) * 60.0f;
constexpr float kSmoothTime = 0.22f;
constexpr size_t kTrail = 36;

struct Follower {
    Vec2 pos;
    Vec2 vel;
    std::deque<Vec2> trail;
    float lag = 0;  // 与目标距离的滑动平均
};

class FollowDemo final : public Demo {
public:
    FollowDemo() { reset(); }

    void reset() override {
        clock_ = 0;
        target_ = path(0);
        for (Follower& f : followers_) f = {target_, {}, {}, 0};
    }

    bool step() override {
        const float dt = 1.0f / fps_;
        clock_ += dt;
        target_ = mouseHeld_ ? mouse_ : path(clock_);

        Follower& naive = followers_[0];
        Follower& decay = followers_[1];
        Follower& damp = followers_[2];
        naive.pos = naiveFollow(naive.pos, target_, kNaiveFactor);
        decay.pos = expDecay(decay.pos, target_, kDecay, dt);
        damp.pos = smoothDamp(damp.pos, target_, damp.vel, kSmoothTime, dt);

        for (Follower& f : followers_) {
            f.trail.push_back(f.pos);
            if (f.trail.size() > kTrail) f.trail.pop_front();
            f.lag = lerp(f.lag, distance(f.pos, target_), 0.05f);
        }
        return true;
    }

    void update() override {
        Vector2 m = GetMousePosition();
        mouseHeld_ = IsMouseButtonDown(MOUSE_BUTTON_LEFT) && !ui::wantsMouse() && m.y > kTop;
        if (mouseHeld_) mouse_ = {m.x, m.y};
    }

    void draw() const override {
        const Palette& p = palette();
        const float hl = hairline();
        DrawLineEx({0, kTop}, {float(kWidth), kTop}, hl, p.gridLine);

        // 目标轨迹（参考）
        Vector2 prev{};
        for (int i = 0; i <= 240; ++i) {
            Vec2 q = path(i / 240.0f * 2 * kPi / 0.9f);  // 一个完整周期
            Vector2 cur{q.x, q.y};
            if (i > 0) DrawLineEx(prev, cur, hl, p.gridLine);
            prev = cur;
        }

        const Color colors[3] = {p.goal, p.start, p.frontier};
        // 绘制顺序：smoothDamp、naive、exp decay（最小，画在最上面，60fps 时能看出它与 naive 重合）
        for (int i : {2, 0, 1}) {
            const Follower& f = followers_[i];
            size_t n = f.trail.size();
            for (size_t k = 0; k < n; ++k) {
                float t = float(k + 1) / float(n);
                DrawCircleV({f.trail[k].x, f.trail[k].y}, 2, Fade(colors[i], 0.1f + 0.4f * t));
            }
            DrawCircleV({f.pos.x, f.pos.y}, i == 1 ? 4.5f : 8.5f, colors[i]);
        }
        // 目标：空心圆
        DrawRing({target_.x, target_.y}, 9, 11, 0, 360, 32, p.text);

        // 图例
        const char* names[3] = {"naive lerp 0.1/frame", "exp decay", "smoothDamp"};
        float y = kHeight - 74;
        for (int i = 0; i < 3; ++i) {
            DrawCircleV({28, y + 6}, 5, colors[i]);
            drawText(names[i], {42, y}, 11, p.text);
            char buf[32];
            std::snprintf(buf, sizeof buf, "lag %5.1f", followers_[i].lag);
            drawText(buf, {220, y}, 11, p.textMuted);
            y += 20;
        }
    }

    void drawUi() override {
        if (ui::slider({16, 13, 250, 26}, "fps", &fps_, 15, 144, "%.0f", 1)) setStepsPerSecond(fps_);
        const Palette& p = palette();
        drawText("hold mouse to move the target", {kWidth - 236.0f, 20}, 11, p.textMuted);
    }

    std::string status() const override {
        char buf[96];
        std::snprintf(buf, sizeof buf, "fps %.0f · lag naive %.0f · decay %.0f · damp %.0f", fps_,
                      followers_[0].lag, followers_[1].lag, followers_[2].lag);
        return buf;
    }

    float fps() const { return fps_; }

private:
    // 目标的固定轨迹：一条利萨如曲线
    static Vec2 path(float t) {
        return {400 + 300 * std::sin(0.9f * t), 240 + 110 * std::sin(1.8f * t + 0.6f)};
    }

    Follower followers_[3];
    Vec2 target_;
    Vec2 mouse_;
    bool mouseHeld_ = false;
    float clock_ = 0;
    float fps_ = 60;
};

}  // namespace

int main() {
    static FollowDemo demo;
    ga::DemoConfig config;
    config.title = "Smooth Follow";
    config.width = kWidth;
    config.height = kHeight;
    config.stepsPerSecond = demo.fps();
    return ga::runDemo(demo, config);
}
