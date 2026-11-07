// 转向行为演示：一个角色，七种行为
//   - seek / flee / arrive：目标是十字准星，点击或拖动场地移动它
//   - pursue / evade：红点自己在场地里漫游（pursue）或追赶角色（evade）
//   - wander：前方的小圆和圆上的目标点
//   - avoid：穿过障碍场去目标点，到了就换到另一边。priority 切换按优先级累加和加权相加
//   - reaction：反应时间。调大后，抵达会冲过头，转弯变得迟钝
//   黑色箭头是速度，虚线是想要的速度，红色箭头是转向力

#include <cmath>
#include <cstdio>
#include <deque>
#include <string>

#include "demo.h"
#include "steering.h"
#include "ui.h"

namespace {

using namespace ga;
using namespace ga::steering;

constexpr int kWidth = 800;
constexpr int kHeight = 500;
constexpr float kTop = 52;
constexpr Rectangle kArena{16, kTop + 12, kWidth - 32, kHeight - kTop - 28};
constexpr float kDt = 1.0f / 60;

enum Mode { Seek, Flee, Arrive, Pursue, Evade, WanderMode, Avoid, kModeCount };
const char* kModeNames[] = {"seek", "flee", "arrive", "pursue", "evade", "wander", "avoid"};

inline Vector2 rl(Vec2 v) { return {v.x, v.y}; }

// 离场地边缘太近时，把角色推回来（优先级最高）
Vec2 containment(const Agent& a, float margin) {
    Vec2 push{};
    if (a.pos.x < kArena.x + margin) push.x += 1;
    if (a.pos.x > kArena.x + kArena.width - margin) push.x -= 1;
    if (a.pos.y < kArena.y + margin) push.y += 1;
    if (a.pos.y > kArena.y + kArena.height - margin) push.y -= 1;
    if (lengthSq(push) == 0) return {};
    return steerToward(a, normalized(push) * a.maxSpeed);
}

class SteeringDemo final : public Demo {
public:
    SteeringDemo() {
        // 障碍：固定的一组圆，留出足够宽的缝
        Pcg32 rng(8);
        while (obstacles_.size() < 11) {
            Circle c{{200 + rng.nextFloat() * 400, kArena.y + 40 + rng.nextFloat() * (kArena.height - 80)}, 18 + rng.nextFloat() * 26};
            bool ok = true;
            for (const Circle& o : obstacles_) ok &= distance(o.c, c.c) > o.r + c.r + 34;
            if (ok) obstacles_.push_back(c);
        }
        reset();
    }

    void reset() override {
        agent_ = Agent{};
        agent_.pos = {150, 300}, agent_.vel = {60, -20};
        other_ = Agent{};
        other_.pos = {560, 200}, other_.vel = {-40, 30};
        other_.maxSpeed = mode_ == Evade ? 95 : 70;
        target_ = mode_ == Avoid ? Vec2{720, 260} : Vec2{460, 280};
        wander_ = Wander{};
        otherWander_ = Wander{};
        rng_ = Pcg32(4);
        trail_.clear();
        hits_ = 0, touching_ = false;
    }

    bool step() override {
        agent_.reaction = reaction_;
        Vec2 desired{};
        Vec2 f{};
        // 红点：pursue 模式下漫游，evade 模式下追赶角色
        if (mode_ == Pursue || mode_ == Evade) {
            Vec2 g = mode_ == Pursue ? wander(other_, otherWander_, kDt, rng_) : pursue(other_, agent_.pos, agent_.vel);
            Vec2 total{};
            accumulate(total, containment(other_, 50), other_.maxForce);
            accumulate(total, g, other_.maxForce);
            integrate(other_, total, kDt);
        }
        Vec2 behavior{};
        switch (mode_) {
        case Seek: behavior = seek(agent_, target_); break;
        case Flee: behavior = flee(agent_, target_, 200); break;
        case Arrive: behavior = arrive(agent_, target_, kSlowRadius); break;
        case Pursue: behavior = pursue(agent_, other_.pos, other_.vel); break;
        case Evade: behavior = evade(agent_, other_.pos, other_.vel, 220); break;
        case WanderMode: behavior = wander(agent_, wander_, kDt, rng_); break;
        case Avoid: {
            if (distance(agent_.pos, target_) < 12)  // 到了：换到场地另一边
                target_ = {target_.x > 400 ? 80.0f : 720.0f, kArena.y + 40 + rng_.nextFloat() * (kArena.height - 80)};
            avoidForce_ = avoidObstacles(agent_, obstacles_, 0.3f, &hit_);
            Vec2 go = arrive(agent_, target_, 60);
            if (priority_) {
                accumulate(f, containment(agent_, 30), agent_.maxForce);
                accumulate(f, avoidForce_, agent_.maxForce);
                accumulate(f, go, agent_.maxForce);
            } else {
                f = weightedSum({{containment(agent_, 30), 1}, {avoidForce_, 1}, {go, 1}}, agent_.maxForce);
            }
            behavior = go;
            break;
        }
        default: break;
        }
        if (mode_ != Avoid) {
            accumulate(f, containment(agent_, 40), agent_.maxForce);
            accumulate(f, behavior, agent_.maxForce);
        }
        desired = behavior * agent_.reaction + agent_.vel;  // 反推出这个行为"想要的速度"，用于绘制
        desired_ = desired;
        force_ = truncate(f, agent_.maxForce);
        integrate(agent_, f, kDt);
        if (mode_ == Avoid) {  // 统计撞上障碍的次数（进入障碍算一次）
            bool touching = false;
            for (const Circle& o : obstacles_) touching |= distance(agent_.pos, o.c) < o.r + agent_.radius * 0.5f;
            if (touching && !touching_) ++hits_;
            touching_ = touching;
        }
        trail_.push_back(agent_.pos);
        if (trail_.size() > 150) trail_.pop_front();
        return true;
    }

    void update() override {
        Vector2 m = GetMousePosition();
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT) && !ui::wantsMouse() && CheckCollisionPointRec(m, kArena) && usesTarget())
            target_ = {m.x, m.y};
    }

    void draw() const override {
        const Palette& pal = palette();
        const float hl = hairline();
        DrawRectangleLinesEx(kArena, hl * 1.5f, pal.textMuted);
        if (mode_ == Avoid)
            for (const Circle& o : obstacles_) {
                DrawCircleV(rl(o.c), o.r, Fade(pal.wall, &o == hit_ ? 0.55f : 0.3f));
                DrawCircleLinesV(rl(o.c), o.r, Fade(pal.wall, 0.8f));
            }
        // 轨迹
        for (size_t i = 0; i < trail_.size(); i += 3)
            DrawCircleV(rl(trail_[i]), 1.6f, Fade(pal.textMuted, 0.15f + 0.6f * i / trail_.size()));
        // 目标
        if (usesTarget()) {
            Vector2 t = rl(target_);
            Color c = mode_ == Flee ? pal.path : pal.text;
            DrawLineEx({t.x - 9, t.y}, {t.x + 9, t.y}, 1.5f, c);
            DrawLineEx({t.x, t.y - 9}, {t.x, t.y + 9}, 1.5f, c);
            DrawCircleLinesV(t, 5, c);
            if (mode_ == Arrive) {
                DrawCircleLinesV(t, kSlowRadius, Fade(pal.textMuted, 0.5f));
                drawTextCentered("slow radius", {t.x, t.y - kSlowRadius - 9}, 10, pal.textMuted);
            }
            if (mode_ == Flee) DrawCircleLinesV(t, 200, Fade(pal.path, 0.3f));
        }
        if (mode_ == Pursue || mode_ == Evade) {
            DrawCircleV(rl(other_.pos), 8, pal.path);
            const Agent& chaser = mode_ == Pursue ? agent_ : other_;
            Vec2 qPos = mode_ == Pursue ? other_.pos : agent_.pos, qVel = mode_ == Pursue ? other_.vel : agent_.vel;
            Vec2 predicted = predictPosition(chaser, qPos, qVel, 2);
            drawDashedLine(rl(qPos), rl(predicted), 1.2f, Fade(pal.path, 0.7f), 4, 3);
            DrawCircleLinesV(rl(predicted), 5, pal.path);
            drawTextCentered("predicted", {predicted.x, predicted.y - 13}, 10, pal.textMuted);
            if (mode_ == Evade) DrawCircleLinesV(rl(other_.pos), 220, Fade(pal.path, 0.25f));
        }
        if (mode_ == WanderMode) {
            Vec2 fwd = normalized(agent_.vel);
            Vec2 center = agent_.pos + fwd * wander_.distance;
            DrawCircleLinesV(rl(center), wander_.radius, Fade(pal.textMuted, 0.6f));
            DrawLineEx(rl(agent_.pos), rl(center), hl, Fade(pal.textMuted, 0.6f));
            Vec2 t = wanderTarget(agent_, wander_);
            DrawCircleV(rl(t), 4, pal.path);
        }
        if (mode_ == Avoid) {  // 探测盒
            float speed = length(agent_.vel);
            if (speed > 1e-3f) {
                Vec2 fwd = agent_.vel / speed, side = perp(fwd);
                float len = agent_.radius + speed * 0.3f, r = agent_.radius;
                Vec2 a = agent_.pos + side * r, b = agent_.pos - side * r;
                Color c = Fade(hit_ ? pal.path : pal.textMuted, 0.8f);
                DrawLineEx(rl(a), rl(a + fwd * len), hl * 1.5f, c);
                DrawLineEx(rl(b), rl(b + fwd * len), hl * 1.5f, c);
                DrawLineEx(rl(a + fwd * len), rl(b + fwd * len), hl * 1.5f, c);
            }
        }
        drawAgent();
        drawLegend();
    }

    void drawUi() override {
        for (int m = 0; m < kModeCount; ++m)
            if (ui::button({16 + m * 62.0f, 13, 58, 26}, kModeNames[m], mode_ == m)) {
                bool changed = mode_ != m;
                mode_ = Mode(m);
                if (changed && (m == Avoid || m == Evade || m == Pursue)) reset();
                trail_.clear();
            }
        ui::slider({456, 13, 210, 26}, "reaction", &reaction_, 0.05f, 1.0f, "%.2fs", 0.05f);
        if (mode_ == Avoid) ui::toggle({676, 13, 108, 26}, "priority", &priority_);
    }

    std::string status() const override {
        char buf[128];
        int n = std::snprintf(buf, sizeof buf, "%s · speed %.0f", kModeNames[mode_], length(agent_.vel));
        if (usesTarget()) n += std::snprintf(buf + n, sizeof buf - n, " · %.1f px to target", distance(agent_.pos, target_));
        if (mode_ == Avoid) std::snprintf(buf + n, sizeof buf - n, " · %s · %d hits", priority_ ? "priority" : "weighted sum", hits_);
        return buf;
    }

private:
    static constexpr float kSlowRadius = 100;

    bool usesTarget() const { return mode_ == Seek || mode_ == Flee || mode_ == Arrive || mode_ == Avoid; }

    void drawAgent() const {
        const Palette& pal = palette();
        Vec2 fwd = lengthSq(agent_.vel) > 1e-4f ? normalized(agent_.vel) : Vec2{1, 0}, side = perp(fwd);
        Vec2 p = agent_.pos;
        DrawTriangle(rl(p + fwd * 12), rl(p - fwd * 7 - side * 7), rl(p - fwd * 7 + side * 7), pal.text);
        DrawTriangle(rl(p + fwd * 12), rl(p - fwd * 7 + side * 7), rl(p - fwd * 7 - side * 7), pal.text);  // 两种绕序都画，避免被剔除
        // 速度、想要的速度、转向力（力按 0.15 秒的效果缩放）
        if (lengthSq(agent_.vel) > 1) drawArrow(rl(p), rl(p + agent_.vel * 0.6f), 1.8f, pal.text, 7);
        if (lengthSq(desired_) > 1) drawDashedLine(rl(p), rl(p + desired_ * 0.6f), 1.4f, pal.frontier, 5, 4);
        if (lengthSq(force_) > 1) drawArrow(rl(p), rl(p + force_ * 0.15f), 1.8f, pal.path, 7);
    }

    void drawLegend() const {
        const Palette& pal = palette();
        float x = kArena.x + 10, y = kArena.y + kArena.height - 18;
        DrawLineEx({x, y + 6}, {x + 20, y + 6}, 1.8f, pal.text);
        drawText("velocity", {x + 26, y}, 10, pal.textMuted);
        drawDashedLine({x + 90, y + 6}, {x + 110, y + 6}, 1.4f, pal.frontier, 5, 4);
        drawText("desired", {x + 116, y}, 10, pal.textMuted);
        DrawLineEx({x + 174, y + 6}, {x + 194, y + 6}, 1.8f, pal.path);
        drawText("steering force", {x + 200, y}, 10, pal.textMuted);
        const char* hint = mode_ == Avoid ? "click to move the goal" : usesTarget() ? "click or drag to move the target" : "";
        drawText(hint, {kArena.x + kArena.width - measureText(hint, 10).x - 10, y}, 10, pal.textMuted);
    }

    Agent agent_, other_;
    Wander wander_, otherWander_;
    Vec2 target_{460, 280}, desired_, force_, avoidForce_;
    const Circle* hit_ = nullptr;
    std::vector<Circle> obstacles_;
    std::deque<Vec2> trail_;
    Pcg32 rng_{4};
    Mode mode_ = Arrive;
    float reaction_ = 0.1f;
    bool priority_ = true, touching_ = false;
    int hits_ = 0;
};

}  // namespace

int main() {
    static SteeringDemo demo;
    ga::DemoConfig config;
    config.title = "Steering behaviors";
    config.width = kWidth;
    config.height = kHeight;
    config.stepsPerSecond = 60.0f;
    return ga::runDemo(demo, config);
}
