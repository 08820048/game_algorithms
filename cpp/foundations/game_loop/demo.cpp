// 游戏循环演示：同一个跳跃，三种循环写法并排对比
//   A 可变步长：物理直接用帧时间推进，跳跃高度随帧率变化
//   B 固定步长：物理以 60Hz 推进，渲染直接用最新状态 —— 帧率不整除 60 时会抖
//   C 固定步长 + 插值：渲染时在上一步和当前步之间插值 —— 稳定又平滑
// 每一"步"代表模拟帧率下的一帧；轨迹上的每个点就是一次渲染。

#include <cmath>
#include <cstdio>
#include <deque>
#include <random>
#include <string>

#include "demo.h"
#include "fixed_timestep.h"
#include "ui.h"

namespace {

using namespace ga;

constexpr int kWidth = 800;
constexpr int kHeight = 500;
constexpr float kTop = 56;                              // 顶部控件区
constexpr float kLaneH = (kHeight - kTop) / 3.0f;       // 每条泳道高度
constexpr float kLaneLeft = 150;                        // 左侧标签宽度
constexpr float kLaneRight = kWidth - 24;
constexpr float kGroundPad = 14;

constexpr Vec2 kGravity{0, 1400};
constexpr Vec2 kLaunch{150, -480};
constexpr double kPhysicsDt = 1.0 / 60.0;
constexpr size_t kTrailLength = 48;

struct Lane {
    const char* name;
    const char* detail;
    Body body;
    Vec2 previous;            // C 用：上一步的位置
    std::deque<Vec2> trail;   // 每次渲染时的位置
    float jumpTop = 0;        // 本次跳跃中达到的最高点（y 最小值）
    float lastPeak = 0;       // 上一次跳跃的最高高度
    int steps = 0;            // 本帧执行的物理步数
};

// 落地后立刻再跳一次，并记录这次跳跃的最高点
void land(Lane& lane) {
    lane.jumpTop = std::min(lane.jumpTop, lane.body.position.y);
    if (lane.body.position.y > 0) {
        lane.lastPeak = -lane.jumpTop;
        lane.jumpTop = 0;
        lane.body.position.y = 0;
        lane.body.velocity.y = kLaunch.y;
    }
}

class GameLoopDemo final : public Demo {
public:
    GameLoopDemo() { reset(); }

    void reset() override {
        lanes_[0] = {"A  variable dt", "physics uses frame time", {}, {}, {}, 0, 0, 0};
        lanes_[1] = {"B  fixed 60 Hz", "render latest state", {}, {}, {}, 0, 0, 0};
        lanes_[2] = {"C  fixed + lerp", "render interpolated", {}, {}, {}, 0, 0, 0};
        for (Lane& l : lanes_) l.body = {{0, 0}, kLaunch};
        fixedB_.reset();
        fixedC_.reset();
        rng_.seed(7);
        frameTime_ = 1.0 / fps_;
    }

    bool step() override {
        frameTime_ = 1.0 / fps_;
        if (jitter_) frameTime_ *= std::uniform_real_distribution<double>(0.4, 1.6)(rng_);

        // A：可变步长
        Lane& a = lanes_[0];
        variableStep(a.body, kGravity, frameTime_);
        land(a);
        a.steps = 1;
        record(a, a.body.position);

        // B：固定步长，渲染最新状态
        Lane& b = lanes_[1];
        b.steps = fixedB_.advance(frameTime_, [&](double dt) {
            integrate(b.body, kGravity, static_cast<float>(dt));
            land(b);
        });
        record(b, b.body.position);

        // C：固定步长 + 插值
        Lane& c = lanes_[2];
        c.steps = fixedC_.advance(frameTime_, [&](double dt) {
            c.previous = c.body.position;
            integrate(c.body, kGravity, static_cast<float>(dt));
            land(c);
        });
        record(c, interpolate(c.previous, c.body.position, fixedC_.alpha()));
        return true;
    }

    void draw() const override {
        const Palette& p = palette();
        const float hl = hairline();
        const float idealPeak = kLaunch.y * kLaunch.y / (2 * kGravity.y);

        DrawLineEx({0, kTop}, {float(kWidth), kTop}, hl, p.gridLine);

        for (int i = 0; i < 3; ++i) {
            const Lane& lane = lanes_[i];
            float top = kTop + kLaneH * i;
            float ground = top + kLaneH - kGroundPad;
            if (i > 0) DrawLineEx({0, top}, {float(kWidth), top}, hl, p.gridLine);

            // 标签
            drawText(lane.name, {16, top + 14}, 12, p.text);
            drawText(lane.detail, {16, top + 32}, 10, p.textMuted);
            char buf[64];
            float overshoot = lane.lastPeak - idealPeak;
            std::snprintf(buf, sizeof buf, "peak %.1f", lane.lastPeak);
            drawText(buf, {16, top + 58}, 11, std::abs(overshoot) > 6 ? p.goal : p.text);
            std::snprintf(buf, sizeof buf, "steps/frame %d", lane.steps);
            drawText(buf, {16, top + 76}, 10, p.textMuted);

            // 地面与理想高度
            DrawLineEx({kLaneLeft, ground}, {kLaneRight, ground}, 1, p.wall);
            drawDashedLine({kLaneLeft, ground - idealPeak}, {kLaneRight, ground - idealPeak}, hl, p.textMuted);
            drawText("ideal", {kLaneRight - 30, ground - idealPeak - 14}, 9, p.textMuted);

            // 轨迹：每个点是一次渲染，越旧越淡
            size_t n = lane.trail.size();
            for (size_t k = 0; k < n; ++k) {
                Vector2 pt = toScreen(lane.trail[k], ground);
                float t = static_cast<float>(k + 1) / static_cast<float>(n);
                DrawCircleV(pt, 2.4f, Fade(p.textMuted, 0.25f + 0.75f * t));
            }
            if (n > 0) DrawCircleV(toScreen(lane.trail.back(), ground), 8, i == 0 ? p.goal : p.start);
        }
    }

    void drawUi() override {
        if (ui::slider({16, 15, 250, 26}, "fps", &fps_, 10, 144, "%.0f", 1)) setStepsPerSecond(fps_);
        ui::toggle({286, 15, 96, 26}, "jitter", &jitter_);

        const Palette& p = palette();
        char buf[64];
        std::snprintf(buf, sizeof buf, "frame %.1f ms   physics %.0f Hz", frameTime_ * 1000.0, 1.0 / kPhysicsDt);
        Vector2 m = measureText(buf, 11);
        drawText(buf, {kWidth - 20 - m.x, 22}, 11, p.textMuted);
    }

    std::string status() const override {
        char buf[64];
        std::snprintf(buf, sizeof buf, "fps %.0f · frame %.1f ms%s", fps_, frameTime_ * 1000.0,
                      jitter_ ? " · jitter" : "");
        return buf;
    }

    float fps() const { return fps_; }

private:
    void record(Lane& lane, Vec2 pos) {
        lane.trail.push_back(pos);
        if (lane.trail.size() > kTrailLength) lane.trail.pop_front();
    }

    // 物理坐标 -> 屏幕坐标：x 在泳道内循环，y 以地面为 0 向上
    static Vector2 toScreen(Vec2 pos, float ground) {
        float span = kLaneRight - kLaneLeft - 16;
        float x = std::fmod(pos.x, span);
        return {kLaneLeft + 8 + x, ground + pos.y};
    }

    Lane lanes_[3];
    FixedTimestep fixedB_{kPhysicsDt};
    FixedTimestep fixedC_{kPhysicsDt};
    std::mt19937 rng_{7};
    float fps_ = 40;
    bool jitter_ = false;
    double frameTime_ = 1.0 / 40.0;
};

}  // namespace

int main() {
    static GameLoopDemo demo;
    ga::DemoConfig config;
    config.title = "Game Loop - Fixed Timestep";
    config.width = kWidth;
    config.height = kHeight;
    config.stepsPerSecond = demo.fps();
    return ga::runDemo(demo, config);
}
