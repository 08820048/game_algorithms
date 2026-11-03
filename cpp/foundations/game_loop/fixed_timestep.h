#pragma once

#include <algorithm>

#include "vec2.h"

namespace ga {

// #region body
// 一个最简单的受重力影响的物体
struct Body {
    Vec2 position;
    Vec2 velocity;
};

// 显式欧拉积分：先用旧速度移动，再更新速度。结果会随 dt 的大小而变化
inline void integrate(Body& body, Vec2 gravity, float dt) {
    body.position += body.velocity * dt;
    body.velocity += gravity * dt;
}
// #endregion

// #region variable-loop
// 写法一：可变时间步长。每帧用"这一帧花了多久"推进模拟
inline void variableStep(Body& body, Vec2 gravity, double frameTime) {
    integrate(body, gravity, static_cast<float>(frameTime));
}
// #endregion

// #region fixed-timestep
// 写法二：固定时间步长。把真实时间存进"蓄水池"，每攒够 dt 就推进一步
class FixedTimestep {
public:
    // dt：每一步模拟的时长；maxFrameTime：单帧最多补多少时间，防止"死亡螺旋"
    explicit FixedTimestep(double dt, double maxFrameTime = 0.25) : dt_(dt), maxFrameTime_(maxFrameTime) {}

    // 传入这一帧真实经过的时间，按固定步长调用 update(dt) 零到多次，返回调用次数
    template <class Update>
    int advance(double frameTime, Update&& update) {
        accumulator_ += std::min(frameTime, maxFrameTime_);
        int steps = 0;
        while (accumulator_ >= dt_) {
            update(dt_);
            accumulator_ -= dt_;
            ++steps;
        }
        return steps;
    }

    // 剩余时间占一步的比例 ∈ [0, 1)：渲染时用它在上一个和当前状态之间插值
    double alpha() const { return accumulator_ / dt_; }
    double dt() const { return dt_; }
    void reset() { accumulator_ = 0.0; }

private:
    double dt_;
    double maxFrameTime_;
    double accumulator_ = 0.0;
};
// #endregion

// #region interpolate
// 渲染用的插值：previous 是上一步的状态，current 是最新状态
inline Vec2 interpolate(Vec2 previous, Vec2 current, double alpha) {
    float a = static_cast<float>(alpha);
    return previous * (1.0f - a) + current * a;
}
// #endregion

// #region game-loop
// 完整的游戏循环骨架。now() 返回秒数，update(dt) 推进一步模拟，render(alpha) 绘制插值后的画面
template <class Now, class Running, class Update, class Render>
void runGameLoop(double dt, Now now, Running running, Update update, Render render) {
    FixedTimestep timestep(dt);
    double previous = now();
    while (running()) {
        double current = now();
        double frameTime = current - previous;  // 这一帧真实经过的时间
        previous = current;

        // processInput();                      // 输入通常在这里收集
        timestep.advance(frameTime, update);    // 零到多次固定步长更新
        render(timestep.alpha());               // 用 alpha 在两个状态之间插值
    }
}
// #endregion

}  // namespace ga
