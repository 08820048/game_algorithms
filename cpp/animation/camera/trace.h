#pragma once

// 一段自动生成的平台跳跃轨迹（测试、测量和演示共用）：
//   走一段、停一下、原地左右小碎步、起跳落到更高或更低的平台、偶尔掉下台阶。
//   坐标是屏幕方向：x 向右，y 向下（像素），重力向 +y

#include <cmath>
#include <vector>

#include "../../foundations/random/random.h"
#include "camera.h"

namespace ga::camera {

// #region trace
struct TraceOptions {
    float walkSpeed = 220;     // px/s
    float jumpSpeed = 620;     // 起跳的竖直速度
    float gravity = 1800;
    float fidgetChance = 0.25f;  // 每段动作后来一次左右小碎步的概率
    uint64_t seed = 1;
};

class Autopilot {
public:
    explicit Autopilot(const TraceOptions& o = {}) : o_(o), rng_(o.seed) { next(); }

    PlayerSample sample() const { return {pos_, vel_, grounded_}; }

    void step(float dt) {
        timer_ -= dt;
        if (grounded_) {
            vel_.x = dir_ * speed_;
            if (timer_ <= 0) next();
        } else {
            vel_.x = dir_ * o_.walkSpeed;
        }
        pos_.x += vel_.x * dt;
        if (!grounded_) {
            vel_.y += o_.gravity * dt;
            pos_.y += vel_.y * dt;
            if (vel_.y > 0 && pos_.y >= ground_) pos_.y = ground_, vel_.y = 0, grounded_ = true, timer_ = 0.15f + rng_.nextFloat() * 0.3f, speed_ = o_.walkSpeed;
        }
    }

private:
    // 选下一段动作
    void next() {
        float r = rng_.nextFloat();
        if (pending_ > 0) {  // 小碎步：往回走一小段，再转回来
            dir_ = -dir_, speed_ = o_.walkSpeed * 0.6f, timer_ = 0.12f + rng_.nextFloat() * 0.15f, --pending_;
            return;
        }
        dir_ = 1;
        if (r < 0.35f) {  // 走
            speed_ = o_.walkSpeed, timer_ = 0.4f + rng_.nextFloat() * 1.2f;
        } else if (r < 0.5f) {  // 停
            speed_ = 0, timer_ = 0.3f + rng_.nextFloat() * 0.8f;
        } else if (r < 0.85f) {  // 跳到另一个高度的平台
            grounded_ = false, vel_.y = -o_.jumpSpeed;
            ground_ += (rng_.nextFloat() * 2 - 1) * 90;
            timer_ = 0;
        } else {  // 往回走一段（比如回头捡东西）
            dir_ = -1, speed_ = o_.walkSpeed, timer_ = 0.3f + rng_.nextFloat() * 0.6f;
        }
        if (rng_.chance(o_.fidgetChance)) pending_ = 2 + int(rng_.below(3));
    }

    TraceOptions o_;
    Pcg32 rng_;
    Vec2 pos_{0, 0}, vel_{0, 0};
    float ground_ = 0, timer_ = 0, speed_ = 0, dir_ = 1;
    bool grounded_ = true;
    int pending_ = 0;
};
// #endregion

}  // namespace ga::camera
