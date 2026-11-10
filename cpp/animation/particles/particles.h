#pragma once

// 粒子系统
//   - 存储：SoA（每个属性一个数组）+ 交换删除：死掉的粒子和最后一个交换，O(1)，但打乱顺序
//   - 环形缓冲：寿命相同时，粒子按出生顺序死亡，只需移动队头；顺序 = 年龄顺序，绘制时天然"老的在下面"
//   - 发射器：每帧要发射 rate·dt 个，不是整数。直接取整在高帧率下一个都发不出；累加器把小数部分留到下一帧
//   - 帧内出生时刻：一帧里发射的几个粒子其实出生在这一帧的不同时刻。补上"已经过去的那一小段时间"，
//     并把发射器的位置插值到那个时刻，否则快速移动的发射器会留下一团一团的拖尾
//   - 积分：重力 + 线性阻力有精确解；显式 / 半隐式欧拉的轨迹随帧率变化。精确解还可以做"无状态粒子"：
//     只存出生时刻和初始状态，任意时刻的位置直接算出来（GPU 粒子常用）

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include "../../foundations/random/random.h"
#include "vec2.h"

namespace ga::particles {

// #region integrate
enum class Integrator { Explicit, SemiImplicit, Exact };

// 重力 g、线性阻力 k（加速度 = g − k·v）下推进 dt
inline void integrate(Vec2& p, Vec2& v, Vec2 g, float k, float dt, Integrator m) {
    switch (m) {
        case Integrator::Explicit: {  // 先用旧速度移动位置，再更新速度
            Vec2 a = g - v * k;
            p += v * dt;
            v += a * dt;
            break;
        }
        case Integrator::SemiImplicit: {  // 先更新速度，再用新速度移动位置（辛欧拉）
            v += (g - v * k) * dt;
            p += v * dt;
            break;
        }
        case Integrator::Exact: {
            if (k < 1e-6f) {
                p += v * dt + g * (0.5f * dt * dt);
                v += g * dt;
                break;
            }
            // 速度按 e^(−k t) 趋向终端速度 g/k
            Vec2 vt = g / k;
            float e = std::exp(-k * dt);
            p += vt * dt + (v - vt) * ((1 - e) / k);
            v = vt + (v - vt) * e;
            break;
        }
    }
}
// 无状态：出生后经过 t 秒的位置（和 Exact 一步走完 t 相同）
inline Vec2 positionAt(Vec2 p0, Vec2 v0, Vec2 g, float k, float t) {
    Vec2 p = p0, v = v0;
    integrate(p, v, g, k, t, Integrator::Exact);
    return p;
}
// #endregion

// #region pool
// SoA 粒子池。kill 用交换删除：O(1)，但会打乱顺序
struct Pool {
    std::vector<Vec2> pos, vel;
    std::vector<float> age, life;
    std::vector<uint32_t> id;  // 出生序号，用于测试和按年龄排序

    size_t size() const { return pos.size(); }
    void add(Vec2 p, Vec2 v, float life_, uint32_t id_, float age_ = 0) {
        pos.push_back(p), vel.push_back(v), age.push_back(age_), life.push_back(life_), id.push_back(id_);
    }
    void kill(size_t i) {
        size_t last = size() - 1;
        if (i != last) pos[i] = pos[last], vel[i] = vel[last], age[i] = age[last], life[i] = life[last], id[i] = id[last];
        pos.pop_back(), vel.pop_back(), age.pop_back(), life.pop_back(), id.pop_back();
    }
    // 推进所有粒子；死掉的就地交换删除（倒序遍历，换过来的粒子已经处理过）
    void update(float dt, Vec2 g, float k, Integrator m) {
        for (size_t i = size(); i-- > 0;) {
            age[i] += dt;
            if (age[i] >= life[i]) {
                kill(i);
                continue;
            }
            integrate(pos[i], vel[i], g, k, dt, m);
        }
    }
    void clear() { pos.clear(), vel.clear(), age.clear(), life.clear(), id.clear(); }
};
// #endregion

// #region ring
// 环形缓冲：所有粒子寿命相同（或者只要求"先生先死"）时，死亡总发生在队头。
// 容量固定，满了就覆盖最老的粒子；遍历顺序就是年龄从大到小
template <class T>
class Ring {
public:
    explicit Ring(size_t capacity) : buf_(capacity) {}
    size_t size() const { return count_; }
    size_t capacity() const { return buf_.size(); }
    T& operator[](size_t i) { return buf_[(head_ + i) % buf_.size()]; }  // 0 = 最老
    const T& operator[](size_t i) const { return buf_[(head_ + i) % buf_.size()]; }
    void push(const T& v) {
        if (count_ == buf_.size()) head_ = (head_ + 1) % buf_.size(), --count_;  // 满了：丢掉最老的
        buf_[(head_ + count_) % buf_.size()] = v;
        ++count_;
    }
    void popOldest() {
        head_ = (head_ + 1) % buf_.size();
        --count_;
    }
    void clear() { head_ = count_ = 0; }

private:
    std::vector<T> buf_;
    size_t head_ = 0, count_ = 0;
};
// #endregion

// #region emitter
enum class EmitMode {
    Truncate,     // int(rate·dt)：小数部分丢掉
    Round,        // 四舍五入
    Accumulate,   // 小数部分留到下一帧
    Subframe,     // 累加 + 每个粒子按真实的出生时刻补走剩下的时间、发射器位置插值到出生时刻
};

class Emitter {
public:
    float rate = 100;  // 每秒发射几个
    EmitMode mode = EmitMode::Subframe;

    // 这一帧（时长 dt）发射器从 from 移到 to。对每个新粒子调用 spawn(位置, 已经存在了多久)：
    // 已经存在的时间 ∈ [0, dt)，调用方应把粒子先推进这么久
    template <class Spawn>
    int emit(float dt, Vec2 from, Vec2 to, Spawn&& spawn) {
        int n = 0;
        switch (mode) {
            case EmitMode::Truncate: n = int(rate * dt); break;
            case EmitMode::Round: n = int(std::lround(rate * dt)); break;
            case EmitMode::Accumulate:
            case EmitMode::Subframe: {
                acc_ += rate * dt;
                n = int(acc_);
                acc_ -= float(n);
                break;
            }
        }
        if (mode != EmitMode::Subframe) {
            for (int i = 0; i < n; ++i) spawn(to, 0.0f);
            return n;
        }
        // acc_ 是这一帧结束时"欠下的"小数部分：最后一个粒子出生在帧末之前 acc_/rate 秒，前面的依次早 1/rate
        for (int i = 0; i < n; ++i) {
            float ageAtEnd = (acc_ + float(n - 1 - i)) / rate;
            float f = dt > 0 ? 1 - ageAtEnd / dt : 1;  // 出生时刻在这一帧里的比例
            spawn(from + (to - from) * std::clamp(f, 0.0f, 1.0f), ageAtEnd);
        }
        return n;
    }
    void reset() { acc_ = 0; }

private:
    float acc_ = 0;
};
// #endregion

// #region shapes
// 圆盘内均匀分布：半径取 R·√u。直接取 R·u 会让点挤在圆心附近（面积和 r² 成正比）
inline Vec2 uniformDisk(Pcg32& rng, float radius) {
    float r = radius * std::sqrt(rng.nextFloat()), a = rng.nextFloat() * 2 * kPi;
    return {r * std::cos(a), r * std::sin(a)};
}
inline Vec2 naiveDisk(Pcg32& rng, float radius) {
    float r = radius * rng.nextFloat(), a = rng.nextFloat() * 2 * kPi;
    return {r * std::cos(a), r * std::sin(a)};
}
// 锥形：方向在 dir 两侧 ±spread 弧度内均匀，速度在 [lo, hi] 内均匀
inline Vec2 cone(Pcg32& rng, Vec2 dir, float spread, float lo, float hi) {
    float a = angleOf(dir) + (rng.nextFloat() * 2 - 1) * spread;
    return fromAngle(a) * (lo + (hi - lo) * rng.nextFloat());
}
// #endregion

// #region gradient
// 随寿命变化的量（颜色、大小、透明度）：若干个关键点之间线性插值，t ∈ [0, 1] 是年龄 / 寿命
struct Key {
    float t, v;
};
inline float evaluate(const std::vector<Key>& keys, float t) {
    if (t <= keys.front().t) return keys.front().v;
    if (t >= keys.back().t) return keys.back().v;
    for (size_t i = 1; i < keys.size(); ++i)
        if (t <= keys[i].t) {
            float u = (t - keys[i - 1].t) / (keys[i].t - keys[i - 1].t);
            return keys[i - 1].v + (keys[i].v - keys[i - 1].v) * u;
        }
    return keys.back().v;
}
// #endregion

}  // namespace ga::particles
