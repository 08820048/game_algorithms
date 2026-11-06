#pragma once

// 数值积分：已知加速度 a(x, v)，一步一步推进位置和速度
//   - explicitEuler       显式欧拉：用旧速度移动，一阶，能量不断增长
//   - semiImplicitEuler   半隐式（辛）欧拉：先更新速度，再用新速度移动，一阶，能量有界
//   - velocityVerlet      速度 Verlet：二阶，能量有界，每步一次加速度计算
//   - positionVerlet      位置 Verlet（Störmer）：不存速度，用上一帧的位置，与速度 Verlet 等价
//   - rk4                 经典四阶 Runge–Kutta：四阶，每步四次加速度计算，能量缓慢衰减
//
// 所有函数对向量类型 V 写成模板：游戏里用 float 的 Vec2，测试和对比用 double 的 Vec2d

#include <cmath>

#include "vec2.h"

namespace ga {

// 双精度二维向量：只用于测量误差，避免 float 的舍入误差掩盖算法本身的误差
struct Vec2d {
    double x = 0, y = 0;
    constexpr Vec2d operator+(Vec2d o) const { return {x + o.x, y + o.y}; }
    constexpr Vec2d operator-(Vec2d o) const { return {x - o.x, y - o.y}; }
    constexpr Vec2d operator*(double s) const { return {x * s, y * s}; }
    constexpr Vec2d operator/(double s) const { return {x / s, y / s}; }
    constexpr Vec2d operator-() const { return {-x, -y}; }
};
constexpr double dot(Vec2d a, Vec2d b) { return a.x * b.x + a.y * b.y; }
inline double length(Vec2d v) { return std::sqrt(dot(v, v)); }

// #region state
template <class V>
struct StateT {
    V x;  // 位置
    V v;  // 速度
};
using State = StateT<Vec2>;
// #endregion

// #region euler
// 显式欧拉：位置和速度都用这一步开始时的值。就是 0.1《游戏循环》里的写法
template <class V, class Accel>
StateT<V> explicitEuler(const StateT<V>& s, double dt, Accel&& accel) {
    V a = accel(s.x, s.v);
    return {s.x + s.v * dt, s.v + a * dt};
}

// 半隐式欧拉：只是把两行交换顺序——先更新速度，再用"新"速度移动位置
template <class V, class Accel>
StateT<V> semiImplicitEuler(const StateT<V>& s, double dt, Accel&& accel) {
    V v = s.v + accel(s.x, s.v) * dt;
    return {s.x + v * dt, v};
}
// #endregion

// #region verlet
// 速度 Verlet：用这一步开始和结束时加速度的平均值更新速度。
// 结束时的加速度留给下一步当"开始时的加速度"，所以每步只需计算一次加速度
template <class V>
struct VerletStateT {
    V x, v;
    V a;  // 当前位置上的加速度
};

template <class V, class Accel>
VerletStateT<V> makeVerlet(const StateT<V>& s, Accel&& accel) {
    return {s.x, s.v, accel(s.x, s.v)};
}

template <class V, class Accel>
VerletStateT<V> velocityVerlet(const VerletStateT<V>& s, double dt, Accel&& accel) {
    V x = s.x + s.v * dt + s.a * (0.5 * dt * dt);
    // 加速度若与速度有关（阻尼、空气阻力），这里只能先用估计的速度
    V a = accel(x, s.v + s.a * dt);
    V v = s.v + (s.a + a) * (0.5 * dt);
    return {x, v, a};
}
// #endregion

// #region position-verlet
// 位置 Verlet（Störmer–Verlet）：只存当前和上一步的位置，速度隐含在两者之差里。
// x[n+1] = 2·x[n] − x[n−1] + a·dt²。Jakobsen 的布料、绳子模拟就基于它：直接修改位置，速度自动跟着变
template <class V>
struct PositionVerletStateT {
    V x, prev;
    double dt;  // 要求固定步长：上一步的位移隐含了上一步的 dt
};

template <class V, class Accel>
PositionVerletStateT<V> makePositionVerlet(const StateT<V>& s, double dt, Accel&& accel) {
    // 往回推一步得到"上一步的位置"（泰勒展开到二阶，保证与速度 Verlet 的轨迹一致）
    return {s.x, s.x - s.v * dt + accel(s.x, s.v) * (0.5 * dt * dt), dt};
}

template <class V, class Accel>
PositionVerletStateT<V> positionVerlet(const PositionVerletStateT<V>& s, Accel&& accel) {
    V vEstimate = (s.x - s.prev) / s.dt;  // 只有一阶精度，速度相关的力会因此带上误差
    V next = s.x * 2.0 - s.prev + accel(s.x, vEstimate) * (s.dt * s.dt);
    return {next, s.x, s.dt};
}

// 位置 Verlet 的速度：中心差分
template <class V>
V velocityOf(const PositionVerletStateT<V>& s, const V& next) {
    return (next - s.prev) / (2 * s.dt);
}
// #endregion

// #region rk4
// 经典四阶 Runge–Kutta：在这一步内取四个点的斜率，按 1:2:2:1 加权平均
template <class V, class Accel>
StateT<V> rk4(const StateT<V>& s, double dt, Accel&& accel) {
    // 每个 k 是一对（速度，加速度），也就是状态 (x, v) 的变化率
    V k1x = s.v, k1v = accel(s.x, s.v);
    V k2x = s.v + k1v * (dt / 2), k2v = accel(s.x + k1x * (dt / 2), k2x);
    V k3x = s.v + k2v * (dt / 2), k3v = accel(s.x + k2x * (dt / 2), k3x);
    V k4x = s.v + k3v * dt, k4v = accel(s.x + k3x * dt, k4x);
    return {s.x + (k1x + k2x * 2.0 + k3x * 2.0 + k4x) * (dt / 6),
            s.v + (k1v + k2v * 2.0 + k3v * 2.0 + k4v) * (dt / 6)};
}
// #endregion

// ---- 常用的加速度 ----

// #region forces
// 弹簧：a = −ω²·x（简谐振动，ω 是角频率）
struct Spring {
    double omega;
    template <class V>
    V operator()(const V& x, const V&) const { return x * -(omega * omega); }
    template <class V>
    double energy(const StateT<V>& s) const { return 0.5 * dot(s.v, s.v) + 0.5 * omega * omega * dot(s.x, s.x); }
};

// 万有引力：a = −GM·x / |x|³（中心天体在原点）
struct Gravity {
    double gm;
    template <class V>
    V operator()(const V& x, const V&) const {
        double r = length(x);
        return x * (-gm / (r * r * r));
    }
    template <class V>
    double energy(const StateT<V>& s) const { return 0.5 * dot(s.v, s.v) - gm / length(s.x); }
};

// 恒定重力：a = g（抛体、跳跃）
template <class V>
struct Constant {
    V g;
    V operator()(const V&, const V&) const { return g; }
};
// #endregion

}  // namespace ga
