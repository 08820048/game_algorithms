#pragma once

// 碰撞响应：基于冲量的刚体物理（思路同 Erin Catto 的 Box2D Lite）
//   - 刚体：盒子或圆，有质量、转动惯量、速度、角速度
//   - 接触：用 2.1 / 2.2 的相交测试得到法线、接触点与穿透深度
//   - 冲量：在每个接触点施加一个瞬时的"冲击"，让两个物体在法线方向上不再互相靠近；
//          切线方向上按库仑摩擦定律限制
//   - 顺序冲量：多个接触互相影响，逐个求解、反复迭代；累积冲量被限制为非负（可以减小，但不能变成拉力）
//   - 热启动：上一帧的冲量作为这一帧的初值，堆叠才能稳住
//   - 位置修正：把一部分穿透深度转换成额外的分离速度（Baumgarte）
//
// 坐标：单位是米，y 轴朝上，重力 (0, -10)

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <vector>

#include "../intersect/intersect.h"
#include "../sat/sat.h"

namespace ga {

// #region body
struct Body {
    enum class Shape { Box, Circle };
    Shape shape = Shape::Box;
    Vec2 half{0.5f, 0.5f};  // 盒子的半边长
    float radius = 0.5f;    // 圆的半径

    Vec2 pos{0, 0};
    float angle = 0;
    Vec2 vel{0, 0};
    float angVel = 0;  // 角速度（弧度 / 秒，逆时针为正）

    float invMass = 0;     // 质量的倒数；0 表示静止物体（无穷大质量）
    float invInertia = 0;  // 转动惯量的倒数
    float friction = 0.5f;
    float restitution = 0;

    float mass() const { return invMass > 0 ? 1 / invMass : 0; }
    float inertia() const { return invInertia > 0 ? 1 / invInertia : 0; }
};

// 密度为 density 的盒子：m = ρ·w·h，I = m·(w² + h²) / 12。density = 0 表示静止
inline Body makeBoxBody(Vec2 pos, Vec2 half, float density, float angle = 0) {
    Body b;
    b.shape = Body::Shape::Box, b.half = half, b.pos = pos, b.angle = angle;
    if (density > 0) {
        float m = density * 4 * half.x * half.y;
        b.invMass = 1 / m;
        b.invInertia = 1 / (m * (4 * half.x * half.x + 4 * half.y * half.y) / 12);
    }
    return b;
}

// 圆：m = ρ·πr²，I = m·r² / 2
inline Body makeCircleBody(Vec2 pos, float radius, float density) {
    Body b;
    b.shape = Body::Shape::Circle, b.radius = radius, b.pos = pos;
    if (density > 0) {
        float m = density * kPi * radius * radius;
        b.invMass = 1 / m;
        b.invInertia = 1 / (0.5f * m * radius * radius);
    }
    return b;
}
// #endregion

inline ConvexPolygon worldPolygon(const Body& b) { return makeBox(b.pos, b.half, b.angle); }

// 二维里的"叉积"：角速度 w（标量）× 向量 r = (−w·r.y, w·r.x)
inline Vec2 crossSV(float w, Vec2 r) { return {-w * r.y, w * r.x}; }

// 物体上 r（相对质心）处的点的速度
inline Vec2 pointVelocity(const Body& b, Vec2 r) { return b.vel + crossSV(b.angVel, r); }

// #region manifold
struct ContactPoint {
    Vec2 point;          // 世界坐标
    float depth;         // 穿透深度
    Vec2 rA, rB;         // 从两个质心到接触点
    float normalMass;    // 法线方向的"有效质量"
    float tangentMass;
    float bias;          // 位置修正与弹性带来的目标分离速度
    float pn = 0, pt = 0;  // 累积的法线、切线冲量
};

struct Arbiter {
    int a, b;  // 物体下标，法线从 a 指向 b
    Vec2 normal;
    int count = 0;
    ContactPoint points[2];
    float friction, restitution;
};

// 两个物体的接触。没有接触时 count = 0
inline Arbiter collideBodies(const std::vector<Body>& bodies, int ia, int ib) {
    const Body &A = bodies[ia], &B = bodies[ib];
    Arbiter arb{ia, ib, {0, 0}, 0, {}, std::sqrt(A.friction * B.friction), std::max(A.restitution, B.restitution)};
    using S = Body::Shape;
    if (A.shape == S::Circle && B.shape == S::Circle) {
        if (auto c = contact(Circle{A.pos, A.radius}, Circle{B.pos, B.radius})) {
            arb.normal = c->normal, arb.count = 1;
            arb.points[0].point = c->point, arb.points[0].depth = c->depth;
        }
    } else if (A.shape == S::Box && B.shape == S::Box) {
        if (auto m = collide(worldPolygon(A), worldPolygon(B))) {
            arb.normal = m->normal, arb.count = m->count;
            for (int k = 0; k < m->count; ++k) arb.points[k].point = m->points[k], arb.points[k].depth = m->depths[k];
        }
    } else {
        // 盒子与圆：contact(多边形, 圆) 的法线从多边形指向圆
        bool boxIsA = A.shape == S::Box;
        const Body& box = boxIsA ? A : B;
        const Body& circle = boxIsA ? B : A;
        if (auto c = contact(worldPolygon(box), Circle{circle.pos, circle.radius})) {
            arb.normal = boxIsA ? c->normal : -c->normal, arb.count = 1;
            arb.points[0].point = c->point, arb.points[0].depth = c->depth;
        }
    }
    return arb;
}
// #endregion

class World {
public:
    struct Options {
        int iterations = 10;
        bool warmStart = true;
        bool accumulate = true;     // false：每次迭代单独把增量冲量夹到非负（错误的做法，用于对比）
        float baumgarte = 0.2f;     // 每步修正多少比例的穿透
        float slop = 0.01f;         // 允许的穿透，不去修正它，避免接触来回抖动
        float restitutionThreshold = 1.0f;  // 相对速度低于它就不反弹，静止的物体才能停稳
    };

    Vec2 gravity{0, -10};
    Options opt;
    std::vector<Body> bodies;

    const std::vector<Arbiter>& arbiters() const { return arbiters_; }

    // #region step
    void step(float dt) {
        findContacts();
        // 1. 积分外力：只改变速度（半隐式欧拉的前半步）
        for (Body& b : bodies)
            if (b.invMass > 0) b.vel += gravity * dt;
        // 2. 预计算每个接触点的有效质量与目标速度。必须先全部算完，再热启动：
        //    否则后面的接触看到的是已经被前面的热启动改变过的速度，会凭空算出反弹速度
        for (Arbiter& arb : arbiters_) prepare(arb, dt);
        for (Arbiter& arb : arbiters_) warmStart(arb);
        // 3. 顺序冲量：逐个接触求解，反复迭代
        for (int it = 0; it < opt.iterations; ++it)
            for (Arbiter& arb : arbiters_) solve(arb);
        // 4. 用新速度更新位置（半隐式欧拉的后半步）
        for (Body& b : bodies) b.pos += b.vel * dt, b.angle += b.angVel * dt;
    }
    // #endregion

private:
    static uint64_t key(int a, int b) { return (uint64_t(uint32_t(a)) << 32) | uint32_t(b); }

    void findContacts() {
        std::map<uint64_t, Arbiter> old;
        for (const Arbiter& arb : arbiters_) old.emplace(key(arb.a, arb.b), arb);
        arbiters_.clear();
        int n = static_cast<int>(bodies.size());
        for (int i = 0; i < n; ++i)
            for (int j = i + 1; j < n; ++j) {
                if (bodies[i].invMass == 0 && bodies[j].invMass == 0) continue;
                if (!boundsOverlap(bodies[i], bodies[j])) continue;
                Arbiter arb = collideBodies(bodies, i, j);
                if (arb.count == 0) continue;
                // 热启动：上一帧同一对物体里，位置相近的接触点继承它的累积冲量
                if (auto it = old.find(key(i, j)); it != old.end() && opt.warmStart)
                    for (int k = 0; k < arb.count; ++k)
                        for (int m = 0; m < it->second.count; ++m)
                            if (lengthSq(arb.points[k].point - it->second.points[m].point) < kMatch * kMatch) {
                                arb.points[k].pn = it->second.points[m].pn;
                                arb.points[k].pt = it->second.points[m].pt;
                            }
                arbiters_.push_back(arb);
            }
    }

    static bool boundsOverlap(const Body& a, const Body& b) {
        auto reach = [](const Body& x) { return x.shape == Body::Shape::Circle ? x.radius : length(x.half); };
        float r = reach(a) + reach(b);
        return lengthSq(a.pos - b.pos) < r * r;
    }

    // #region prepare
    void prepare(Arbiter& arb, float dt) {
        Body &A = bodies[arb.a], &B = bodies[arb.b];
        Vec2 n = arb.normal, t = perp(n);
        for (int k = 0; k < arb.count; ++k) {
            ContactPoint& c = arb.points[k];
            c.rA = c.point - A.pos, c.rB = c.point - B.pos;
            // 有效质量：在接触点沿 n 施加单位冲量，两个物体的相对速度改变多少，它的倒数
            float rnA = cross(c.rA, n), rnB = cross(c.rB, n);
            c.normalMass = 1 / (A.invMass + B.invMass + A.invInertia * rnA * rnA + B.invInertia * rnB * rnB);
            float rtA = cross(c.rA, t), rtB = cross(c.rB, t);
            c.tangentMass = 1 / (A.invMass + B.invMass + A.invInertia * rtA * rtA + B.invInertia * rtB * rtB);
            // 目标分离速度：位置修正（把超出 slop 的穿透在若干步内推开）与弹性，取较大者
            float vn = dot(pointVelocity(B, c.rB) - pointVelocity(A, c.rA), n);
            float positional = opt.baumgarte / dt * std::max(0.0f, c.depth - opt.slop);
            float bounce = vn < -opt.restitutionThreshold ? -arb.restitution * vn : 0;
            c.bias = std::max(positional, bounce);
        }
    }
    // #endregion

    // #region warm
    // 热启动：先施加上一帧的累积冲量，迭代只需在此基础上微调
    void warmStart(Arbiter& arb) {
        Body &A = bodies[arb.a], &B = bodies[arb.b];
        Vec2 n = arb.normal, t = perp(n);
        for (int k = 0; k < arb.count; ++k) {
            ContactPoint& c = arb.points[k];
            if (opt.warmStart) apply(A, B, c, n * c.pn + t * c.pt);
            else c.pn = c.pt = 0;
        }
    }
    // #endregion

    static void apply(Body& A, Body& B, const ContactPoint& c, Vec2 P) {
        A.vel -= P * A.invMass, A.angVel -= A.invInertia * cross(c.rA, P);
        B.vel += P * B.invMass, B.angVel += B.invInertia * cross(c.rB, P);
    }

    // #region solve
    void solve(Arbiter& arb) {
        Body &A = bodies[arb.a], &B = bodies[arb.b];
        Vec2 n = arb.normal, t = perp(n);
        for (int k = 0; k < arb.count; ++k) {
            ContactPoint& c = arb.points[k];
            // 法线：让分离速度达到 bias（至少不再互相靠近）
            float vn = dot(pointVelocity(B, c.rB) - pointVelocity(A, c.rA), n);
            float dPn = c.normalMass * (c.bias - vn);
            if (opt.accumulate) {
                // 正确做法：限制的是"累积"冲量 ≥ 0。之前推得太多，这一次可以往回收
                float old = c.pn;
                c.pn = std::max(old + dPn, 0.0f);
                dPn = c.pn - old;
            } else {
                dPn = std::max(dPn, 0.0f);  // 错误做法：每次的增量都只能是推，推多了收不回来
                c.pn += dPn;
            }
            apply(A, B, c, n * dPn);

            // 切线（摩擦）：让切向相对速度变为 0，但摩擦冲量不能超过 μ·法线冲量
            float vt = dot(pointVelocity(B, c.rB) - pointVelocity(A, c.rA), t);
            float dPt = c.tangentMass * -vt;
            float maxPt = arb.friction * c.pn;
            float old = c.pt;
            c.pt = std::clamp(old + dPt, -maxPt, maxPt);
            dPt = c.pt - old;
            apply(A, B, c, t * dPt);
        }
    }
    // #endregion

    static constexpr float kMatch = 0.05f;  // 热启动时，两帧的接触点相距多近算"同一个"
    std::vector<Arbiter> arbiters_;
};

}  // namespace ga
