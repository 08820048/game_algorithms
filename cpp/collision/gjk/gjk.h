#pragma once

// GJK（Gilbert–Johnson–Keerthi）与 EPA（Expanding Polytope Algorithm）。
// 两者都只通过"支撑函数"访问形状，因此同一份代码适用于任意凸形状：多边形、圆、胶囊体、圆角矩形……

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>

#include "../intersect/intersect.h"
#include "vec2.h"

namespace ga {

// #region shape
// 凸形状 = 若干个"核心点"的凸包，再向外膨胀 radius。
//   多边形：顶点，radius = 0      圆：1 个点（圆心），radius = r
//   胶囊体：2 个点（线段端点）     圆角矩形：4 个顶点，radius = 圆角半径
struct ConvexShape {
    std::vector<Vec2> points;
    float radius = 0;
};

// 支撑函数：形状上沿方向 d 最远的点。GJK 和 EPA 只需要这一个函数
inline Vec2 support(const ConvexShape& s, Vec2 d) {
    int best = 0;
    float bestDot = dot(s.points[0], d);
    for (int i = 1; i < static_cast<int>(s.points.size()); ++i) {
        float t = dot(s.points[i], d);
        if (t > bestDot) bestDot = t, best = i;
    }
    Vec2 p = s.points[best];
    if (s.radius > 0) p += normalized(d) * s.radius;  // 膨胀：再沿 d 走 radius
    return p;
}
// #endregion

inline Vec2 centerOf(const ConvexShape& s) {
    Vec2 c{0, 0};
    for (Vec2 p : s.points) c += p;
    return c / static_cast<float>(s.points.size());
}

// #region minkowski
// 闵可夫斯基差 A − B = { a − b }。它的支撑点 = A 沿 d 的支撑点 − B 沿 −d 的支撑点。
// 同时记下 a 和 b，最后才能还原出两个形状上的最近点 / 接触点
struct SupportPoint {
    Vec2 p;  // p = a − b
    Vec2 a, b;
};

inline SupportPoint support(const ConvexShape& a, const ConvexShape& b, Vec2 d) {
    Vec2 pa = support(a, d), pb = support(b, -d);
    return {pa - pb, pa, pb};
}
// #endregion

// 单纯形：GJK 维护的 1~3 个支撑点，以及原点在其上的最近点的重心坐标
struct Simplex {
    SupportPoint v[3];
    float w[3] = {1, 0, 0};
    int count = 0;

    Vec2 closest() const { return blend(&SupportPoint::p); }
    Vec2 pointA() const { return blend(&SupportPoint::a); }
    Vec2 pointB() const { return blend(&SupportPoint::b); }

private:
    Vec2 blend(Vec2 SupportPoint::* m) const {
        Vec2 r{0, 0};
        for (int i = 0; i < count; ++i) r += v[i].*m * w[i];
        return r;
    }
};

// #region reduce
// 求单纯形上离原点最近的点，并丢掉用不到的顶点（只保留最近点所在的那个顶点 / 边）
inline void keep(Simplex& s, std::initializer_list<int> ids, std::initializer_list<float> ws) {
    SupportPoint v[3];
    int k = 0;
    for (int i : ids) v[k++] = s.v[i];
    k = 0;
    for (float w : ws) s.w[k++] = w;
    for (int i = 0; i < k; ++i) s.v[i] = v[i];
    s.count = k;
}

// 线段 AB
inline void reduceSegment(Simplex& s) {
    Vec2 a = s.v[0].p, ab = s.v[1].p - a;
    float t = dot(-a, ab) / dot(ab, ab);
    if (t <= 0) keep(s, {0}, {1});
    else if (t >= 1) keep(s, {1}, {1});
    else keep(s, {0, 1}, {1 - t, t});
}

// 三角形 ABC：依次判断原点落在哪个顶点区域、边区域，还是三角形内部（Ericson 5.1.5）
inline void reduceTriangle(Simplex& s) {
    Vec2 a = s.v[0].p, b = s.v[1].p, c = s.v[2].p;
    Vec2 ab = b - a, ac = c - a;
    float d1 = dot(ab, -a), d2 = dot(ac, -a);
    if (d1 <= 0 && d2 <= 0) return keep(s, {0}, {1});
    float d3 = dot(ab, -b), d4 = dot(ac, -b);
    if (d3 >= 0 && d4 <= d3) return keep(s, {1}, {1});
    float vc = d1 * d4 - d3 * d2;
    if (vc <= 0 && d1 >= 0 && d3 <= 0) {
        float t = d1 / (d1 - d3);
        return keep(s, {0, 1}, {1 - t, t});
    }
    float d5 = dot(ab, -c), d6 = dot(ac, -c);
    if (d6 >= 0 && d5 <= d6) return keep(s, {2}, {1});
    float vb = d5 * d2 - d1 * d6;
    if (vb <= 0 && d2 >= 0 && d6 <= 0) {
        float t = d2 / (d2 - d6);
        return keep(s, {0, 2}, {1 - t, t});
    }
    float va = d3 * d6 - d5 * d4;
    if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0) {
        float t = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return keep(s, {1, 2}, {1 - t, t});
    }
    // 原点在三角形内部：保留三个顶点
    float inv = 1 / (va + vb + vc);
    s.w[1] = vb * inv, s.w[2] = vc * inv, s.w[0] = 1 - s.w[1] - s.w[2];
}
// #endregion

// #region gjk
// 一步一步执行的 GJK：每一步沿"从当前最近点指向原点"的方向取一个新的支撑点，
// 把它加入单纯形，再收缩到离原点最近的部分
class Gjk {
public:
    enum class Status { Running, Intersecting, Separated };

    Gjk(const ConvexShape& a, const ConvexShape& b, float tolerance = 1e-3f) : a_(&a), b_(&b), tol_(tolerance) {
        Vec2 d = centerOf(b) - centerOf(a);  // A − B 的中心大约在 cA − cB，朝原点的方向就是 cB − cA
        if (lengthSq(d) < 1e-12f) d = {1, 0};
        simplex_.v[0] = last_ = support(a, b, d);
        simplex_.count = 1;
    }

    Status step() {
        if (status_ != Status::Running) return status_;
        ++iterations_;
        Vec2 c = simplex_.closest();
        if (simplex_.count == 1) {
            if (lengthSq(c) < 1e-12f) return status_ = Status::Intersecting;  // 原点就是这个顶点
            dir_ = -c;
        } else {
            // 线段：搜索方向取线段的垂线（朝向原点的一侧），而不是 −c。
            // 原点几乎落在线段上时，c 接近零，它的方向完全由浮点误差决定
            Vec2 a = simplex_.v[0].p, ab = simplex_.v[1].p - a;
            float side = cross(ab, -a);
            if (std::abs(side) <= 1e-5f * length(ab)) return status_ = Status::Intersecting;  // 原点在线段上
            dir_ = side > 0 ? perp(ab) : -perp(ab);
        }
        SupportPoint w = last_ = support(*a_, *b_, dir_);
        // 新的支撑点没有比当前最近点更接近原点：当前最近点就是答案
        if (dot(w.p - c, normalized(dir_)) <= tol_) return status_ = Status::Separated;
        for (int i = 0; i < simplex_.count; ++i)
            if (lengthSq(simplex_.v[i].p - w.p) < 1e-12f) return status_ = Status::Separated;

        simplex_.v[simplex_.count++] = w;
        if (simplex_.count == 2) reduceSegment(simplex_);
        else reduceTriangle(simplex_);
        if (simplex_.count == 3) return status_ = Status::Intersecting;  // 原点被三角形包住了
        if (iterations_ >= kMaxIterations) return status_ = Status::Separated;
        return status_;
    }
    // #endregion

    Status run() {
        while (step() == Status::Running) {}
        return status_;
    }

    Status status() const { return status_; }
    const Simplex& simplex() const { return simplex_; }
    Vec2 direction() const { return dir_; }        // 最近一次的搜索方向
    const SupportPoint& lastSupport() const { return last_; }
    int iterations() const { return iterations_; }
    // 分离时：两个形状之间的距离，以及各自身上的最近点
    float distance() const { return length(simplex_.closest()); }
    Vec2 pointA() const { return simplex_.pointA(); }
    Vec2 pointB() const { return simplex_.pointB(); }

    static constexpr int kMaxIterations = 32;

private:
    const ConvexShape* a_;
    const ConvexShape* b_;
    float tol_;
    Simplex simplex_;
    Status status_ = Status::Running;
    Vec2 dir_{0, 0};
    SupportPoint last_{};
    int iterations_ = 0;
};

inline bool overlaps(const ConvexShape& a, const ConvexShape& b) {
    return Gjk(a, b).run() == Gjk::Status::Intersecting;
}

// #region epa
// EPA：GJK 判定相交后，从它留下的三角形出发，不断找离原点最近的边，
// 沿这条边的法线取支撑点，把多边形往外扩，直到扩不动为止。
// 那条最近的边到原点的距离就是穿透深度，它的法线就是分离方向
class Epa {
public:
    Epa(const ConvexShape& a, const ConvexShape& b, const Simplex& s, float tolerance = 1e-3f)
        : a_(&a), b_(&b), tol_(tolerance) {
        for (int i = 0; i < s.count; ++i) poly_.push_back(s.v[i]);
        // 两个形状恰好接触时，GJK 可能只留下 1~2 个点：补成三角形
        if (poly_.size() == 1) addIfNew(support(a, b, {1, 0})) || addIfNew(support(a, b, {-1, 0}));
        if (poly_.size() == 2) {
            Vec2 n = perp(poly_[1].p - poly_[0].p);
            addIfNew(support(a, b, n)) || addIfNew(support(a, b, -n));
        }
        // 统一成屏幕上的顺时针，这样边 e 的外法线是 (e.y, -e.x)
        float area = 0;
        for (size_t i = 0; i < poly_.size(); ++i) area += cross(poly_[i].p, poly_[(i + 1) % poly_.size()].p);
        if (area < 0) std::reverse(poly_.begin(), poly_.end());
        findClosestEdge();
    }

    // 扩张一次；收敛后返回 false
    bool step() {
        if (done_) return false;
        ++iterations_;
        SupportPoint w = support(*a_, *b_, normal_);
        // 沿法线再也走不远了：这条边就在 A − B 的边界上
        if (dot(w.p, normal_) - depth_ <= tol_ || iterations_ >= kMaxIterations || poly_.size() < 3) {
            done_ = true;
            return false;
        }
        poly_.insert(poly_.begin() + edge_ + 1, w);
        findClosestEdge();
        return true;
    }
    // #endregion

    void run() {
        while (step()) {}
    }

    bool done() const { return done_; }
    const std::vector<SupportPoint>& polytope() const { return poly_; }
    int closestEdge() const { return edge_; }
    Vec2 normal() const { return normal_; }  // 从 A 指向 B：把 B 沿它移动 depth 就能分开
    float depth() const { return depth_; }
    int iterations() const { return iterations_; }
    // 接触点：原点在最近边上的投影，换算回两个形状
    Vec2 pointA() const { return blend(&SupportPoint::a); }
    Vec2 pointB() const { return blend(&SupportPoint::b); }

    static constexpr int kMaxIterations = 64;

private:
    bool addIfNew(const SupportPoint& w) {
        for (auto& v : poly_)
            if (lengthSq(v.p - w.p) < 1e-10f) return false;
        if (poly_.size() == 2 && std::abs(cross(poly_[1].p - poly_[0].p, w.p - poly_[0].p)) < 1e-8f) return false;
        poly_.push_back(w);
        return true;
    }

    void findClosestEdge() {
        depth_ = std::numeric_limits<float>::infinity();
        int n = static_cast<int>(poly_.size());
        for (int i = 0; i < n; ++i) {
            Vec2 e = poly_[(i + 1) % n].p - poly_[i].p;
            if (lengthSq(e) < 1e-12f) continue;
            Vec2 nrm = normalized({e.y, -e.x});
            float d = dot(poly_[i].p, nrm);
            if (d < depth_) depth_ = d, edge_ = i, normal_ = nrm;
        }
        if (depth_ < 0) depth_ = 0;  // 原点恰好在边界上
    }

    Vec2 blend(Vec2 SupportPoint::* m) const {
        const SupportPoint& p = poly_[edge_];
        const SupportPoint& q = poly_[(edge_ + 1) % poly_.size()];
        Vec2 e = q.p - p.p;
        float t = std::clamp(dot(-p.p, e) / std::max(dot(e, e), 1e-12f), 0.0f, 1.0f);
        return p.*m * (1 - t) + q.*m * t;
    }

    const ConvexShape* a_;
    const ConvexShape* b_;
    float tol_;
    std::vector<SupportPoint> poly_;
    int edge_ = 0;
    Vec2 normal_{1, 0};
    float depth_ = 0;
    int iterations_ = 0;
    bool done_ = false;
};

// #region query
// 一次完整的查询：分离时给出距离和最近点，相交时给出穿透深度、方向和接触点
struct Proximity {
    bool intersecting;
    float distance;  // 分离时为正；相交时为负的穿透深度
    Vec2 normal;     // 从 A 指向 B
    Vec2 pointA, pointB;
};

inline Proximity query(const ConvexShape& a, const ConvexShape& b) {
    Gjk gjk(a, b);
    if (gjk.run() == Gjk::Status::Separated) {
        Vec2 pa = gjk.pointA(), pb = gjk.pointB();
        return {false, gjk.distance(), normalized(pb - pa), pa, pb};
    }
    Epa epa(a, b, gjk.simplex());
    epa.run();
    return {true, -epa.depth(), epa.normal(), epa.pointA(), epa.pointB()};
}
// #endregion

}  // namespace ga
