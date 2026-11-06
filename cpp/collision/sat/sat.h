#pragma once

// 分离轴定理（Separating Axis Theorem，SAT）：两个凸形状不相交，当且仅当存在一条轴，
// 使它们在这条轴上的投影不重叠。对凸多边形来说，只需要检查两者所有边的法线。

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>

#include "../intersect/intersect.h"
#include "vec2.h"

namespace ga {

// #region polygon
// 凸多边形。顶点按屏幕上的顺时针排列（y 朝下），此时边 e 的外法线是 (e.y, -e.x)
struct ConvexPolygon {
    std::vector<Vec2> v;

    int size() const { return static_cast<int>(v.size()); }
    Vec2 vertex(int i) const { return v[i % v.size()]; }
    Vec2 edge(int i) const { return vertex(i + 1) - vertex(i); }
    // 第 i 条边（v[i] → v[i+1]）的单位外法线
    Vec2 normal(int i) const {
        Vec2 e = edge(i);
        return normalized({e.y, -e.x});
    }
};
// #endregion

// 有向面积的两倍：顺时针（屏幕上）为正
inline float signedArea2(const std::vector<Vec2>& pts) {
    float s = 0;
    for (size_t i = 0; i < pts.size(); ++i) s += cross(pts[i], pts[(i + 1) % pts.size()]);
    return s;
}

// 从任意顺序的凸多边形顶点构造：统一成顺时针
inline ConvexPolygon makePolygon(std::vector<Vec2> pts) {
    if (signedArea2(pts) < 0) std::reverse(pts.begin(), pts.end());
    return {std::move(pts)};
}

inline ConvexPolygon regularPolygon(Vec2 center, float radius, int n, float angle = 0) {
    ConvexPolygon p;
    for (int i = 0; i < n; ++i) p.v.push_back(center + fromAngle(angle + 2 * kPi * i / n) * radius);
    return p;
}

// 旋转后的矩形（Oriented Bounding Box，OBB）
inline ConvexPolygon makeBox(Vec2 center, Vec2 half, float angle = 0) {
    ConvexPolygon p;
    for (Vec2 c : {Vec2{-half.x, -half.y}, Vec2{half.x, -half.y}, Vec2{half.x, half.y}, Vec2{-half.x, half.y}})
        p.v.push_back(center + rotate(c, angle));
    return p;
}

// 把局部坐标下的形状旋转 angle、平移到 pos
inline ConvexPolygon transformed(const ConvexPolygon& local, Vec2 pos, float angle) {
    ConvexPolygon p;
    p.v.reserve(local.v.size());
    for (Vec2 q : local.v) p.v.push_back(pos + rotate(q, angle));
    return p;
}

inline Vec2 centroid(const ConvexPolygon& p) {
    Vec2 s{0, 0};
    for (Vec2 q : p.v) s += q;
    return s / static_cast<float>(p.size());
}

// 严格在内部（落在边上不算）
inline bool contains(const ConvexPolygon& p, Vec2 q) {
    for (int i = 0; i < p.size(); ++i)
        if (dot(q - p.vertex(i), p.normal(i)) >= 0) return false;
    return true;
}

// #region project
// 形状在单位向量 axis 上的投影是一个区间：所有顶点与 axis 点积的最小值和最大值
struct Interval {
    float min, max;
};

inline Interval project(const ConvexPolygon& p, Vec2 axis) {
    float lo = dot(p.v[0], axis), hi = lo;
    for (int i = 1; i < p.size(); ++i) {
        float d = dot(p.v[i], axis);
        lo = std::min(lo, d);
        hi = std::max(hi, d);
    }
    return {lo, hi};
}

// 圆的投影：圆心的投影 ± 半径
inline Interval project(const Circle& c, Vec2 axis) {
    float m = dot(c.center, axis);
    return {m - c.radius, m + c.radius};
}
// #endregion

// #region sat
// 只要有一条轴把两个投影分开，就一定不相交 —— 可以立即返回
template <class A, class B>
bool separatedOn(const A& a, const B& b, Vec2 axis) {
    Interval ia = project(a, axis), ib = project(b, axis);
    return ia.max <= ib.min || ib.max <= ia.min;
}

inline bool overlaps(const ConvexPolygon& a, const ConvexPolygon& b) {
    for (int i = 0; i < a.size(); ++i)
        if (separatedOn(a, b, a.normal(i))) return false;
    for (int i = 0; i < b.size(); ++i)
        if (separatedOn(a, b, b.normal(i))) return false;
    return true;  // 所有候选轴上都重叠：相交
}
// #endregion

// #region circle-axes
// 多边形与圆：圆没有边，候选轴是多边形的边法线，再加上"离圆心最近的顶点 → 圆心"这一条
inline Vec2 closestVertexAxis(const ConvexPolygon& p, Vec2 center) {
    Vec2 best = p.v[0];
    for (Vec2 q : p.v)
        if (lengthSq(q - center) < lengthSq(best - center)) best = q;
    Vec2 d = normalized(center - best);
    return d == Vec2{0, 0} ? Vec2{1, 0} : d;
}

inline bool overlaps(const ConvexPolygon& a, const Circle& b) {
    for (int i = 0; i < a.size(); ++i)
        if (separatedOn(a, b, a.normal(i))) return false;
    return !separatedOn(a, b, closestVertexAxis(a, b.center));
}
// #endregion

// #region axis-test
// 在一条轴上，b 可以往 +axis 推出（推到 a 的投影之外），也可以往 -axis 推出，取较短的一个。
// push ≤ 0 表示这条轴已经把两者分开了
struct AxisTest {
    Vec2 axis;    // 推出方向（已带符号）：把 b 沿它移动 push 就能在这条轴上分开
    float push;
};

inline AxisTest testAxis(Interval a, Interval b, Vec2 axis) {
    float forward = a.max - b.min;   // 沿 +axis 推出 b 需要的距离
    float backward = b.max - a.min;  // 沿 -axis 推出 b 需要的距离
    return forward < backward ? AxisTest{axis, forward} : AxisTest{-axis, backward};
}
// #endregion

// #region contact
// 最小分离向量：所有候选轴里 push 最小的那条。返回的 normal 从 a 指向 b
inline std::optional<Contact> contact(const ConvexPolygon& a, const ConvexPolygon& b) {
    AxisTest best{{0, 0}, std::numeric_limits<float>::infinity()};
    auto consider = [&](Vec2 axis) {
        AxisTest t = testAxis(project(a, axis), project(b, axis), axis);
        if (t.push < best.push) best = t;
        return t.push > 0;  // false：找到了分离轴
    };
    for (int i = 0; i < a.size(); ++i)
        if (!consider(a.normal(i))) return std::nullopt;
    for (int i = 0; i < b.size(); ++i)
        if (!consider(b.normal(i))) return std::nullopt;
    return Contact{best.axis, best.push, {0, 0}};  // 接触点由 collide() 计算
}
// #endregion

inline std::optional<Contact> contact(const ConvexPolygon& a, const Circle& b) {
    AxisTest best{{0, 0}, std::numeric_limits<float>::infinity()};
    auto consider = [&](Vec2 axis) {
        AxisTest t = testAxis(project(a, axis), project(b, axis), axis);
        if (t.push < best.push) best = t;
        return t.push > 0;
    };
    for (int i = 0; i < a.size(); ++i)
        if (!consider(a.normal(i))) return std::nullopt;
    if (!consider(closestVertexAxis(a, b.center))) return std::nullopt;
    // 圆上最深入多边形的点
    return Contact{best.axis, best.push, b.center - best.axis * b.radius};
}

// #region manifold
// 接触流形：物理引擎需要的不只是一个点。两个盒子面贴面压在一起时，接触是一条线段，
// 用两个端点表示，才能让箱子稳稳地放平而不是绕着一个点摇晃。
struct Manifold {
    Vec2 normal;        // 从 a 指向 b
    float depth;
    int count = 0;      // 接触点个数：1 或 2
    Vec2 points[2];
    float depths[2];    // 每个接触点各自的穿透深度
};

// 外法线与 n 最接近的边
inline int mostAlignedEdge(const ConvexPolygon& p, Vec2 n) {
    int best = 0;
    float bestDot = -std::numeric_limits<float>::infinity();
    for (int i = 0; i < p.size(); ++i) {
        float d = dot(p.normal(i), n);
        if (d > bestDot) bestDot = d, best = i;
    }
    return best;
}

// 保留线段 in 上满足 dot(p, dir) ≤ limit 的部分
inline int clipSegment(const Vec2 in[2], Vec2 out[2], Vec2 dir, float limit) {
    int k = 0;
    float d0 = dot(in[0], dir) - limit, d1 = dot(in[1], dir) - limit;
    if (d0 <= 0) out[k++] = in[0];
    if (d1 <= 0) out[k++] = in[1];
    if (d0 * d1 < 0) out[k++] = in[0] + (in[1] - in[0]) * (d0 / (d0 - d1));
    return k;
}

inline std::optional<Manifold> collide(const ConvexPolygon& a, const ConvexPolygon& b) {
    auto c = contact(a, b);
    if (!c) return std::nullopt;
    Vec2 n = c->normal;
    // 1. 两边各找一条最"正对"对方的边。更垂直于 n 的那条作为参考边，另一条是入射边
    int ea = mostAlignedEdge(a, n), eb = mostAlignedEdge(b, -n);
    bool refIsA = dot(a.normal(ea), n) >= dot(b.normal(eb), -n) - 1e-3f;  // 略偏向 a，避免来回跳
    const ConvexPolygon& ref = refIsA ? a : b;
    const ConvexPolygon& inc = refIsA ? b : a;
    int er = refIsA ? ea : eb, ei = refIsA ? eb : ea;

    // 2. 用参考边两端的侧面裁剪入射边
    Vec2 r0 = ref.vertex(er), r1 = ref.vertex(er + 1);
    Vec2 t = normalized(r1 - r0);
    Vec2 seg[2] = {inc.vertex(ei), inc.vertex(ei + 1)}, tmp[2], out[2];
    if (clipSegment(seg, tmp, -t, -dot(r0, t)) < 2) return Manifold{n, c->depth, 1, {seg[0]}, {c->depth}};
    if (clipSegment(tmp, out, t, dot(r1, t)) < 2) return Manifold{n, c->depth, 1, {tmp[0]}, {c->depth}};

    // 3. 只保留在参考边"后面"（已经穿透进去）的点
    Manifold m{n, c->depth};
    Vec2 rn = ref.normal(er);
    float face = dot(r0, rn);
    for (Vec2 p : out) {
        float sep = dot(p, rn) - face;
        if (sep <= 0) {
            m.points[m.count] = p;
            m.depths[m.count++] = -sep;
        }
    }
    if (m.count == 0) {  // 数值误差导致两个点都在外面：退化为一个点
        m.count = 1;
        m.points[0] = (out[0] + out[1]) * 0.5f;
        m.depths[0] = c->depth;
    }
    return m;
}
// #endregion

// #region cache
// 时间相干性：物体每帧只移动一点点，上一帧的分离轴这一帧很可能仍然有效。
// 记住它，下一帧先试这一条。axis 是候选轴的编号：[0, a.size()) 属于 a，之后属于 b
struct SatCache {
    int axis = -1;
};

inline bool overlapsCached(const ConvexPolygon& a, const ConvexPolygon& b, SatCache& cache) {
    int na = a.size(), n = na + b.size();
    auto axisAt = [&](int i) { return i < na ? a.normal(i) : b.normal(i - na); };
    if (cache.axis >= 0 && cache.axis < n && separatedOn(a, b, axisAt(cache.axis))) return false;
    for (int i = 0; i < n; ++i) {
        if (i != cache.axis && separatedOn(a, b, axisAt(i))) {
            cache.axis = i;
            return false;
        }
    }
    cache.axis = -1;
    return true;
}
// #endregion

}  // namespace ga
