#pragma once

// 贝塞尔曲线与 Catmull-Rom 样条
//   - 贝塞尔：控制点决定形状，曲线只经过首尾两点。de Casteljau（反复线性插值）求值最稳定
//   - 参数 t 不是弧长：t 匀速增加时，点在曲线上忽快忽慢。要匀速移动，先算弧长表再反查
//   - 画出来要折成线段：自适应细分按平直程度切，Wang 公式直接给出均匀切分的段数上界
//   - Catmull-Rom：经过每一个控制点，切线由相邻两点决定。节点间距取 |Δp|^α：
//     α = 0 均匀，α = 0.5 向心（centripetal），α = 1 弦长（chordal）。向心参数化保证段内没有尖点和自交

#include <algorithm>
#include <cmath>
#include <vector>

#include "vec2.h"

namespace ga::spline {

// #region casteljau
// de Casteljau：相邻控制点按 t 线性插值，点数减一；重复到只剩一个点。n 个控制点（n - 1 次曲线）
// 只用凸组合（系数 1 - t 和 t 都在 [0, 1] 内），舍入误差不会被放大
template <class P, class T>
P deCasteljau(const P* pts, int n, T t) {
    P tmp[32];
    for (int i = 0; i < n; ++i) tmp[i] = pts[i];
    for (int k = n - 1; k > 0; --k)
        for (int i = 0; i < k; ++i) tmp[i] = tmp[i] * (T(1) - t) + tmp[i + 1] * t;
    return tmp[0];
}

// Bernstein 形式：B(t) = Σ C(d, i) tⁱ (1 - t)^(d - i) Pᵢ，直接相加
template <class P, class T>
P bernstein(const P* pts, int n, T t) {
    int d = n - 1;
    P sum = pts[0] * T(0);
    T binom = 1;
    for (int i = 0; i <= d; ++i) {
        T b = binom;
        for (int k = 0; k < i; ++k) b *= t;
        for (int k = 0; k < d - i; ++k) b *= T(1) - t;
        sum = sum + pts[i] * b;
        binom = binom * T(d - i) / T(i + 1);
    }
    return sum;
}

// 幂基（多项式系数）：先把控制点换成 a₀ + a₁t + … + a_d t^d 的系数，再用 Horner 求值。
// 每次求值最便宜，但换基时系数是控制点的正负交替组合，次数高了会互相抵消、丢掉有效数字
template <class P, class T>
std::vector<P> toPowerBasis(const P* pts, int n) {
    int d = n - 1;
    std::vector<P> a(size_t(n), pts[0] * T(0));
    T cdk = 1;  // C(d, k)
    for (int k = 0; k <= d; ++k) {
        P s = pts[0] * T(0);
        T cki = 1;  // C(k, i)
        for (int i = 0; i <= k; ++i) {
            T sign = ((k - i) & 1) ? T(-1) : T(1);
            s = s + pts[i] * (sign * cki);
            cki = cki * T(k - i) / T(i + 1);
        }
        a[size_t(k)] = s * cdk;
        cdk = cdk * T(d - k) / T(k + 1);
    }
    return a;
}
template <class P, class T>
P horner(const std::vector<P>& a, T t) {
    P r = a.back();
    for (int k = int(a.size()) - 2; k >= 0; --k) r = r * t + a[size_t(k)];
    return r;
}
// #endregion

inline Vec2 lerp(Vec2 a, Vec2 b, float t) { return a + (b - a) * t; }

// #region cubic
struct Cubic {
    Vec2 p[4];

    Vec2 at(float t) const {
        float u = 1 - t;
        return p[0] * (u * u * u) + p[1] * (3 * u * u * t) + p[2] * (3 * u * t * t) + p[3] * (t * t * t);
    }
    // 导数也是贝塞尔曲线（"速端曲线"），控制点是相邻控制点之差的 3 倍
    Vec2 d1(float t) const {
        float u = 1 - t;
        return (p[1] - p[0]) * (3 * u * u) + (p[2] - p[1]) * (6 * u * t) + (p[3] - p[2]) * (3 * t * t);
    }
    Vec2 d2(float t) const { return (p[2] - p[1] * 2 + p[0]) * (6 * (1 - t)) + (p[3] - p[2] * 2 + p[1]) * (6 * t); }
    // 有符号曲率 κ = (B' × B'') / |B'|³；|B'| = 0（尖点）处没有定义
    float curvature(float t) const {
        Vec2 a = d1(t), b = d2(t);
        float l = length(a);
        return l > 1e-12f ? cross(a, b) / (l * l * l) : 0;
    }
    // 在 t 处切成两条三次曲线：de Casteljau 的中间点正好是两半的控制点
    void split(float t, Cubic* left, Cubic* right) const {
        Vec2 a = lerp(p[0], p[1], t), b = lerp(p[1], p[2], t), c = lerp(p[2], p[3], t);
        Vec2 d = lerp(a, b, t), e = lerp(b, c, t), f = lerp(d, e, t);
        *left = {{p[0], a, d, f}};
        *right = {{f, e, c, p[3]}};
    }
};
// #endregion

// #region gauss
// Gauss–Legendre 求积：n 个节点精确积分 2n - 1 次多项式。节点是 Legendre 多项式 Pₙ 的根，用牛顿法求
struct Quadrature {
    std::vector<double> x, w;  // [-1, 1] 上的节点和权重
};
inline Quadrature gaussLegendre(int n) {
    Quadrature q;
    q.x.resize(size_t(n)), q.w.resize(size_t(n));
    for (int i = 0; i < n; ++i) {
        double z = std::cos(M_PI * (i + 0.75) / (n + 0.5)), dp = 0;
        for (int it = 0; it < 100; ++it) {
            double p0 = 1, p1 = z;  // 递推 P_k
            for (int k = 2; k <= n; ++k) {
                double p2 = ((2 * k - 1) * z * p1 - (k - 1) * p0) / k;
                p0 = p1, p1 = p2;
            }
            dp = n * (z * p1 - p0) / (z * z - 1);
            double dz = p1 / dp;
            z -= dz;
            if (std::fabs(dz) < 1e-15) break;
        }
        q.x[size_t(i)] = z;
        q.w[size_t(i)] = 2 / ((1 - z * z) * dp * dp);
    }
    return q;
}
// ∫ f(t) dt，t ∈ [a, b]
template <class F>
double integrate(const Quadrature& q, double a, double b, F&& f) {
    double h = (b - a) / 2, m = (a + b) / 2, s = 0;
    for (size_t i = 0; i < q.x.size(); ++i) s += q.w[i] * f(m + h * q.x[i]);
    return s * h;
}
// #endregion

// #region arclength
// 弧长 = ∫|B'(t)|dt。|B'| 不是多项式（有开方），尖点附近变化剧烈，所以把 [0, 1] 等分成若干段分别积分
inline double arcLength(const Cubic& c, double t0 = 0, double t1 = 1, int pieces = 8, int order = 5) {
    static const Quadrature q5 = gaussLegendre(5);
    Quadrature qn = order == 5 ? q5 : gaussLegendre(order);
    double s = 0;
    for (int i = 0; i < pieces; ++i) {
        double a = t0 + (t1 - t0) * i / pieces, b = t0 + (t1 - t0) * (i + 1) / pieces;
        s += integrate(qn, a, b, [&](double t) { return double(length(c.d1(float(t)))); });
    }
    return s;
}

// 弧长表：等分 t 记下累计弧长，反查时二分找到所在区间再线性插值；refine 时再用牛顿法修正一两步
class ArcLengthTable {
public:
    ArcLengthTable() = default;
    ArcLengthTable(const Cubic& c, int samples) : c_(c) {
        s_.assign(size_t(samples) + 1, 0);
        for (int i = 0; i < samples; ++i) s_[size_t(i) + 1] = s_[size_t(i)] + arcLength(c, double(i) / samples, double(i + 1) / samples, 1, 5);
    }
    double length() const { return s_.back(); }
    // 走过弧长 s 时的参数 t
    float paramAt(double s, int newtonSteps = 0) const {
        int n = int(s_.size()) - 1;
        if (s <= 0) return 0;
        if (s >= s_.back()) return 1;
        int i = int(std::upper_bound(s_.begin(), s_.end(), s) - s_.begin()) - 1;
        i = std::clamp(i, 0, n - 1);
        double seg = s_[size_t(i) + 1] - s_[size_t(i)];
        double t = (i + (seg > 0 ? (s - s_[size_t(i)]) / seg : 0)) / n;
        // 牛顿法：f(t) = 表里的累计弧长 + ∫(区间起点..t)|B'| − s，f'(t) = |B'(t)|。
        // 速度接近零（接近尖点）时牛顿步会冲出区间，所以维护一个包含根的区间，出界就改用二分
        double lo = double(i) / n, hi = double(i + 1) / n;
        for (int k = 0; k < newtonSteps; ++k) {
            double f = s_[size_t(i)] + arcLength(c_, double(i) / n, t, 1, 5) - s;
            if (f == 0) break;
            if (f > 0) hi = t;
            else lo = t;
            double d = ga::length(c_.d1(float(t)));
            double nt = d > 0 ? t - f / d : lo - 1;
            t = nt >= lo && nt <= hi ? nt : (lo + hi) / 2;
        }
        return float(t);
    }

private:
    Cubic c_{};
    std::vector<double> s_;
};
// #endregion

// #region flatten
// 点到线段的距离
inline float distToSegment(Vec2 p, Vec2 a, Vec2 b) {
    Vec2 ab = b - a;
    float l2 = lengthSq(ab);
    float t = l2 > 0 ? std::clamp(dot(p - a, ab) / l2, 0.0f, 1.0f) : 0;
    return length(p - (a + ab * t));
}

// 自适应细分：曲线在控制多边形的凸包里，所以 P1、P2 到弦 P0P3 的距离不超过 tol 时，整段离弦也不超过 tol
inline void flattenAdaptive(const Cubic& c, float tol, std::vector<Vec2>* out, int depth = 0) {
    if (out->empty()) out->push_back(c.p[0]);
    bool flat = distToSegment(c.p[1], c.p[0], c.p[3]) <= tol && distToSegment(c.p[2], c.p[0], c.p[3]) <= tol;
    if (flat || depth >= 16) {
        out->push_back(c.p[3]);
        return;
    }
    Cubic l, r;
    c.split(0.5f, &l, &r);
    flattenAdaptive(l, tol, out, depth + 1);
    flattenAdaptive(r, tol, out, depth + 1);
}

// Wang 公式：按 t 均匀切成 n 段，n = ⌈√(d(d - 1)·M / (8·tol))⌉，M = max |Pᵢ − 2Pᵢ₊₁ + Pᵢ₊₂|（二阶差分）。
// 这是误差的上界，所以保证够用，但通常偏多
inline int wangSegments(const Cubic& c, float tol) {
    float m = std::max(length(c.p[0] - c.p[1] * 2 + c.p[2]), length(c.p[1] - c.p[2] * 2 + c.p[3]));
    return std::max(1, int(std::ceil(std::sqrt(3 * 2 * m / (8 * tol)))));
}
inline void flattenUniform(const Cubic& c, int n, std::vector<Vec2>* out) {
    for (int i = 0; i <= n; ++i) out->push_back(c.at(float(i) / n));
}
// #endregion

// #region catmull
// Catmull-Rom 的一段：经过 p1、p2，切线由 p0、p3 决定。节点 tᵢ₊₁ = tᵢ + |pᵢ₊₁ − pᵢ|^α。
// Barry–Goldman 金字塔：三次线性插值（按节点区间），再两次，再一次。u ∈ [0, 1] 对应 t ∈ [t1, t2]
inline float knotStep(Vec2 a, Vec2 b, float alpha) {
    float d = std::pow(lengthSq(b - a), alpha * 0.5f);
    return std::max(d, 1e-4f);  // 两点重合时节点间距为 0，会除以零
}
inline Vec2 catmullRom(Vec2 p0, Vec2 p1, Vec2 p2, Vec2 p3, float alpha, float u) {
    float t0 = 0, t1 = t0 + knotStep(p0, p1, alpha), t2 = t1 + knotStep(p1, p2, alpha), t3 = t2 + knotStep(p2, p3, alpha);
    float t = t1 + (t2 - t1) * u;
    auto mix = [](Vec2 a, Vec2 b, float ta, float tb, float t) { return a * ((tb - t) / (tb - ta)) + b * ((t - ta) / (tb - ta)); };
    Vec2 a1 = mix(p0, p1, t0, t1, t), a2 = mix(p1, p2, t1, t2, t), a3 = mix(p2, p3, t2, t3, t);
    Vec2 b1 = mix(a1, a2, t0, t2, t), b2 = mix(a2, a3, t1, t3, t);
    return mix(b1, b2, t1, t2, t);
}
// 同一段写成三次贝塞尔：端点切线 m1 = (p1−p0)/(t1−t0) − (p2−p0)/(t2−t0) + (p2−p1)/(t2−t1)，乘以 (t2 − t1) 后按 u 计。
// 这样就能复用贝塞尔的弧长、细分和绘制
inline Cubic catmullRomToBezier(Vec2 p0, Vec2 p1, Vec2 p2, Vec2 p3, float alpha) {
    float d01 = knotStep(p0, p1, alpha), d12 = knotStep(p1, p2, alpha), d23 = knotStep(p2, p3, alpha);
    Vec2 m1 = ((p1 - p0) / d01 - (p2 - p0) / (d01 + d12) + (p2 - p1) / d12) * d12;
    Vec2 m2 = ((p2 - p1) / d12 - (p3 - p1) / (d12 + d23) + (p3 - p2) / d23) * d12;
    return {{p1, p1 + m1 / 3, p2 - m2 / 3, p2}};
}

// 一串点连成样条。closed = 首尾相接；否则在两端各外推一个虚拟点（p₋₁ = 2p₀ − p₁）
inline std::vector<Cubic> catmullRomChain(const std::vector<Vec2>& pts, float alpha, bool closed) {
    std::vector<Cubic> out;
    int n = int(pts.size());
    if (n < 2) return out;
    auto at = [&](int i) -> Vec2 {
        if (closed) return pts[size_t((i % n + n) % n)];
        if (i < 0) return pts[0] * 2 - pts[1];
        if (i >= n) return pts[size_t(n - 1)] * 2 - pts[size_t(n - 2)];
        return pts[size_t(i)];
    };
    int segs = closed ? n : n - 1;
    for (int i = 0; i < segs; ++i) out.push_back(catmullRomToBezier(at(i - 1), at(i), at(i + 1), at(i + 2), alpha));
    return out;
}
// #endregion

// #region defects
// 尖点：段内速度为零的点（端点处速度为零不算：那里接上了相邻的一段）。速端曲线 B'(t) 是二次贝塞尔曲线（控制点 3(Pᵢ₊₁ − Pᵢ)），
// 先求 x 分量的根，再看 y 分量在那里是否也为零（相对于速度的尺度）
inline bool hasCusp(const Cubic& c, float relTol = 1e-4f) {
    Vec2 h0 = (c.p[1] - c.p[0]) * 3, h1 = (c.p[2] - c.p[1]) * 3, h2 = (c.p[3] - c.p[2]) * 3;
    float scale = std::max({length(h0), length(h1), length(h2)});
    if (scale == 0) return true;
    auto H = [&](double t) { return c.d1(float(t)); };
    // x(t) = A t² + B t + C
    double A = double(h0.x) - 2.0 * h1.x + h2.x, B = 2.0 * (double(h1.x) - h0.x), C = h0.x;
    std::vector<double> roots;
    if (std::fabs(A) < 1e-12 * scale) {
        if (std::fabs(B) > 1e-12 * scale) roots.push_back(-C / B);
        else for (int i = 0; i <= 64; ++i) roots.push_back(i / 64.0);  // x 分量恒为零（竖直的直线）：退化，逐点看 y
    } else {
        double disc = B * B - 4 * A * C;
        if (disc >= 0) roots.push_back((-B + std::sqrt(disc)) / (2 * A)), roots.push_back((-B - std::sqrt(disc)) / (2 * A));
    }
    for (double t : roots)
        if (t > 1e-3 && t < 1 - 1e-3 && length(H(t)) <= relTol * scale) return true;
    return false;
}
// 自交：折线化后有不相邻的线段相交
inline bool segmentsCross(Vec2 a, Vec2 b, Vec2 c, Vec2 d) {
    float d1 = cross(b - a, c - a), d2 = cross(b - a, d - a), d3 = cross(d - c, a - c), d4 = cross(d - c, b - c);
    return ((d1 > 0) != (d2 > 0)) && ((d3 > 0) != (d4 > 0)) && d1 != 0 && d2 != 0 && d3 != 0 && d4 != 0;
}
struct Defects {
    bool cusp = false, loop = false;
};
inline bool polylineSelfCrosses(const Cubic& c, int samples) {
    std::vector<Vec2> poly;
    flattenUniform(c, samples, &poly);
    for (int i = 0; i + 1 < int(poly.size()); ++i)
        for (int j = i + 2; j + 1 < int(poly.size()); ++j)
            if (segmentsCross(poly[size_t(i)], poly[size_t(i) + 1], poly[size_t(j)], poly[size_t(j) + 1])) return true;
    return false;
}
// 急转弯处相邻的线段几乎共线，叉积接近 0，舍入可能把它们误判为相交（是否误报还取决于编译器是否合并 FMA，见 6.4）。
// 所以发现相交后用 10 倍的采样再确认一次
inline Defects findDefects(const Cubic& c, int samples = 200) {
    Defects r;
    r.cusp = hasCusp(c);
    r.loop = polylineSelfCrosses(c, samples) && polylineSelfCrosses(c, samples * 10);
    return r;
}
// #endregion

}  // namespace ga::spline
