#pragma once

// 反向动力学（IK）：已知末端要到达的位置，反过来求每个关节的角度
//   - 链：p[0] 是固定的基座，p[i] → p[i+1] 是第 i 节骨头，长度 len[i] 不变
//   - CCD（循环坐标下降）：从最末一个关节往回，每个关节转一下，让"关节 → 末端"对准"关节 → 目标"
//   - FABRIK：先把末端放到目标上，沿链往回拉（每节保持长度）；再把基座放回原位，往前拉。来回几趟就收敛
//   - 对照：雅可比转置、阻尼最小二乘（DLS）把关节角当作变量做数值优化；两节骨头有解析解（余弦定理）
// 角度限制：每节骨头相对于上一节（第 0 节相对于 baseDir）的转角限制在 [-limit, limit] 内

#include <algorithm>
#include <cmath>
#include <vector>

#include "vec2.h"

namespace ga::ik {

// #region chain
struct Chain {
    std::vector<Vec2> p;      // n + 1 个关节位置（最后一个是末端）
    std::vector<float> len;   // n 节骨头的长度
    Vec2 baseDir{1, 0};       // 第 0 节骨头的"零角度"方向
    float limit = 0;          // 每个关节的转角上限（弧度）；0 = 不限制

    int bones() const { return int(len.size()); }
    Vec2 end() const { return p.back(); }
    float total() const {
        float s = 0;
        for (float l : len) s += l;
        return s;
    }
};

// 沿 dir 方向伸直的链
inline Chain straightChain(Vec2 base, const std::vector<float>& lengths, Vec2 dir) {
    Chain c;
    c.len = lengths;
    c.baseDir = normalized(dir);
    c.p.push_back(base);
    for (float l : lengths) c.p.push_back(c.p.back() + c.baseDir * l);
    return c;
}

// 每节骨头相对于上一节的转角（弧度，-π..π）
inline std::vector<float> relativeAngles(const Chain& c) {
    std::vector<float> a;
    Vec2 prev = c.baseDir;
    for (int i = 0; i < c.bones(); ++i) {
        Vec2 d = c.p[size_t(i) + 1] - c.p[size_t(i)];
        a.push_back(signedAngle(prev, d));
        prev = d;
    }
    return a;
}
// 正向运动学：由相对转角得到关节位置
inline void applyAngles(Chain& c, const std::vector<float>& a) {
    float abs = angleOf(c.baseDir);
    for (int i = 0; i < c.bones(); ++i) {
        abs += a[size_t(i)];
        c.p[size_t(i) + 1] = c.p[size_t(i)] + fromAngle(abs) * c.len[size_t(i)];
    }
}
// #endregion

// #region ccd
// CCD 的一趟：从最末一个关节到基座，每个关节转动"关节 → 末端"到"关节 → 目标"的夹角，
// 带着它后面的整段链一起转。有角度限制时，把这个关节的相对转角截断到限制以内
inline void ccdSweep(Chain& c, Vec2 target) {
    int n = c.bones();
    for (int i = n - 1; i >= 0; --i) {
        Vec2 pi = c.p[size_t(i)];
        Vec2 toEnd = c.end() - pi, toTarget = target - pi;
        if (lengthSq(toEnd) < 1e-12f || lengthSq(toTarget) < 1e-12f) continue;
        float rot = signedAngle(toEnd, toTarget);
        if (c.limit > 0) {
            Vec2 parent = i == 0 ? c.baseDir : c.p[size_t(i)] - c.p[size_t(i) - 1];
            float rel = signedAngle(parent, c.p[size_t(i) + 1] - pi);
            rot = std::clamp(rel + rot, -c.limit, c.limit) - rel;
        }
        float cs = std::cos(rot), sn = std::sin(rot);
        for (int k = i + 1; k <= n; ++k) {
            Vec2 d = c.p[size_t(k)] - pi;
            c.p[size_t(k)] = pi + Vec2{d.x * cs - d.y * sn, d.x * sn + d.y * cs};
        }
    }
}
// #endregion

// #region fabrik
// 把 dir 截断到和 parent 夹角不超过 limit 的范围内
inline Vec2 clampDirection(Vec2 parent, Vec2 dir, float limit) {
    float a = signedAngle(parent, dir);
    if (std::fabs(a) <= limit) return dir;
    return rotate(normalized(parent), a > 0 ? limit : -limit) * length(dir);
}
// 沿 from → to 方向放一个距离 from 为 len 的点；两点重合时沿 fallback 方向
inline Vec2 placeAlong(Vec2 from, Vec2 to, float len, Vec2 fallback) {
    Vec2 d = to - from;
    float l = length(d);
    return from + (l > 1e-9f ? d / l : normalized(fallback)) * len;
}

// FABRIK 的一趟：后向（末端贴到目标，往基座拉）+ 前向（基座放回原位，往末端拉）。
// 角度限制只在前向时施加：每节骨头的方向截断到相对上一节的限制以内
inline void fabrikSweep(Chain& c, Vec2 target) {
    int n = c.bones();
    Vec2 base = c.p[0];
    c.p[size_t(n)] = target;
    for (int i = n - 1; i >= 0; --i) c.p[size_t(i)] = placeAlong(c.p[size_t(i) + 1], c.p[size_t(i)], c.len[size_t(i)], c.p[size_t(i)] - c.p[size_t(i) + 1]);
    c.p[0] = base;
    Vec2 parent = c.baseDir;
    for (int i = 0; i < n; ++i) {
        Vec2 d = c.p[size_t(i) + 1] - c.p[size_t(i)];
        if (lengthSq(d) < 1e-18f) d = parent;
        if (c.limit > 0) d = clampDirection(parent, d, c.limit);
        c.p[size_t(i) + 1] = placeAlong(c.p[size_t(i)], c.p[size_t(i)] + d, c.len[size_t(i)], parent);
        parent = c.p[size_t(i) + 1] - c.p[size_t(i)];
    }
}
// #endregion

// #region jacobian
// 雅可比矩阵：转动关节 i 时末端的速度 = perp(末端 − p[i])（2 × n）。
// 转置法：Δθ = α Jᵀe，α 取沿这个方向的最优步长；阻尼最小二乘：Δθ = Jᵀ (J Jᵀ + λ² I)⁻¹ e
enum class JacobianMethod { Transpose, DampedLeastSquares };
inline void jacobianStep(Chain& c, Vec2 target, JacobianMethod m, float lambda) {
    int n = c.bones();
    Vec2 e = target - c.end();
    std::vector<Vec2> col(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i) col[size_t(i)] = perp(c.end() - c.p[size_t(i)]);
    std::vector<float> dtheta(static_cast<size_t>(n));
    if (m == JacobianMethod::Transpose) {
        Vec2 jjte{0, 0};
        for (int i = 0; i < n; ++i) jjte += col[size_t(i)] * dot(col[size_t(i)], e);
        float denom = lengthSq(jjte);
        float alpha = denom > 1e-18f ? dot(e, jjte) / denom : 0;
        for (int i = 0; i < n; ++i) dtheta[size_t(i)] = alpha * dot(col[size_t(i)], e);
    } else {
        // J Jᵀ 是 2 × 2 对称矩阵
        double a = lambda * lambda, b = 0, d = lambda * lambda;
        for (int i = 0; i < n; ++i) {
            Vec2 j = col[size_t(i)];
            a += j.x * j.x, b += j.x * j.y, d += j.y * j.y;
        }
        double det = a * d - b * b;
        Vec2 f{float((d * e.x - b * e.y) / det), float((-b * e.x + a * e.y) / det)};
        for (int i = 0; i < n; ++i) dtheta[size_t(i)] = dot(col[size_t(i)], f);
    }
    // 关节角是相对角：转动关节 i 带动它后面的所有骨头，正好对应相对角 i 的增量
    std::vector<float> ang = relativeAngles(c);
    for (int i = 0; i < n; ++i) {
        ang[size_t(i)] += dtheta[size_t(i)];
        if (c.limit > 0) ang[size_t(i)] = std::clamp(ang[size_t(i)], -c.limit, c.limit);
    }
    applyAngles(c, ang);
}
// #endregion

// #region two-bone
// 两节骨头的解析解：余弦定理求肘部的角度。bendSign = ±1 选择肘部弯向哪一边。
// 目标太远或太近时，把距离截断到可达范围 [|l1 − l2|, l1 + l2]，得到离目标最近的姿势
inline void twoBone(Chain& c, Vec2 target, float bendSign = 1) {
    float l1 = c.len[0], l2 = c.len[1];
    Vec2 base = c.p[0], d = target - base;
    float dist = std::clamp(length(d), std::fabs(l1 - l2) + 1e-6f, l1 + l2 - 1e-6f);
    float cosA = std::clamp((l1 * l1 + dist * dist - l2 * l2) / (2 * l1 * dist), -1.0f, 1.0f);  // 基座处的夹角
    Vec2 dir = lengthSq(d) > 0 ? normalized(d) : c.baseDir;
    Vec2 elbow = base + rotate(dir, bendSign * std::acos(cosA)) * l1;
    c.p[1] = elbow;
    c.p[2] = elbow + normalized(base + dir * dist - elbow) * l2;
}
// #endregion

// #region solve
enum class Method { CCD, FABRIK, Transpose, DLS };
struct SolveResult {
    int iterations;
    float error;  // 末端到目标的距离
};
inline void sweep(Chain& c, Vec2 target, Method m, float lambda) {
    switch (m) {
        case Method::CCD: ccdSweep(c, target); break;
        case Method::FABRIK: fabrikSweep(c, target); break;
        case Method::Transpose: jacobianStep(c, target, JacobianMethod::Transpose, lambda); break;
        case Method::DLS: jacobianStep(c, target, JacobianMethod::DampedLeastSquares, lambda); break;
    }
}
// 迭代到误差小于 tol 或者达到 maxIter
inline SolveResult solve(Chain& c, Vec2 target, Method m, int maxIter, float tol, float lambda = -1) {
    if (lambda < 0) lambda = 0.1f * c.total();
    int it = 0;
    float err = length(c.end() - target);
    while (it < maxIter && err > tol) {
        sweep(c, target, m, lambda);
        ++it;
        err = length(c.end() - target);
    }
    return {it, err};
}
// #endregion

}  // namespace ga::ik
