#pragma once

// 视锥剔除与遮挡剔除（二维版：视锥是一个梯形，"屏幕"是一维的）
//   - 视锥 = 四个平面（左、右、近、远）。物体的包围盒在某个平面的外侧就看不见
//   - 平面测试只看"是否在某个平面外"，视锥角落附近的盒子可能每个平面都不在外侧、却和视锥不相交（假阳性）。
//     再加上盒子自己的两个轴（分离轴定理），才是精确的相交测试
//   - 层次剔除：包围盒层次（BVH）的节点在视锥外，整棵子树都跳过；节点完全在某个平面内侧，子树就不用再测这个平面
//   - 遮挡剔除：把挡在前面的大物体（墙、楼）光栅化进一维深度缓冲，每个"像素"记下遮挡物在这一列里最远的深度；
//     物体最近的点比它覆盖的每一列都远，就被挡住了。深度缓冲天然把多个遮挡物"融合"起来
//   - Hi-Z：深度缓冲的最大值金字塔。大物体只需查高层的一两个格子

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "../../collision/intersect/intersect.h"  // Aabb、overlaps
#include "vec2.h"

namespace ga::culling {

constexpr float kInf = std::numeric_limits<float>::infinity();

// #region frustum
// 平面（二维里是直线）：dot(n, p) + d ≥ 0 为内侧
struct Plane {
    Vec2 n;
    float d;
    float distance(Vec2 p) const { return dot(n, p) + d; }
};

struct View {
    Vec2 eye, dir;  // dir 为单位向量
    float halfFov, near, far;
    Vec2 right() const { return perp(dir); }
    // 视图空间：x 向右，z 向前（深度）
    Vec2 toView(Vec2 p) const {
        Vec2 d = p - eye;
        return {dot(d, right()), dot(d, dir)};
    }
};

struct Frustum {
    std::array<Plane, 4> planes;  // 左、右、近、远
    std::array<Vec2, 4> corners;  // 近左、近右、远右、远左
    Aabb bounds;                  // 四个角的包围盒：精确测试用
};

inline Frustum makeFrustum(const View& v) {
    Frustum f;
    Vec2 l = rotate(v.dir, -v.halfFov), r = rotate(v.dir, v.halfFov);
    Vec2 nl = perp(l), nr = -perp(r);  // 内侧法线：指向视线方向那一边
    f.planes[0] = {nl, -dot(nl, v.eye)};
    f.planes[1] = {nr, -dot(nr, v.eye)};
    f.planes[2] = {v.dir, -dot(v.dir, v.eye) - v.near};
    f.planes[3] = {-v.dir, dot(v.dir, v.eye) + v.far};
    float c = std::cos(v.halfFov);
    f.corners = {v.eye + l * (v.near / c), v.eye + r * (v.near / c), v.eye + r * (v.far / c), v.eye + l * (v.far / c)};
    f.bounds = {f.corners[0], f.corners[0]};
    for (Vec2 p : f.corners)
        f.bounds.min = {std::min(f.bounds.min.x, p.x), std::min(f.bounds.min.y, p.y)},
        f.bounds.max = {std::max(f.bounds.max.x, p.x), std::max(f.bounds.max.y, p.y)};
    return f;
}
// #endregion

// #region classify
enum class Cull { Outside, Intersect, Inside };

// 盒子对每个平面：离平面最"内"的角（p 顶点）在外侧 → 整个盒子在外侧；
// 离平面最"外"的角（n 顶点）也在内侧 → 整个盒子在这个平面内侧，以后不用再测（清掉 mask 里的这一位）
inline Cull classify(const Frustum& f, const Aabb& b, unsigned& mask, long& tests) {
    Cull result = Cull::Inside;
    for (int i = 0; i < 4; ++i) {
        if (!(mask & (1u << i))) continue;
        ++tests;
        const Plane& p = f.planes[i];
        Vec2 pv{p.n.x >= 0 ? b.max.x : b.min.x, p.n.y >= 0 ? b.max.y : b.min.y};
        if (p.distance(pv) < 0) return Cull::Outside;
        Vec2 nv{p.n.x >= 0 ? b.min.x : b.max.x, p.n.y >= 0 ? b.min.y : b.max.y};
        if (p.distance(nv) >= 0) mask &= ~(1u << i);
        else result = Cull::Intersect;
    }
    return result;
}
inline Cull classify(const Frustum& f, const Aabb& b) {
    unsigned mask = 15;
    long tests = 0;
    return classify(f, b, mask, tests);
}
// 精确：平面测试没有排除，再测盒子自己的两个轴（视锥的包围盒和它是否重叠）
inline bool overlapsExact(const Frustum& f, const Aabb& b) {
    return classify(f, b) != Cull::Outside && overlaps(f.bounds, b);
}
// 包围圆：圆心到某个平面的距离小于 −r 就在外侧
inline bool circleOutside(const Frustum& f, Vec2 c, float r) {
    for (const Plane& p : f.planes)
        if (p.distance(c) < -r) return true;
    return false;
}
// #endregion

// #region bvh
// 静态包围盒层次：每个节点的物体在 items 里连续存放，"整棵子树都可见"时可以直接整段输出
struct Bvh {
    struct Node {
        Aabb box;
        int left = -1, right = -1, first = 0, count = 0;
        bool leaf() const { return left < 0; }
    };
    std::vector<Node> nodes;
    std::vector<int> items;

    void build(const std::vector<Aabb>& boxes, int leafSize = 4) {
        nodes.clear();
        items.resize(boxes.size());
        for (size_t i = 0; i < boxes.size(); ++i) items[i] = int(i);
        if (!boxes.empty()) buildNode(boxes, 0, int(boxes.size()), leafSize);
    }

private:
    int buildNode(const std::vector<Aabb>& boxes, int first, int count, int leafSize) {
        int id = int(nodes.size());
        nodes.push_back({});
        Aabb box = boxes[items[first]];
        for (int i = first; i < first + count; ++i) {
            const Aabb& b = boxes[items[i]];
            box.min = {std::min(box.min.x, b.min.x), std::min(box.min.y, b.min.y)};
            box.max = {std::max(box.max.x, b.max.x), std::max(box.max.y, b.max.y)};
        }
        nodes[id].box = box, nodes[id].first = first, nodes[id].count = count;
        if (count <= leafSize) return id;
        bool xAxis = box.max.x - box.min.x >= box.max.y - box.min.y;
        auto centre = [&](int i) { const Aabb& b = boxes[i]; return xAxis ? b.min.x + b.max.x : b.min.y + b.max.y; };
        int mid = first + count / 2;
        std::nth_element(items.begin() + first, items.begin() + mid, items.begin() + first + count,
                         [&](int a, int b) { return centre(a) < centre(b); });
        int l = buildNode(boxes, first, mid - first, leafSize);
        int r = buildNode(boxes, mid, first + count - mid, leafSize);
        nodes[id].left = l, nodes[id].right = r;
        return id;
    }
};

struct CullStats {
    long planeTests = 0, nodes = 0;
};

// 逐个物体测试：对照
template <class Visit>
void cullBrute(const Frustum& f, const std::vector<Aabb>& boxes, Visit&& visit, CullStats& st) {
    for (size_t i = 0; i < boxes.size(); ++i) {
        unsigned mask = 15;
        if (classify(f, boxes[i], mask, st.planeTests) != Cull::Outside) visit(int(i));
    }
}

// 层次剔除。useMasks：子树完全在某个平面内侧，就不再测这个平面；全部平面都在内侧，整段输出、不再测试
template <class Visit>
void cullBvh(const Bvh& t, const Frustum& f, const std::vector<Aabb>& boxes, bool useMasks, Visit&& visit,
             CullStats& st, int node = 0, unsigned mask = 15) {
    if (t.nodes.empty()) return;
    const Bvh::Node& n = t.nodes[node];
    ++st.nodes;
    if (!useMasks) mask = 15;
    if (classify(f, n.box, mask, st.planeTests) == Cull::Outside) return;
    if (useMasks && mask == 0) {
        for (int i = n.first; i < n.first + n.count; ++i) visit(t.items[i]);
        return;
    }
    if (n.leaf()) {
        for (int i = n.first; i < n.first + n.count; ++i) {
            unsigned m = useMasks ? mask : 15u;
            if (classify(f, boxes[t.items[i]], m, st.planeTests) != Cull::Outside) visit(t.items[i]);
        }
        return;
    }
    cullBvh(t, f, boxes, useMasks, visit, st, n.left, mask);
    cullBvh(t, f, boxes, useMasks, visit, st, n.right, mask);
}
// #endregion

// #region raster
// 一维深度缓冲。屏幕坐标 s = x / (z·tan(halfFov)) ∈ [−1, 1]，分成 bins 列（2 的幂，Hi-Z 需要）。
// 每列存"这一列被遮挡物完全盖住时，遮挡物在这一列里最远的深度"；没有完全盖住就保持 +∞
class DepthBuffer {
public:
    explicit DepthBuffer(int bins) : n_(bins) {}

    int bins() const { return n_; }
    const View& view() const { return view_; }
    float depth(int i) const { return levels_[0][i]; }
    int levelCount() const { return int(levels_.size()); }
    const std::vector<float>& level(int k) const { return levels_[k]; }

    void begin(const View& v) {
        view_ = v;
        tan_ = std::tan(v.halfFov);
        levels_.assign(1, std::vector<float>(n_, kInf));
    }
    float screenX(Vec2 viewPt) const { return viewPt.x / (viewPt.y * tan_); }
    float binEdge(int i) const { return -1 + 2.0f * float(i) / float(n_); }

    // 光栅化一个凸多边形遮挡物（世界坐标；2 个点 = 一段墙）。返回写入的列数
    int rasterize(const std::vector<Vec2>& world) {
        poly_.clear();
        for (Vec2 p : world) poly_.push_back(view_.toView(p));
        clipNear();
        if (poly_.size() < 2) return 0;
        float smin = kInf, smax = -kInf;
        for (Vec2 p : poly_) smin = std::min(smin, screenX(p)), smax = std::max(smax, screenX(p));
        // 只写被完全盖住的列。lo、hi 只是缩小范围：部分盖住的列，列边上的视线碰不到遮挡物（深度 +∞），本来也不会写
        int lo = std::max(0, int(std::ceil((smin + 1) * 0.5f * float(n_))));
        int hi = std::min(n_ - 1, int(std::floor((smax + 1) * 0.5f * float(n_))) - 1);
        int written = 0;
        for (int i = lo; i <= hi; ++i) {
            float x0 = binEdge(i), x1 = binEdge(i + 1);
            // 前表面的深度在每条边上随 s 单调；凸多边形的前表面朝视点凸出，顶点处只会更近。
            // 所以一列里最远的深度一定在列的两端
            float zmax = std::max(entryDepth(x0), entryDepth(x1));
            if (zmax == kInf) continue;
            if (zmax < levels_[0][i]) levels_[0][i] = zmax, ++written;
        }
        return written;
    }
    // #endregion

    // #region query
    struct Span {
        bool valid;  // false：盒子跨过近平面，保守地当作可见
        int lo, hi;
        float zmin;
    };
    Span project(const Aabb& b) const {
        const Vec2 cs[4] = {b.min, {b.max.x, b.min.y}, b.max, {b.min.x, b.max.y}};
        float smin = kInf, smax = -kInf, zmin = kInf;
        for (Vec2 c : cs) {
            Vec2 v = view_.toView(c);
            if (v.y < view_.near) return {false, 0, 0, 0};
            float s = screenX(v);
            smin = std::min(smin, s), smax = std::max(smax, s), zmin = std::min(zmin, v.y);
        }
        auto bin = [&](float s) { return std::clamp(int(std::floor((s + 1) * 0.5f * float(n_))), 0, n_ - 1); };
        return {true, bin(smin), bin(smax), zmin};
    }
    // 逐列检查：盒子最近的点比它覆盖的每一列都远 → 被挡住
    bool occluded(const Aabb& b, long* texels = nullptr) const {
        Span sp = project(b);
        if (!sp.valid) return false;
        for (int i = sp.lo; i <= sp.hi; ++i) {
            if (texels) ++*texels;
            if (levels_[0][i] >= sp.zmin) return false;
        }
        return true;
    }
    // #endregion

    // #region hiz
    // 最大值金字塔：上一层的每格 = 下一层两格的最大值（更远的那个，保守）
    void buildHiZ() {
        levels_.resize(1);
        while (levels_.back().size() > 1) {
            const std::vector<float>& lower = levels_.back();
            std::vector<float> up(lower.size() / 2);
            for (size_t i = 0; i < up.size(); ++i) up[i] = std::max(lower[2 * i], lower[2 * i + 1]);
            levels_.push_back(std::move(up));
        }
    }
    // 选一层，让盒子只覆盖其中 1～2 格。refine = false：只看这一层（更快、更保守）；
    // refine = true：这一层判断不了就往下细分，结果与逐列检查完全相同
    bool occludedHiZ(const Aabb& b, bool refine, long* texels = nullptr) const {
        Span sp = project(b);
        if (!sp.valid) return false;
        int L = 0;
        while (L + 1 < levelCount() && (sp.hi >> L) - (sp.lo >> L) > 1) ++L;
        for (int i = sp.lo >> L; i <= sp.hi >> L; ++i)
            if (!(refine ? descend(L, i, sp, texels) : (count(texels), levels_[L][i] < sp.zmin))) return false;
        return true;
    }
    // #endregion

private:
    static void count(long* t) {
        if (t) ++*t;
    }
    bool descend(int L, int i, const Span& sp, long* texels) const {
        int first = i << L, last = ((i + 1) << L) - 1;
        if (last < sp.lo || first > sp.hi) return true;
        count(texels);
        if (levels_[L][i] < sp.zmin) return true;
        if (L == 0) return false;
        return descend(L - 1, 2 * i, sp, texels) && descend(L - 1, 2 * i + 1, sp, texels);
    }
    // 近平面裁剪（Sutherland–Hodgman，只有一个平面 z ≥ near）。裁出的那条边在近平面上，比真实的遮挡物更近，仍然保守
    void clipNear() {
        if (poly_.size() == 2) {
            Vec2 a = poly_[0], b = poly_[1];
            float n = view_.near;
            if (a.y < n && b.y < n) return poly_.clear();
            if (a.y < n) a = a + (b - a) * ((n - a.y) / (b.y - a.y));
            if (b.y < n) b = b + (a - b) * ((n - b.y) / (a.y - b.y));
            poly_ = {a, b};
            return;
        }
        tmp_.clear();
        for (size_t i = 0; i < poly_.size(); ++i) {
            Vec2 a = poly_[i], b = poly_[(i + 1) % poly_.size()];
            bool ina = a.y >= view_.near, inb = b.y >= view_.near;
            if (ina) tmp_.push_back(a);
            if (ina != inb) tmp_.push_back(a + (b - a) * ((view_.near - a.y) / (b.y - a.y)));
        }
        poly_.swap(tmp_);
    }
    // 屏幕坐标 s 那条视线碰到遮挡物的深度（碰到的第一条边）；没碰到为 +∞
    float entryDepth(float s) const {
        Vec2 r{s * tan_, 1};
        float best = kInf;
        size_t m = poly_.size() == 2 ? 1 : poly_.size();
        for (size_t j = 0; j < m; ++j) {
            Vec2 p = poly_[j], e = poly_[(j + 1) % poly_.size()] - p;
            float den = cross(r, e);
            if (std::fabs(den) < 1e-12f) continue;
            float t = cross(p, e) / den, u = cross(p, r) / den;
            if (u >= -1e-5f && u <= 1 + 1e-5f && t > 0) best = std::min(best, t);
        }
        return best;
    }

    int n_;
    View view_{};
    float tan_ = 1;
    std::vector<std::vector<float>> levels_{std::vector<float>(n_, kInf)};
    std::vector<Vec2> poly_, tmp_;
};

// 不融合：每个遮挡物单独判断（物体必须被某一个遮挡物完全挡住）。对照用
inline void cullIndividually(DepthBuffer& scratch, const View& v, const std::vector<std::vector<Vec2>>& occluders,
                             const std::vector<Aabb>& boxes, std::vector<char>& hidden) {
    hidden.assign(boxes.size(), 0);
    for (const auto& occ : occluders) {
        scratch.begin(v);
        if (scratch.rasterize(occ) == 0) continue;
        for (size_t i = 0; i < boxes.size(); ++i)
            if (!hidden[i] && scratch.occluded(boxes[i])) hidden[i] = 1;
    }
}

// 参照：沿 rays 条视线（均匀分布在盒子的屏幕范围内）逐条求交。任何一条视线先碰到盒子 → 可见
inline bool visibleByRays(const View& v, const Aabb& box, const std::vector<std::vector<Vec2>>& occluders, int rays) {
    const Vec2 cs[4] = {box.min, {box.max.x, box.min.y}, box.max, {box.min.x, box.max.y}};
    float tan = std::tan(v.halfFov), smin = kInf, smax = -kInf;
    for (Vec2 c : cs) {
        Vec2 p = v.toView(c);
        if (p.y < v.near) return true;
        smin = std::min(smin, p.x / (p.y * tan)), smax = std::max(smax, p.x / (p.y * tan));
    }
    smin = std::max(smin, -1.0f), smax = std::min(smax, 1.0f);
    for (int k = 0; k < rays; ++k) {
        float s = smin + (float(k) + 0.5f) / float(rays) * (smax - smin);
        Vec2 d = v.dir + v.right() * (s * tan);  // 视图深度 z = 参数 t
        // 盒子：slab
        float t0 = v.near, t1 = v.far;
        for (int a = 0; a < 2; ++a) {
            float o = a ? v.eye.y : v.eye.x, dd = a ? d.y : d.x, lo = a ? box.min.y : box.min.x, hi = a ? box.max.y : box.max.x;
            if (std::fabs(dd) < 1e-12f) {
                if (o < lo || o > hi) t0 = kInf;
                continue;
            }
            float ta = (lo - o) / dd, tb = (hi - o) / dd;
            if (ta > tb) std::swap(ta, tb);
            t0 = std::max(t0, ta), t1 = std::min(t1, tb);
        }
        if (t0 > t1) continue;
        // 遮挡物：t ≥ near 的第一个交点；近平面上的点就在凸多边形内部 → 在 near 处被挡
        float tocc = kInf;
        for (const auto& poly : occluders) {
            size_t m = poly.size() == 2 ? 1 : poly.size();
            int pos = 0, neg = 0;  // 近平面上的点在每条边的同一侧 → 在凸多边形内部（与绕向无关）
            Vec2 pn = v.eye + d * v.near;
            for (size_t j = 0; j < m; ++j) {
                Vec2 a = poly[j], e = poly[(j + 1) % poly.size()] - a;
                float side = cross(e, pn - a);
                pos += side > 0, neg += side < 0;
                float den = cross(d, e);
                if (std::fabs(den) < 1e-12f) continue;
                Vec2 q = a - v.eye;
                float t = cross(q, e) / den, u = cross(q, d) / den;
                if (u >= 0 && u <= 1 && t >= v.near) tocc = std::min(tocc, t);
            }
            if (poly.size() > 2 && (pos == 0 || neg == 0)) tocc = std::min(tocc, v.near);
        }
        if (t0 < tocc) return true;
    }
    return false;
}

}  // namespace ga::culling
