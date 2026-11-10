#pragma once

// Delaunay 三角剖分与 Voronoi 图
//   - 谓词：orient（c 在有向线段 ab 的哪一侧）与 inCircle（d 是否在 abc 的外接圆内），三种精度：float、double、精确（整数坐标 + 128 位整数）
//   - Bowyer–Watson 增量插入：找到外接圆包含新点的所有三角形（"空腔"），删掉，再把空腔边界的每条边和新点连成三角形
//   - 凸包外面用一个"无穷远点"补成幽灵三角形，不需要超级三角形；点落在凸包外时，看得见它的凸包边对应的幽灵三角形也在空腔里
//   - 定位新点：从上一个新三角形出发，朝新点"走"过去；插入顺序按希尔伯特曲线排序可以让每次走的步数接近常数
//   - Voronoi 单元：边界框依次被每个 Delaunay 邻居的垂直平分线裁掉一半
//   - Lloyd 松弛：每个点移到自己单元的质心

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <numeric>
#include <vector>

#include "../../foundations/random/random.h"
#include "vec2.h"

namespace ga::delaunay {

// #region predicates
enum class Precision { Float, Double, Exact };

// orient > 0：c 在 a→b 的左侧（数学坐标里的逆时针）。屏幕 y 朝下时看起来是顺时针，但只要全程一致就没有关系
// inCircle > 0：当 orient(a, b, c) > 0 时，d 严格在 abc 的外接圆内
// 精确版要求坐标是整数且 |x| < 2^24：差 < 2^25，平方和 < 2^51，乘积 < 2^102，128 位整数放得下
inline int sgn(double v) { return (v > 0) - (v < 0); }

inline int orient(Vec2 a, Vec2 b, Vec2 c, Precision pr) {
    if (pr == Precision::Float) return sgn((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x));
    if (pr == Precision::Double) return sgn((double(b.x) - a.x) * (double(c.y) - a.y) - (double(b.y) - a.y) * (double(c.x) - a.x));
    int64_t abx = int64_t(b.x) - int64_t(a.x), aby = int64_t(b.y) - int64_t(a.y);
    int64_t acx = int64_t(c.x) - int64_t(a.x), acy = int64_t(c.y) - int64_t(a.y);
    __int128 d = __int128(abx) * acy - __int128(aby) * acx;
    return (d > 0) - (d < 0);
}

template <class T>
inline T inCircleDet(T adx, T ady, T bdx, T bdy, T cdx, T cdy) {
    return (adx * adx + ady * ady) * (bdx * cdy - cdx * bdy) + (bdx * bdx + bdy * bdy) * (cdx * ady - adx * cdy) +
           (cdx * cdx + cdy * cdy) * (adx * bdy - bdx * ady);
}

inline int inCircle(Vec2 a, Vec2 b, Vec2 c, Vec2 d, Precision pr) {
    if (pr == Precision::Float) return sgn(inCircleDet<float>(a.x - d.x, a.y - d.y, b.x - d.x, b.y - d.y, c.x - d.x, c.y - d.y));
    if (pr == Precision::Double)
        return sgn(inCircleDet<double>(double(a.x) - d.x, double(a.y) - d.y, double(b.x) - d.x, double(b.y) - d.y, double(c.x) - d.x,
                                       double(c.y) - d.y));
    auto I = [](float v) { return __int128(int64_t(v)); };
    __int128 r = inCircleDet<__int128>(I(a.x) - I(d.x), I(a.y) - I(d.y), I(b.x) - I(d.x), I(b.y) - I(d.y), I(c.x) - I(d.x), I(c.y) - I(d.y));
    return (r > 0) - (r < 0);
}
// #endregion

// #region mesh
// 三角形 (v0, v1, v2) 按 orient > 0 排列；n[i] 是隔着 v[i] 对边的邻居，也就是边 (v[i+1], v[i+2]) 另一侧的三角形
// 顶点 kInf 是无穷远点：幽灵三角形 (a, b, ∞) 表示凸包边 a→b 外侧的那片半平面
inline constexpr int kInf = -1;

struct Tri {
    std::array<int, 3> v{};
    std::array<int, 3> n{};
    bool ghost() const { return v[0] == kInf || v[1] == kInf || v[2] == kInf; }
};
// #endregion

struct Stats {
    long long walkSteps = 0;     // 定位时穿过的三角形数
    long long cavity = 0;        // 被删掉的三角形数（每次插入的空腔大小之和）
    long long circleTests = 0;   // 冲突测试次数
    long long located = 0;       // 成功插入的点数
    int duplicates = 0;          // 与已有点重合而跳过的点
    int fallbacks = 0;           // 走路走不到、改用逐个扫描的次数（只在谓词不精确时出现）
    bool broken = false;         // 空腔不是一个圆盘（谓词前后矛盾），三角网已损坏，停止插入
};

// 一次插入的过程：定位时穿过的三角形、被删掉的空腔（只记有限三角形），供演示画出来
struct Trace {
    std::vector<std::array<int, 3>> walk, cavity;
};

class Triangulation {
public:
    explicit Triangulation(const std::vector<Vec2>& pts, Precision pr = Precision::Double) : p_(pts), pr_(pr) {}

    const std::vector<Vec2>& points() const { return p_; }
    const std::vector<Tri>& tris() const { return t_; }
    bool alive(int t) const { return alive_[t]; }
    const Stats& stats() const { return st_; }
    bool started() const { return !t_.empty(); }

    // 插入第 i 个点；trace 不为空时记下定位路径和空腔
    void insert(int i, Trace* trace = nullptr) {
        if (st_.broken) return;
        if (!started()) {
            pending_.push_back(i);
            tryStart();
            return;
        }
        insertPoint(i, trace);
    }

    // 所有有限三角形
    std::vector<std::array<int, 3>> triangles() const {
        std::vector<std::array<int, 3>> out;
        for (int t = 0; t < int(t_.size()); ++t)
            if (alive_[t] && !t_[t].ghost()) out.push_back(t_[t].v);
        return out;
    }

    // 凸包：幽灵三角形 (a, b, ∞) 的 a→b 首尾相接
    std::vector<int> hull() const {
        std::vector<int> next(p_.size(), -1);
        int start = -1;
        for (int t = 0; t < int(t_.size()); ++t) {
            if (!alive_[t] || !t_[t].ghost()) continue;
            auto [a, b] = ghostEdge(t);
            next[a] = b, start = a;
        }
        std::vector<int> h;
        for (int v = start; v >= 0 && h.size() <= p_.size(); v = next[v]) {
            h.push_back(v);
            if (next[v] == start) break;
        }
        return h;
    }

    // 每个点的 Delaunay 邻居（包括凸包上的边）
    std::vector<std::vector<int>> neighbors() const {
        std::vector<std::vector<int>> nb(p_.size());
        for (int t = 0; t < int(t_.size()); ++t) {
            if (!alive_[t]) continue;
            for (int e = 0; e < 3; ++e) {
                int a = t_[t].v[(e + 1) % 3], b = t_[t].v[(e + 2) % 3];
                if (a != kInf && b != kInf && a < b) nb[a].push_back(b), nb[b].push_back(a);
            }
        }
        return nb;
    }

    // 结构检查：邻接对称、有限三角形方向为正、每条边恰好两侧各一个三角形
    bool valid() const {
        for (int t = 0; t < int(t_.size()); ++t) {
            if (!alive_[t]) continue;
            const Tri& T = t_[t];
            if (!T.ghost() && orient(p_[T.v[0]], p_[T.v[1]], p_[T.v[2]], Precision::Double) <= 0 && pr_ != Precision::Exact) return false;
            if (!T.ghost() && pr_ == Precision::Exact && orient(p_[T.v[0]], p_[T.v[1]], p_[T.v[2]], pr_) <= 0) return false;
            for (int e = 0; e < 3; ++e) {
                int u = T.n[e];
                if (u < 0 || u >= int(t_.size()) || !alive_[u]) return false;
                int a = T.v[(e + 1) % 3], b = T.v[(e + 2) % 3];
                int back = -1;
                for (int f = 0; f < 3; ++f)
                    if (t_[u].n[f] == t) back = f;
                if (back < 0 || t_[u].v[(back + 1) % 3] != b || t_[u].v[(back + 2) % 3] != a) return false;
            }
        }
        return true;
    }

private:
    std::pair<int, int> ghostEdge(int t) const {  // 幽灵三角形的凸包边 (a, b)，按无穷远点之后的循环顺序
        const Tri& T = t_[t];
        int k = T.v[0] == kInf ? 0 : T.v[1] == kInf ? 1 : 2;
        return {T.v[(k + 1) % 3], T.v[(k + 2) % 3]};
    }

    int newTri(std::array<int, 3> v) {
        int t;
        if (!free_.empty()) t = free_.back(), free_.pop_back(), alive_[t] = 1;
        else t = int(t_.size()), t_.push_back({}), alive_.push_back(1), mark_.push_back(0), conflictMark_.push_back(0), conflict_.push_back(0);
        t_[t].v = v;
        t_[t].n = {-1, -1, -1};
        return t;
    }

    // 前三个不共线的点组成第一个三角形，外面围三个幽灵三角形
    void tryStart() {
        int a = pending_[0], b = -1, c = -1;
        for (int i : pending_)
            if (b < 0 && !(p_[i] == p_[a])) b = i;
        if (b < 0) return;
        for (int i : pending_)  // 显式排除 a、b：float 下编译器把乘减合并成 FMA 时，orient(a, b, b) 不一定是 0
            if (i != a && i != b && orient(p_[a], p_[b], p_[i], pr_) != 0) c = i;
        if (c < 0) return;
        if (orient(p_[a], p_[b], p_[c], pr_) < 0) std::swap(b, c);
        int f = newTri({a, b, c});
        int g0 = newTri({c, b, kInf}), g1 = newTri({a, c, kInf}), g2 = newTri({b, a, kInf});
        // f 的 n[0] 对边 (b, c)，n[1] 对边 (c, a)，n[2] 对边 (a, b)
        t_[f].n = {g0, g1, g2};
        // 幽灵 (c, b, ∞)：n[2] 对边 (c, b) → f；n[0] 对边 (b, ∞) → 含边 (∞, b) 的 g2；n[1] 对边 (∞, c) → 含边 (c, ∞) 的 g1
        t_[g0].n = {g2, g1, f};
        t_[g1].n = {g0, g2, f};
        t_[g2].n = {g1, g0, f};
        last_ = f;
        st_.located += 3;
        std::vector<int> rest;
        for (int i : pending_)
            if (i != a && i != b && i != c) rest.push_back(i);
        pending_.clear();
        for (int i : rest) insertPoint(i, nullptr);
    }

    bool conflict(int t, Vec2 q) {
        ++st_.circleTests;
        const Tri& T = t_[t];
        if (!T.ghost()) return inCircle(p_[T.v[0]], p_[T.v[1]], p_[T.v[2]], q, pr_) > 0;
        auto [a, b] = ghostEdge(t);
        int o = orient(p_[a], p_[b], q, pr_);
        if (o != 0) return o > 0;
        // 恰好落在凸包边所在的直线上：只有在线段 ab 内部才算（外接"圆"退化成半平面加上这段开线段）
        Vec2 A = p_[a], B = p_[b];
        double t0 = (double(q.x) - A.x) * (double(B.x) - A.x) + (double(q.y) - A.y) * (double(B.y) - A.y);
        double len = (double(B.x) - A.x) * (double(B.x) - A.x) + (double(B.y) - A.y) * (double(B.y) - A.y);
        return t0 > 0 && t0 < len;
    }

    // #region locate
    // 从 last_ 出发，只要新点在当前三角形某条边的外侧，就跨过那条边。到达幽灵三角形说明点在凸包外
    int locate(Vec2 q, Trace* trace) {
        int t = last_;
        const long long cap = 4 * (long long)t_.size() + 16;
        for (long long steps = 0; steps < cap; ++steps) {
            const Tri& T = t_[t];
            if (T.ghost()) return t;
            if (trace) trace->walk.push_back(T.v);
            int moved = -1;
            int s = int(steps % 3);  // 轮换起始边，避免在不精确的谓词下来回打转
            for (int k = 0; k < 3 && moved < 0; ++k) {
                int e = (s + k) % 3;
                if (orient(p_[T.v[(e + 1) % 3]], p_[T.v[(e + 2) % 3]], q, pr_) < 0) moved = T.n[e];
            }
            if (moved < 0) return t;
            t = moved;
            ++st_.walkSteps;
        }
        return -1;
    }
    // #endregion

    void insertPoint(int i, Trace* trace) {
        if (st_.broken) return;
        Vec2 q = p_[i];
        int t = locate(q, trace);
        if (t >= 0 && !t_[t].ghost())
            for (int v : t_[t].v)
                if (p_[v] == q) {
                    ++st_.duplicates;
                    return;
                }
        ++stamp_;
        if (t < 0 || !conflictCached(t, q)) {
            // 只有谓词不精确时才会到这里：逐个找一个冲突的三角形
            ++st_.fallbacks;
            t = -1;
            for (int u = 0; u < int(t_.size()) && t < 0; ++u)
                if (alive_[u] && conflictCached(u, q)) t = u;
            if (t < 0) {
                ++st_.duplicates;
                return;
            }
        }
        // #region cavity
        // 空腔：从 t 出发，向外扩展到所有冲突的三角形；冲突与不冲突的交界就是空腔的边界
        struct Edge {
            int a, b, out, outEdge;
        };
        std::vector<int> cav{t};
        std::vector<Edge> boundary;
        mark_[t] = stamp_;
        for (size_t k = 0; k < cav.size(); ++k) {
            int c = cav[k];
            for (int e = 0; e < 3; ++e) {
                int u = t_[c].n[e];
                if (mark_[u] == stamp_) continue;
                if (conflictCached(u, q)) {
                    mark_[u] = stamp_;
                    cav.push_back(u);
                } else {
                    int back = 0;
                    while (t_[u].n[back] != c) ++back;
                    boundary.push_back({t_[c].v[(e + 1) % 3], t_[c].v[(e + 2) % 3], u, back});
                }
            }
        }
        // 每条边界边 (a, b) 和新点连成 (a, b, q)。空腔是圆盘时，边界是一个环：每个顶点恰好是一条边的起点、另一条边的终点
        std::vector<int> next(boundary.size(), -1);
        for (size_t k = 0; k < boundary.size(); ++k) {
            int hits = 0;
            for (size_t m = 0; m < boundary.size(); ++m) {
                if (m != k && boundary[m].a == boundary[k].a) hits = 2;
                if (boundary[m].a == boundary[k].b) next[k] = int(m), ++hits;
            }
            if (hits != 1) {
                st_.broken = true;  // 谓词前后矛盾，空腔不是圆盘
                return;
            }
        }
        if (trace)
            for (int c : cav)
                if (!t_[c].ghost()) trace->cavity.push_back(t_[c].v);
        st_.cavity += (long long)cav.size();
        for (int c : cav) alive_[c] = 0, free_.push_back(c);
        std::vector<int> made(boundary.size());
        for (size_t k = 0; k < boundary.size(); ++k) {
            const Edge& E = boundary[k];
            int nt = newTri({E.a, E.b, i});
            t_[nt].n[2] = E.out;
            t_[E.out].n[E.outEdge] = nt;
            made[k] = nt;
        }
        // (a, b, q) 的 n[0] 对边 (b, q)：它在以 b 为起点的新三角形 (b, c, q) 里是 n[1]
        for (size_t k = 0; k < boundary.size(); ++k) {
            t_[made[k]].n[0] = made[next[k]];
            t_[made[next[k]]].n[1] = made[k];
        }
        // #endregion
        for (int nt : made)
            if (!t_[nt].ghost()) last_ = nt;
        ++st_.located;
    }

    bool conflictCached(int t, Vec2 q) {
        if (conflictMark_[t] != stamp_) conflictMark_[t] = stamp_, conflict_[t] = conflict(t, q);
        return conflict_[t];
    }

    std::vector<Vec2> p_;
    Precision pr_;
    std::vector<Tri> t_;
    std::vector<char> alive_, conflict_;
    std::vector<int> free_, pending_, mark_, conflictMark_;
    int stamp_ = 0, last_ = 0;
    Stats st_;
};

// #region order
// 插入顺序。随机顺序下，从上一个三角形走到新点平均要穿过 O(√n) 个三角形；
// 按希尔伯特曲线排序后，相邻两个点在空间上也相邻，走路的步数接近常数
enum class Order { Given, Random, Hilbert, SortX };

inline uint32_t hilbertIndex(uint32_t x, uint32_t y, int bits) {
    uint32_t d = 0;
    for (uint32_t s = 1u << (bits - 1); s > 0; s >>= 1) {
        uint32_t rx = (x & s) ? 1 : 0, ry = (y & s) ? 1 : 0;
        d += s * s * ((3 * rx) ^ ry);
        if (ry == 0) {  // 旋转象限
            if (rx == 1) x = s - 1 - x, y = s - 1 - y;
            std::swap(x, y);
        }
    }
    return d;
}

inline std::vector<int> insertionOrder(const std::vector<Vec2>& pts, Order order, Pcg32& rng) {
    std::vector<int> idx(pts.size());
    std::iota(idx.begin(), idx.end(), 0);
    if (order == Order::Random) shuffle(idx, rng);
    if (order == Order::SortX) std::sort(idx.begin(), idx.end(), [&](int a, int b) { return pts[a].x < pts[b].x || (pts[a].x == pts[b].x && pts[a].y < pts[b].y); });
    if (order == Order::Hilbert && !pts.empty()) {
        float x0 = pts[0].x, y0 = pts[0].y, x1 = x0, y1 = y0;
        for (const Vec2& p : pts) x0 = std::min(x0, p.x), y0 = std::min(y0, p.y), x1 = std::max(x1, p.x), y1 = std::max(y1, p.y);
        float s = std::max({x1 - x0, y1 - y0, 1e-30f});
        std::vector<uint32_t> key(pts.size());
        for (size_t i = 0; i < pts.size(); ++i)
            key[i] = hilbertIndex(uint32_t((pts[i].x - x0) / s * 65535), uint32_t((pts[i].y - y0) / s * 65535), 16);
        std::sort(idx.begin(), idx.end(), [&](int a, int b) { return key[a] < key[b]; });
    }
    return idx;
}

inline Triangulation triangulate(const std::vector<Vec2>& pts, Order order = Order::Hilbert, Precision pr = Precision::Double, uint64_t seed = 1) {
    Pcg32 rng{seed};
    Triangulation tr(pts, pr);
    for (int i : insertionOrder(pts, order, rng)) tr.insert(i);
    return tr;
}
// #endregion

// #region super
// 对照：经典的超级三角形写法。先放三个离点集很远（scale 倍包围盒）的真实顶点，插完后删掉碰到它们的三角形。
// 超级顶点不在无穷远，凸包附近有些本该存在的三角形会因为外接圆包含了超级顶点而被换掉
inline std::vector<std::array<int, 3>> superTriangleDelaunay(const std::vector<Vec2>& pts, float scale, Precision pr, Order order = Order::Hilbert) {
    float x0 = pts[0].x, y0 = pts[0].y, x1 = x0, y1 = y0;
    for (const Vec2& p : pts) x0 = std::min(x0, p.x), y0 = std::min(y0, p.y), x1 = std::max(x1, p.x), y1 = std::max(y1, p.y);
    float cx = std::round((x0 + x1) / 2), cy = std::round((y0 + y1) / 2), d = std::round(std::max(x1 - x0, y1 - y0) * scale);
    std::vector<Vec2> all = pts;
    int n = int(pts.size());
    all.push_back({cx - 2 * d, cy - d});
    all.push_back({cx + 2 * d, cy - d});
    all.push_back({cx, cy + 2 * d});
    Triangulation tr(all, pr);
    tr.insert(n), tr.insert(n + 1), tr.insert(n + 2);
    Pcg32 rng{1};
    for (int i : insertionOrder(pts, order, rng)) tr.insert(i);
    std::vector<std::array<int, 3>> out;
    for (const auto& t : tr.triangles())
        if (t[0] < n && t[1] < n && t[2] < n) out.push_back(t);
    return out;
}
// #endregion

// #region voronoi
// Voronoi 单元：离点 i 比离其他所有点都近的区域。它是若干半平面的交：对每个邻居 j，保留垂直平分线靠 i 的一侧。
// 只需要 Delaunay 邻居：别的点的平分线不会切到单元（测试里和"对所有点裁剪"对照）
struct P2 {
    double x = 0, y = 0;
};
using Poly = std::vector<P2>;

inline Poly clipBisector(const Poly& poly, Vec2 a, Vec2 b) {  // 保留离 a 不比离 b 远的部分
    double dx = double(b.x) - a.x, dy = double(b.y) - a.y;
    double c = (dx * (double(a.x) + b.x) + dy * (double(a.y) + b.y)) / 2;  // 平分线：dx·x + dy·y = c
    auto side = [&](P2 p) { return dx * p.x + dy * p.y - c; };          // ≤ 0 是 a 这一侧
    Poly out;
    for (size_t k = 0; k < poly.size(); ++k) {
        P2 p = poly[k], q = poly[(k + 1) % poly.size()];
        double sp = side(p), sq = side(q);
        if (sp <= 0) out.push_back(p);
        if ((sp < 0 && sq > 0) || (sp > 0 && sq < 0)) {
            double t = sp / (sp - sq);
            out.push_back({p.x + t * (q.x - p.x), p.y + t * (q.y - p.y)});
        }
    }
    return out;
}

inline Poly boxPoly(double w, double h) { return {{0, 0}, {w, 0}, {w, h}, {0, h}}; }

inline std::vector<Poly> voronoiCells(const std::vector<Vec2>& pts, const std::vector<std::vector<int>>& nb, double w, double h) {
    std::vector<Poly> cells(pts.size());
    for (size_t i = 0; i < pts.size(); ++i) {
        Poly c = boxPoly(w, h);
        for (int j : nb[i]) c = clipBisector(c, pts[i], pts[j]);
        cells[i] = std::move(c);
    }
    return cells;
}
// #endregion

// #region polygon
inline double area(const Poly& p) {
    double s = 0;
    for (size_t k = 0; k < p.size(); ++k) {
        const P2 &a = p[k], &b = p[(k + 1) % p.size()];
        s += a.x * b.y - b.x * a.y;
    }
    return std::abs(s) / 2;
}

inline P2 centroid(const Poly& p) {
    double s = 0, cx = 0, cy = 0;
    for (size_t k = 0; k < p.size(); ++k) {
        const P2 &a = p[k], &b = p[(k + 1) % p.size()];
        double c = a.x * b.y - b.x * a.y;
        s += c, cx += (a.x + b.x) * c, cy += (a.y + b.y) * c;
    }
    if (std::abs(s) < 1e-300) return p.empty() ? P2{} : p[0];
    return {cx / (3 * s), cy / (3 * s)};
}

// ∫|x − s|² dA：扇形三角形 (s, a, b) 的二阶矩是 面积/6 ·(|a−s|² + |b−s|² + (a−s)·(b−s))
inline double secondMoment(const Poly& p, Vec2 s) {
    double m = 0;
    for (size_t k = 0; k < p.size(); ++k) {
        P2 a = p[k], b = p[(k + 1) % p.size()];
        double ux = a.x - s.x, uy = a.y - s.y, vx = b.x - s.x, vy = b.y - s.y;
        double tri = (ux * vy - uy * vx) / 2;  // 有向面积，凸多边形整体同号
        m += tri / 6 * (ux * ux + uy * uy + vx * vx + vy * vy + ux * vx + uy * vy);
    }
    return std::abs(m);
}
// #endregion

// #region lloyd
// Lloyd 松弛：三角剖分 → Voronoi 单元 → 每个点移到单元质心。返回移动前的总能量 Σ∫|x − s|²（每一步都不会增加）
struct LloydStep {
    double energy = 0, meanMove = 0;
};

inline LloydStep lloyd(std::vector<Vec2>& pts, double w, double h) {
    Triangulation tr = triangulate(pts);
    std::vector<Poly> cells = voronoiCells(pts, tr.neighbors(), w, h);
    LloydStep r;
    for (size_t i = 0; i < pts.size(); ++i) {
        r.energy += secondMoment(cells[i], pts[i]);
        P2 c = centroid(cells[i]);
        Vec2 np{float(c.x), float(c.y)};
        r.meanMove += std::hypot(double(np.x) - pts[i].x, double(np.y) - pts[i].y) / double(pts.size());
        pts[i] = np;
    }
    return r;
}
// #endregion

}  // namespace ga::delaunay
