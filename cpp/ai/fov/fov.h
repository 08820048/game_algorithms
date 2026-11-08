#pragma once

// 网格上的视野（field of view）：从一个格子出发，哪些格子看得见。
//
// "看得见"的精确定义（本文所有算法都以它为准）：
//   - 每个格子是一个单位正方形，视线从格子中心出发
//   - 地面格子 T 可见：从起点中心到 T 中心的线段不穿过任何墙的内部（擦过墙的边或角不算），
//     并且不从"斜对角相接的两堵墙"之间的缝隙穿过（线段恰好经过两墙相接的那个角点）
//   - 墙格子 W 可见：存在一条这样的视线到达 W 的内部
//   - 距离限制：中心之间的距离不超过半径
// 这个定义对地面格子是对称的：A 看得见 B，当且仅当 B 看得见 A。
//
// 实现：
//   shadowcast  按八个卦限逐行扫描，用精确的有理数斜率维护"还亮着"的区间，与定义逐格一致
//   canSee      两点之间的视线，直接按定义检查（暴力对照，也可用于"守卫能否看见玩家"）
//   rayCast     从起点向边界画 Bresenham 直线，遇墙停止（常见的简单做法，用于对比）
//   classicShadowcast  经典的递归阴影投射（浮点斜率，RogueBasin 上流传最广的版本，用于对比）

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <numeric>
#include <vector>

namespace ga::fov {

// #region grid
struct Grid {
    int w = 0, h = 0;
    std::vector<uint8_t> wall;
    Grid() = default;
    Grid(int w_, int h_) : w(w_), h(h_), wall(size_t(w_) * h_, 0) {}
    bool inside(int x, int y) const { return x >= 0 && y >= 0 && x < w && y < h; }
    bool blocked(int x, int y) const { return !inside(x, y) || wall[size_t(y) * w + x]; }  // 地图外当作墙
    int index(int x, int y) const { return y * w + x; }
};
// #endregion

// #region slope
// 斜率 n/d（d > 0），用整数交叉相乘比较，没有舍入误差
struct Slope {
    int64_t n, d;
    friend bool operator<(Slope a, Slope b) { return a.n * b.d < b.n * a.d; }
    friend bool operator==(Slope a, Slope b) { return a.n * b.d == b.n * a.d; }
    friend bool operator<=(Slope a, Slope b) { return !(b < a); }
};

// 一段亮着的斜率区间，两端可以是开的或闭的
struct Span {
    Slope lo, hi;
    bool loClosed, hiClosed;
    bool contains(Slope s) const { return (lo < s || (loClosed && lo == s)) && (s < hi || (hiClosed && s == hi)); }
    // 与另一个区间是否有公共点
    bool meets(const Span& o) const {
        Slope L = std::max(lo, o.lo), H = std::min(hi, o.hi);
        if (L < H) return true;
        return L == H && contains(L) && o.contains(L);
    }
    bool meetsOpen(Slope a, Slope b) const { return meets({a, b, false, false}); }
};
// #endregion

// #region shadowcast
// 一个卦限内：第 r 行（离起点 r 格）第 c 列（0 <= c <= r），斜率 = c / r。
// 墙 (c, r) 的正方形挡住的斜率是开区间 ((c - 1/2) / (r + 1/2), (c + 1/2) / (r - 1/2))
inline void scanOctant(const Grid& g, int ox, int oy, int radius, int xx, int xy, int yx, int yy, std::vector<uint8_t>& out) {
    auto cellX = [&](int c, int r) { return ox + c * xx + r * xy; };
    auto cellY = [&](int c, int r) { return oy + c * yx + r * yy; };
    auto isWall = [&](int c, int r) { return g.blocked(cellX(c, r), cellY(c, r)); };

    std::vector<Span> lit{{{0, 1}, {1, 1}, true, true}}, next;
    for (int r = 1; r <= radius && !lit.empty(); ++r) {
        // 1. 斜对角相接的两堵墙 (c+1, r-1) 和 (c, r) 之间的缝：恰好经过角点的那条视线被挡住
        for (int c = 0; c < r; ++c)
            if (isWall(c + 1, r - 1) && isWall(c, r)) {
                Slope p{2 * c + 1, 2 * r - 1};
                next.clear();
                for (const Span& s : lit) {
                    if (!s.contains(p)) {
                        next.push_back(s);
                        continue;
                    }
                    if (s.lo < p) next.push_back({s.lo, p, s.loClosed, false});
                    if (p < s.hi) next.push_back({p, s.hi, false, s.hiClosed});
                }
                lit.swap(next);
            }
        // 2. 这一行的格子：地面看中心是否亮着，墙看它挡住的区间里是否有亮的部分
        for (int c = 0; c <= r; ++c) {
            int x = cellX(c, r), y = cellY(c, r);
            if (!g.inside(x, y) || c * c + r * r > radius * radius) continue;
            bool wall = g.wall[g.index(x, y)];
            // 墙能被看见的视线：它挡住的开区间。但如果同一行靠斜率 0 一侧的邻居 (c-1, r) 也是墙，
            // 从左侧边进入的那些视线会先穿过邻居，只剩从底边进入的部分 [(c-1/2)/(r-1/2), 上界)
            Span hit = c >= 1 && isWall(c - 1, r) ? Span{{2 * c - 1, 2 * r - 1}, {2 * c + 1, 2 * r - 1}, true, false}
                                                  : Span{{2 * c - 1, 2 * r + 1}, {2 * c + 1, 2 * r - 1}, false, false};
            for (const Span& s : lit)
                if (wall ? s.meets(hit) : s.contains(Slope{c, r})) {
                    out[g.index(x, y)] = 1;
                    break;
                }
        }
        // 3. 这一行的墙挡住的部分从亮区间中去掉（开区间：端点仍然亮着）
        for (int c = 0; c <= r; ++c) {
            if (!isWall(c, r)) continue;
            Slope a{2 * c - 1, 2 * r + 1}, b{2 * c + 1, 2 * r - 1};
            next.clear();
            for (const Span& s : lit) {
                if (!s.meetsOpen(a, b)) {
                    next.push_back(s);
                    continue;
                }
                if (s.lo < a || (s.lo == a && s.loClosed)) next.push_back({s.lo, a, s.loClosed, true});
                if (b < s.hi || (b == s.hi && s.hiClosed)) next.push_back({b, s.hi, true, s.hiClosed});
            }
            lit.swap(next);
        }
    }
}

inline void shadowcast(const Grid& g, int ox, int oy, int radius, std::vector<uint8_t>& out) {
    out.assign(g.wall.size(), 0);
    if (!g.inside(ox, oy)) return;
    out[g.index(ox, oy)] = 1;
    // 八个卦限：(c, r) -> (ox + c*xx + r*xy, oy + c*yx + r*yy)
    static constexpr int t[8][4] = {{1, 0, 0, 1}, {0, 1, 1, 0}, {0, -1, 1, 0}, {-1, 0, 0, 1},
                                    {-1, 0, 0, -1}, {0, -1, -1, 0}, {0, 1, -1, 0}, {1, 0, 0, -1}};
    for (auto& o : t) scanOctant(g, ox, oy, radius, o[0], o[1], o[2], o[3], out);
}
// #endregion

// #region can-see
// 两个格子中心之间的视线，直接按定义检查。坐标放大一倍：格子中心在偶数点，格子的角在奇数点
inline bool canSee(const Grid& g, int ax, int ay, int bx, int by) {
    const int64_t X = 2 * int64_t(bx - ax), Y = 2 * int64_t(by - ay);
    // 线段 p(t) = (2ax, 2ay) + t (X, Y)，t ∈ [0, 1]
    int x0 = std::min(ax, bx), x1 = std::max(ax, bx), y0 = std::min(ay, by), y1 = std::max(ay, by);
    for (int y = y0; y <= y1; ++y)
        for (int x = x0; x <= x1; ++x) {
            if (!g.blocked(x, y) || (x == ax && y == ay) || (x == bx && y == by)) continue;
            // 开正方形 (2x-1, 2x+1) × (2y-1, 2y+1)：求 t 的开区间，用分数 n/d 表示
            int64_t loN = 0, loD = 1, hiN = 1, hiD = 1;  // t 的下界和上界
            auto clip = [&](int64_t p0, int64_t dir, int64_t lo, int64_t hi) {  // lo < p0 + t*dir < hi
                if (dir == 0) return lo < p0 && p0 < hi;
                int64_t a = lo - p0, b = hi - p0;
                if (dir < 0) a = -a, b = -b, dir = -dir, std::swap(a, b);
                if (a * loD > loN * dir) loN = a, loD = dir;  // 下界取 max
                if (b * hiD < hiN * dir) hiN = b, hiD = dir;  // 上界取 min
                return true;
            };
            if (!clip(2 * ax, X, 2 * x - 1, 2 * x + 1) || !clip(2 * ay, Y, 2 * y - 1, 2 * y + 1)) continue;
            if (loN * hiD < hiN * loD) return false;  // 区间非空：线段穿过这堵墙的内部
        }
    // 线段经过的格子角点（坐标都是奇数）：两侧的格子都是墙则挡住
    if (X != 0 && Y != 0) {
        int64_t k = std::gcd(std::llabs(X), std::llabs(Y)), sx = X > 0 ? 1 : -1, sy = Y > 0 ? 1 : -1;
        for (int64_t i = 1; i < k; ++i) {
            int64_t px = 2 * ax + X / k * i, py = 2 * ay + Y / k * i;
            if (!(px & 1) || !(py & 1)) continue;
            // 线段从左下（相对方向）穿到右上；挡路的是另外两个格子
            int cx1 = int((px + sx) >> 1), cy1 = int((py - sy) >> 1);
            int cx2 = int((px - sx) >> 1), cy2 = int((py + sy) >> 1);
            if (g.blocked(cx1, cy1) && g.blocked(cx2, cy2)) return false;
        }
    }
    return true;
}

// 地面格子的视野：逐个格子检查视线（暴力，O(格子数 × 半径²)）
inline void fovBrute(const Grid& g, int ox, int oy, int radius, std::vector<uint8_t>& out) {
    out.assign(g.wall.size(), 0);
    for (int y = 0; y < g.h; ++y)
        for (int x = 0; x < g.w; ++x) {
            int dx = x - ox, dy = y - oy;
            if (dx * dx + dy * dy <= radius * radius && !g.wall[g.index(x, y)] && canSee(g, ox, oy, x, y)) out[g.index(x, y)] = 1;
        }
}
// #endregion

// #region ray-cast
// 对比：从起点向半径为 radius 的方框边界上的每个格子画 Bresenham 直线，沿线点亮，遇墙停止
inline void rayCast(const Grid& g, int ox, int oy, int radius, std::vector<uint8_t>& out) {
    out.assign(g.wall.size(), 0);
    if (!g.inside(ox, oy)) return;
    auto cast = [&](int tx, int ty) {
        int x = ox, y = oy, dx = std::abs(tx - ox), dy = -std::abs(ty - oy);
        int sx = ox < tx ? 1 : -1, sy = oy < ty ? 1 : -1, err = dx + dy;
        while (true) {
            if (!g.inside(x, y)) return;
            int ddx = x - ox, ddy = y - oy;
            if (ddx * ddx + ddy * ddy > radius * radius) return;
            out[g.index(x, y)] = 1;
            if (g.wall[g.index(x, y)] || (x == tx && y == ty)) return;
            int e2 = 2 * err;
            if (e2 >= dy) err += dy, x += sx;
            if (e2 <= dx) err += dx, y += sy;
        }
    };
    for (int k = -radius; k <= radius; ++k) {
        cast(ox + k, oy - radius), cast(ox + k, oy + radius);
        cast(ox - radius, oy + k), cast(ox + radius, oy + k);
    }
}
// #endregion

// #region classic
// 对比：经典的递归阴影投射（Björn Bergström，2001）。浮点斜率，被光束碰到的格子都点亮
inline void classicOctant(const Grid& g, int ox, int oy, int radius, int row, double start, double end, int xx, int xy, int yx, int yy,
                          std::vector<uint8_t>& out) {
    if (start < end) return;
    double newStart = 0;
    for (int j = row; j <= radius; ++j) {
        int dx = -j - 1, dy = -j;
        bool blocked = false;
        while (dx <= 0) {
            ++dx;
            int X = ox + dx * xx + dy * xy, Y = oy + dx * yx + dy * yy;
            double lSlope = (dx - 0.5) / (dy + 0.5), rSlope = (dx + 0.5) / (dy - 0.5);
            if (start < rSlope) continue;
            if (end > lSlope) break;
            if (dx * dx + dy * dy <= radius * radius && g.inside(X, Y)) out[g.index(X, Y)] = 1;
            if (blocked) {
                if (g.blocked(X, Y)) {
                    newStart = rSlope;
                    continue;
                }
                blocked = false;
                start = newStart;
            } else if (g.blocked(X, Y) && j < radius) {
                blocked = true;
                classicOctant(g, ox, oy, radius, j + 1, start, lSlope, xx, xy, yx, yy, out);
                newStart = rSlope;
            }
        }
        if (blocked) break;
    }
}

inline void classicShadowcast(const Grid& g, int ox, int oy, int radius, std::vector<uint8_t>& out) {
    out.assign(g.wall.size(), 0);
    if (!g.inside(ox, oy)) return;
    out[g.index(ox, oy)] = 1;
    static constexpr int t[8][4] = {{1, 0, 0, 1}, {0, 1, 1, 0}, {0, -1, 1, 0}, {-1, 0, 0, 1},
                                    {-1, 0, 0, -1}, {0, -1, -1, 0}, {0, 1, -1, 0}, {1, 0, 0, -1}};
    for (auto& o : t) classicOctant(g, ox, oy, radius, 1, 1.0, 0.0, o[0], o[1], o[2], o[3], out);
}
// #endregion

// #region perception
// 感知：守卫能否看见目标 = 距离 + 视野锥 + 视线。三项都便宜，按从便宜到贵的顺序检查
struct Viewer {
    float x, y;        // 所在格子的中心（格子坐标 + 0.5）
    float facing;      // 朝向（弧度）
    float halfAngle;   // 视野锥的半角
    float range;       // 最远看多少格
};

inline bool inCone(const Viewer& v, float tx, float ty) {
    float dx = tx - v.x, dy = ty - v.y;
    float d2 = dx * dx + dy * dy;
    if (d2 > v.range * v.range) return false;
    if (d2 < 1e-6f) return true;
    float fx = std::cos(v.facing), fy = std::sin(v.facing);
    return (dx * fx + dy * fy) >= std::cos(v.halfAngle) * std::sqrt(d2);  // 夹角不超过半角
}

inline bool sees(const Grid& g, const Viewer& v, int tx, int ty) {
    return inCone(v, tx + 0.5f, ty + 0.5f) && canSee(g, int(v.x), int(v.y), tx, ty);
}

// 警觉度：看见时上涨，越近涨得越快；看不见时缓慢回落。到 1 才算"发现"，避免一瞥即中
inline float updateAwareness(float a, bool visible, float distance, float range, float dt) {
    if (visible) a += dt * (0.6f + 2.4f * (1 - std::clamp(distance / range, 0.0f, 1.0f)));
    else a -= dt * 0.25f;
    return std::clamp(a, 0.0f, 1.0f);
}
// #endregion

}  // namespace ga::fov
