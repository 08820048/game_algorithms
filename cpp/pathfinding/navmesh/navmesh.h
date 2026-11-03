#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include "vec2.h"

namespace ga {

// #region navmesh
// 导航网格：可行走区域被划分成若干个凸多边形。
// 相邻多边形之间共享的线段叫"门"（portal），单位从一个多边形穿过门进入另一个。
struct NavLink {
    int to;    // 相邻多边形的下标
    Vec2 a, b; // 门的两个端点
};

struct NavPoly {
    std::vector<Vec2> verts;  // 凸多边形顶点
    Vec2 center;
    std::vector<NavLink> links;
};

using Polygon = std::vector<Vec2>;

// 点是否在凸多边形内（含边界）：对每条边，点都必须在同一侧
inline bool insideConvex(const Polygon& poly, Vec2 p, float eps = 1e-3f) {
    bool pos = false, neg = false;
    for (size_t i = 0; i < poly.size(); ++i) {
        float c = cross(poly[(i + 1) % poly.size()] - poly[i], p - poly[i]);
        if (c > eps) pos = true;
        if (c < -eps) neg = true;
        if (pos && neg) return false;
    }
    return true;
}

class NavMesh {
public:
    std::vector<NavPoly> polys;

    // 找到包含点 p 的多边形，找不到（在障碍里或地图外）返回 -1
    int findPoly(Vec2 p) const {
        for (int i = 0; i < static_cast<int>(polys.size()); ++i) {
            if (insideConvex(polys[i].verts, p)) return i;
        }
        return -1;
    }
};
// #endregion

// 点是否严格在多边形内部（射线法，适用于任意简单多边形）
inline bool insidePolygon(const Polygon& poly, Vec2 p) {
    bool inside = false;
    for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
        Vec2 a = poly[i], b = poly[j];
        if ((a.y > p.y) != (b.y > p.y) && p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x) inside = !inside;
    }
    return inside;
}

// #region build
// 用"梯形分解"从障碍物生成导航网格：
//   1. 在每个障碍物顶点的 x 坐标处画一条竖线，把地图切成若干竖条
//   2. 每个竖条内部没有顶点，穿过它的障碍物边互不相交，可以按高度排序
//   3. 相邻两条边之间如果不在障碍物里，就是一块可行走的梯形
//   4. 左右相邻、上下边界来自同一对边的梯形合并成一个
//   5. 左右两侧竖边有重叠的梯形互相连通，重叠部分就是门
// 障碍物必须是互不重叠的凸多边形，且完全位于 [min, max] 矩形内部。
inline NavMesh buildNavMesh(Vec2 min, Vec2 max, const std::vector<Polygon>& obstacles) {
    constexpr float kEps = 1e-3f;  // 横坐标相差小于它视为同一条竖线，保证每个竖条都有一定宽度
    struct Edge {
        Vec2 p, q;  // p.x < q.x
        float yAt(float x) const { return p.y + (q.y - p.y) * (x - p.x) / (q.x - p.x); }
    };
    std::vector<Edge> edges{{{min.x, min.y}, {max.x, min.y}}, {{min.x, max.y}, {max.x, max.y}}};  // 上下边界
    std::vector<float> xs{min.x, max.x};
    for (const Polygon& o : obstacles) {
        for (size_t i = 0; i < o.size(); ++i) {
            Vec2 a = o[i], b = o[(i + 1) % o.size()];
            xs.push_back(a.x);
            if (std::abs(a.x - b.x) > kEps) edges.push_back(a.x < b.x ? Edge{a, b} : Edge{b, a});
        }
    }
    std::sort(xs.begin(), xs.end());
    xs.erase(std::unique(xs.begin(), xs.end(), [](float a, float b) { return b - a < kEps; }), xs.end());

    // 一块梯形：左右两条竖边 x0 / x1，上边界是 edges[top]，下边界是 edges[bottom]
    struct Trap {
        float x0, x1;
        int top, bottom;
    };
    std::vector<Trap> traps;
    std::vector<Trap> prevSlab;
    for (size_t s = 0; s + 1 < xs.size(); ++s) {
        float x0 = xs[s], x1 = xs[s + 1], xm = (x0 + x1) / 2;
        std::vector<int> crossing;  // 穿过这个竖条的边
        for (int e = 0; e < static_cast<int>(edges.size()); ++e) {
            if (edges[e].p.x <= x0 + kEps && edges[e].q.x >= x1 - kEps) crossing.push_back(e);
        }
        std::sort(crossing.begin(), crossing.end(),
                  [&](int a, int b) { return edges[a].yAt(xm) < edges[b].yAt(xm); });

        std::vector<Trap> slab;
        for (size_t k = 0; k + 1 < crossing.size(); ++k) {
            int top = crossing[k], bottom = crossing[k + 1];
            Vec2 mid{xm, (edges[top].yAt(xm) + edges[bottom].yAt(xm)) / 2};
            bool blocked = std::any_of(obstacles.begin(), obstacles.end(),
                                       [&](const Polygon& o) { return insidePolygon(o, mid); });
            if (blocked) continue;
            // 和左边竖条里上下边界相同的梯形合并
            auto same = std::find_if(prevSlab.begin(), prevSlab.end(), [&](const Trap& t) {
                return t.top == top && t.bottom == bottom && std::abs(t.x1 - x0) < kEps;
            });
            if (same != prevSlab.end()) {
                Trap merged = *same;
                merged.x1 = x1;
                slab.push_back(merged);
                prevSlab.erase(same);
            } else {
                slab.push_back({x0, x1, top, bottom});
            }
        }
        traps.insert(traps.end(), prevSlab.begin(), prevSlab.end());  // 左边没被合并的已经完整
        prevSlab = std::move(slab);
    }
    traps.insert(traps.end(), prevSlab.begin(), prevSlab.end());

    NavMesh mesh;
    for (const Trap& t : traps) {
        const Edge &top = edges[t.top], &bottom = edges[t.bottom];
        Polygon v{{t.x0, top.yAt(t.x0)}, {t.x1, top.yAt(t.x1)}, {t.x1, bottom.yAt(t.x1)}, {t.x0, bottom.yAt(t.x0)}};
        // 上下边界在端点处相交时，梯形退化成三角形：去掉重复顶点
        Polygon verts;
        for (Vec2 p : v) {
            if (verts.empty() || distance(verts.back(), p) > 1e-4f) verts.push_back(p);
        }
        if (verts.size() > 1 && distance(verts.front(), verts.back()) < 1e-4f) verts.pop_back();
        Vec2 c{0, 0};
        for (Vec2 p : verts) c += p;
        mesh.polys.push_back({verts, c / static_cast<float>(verts.size()), {}});
    }
    // 连通关系：a 的右边与 b 的左边在同一条竖线上，且竖直方向有重叠
    for (size_t a = 0; a < traps.size(); ++a) {
        for (size_t b = 0; b < traps.size(); ++b) {
            const Trap &ta = traps[a], &tb = traps[b];
            if (std::abs(ta.x1 - tb.x0) > kEps) continue;
            float x = ta.x1;
            float y0 = std::max(edges[ta.top].yAt(x), edges[tb.top].yAt(x));
            float y1 = std::min(edges[ta.bottom].yAt(x), edges[tb.bottom].yAt(x));
            if (y1 - y0 < 1e-3f) continue;
            mesh.polys[a].links.push_back({static_cast<int>(b), {x, y0}, {x, y1}});
            mesh.polys[b].links.push_back({static_cast<int>(a), {x, y0}, {x, y1}});
        }
    }
    return mesh;
}
// #endregion

}  // namespace ga
