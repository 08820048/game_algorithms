#include <cmath>
#include <cstdio>

#include "../../foundations/random/random.h"
#include "check.h"
#include "funnel.h"
#include "navmesh.h"

using namespace ga;

static float area(const Polygon& p) {
    float a = 0;
    for (size_t i = 0; i < p.size(); ++i) a += cross(p[i], p[(i + 1) % p.size()]);
    return std::abs(a) / 2;
}

// 点严格在凸多边形内部（离每条边都有一点距离）
static bool strictlyInside(const Polygon& o, Vec2 p) {
    bool pos = true, neg = true;
    for (size_t i = 0; i < o.size(); ++i) {
        Vec2 e = o[(i + 1) % o.size()] - o[i];
        float c = cross(e, p - o[i]) / length(e);
        pos = pos && c > 1e-2f;
        neg = neg && c < -1e-2f;
    }
    return pos || neg;
}

// 两条线段是否真正交叉。用点到直线的距离判断，贴近到 0.05 以内的接触不算（浮点误差）
static bool properIntersect(Vec2 a, Vec2 b, Vec2 c, Vec2 d) {
    float d1 = cross(b - a, c - a) / length(b - a), d2 = cross(b - a, d - a) / length(b - a);
    float d3 = cross(d - c, a - c) / length(d - c), d4 = cross(d - c, b - c) / length(d - c);
    const float e = 0.05f;
    return ((d1 > e && d2 < -e) || (d1 < -e && d2 > e)) && ((d3 > e && d4 < -e) || (d3 < -e && d4 > e));
}

// 线段是否不穿过任何障碍物内部（贴着边、经过顶点都允许）
static bool segmentFree(Vec2 a, Vec2 b, const std::vector<Polygon>& obstacles) {
    for (const Polygon& o : obstacles) {
        for (size_t i = 0; i < o.size(); ++i) {
            if (properIntersect(a, b, o[i], o[(i + 1) % o.size()])) return false;
        }
        for (float t : {0.25f, 0.5f, 0.75f}) {
            if (strictlyInside(o, a + (b - a) * t)) return false;
        }
    }
    return true;
}

// 暴力对照：可见性图 + Dijkstra。多边形障碍之间的最短路径只会在障碍物顶点处拐弯
static float optimalLength(Vec2 s, Vec2 t, const std::vector<Polygon>& obstacles) {
    std::vector<Vec2> nodes{s, t};
    for (const Polygon& o : obstacles) nodes.insert(nodes.end(), o.begin(), o.end());
    const size_t n = nodes.size();
    std::vector<float> dist(n, 1e30f);
    std::vector<char> done(n, 0);
    dist[0] = 0;
    for (size_t iter = 0; iter < n; ++iter) {
        size_t u = n;
        for (size_t i = 0; i < n; ++i) {
            if (!done[i] && (u == n || dist[i] < dist[u])) u = i;
        }
        if (u == n || dist[u] >= 1e30f) break;
        done[u] = 1;
        for (size_t v = 0; v < n; ++v) {
            if (done[v]) continue;
            float d = dist[u] + distance(nodes[u], nodes[v]);
            if (d < dist[v] && segmentFree(nodes[u], nodes[v], obstacles)) dist[v] = d;
        }
    }
    return dist[1];
}

static std::vector<Polygon> randomObstacles(Pcg32& rng, Vec2 min, Vec2 max, int count) {
    std::vector<Polygon> result;
    std::vector<std::pair<Vec2, float>> circles;
    for (int attempt = 0; attempt < 400 && static_cast<int>(result.size()) < count; ++attempt) {
        float r = 25 + rng.nextFloat() * 60;
        Vec2 c{min.x + r + 10 + rng.nextFloat() * (max.x - min.x - 2 * r - 20),
               min.y + r + 10 + rng.nextFloat() * (max.y - min.y - 2 * r - 20)};
        bool overlap = false;
        for (auto [oc, orad] : circles) overlap = overlap || distance(c, oc) < r + orad + 8;
        if (overlap) continue;
        int sides = 3 + static_cast<int>(rng.below(4));
        float phase = rng.nextFloat() * 2 * kPi;
        Polygon p;
        for (int k = 0; k < sides; ++k) {
            float ang = phase + 2 * kPi * (k + 0.3f * rng.nextFloat()) / sides;
            p.push_back(c + fromAngle(ang) * r);
        }
        result.push_back(p);
        circles.push_back({c, r});
    }
    return result;
}

int main() {
    const Vec2 kMin{0, 0}, kMax{800, 450};
    Pcg32 rng(17);
    int optimalCount = 0, queries = 0;
    for (int round = 0; round < 40; ++round) {
        auto obstacles = randomObstacles(rng, kMin, kMax, 4 + static_cast<int>(rng.below(8)));
        NavMesh mesh = buildNavMesh(kMin, kMax, obstacles);

        // 导航网格恰好覆盖空地：面积相等，每个多边形都是凸的，连通关系对称
        float free = (kMax.x - kMin.x) * (kMax.y - kMin.y);
        for (const Polygon& o : obstacles) free -= area(o);
        float covered = 0;
        for (const NavPoly& p : mesh.polys) {
            covered += area(p.verts);
            CHECK(p.verts.size() >= 3);
            for (size_t i = 0; i < p.verts.size(); ++i) {
                Vec2 a = p.verts[i], b = p.verts[(i + 1) % p.verts.size()], c = p.verts[(i + 2) % p.verts.size()];
                CHECK(cross(b - a, c - b) >= -1e-2f);  // 屏幕坐标下顺时针，凸
            }
            for (const NavLink& l : p.links) {
                const auto& back = mesh.polys[l.to].links;
                CHECK(std::any_of(back.begin(), back.end(), [&](const NavLink& x) { return &mesh.polys[x.to] == &p; }));
            }
        }
        CHECK(std::abs(covered - free) < 1.0f);

        for (int q = 0; q < 25; ++q) {
            Vec2 s{rng.nextFloat() * 800, rng.nextFloat() * 450}, t{rng.nextFloat() * 800, rng.nextFloat() * 450};
            bool sFree = std::none_of(obstacles.begin(), obstacles.end(), [&](const Polygon& o) { return insidePolygon(o, s); });
            bool tFree = std::none_of(obstacles.begin(), obstacles.end(), [&](const Polygon& o) { return insidePolygon(o, t); });
            CHECK((mesh.findPoly(s) >= 0) == sFree);
            if (!sFree || !tFree) continue;

            auto channel = findChannel(mesh, s, t);
            CHECK(!channel.empty());  // 障碍物互不重叠，空地总是连通的
            auto portals = channelPortals(mesh, channel, s, t);
            auto path = stringPull(portals);
            CHECK(path.front() == s);
            CHECK(path.back() == t);
            for (size_t i = 1; i < path.size(); ++i) CHECK(segmentFree(path[i - 1], path[i], obstacles));
            float len = pathLength(path), best = optimalLength(s, t, obstacles);
            CHECK(len <= pathLength(midpointPath(portals)) + 1e-2f);
            CHECK(len >= best - 1e-2f);
            ++queries;
            if (len <= best + 1e-2f) ++optimalCount;
        }
    }
    std::printf("funnel path is globally optimal in %d / %d queries\n", optimalCount, queries);
    return report("navmesh_test");
}
