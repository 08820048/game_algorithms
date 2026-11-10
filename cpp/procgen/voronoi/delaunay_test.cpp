#include "delaunay.h"

#include <algorithm>
#include <cstdio>
#include <set>
#include <vector>

#include "check.h"

using namespace ga;
using namespace ga::delaunay;

using Tri3 = std::array<int, 3>;
static Tri3 canon(Tri3 t) {
    while (!(t[0] < t[1] && t[0] < t[2])) t = {t[1], t[2], t[0]};
    return t;
}
static std::set<Tri3> canonSet(const std::vector<Tri3>& ts) {
    std::set<Tri3> s;
    for (const Tri3& t : ts) s.insert(canon(t));
    return s;
}

// ---- 参照写法 ----
// 逐个三元组：方向为正、外接圆内严格没有别的点。没有四点共圆时，这就是唯一的 Delaunay 三角剖分
static std::set<Tri3> bruteDelaunay(const std::vector<Vec2>& p, bool* cocircular) {
    std::set<Tri3> out;
    int n = int(p.size());
    *cocircular = false;
    for (int a = 0; a < n; ++a)
        for (int b = 0; b < n; ++b)
            for (int c = 0; c < n; ++c) {
                if (a == b || b == c || a == c || orient(p[a], p[b], p[c], Precision::Exact) <= 0) continue;
                bool empty = true;
                for (int d = 0; d < n; ++d) {
                    if (d == a || d == b || d == c) continue;
                    int s = inCircle(p[a], p[b], p[c], p[d], Precision::Exact);
                    if (s > 0) empty = false;
                    if (s == 0) *cocircular = true;
                }
                if (empty) out.insert(canon({a, b, c}));
            }
    return out;
}

// 凸包上的点数（包括落在凸包边上的共线点）：单调链
static int hullCount(std::vector<Vec2> p, Precision pr = Precision::Exact) {
    std::sort(p.begin(), p.end(), [](Vec2 a, Vec2 b) { return a.x < b.x || (a.x == b.x && a.y < b.y); });
    p.erase(std::unique(p.begin(), p.end()), p.end());
    if (p.size() < 3) return int(p.size());
    std::vector<Vec2> h(2 * p.size());
    int k = 0;
    for (size_t i = 0; i < p.size(); ++i) {
        while (k >= 2 && orient(h[k - 2], h[k - 1], p[i], pr) < 0) --k;
        h[k++] = p[i];
    }
    for (int i = int(p.size()) - 2, t = k + 1; i >= 0; --i) {
        while (k >= t && orient(h[k - 2], h[k - 1], p[i], pr) < 0) --k;
        h[k++] = p[i];
    }
    return k - 1;
}

static int distinctCount(std::vector<Vec2> p) {
    std::sort(p.begin(), p.end(), [](Vec2 a, Vec2 b) { return a.x < b.x || (a.x == b.x && a.y < b.y); });
    return int(std::unique(p.begin(), p.end()) - p.begin());
}

// 每个三角形的外接圆里严格没有点
static long long emptyCircleViolations(const std::vector<Vec2>& p, const std::vector<Tri3>& ts) {
    long long bad = 0;
    for (const Tri3& t : ts)
        for (size_t d = 0; d < p.size(); ++d)
            bad += inCircle(p[t[0]], p[t[1]], p[t[2]], p[d], Precision::Exact) > 0;
    return bad;
}

static std::vector<Vec2> randomInts(Pcg32& rng, int n, int range) {
    std::vector<Vec2> p(n);
    for (Vec2& a : p) a = {float(rng.below(uint32_t(range))), float(rng.below(uint32_t(range)))};
    return p;
}

static void testPredicates() {
    Precision E = Precision::Exact, D = Precision::Double;
    CHECK(orient({0, 0}, {1, 0}, {0, 1}, E) == 1);
    CHECK(orient({0, 0}, {1, 0}, {0, -1}, E) == -1);
    CHECK(orient({0, 0}, {1, 1}, {5, 5}, E) == 0);
    CHECK(inCircle({0, 0}, {2, 0}, {0, 2}, {1, 1}, E) == 1);   // 圆心
    CHECK(inCircle({0, 0}, {2, 0}, {0, 2}, {2, 2}, E) == 0);   // 正方形四点共圆
    CHECK(inCircle({0, 0}, {2, 0}, {0, 2}, {3, 3}, E) == -1);
    // 小整数坐标下 double 也是精确的：随机四元组的符号全部一致
    Pcg32 rng{7};
    int disagree = 0, zeros = 0;
    for (int k = 0; k < 200000; ++k) {
        Vec2 a{float(rng.below(64)), float(rng.below(64))}, b{float(rng.below(64)), float(rng.below(64))};
        Vec2 c{float(rng.below(64)), float(rng.below(64))}, d{float(rng.below(64)), float(rng.below(64))};
        disagree += inCircle(a, b, c, d, E) != inCircle(a, b, c, d, D) || orient(a, b, c, E) != orient(a, b, c, D);
        zeros += inCircle(a, b, c, d, E) == 0;
    }
    CHECK(disagree == 0);
    CHECK(zeros > 100);
    // 大坐标、几乎共圆：float 会算错符号，double 不会（d 取离 abc 外接圆最近的整数点）
    int wrongF = 0, wrongD = 0, total = 0;
    while (total < 20000) {
        uint32_t range = 1u << 23;
        Vec2 a{float(rng.below(range)), float(rng.below(range))}, b{float(rng.below(range)), float(rng.below(range))}, c{float(rng.below(range)), float(rng.below(range))};
        double bx = double(b.x) - a.x, by = double(b.y) - a.y, cx = double(c.x) - a.x, cy = double(c.y) - a.y, D = 2 * (bx * cy - by * cx);
        if (std::abs(D) < 1) continue;
        double ux = (cy * (bx * bx + by * by) - by * (cx * cx + cy * cy)) / D, uy = (bx * (cx * cx + cy * cy) - cx * (bx * bx + by * by)) / D;
        double R = std::hypot(ux, uy), th = rng.nextDouble() * 6.283185307179586;
        double dx = std::round(a.x + ux + R * std::cos(th)), dy = std::round(a.y + uy + R * std::sin(th));
        if (dx < 0 || dy < 0 || dx >= range || dy >= range) continue;
        Vec2 d{float(dx), float(dy)};
        int ex = inCircle(a, b, c, d, E);
        ++total, wrongF += inCircle(a, b, c, d, Precision::Float) != ex, wrongD += inCircle(a, b, c, d, Precision::Double) != ex;
    }
    CHECK(wrongF > total / 50);
    CHECK(wrongD == 0);
}

static void testSmallVsBrute() {
    // 小点集：所有插入顺序、两种精度，都和逐个三元组的参照一致
    int compared = 0;
    for (int s = 0; s < 600; ++s) {
        Pcg32 rng{uint64_t(100 + s)};
        int n = 3 + s % 12;
        std::vector<Vec2> p = randomInts(rng, n, s % 3 == 0 ? 8 : 1000);
        if (distinctCount(p) != n) continue;
        bool cocirc;
        std::set<Tri3> ref = bruteDelaunay(p, &cocirc);
        for (Order o : {Order::Given, Order::Random, Order::Hilbert, Order::SortX})
            for (Precision pr : {Precision::Exact, Precision::Double}) {
                Triangulation tr = triangulate(p, o, pr, uint64_t(s));
                CHECK(tr.valid());
                std::vector<Tri3> ts = tr.triangles();
                CHECK(emptyCircleViolations(p, ts) == 0);
                if (!ref.empty()) CHECK(int(ts.size()) == 2 * n - 2 - hullCount(p));
                CHECK(int(tr.hull().size()) == (ref.empty() ? 0 : hullCount(p)));
                if (!cocirc) {
                    CHECK(canonSet(ts) == ref);
                    ++compared;
                }
            }
    }
    CHECK(compared > 2000);
}

static void testDegenerate() {
    // 整数网格：到处是四点共圆，边界上全是共线点
    for (int k : {2, 3, 5, 10, 17}) {
        std::vector<Vec2> p;
        for (int y = 0; y < k; ++y)
            for (int x = 0; x < k; ++x) p.push_back({float(x), float(y)});
        int n = k * k;
        for (Order o : {Order::Given, Order::Random, Order::Hilbert}) {
            Triangulation tr = triangulate(p, o, Precision::Exact, 3);
            CHECK(tr.valid());
            std::vector<Tri3> ts = tr.triangles();
            CHECK(int(ts.size()) == 2 * (k - 1) * (k - 1));
            CHECK(int(ts.size()) == 2 * n - 2 - 4 * (k - 1));
            CHECK(int(tr.hull().size()) == 4 * (k - 1));
            CHECK(emptyCircleViolations(p, ts) == 0);
        }
        // 每个点重复一次：重复的全部被跳过
        std::vector<Vec2> twice = p;
        twice.insert(twice.end(), p.begin(), p.end());
        Triangulation tr = triangulate(twice, Order::Random, Precision::Exact, 5);
        CHECK(tr.stats().duplicates == n);
        CHECK(int(tr.triangles().size()) == 2 * (k - 1) * (k - 1));
    }
    // 全部共线：没有三角形；之后再来一个不共线的点，就从扇形开始
    std::vector<Vec2> line;
    for (int i = 0; i < 10; ++i) line.push_back({float(i), float(2 * i)});
    {
        Triangulation tr(line, Precision::Exact);
        for (int i = 0; i < 10; ++i) tr.insert(i);
        CHECK(!tr.started() && tr.triangles().empty());
    }
    line.push_back({3, 0});
    {
        Triangulation tr(line, Precision::Exact);
        for (int i = 0; i < 11; ++i) tr.insert(i);
        CHECK(tr.valid());
        CHECK(int(tr.triangles().size()) == 9);
        CHECK(emptyCircleViolations(line, tr.triangles()) == 0);
    }
    // 点落在凸包边上（共线）以及凸包边的延长线上
    std::vector<Vec2> p = {{0, 0}, {10, 0}, {0, 10}, {5, 0}, {20, 0}, {-5, 0}, {0, 5}, {5, 5}, {0, 20}};
    for (Order o : {Order::Given, Order::Random, Order::SortX}) {
        Triangulation tr = triangulate(p, o, Precision::Exact, 9);
        CHECK(tr.valid());
        CHECK(int(tr.triangles().size()) == 2 * 9 - 2 - hullCount(p));
        CHECK(emptyCircleViolations(p, tr.triangles()) == 0);
    }
}

static void testLarge() {
    // 大点集：结构、空外接圆（精确检查）、三角形数 = 2n − 2 − h、三角形总数（含幽灵）= 2n − 2
    for (int s = 0; s < 4; ++s) {
        Pcg32 rng{uint64_t(500 + s)};
        std::vector<Vec2> p = randomInts(rng, 3000, s % 2 ? 1 << 20 : 300);
        int n = distinctCount(p);
        for (Order o : {Order::Random, Order::Hilbert}) {
            Triangulation tr = triangulate(p, o, Precision::Exact, uint64_t(s));
            CHECK(tr.valid());
            std::vector<Tri3> ts = tr.triangles();
            CHECK(emptyCircleViolations(p, ts) == 0);
            CHECK(int(ts.size()) == 2 * n - 2 - hullCount(p));
            CHECK(tr.stats().located == n);
            CHECK(tr.stats().duplicates == int(p.size()) - n);
            CHECK(tr.stats().fallbacks == 0);
            int alive = 0;
            for (int t = 0; t < int(tr.tris().size()); ++t) alive += tr.alive(t);
            CHECK(alive == 2 * n - 2);
            // 每次插入删掉 c 个三角形、新建 c + 2 个：新建总数 = 删除总数 + 2(n − 3)
            CHECK(tr.stats().cavity >= 2 * (n - 3));
        }
    }
    // 浮点坐标 + double 谓词：结果和整数坐标版一样合法
    Pcg32 rng{77};
    std::vector<Vec2> p(5000);
    for (Vec2& a : p) a = {rng.nextFloat() * 100, rng.nextFloat() * 60};
    Triangulation tr = triangulate(p, Order::Hilbert, Precision::Double);
    CHECK(tr.valid());
    CHECK(int(tr.triangles().size()) == 2 * 5000 - 2 - int(tr.hull().size()));
    CHECK(int(tr.hull().size()) == hullCount(p, Precision::Double));
}

static void testCavityRecord() {
    // 记录下来的空腔：每个三角形的外接圆都包含新点；空腔之外的三角形都不包含
    Pcg32 rng{31};
    std::vector<Vec2> p = randomInts(rng, 300, 1000);
    Triangulation tr(p, Precision::Exact);
    int checked = 0;
    for (int i = 0; i < 300; ++i) {
        std::vector<Tri3> before = tr.triangles();
        Trace trace;
        tr.insert(i, &trace);
        const std::vector<Tri3>& cav = trace.cavity;
        if (!tr.started() || before.empty()) continue;
        std::set<Tri3> cs = canonSet(cav);
        for (const Tri3& t : before) {
            bool in = inCircle(p[t[0]], p[t[1]], p[t[2]], p[i], Precision::Exact) > 0;
            CHECK(in == (cs.count(canon(t)) > 0));
        }
        // 定位路径相邻的三角形共享一条边
        for (size_t k = 0; k + 1 < trace.walk.size(); ++k) {
            int shared = 0;
            for (int a : trace.walk[k])
                for (int b : trace.walk[k + 1]) shared += a == b;
            CHECK(shared == 2);
        }
        ++checked;
    }
    CHECK(checked > 290);
}

static bool insideConvex(const Poly& c, P2 q) {
    for (size_t k = 0; k < c.size(); ++k) {
        P2 a = c[k], b = c[(k + 1) % c.size()];
        double cr = (b.x - a.x) * (q.y - a.y) - (b.y - a.y) * (q.x - a.x);
        if (cr < -1e-9) return false;
    }
    return true;
}

static void testVoronoi() {
    for (int s = 0; s < 6; ++s) {
        Pcg32 rng{uint64_t(900 + s)};
        double w = 50 + 10 * s, h = 40;
        std::vector<Vec2> p(200 + 50 * s);
        for (Vec2& a : p) a = {float(rng.nextDouble() * w), float(rng.nextDouble() * h)};
        Triangulation tr = triangulate(p);
        std::vector<std::vector<int>> nb = tr.neighbors();
        std::vector<Poly> cells = voronoiCells(p, nb, w, h);
        double total = 0, worst = 0;
        for (size_t i = 0; i < p.size(); ++i) {
            Poly brute = boxPoly(w, h);  // 对所有点的平分线裁剪
            for (size_t j = 0; j < p.size(); ++j)
                if (j != i) brute = clipBisector(brute, p[i], p[j]);
            worst = std::max(worst, std::abs(area(brute) - area(cells[i])));
            total += area(cells[i]);
        }
        CHECK(worst < 1e-9 * w * h);
        CHECK(std::abs(total - w * h) < 1e-8 * w * h);
        // 平均邻居数 < 6（Euler：内部点平均 6，凸包上的点邻居少）
        double mean = 0;
        for (auto& v : nb) mean += double(v.size()) / p.size();
        CHECK(mean < 6 && mean > 5);
        // 随机查询点：落在单元 i 里，就一定离点 i 最近
        int wrong = 0;
        for (int q = 0; q < 3000; ++q) {
            P2 x{rng.nextDouble() * w, rng.nextDouble() * h};
            int best = 0;
            double bd = INFINITY;
            for (size_t j = 0; j < p.size(); ++j) {
                double d = (x.x - p[j].x) * (x.x - p[j].x) + (x.y - p[j].y) * (x.y - p[j].y);
                if (d < bd) bd = d, best = int(j);
            }
            wrong += !insideConvex(cells[best], x);
        }
        CHECK(wrong == 0);
        // 单元的内部顶点（不在边界框上）是某个 Delaunay 三角形的外心
        std::vector<P2> cc;
        for (const Tri3& t : tr.triangles()) {
            double ax = p[t[0]].x, ay = p[t[0]].y, bx = p[t[1]].x - ax, by = p[t[1]].y - ay, cx = p[t[2]].x - ax, cy = p[t[2]].y - ay;
            double d = 2 * (bx * cy - by * cx);
            cc.push_back({ax + (cy * (bx * bx + by * by) - by * (cx * cx + cy * cy)) / d, ay + (bx * (cx * cx + cy * cy) - cx * (bx * bx + by * by)) / d});
        }
        int interior = 0, unmatched = 0;
        for (size_t i = 0; i < p.size(); i += 7)
            for (P2 v : cells[i]) {
                if (v.x < 1e-9 || v.y < 1e-9 || v.x > w - 1e-9 || v.y > h - 1e-9) continue;
                ++interior;
                double bd = INFINITY;
                for (P2 c : cc) bd = std::min(bd, std::hypot(c.x - v.x, c.y - v.y));
                unmatched += bd > 1e-6;
            }
        CHECK(interior > 50);
        CHECK(unmatched == 0);
    }
}

static void testMoments() {
    // 质心与二阶矩：和细网格上的数值积分对照（凸多边形，点在多边形外也成立）
    Pcg32 rng{41};
    for (int s = 0; s < 10; ++s) {
        Poly c = boxPoly(10, 8);
        Vec2 a{float(rng.nextDouble() * 10), float(rng.nextDouble() * 8)};
        for (int k = 0; k < 5; ++k) c = clipBisector(c, a, {float(rng.nextDouble() * 10), float(rng.nextDouble() * 8)});
        if (area(c) < 1) continue;
        Vec2 site{float(rng.nextDouble() * 12 - 1), float(rng.nextDouble() * 10 - 1)};
        const int res = 1000;
        double A = 0, cx = 0, cy = 0, m = 0, dA = (10.0 / res) * (8.0 / res);
        for (int y = 0; y < res; ++y)
            for (int x = 0; x < res; ++x) {
                P2 q{(x + 0.5) * 10 / res, (y + 0.5) * 8 / res};
                if (!insideConvex(c, q)) continue;
                A += dA, cx += q.x * dA, cy += q.y * dA, m += ((q.x - site.x) * (q.x - site.x) + (q.y - site.y) * (q.y - site.y)) * dA;
            }
        CHECK(std::abs(A - area(c)) < 0.01 * area(c));
        P2 g = centroid(c);
        CHECK(std::abs(cx / A - g.x) < 0.01 && std::abs(cy / A - g.y) < 0.01);
        CHECK(std::abs(m - secondMoment(c, site)) < 0.01 * m);
    }
}

static void testLloyd() {
    // 能量每一步都不增加；点始终在区域内
    for (int s = 0; s < 3; ++s) {
        Pcg32 rng{uint64_t(60 + s)};
        std::vector<Vec2> p(400);
        for (Vec2& a : p) a = {rng.nextFloat() * 40, rng.nextFloat() * 30};
        double prev = INFINITY;
        // 返回的是移动之前的能量：和细网格上"每个采样点到最近点的距离平方"的积分对照
        if (s == 0) {
            std::vector<Vec2> q(p.begin(), p.begin() + 60);
            for (Vec2& a : q) a = {a.x / 4, a.y / 3};  // 缩到 10×10
            std::vector<Vec2> before = q;
            const int res = 500;
            double brute = 0, dA = (10.0 / res) * (10.0 / res);
            for (int y = 0; y < res; ++y)
                for (int x = 0; x < res; ++x) {
                    double px = (x + 0.5) * 10 / res, py = (y + 0.5) * 10 / res, bd = INFINITY;
                    for (const Vec2& a : before) bd = std::min(bd, (px - a.x) * (px - a.x) + (py - a.y) * (py - a.y));
                    brute += bd * dA;
                }
            LloydStep r = lloyd(q, 10, 10);
            CHECK(std::abs(r.energy - brute) < 0.005 * brute);
        }
        for (int it = 0; it < 30; ++it) {
            LloydStep r = lloyd(p, 40, 30);
            CHECK(r.energy <= prev * (1 + 1e-9));
            prev = r.energy;
        }
        for (const Vec2& a : p) CHECK(a.x >= 0 && a.y >= 0 && a.x <= 40 && a.y <= 30);
        // 收敛后的平均能量接近六边形的下界：每单位面积 5√3/54 · (面积/点)
        double hex = 5 * std::sqrt(3.0) / 54 * (40.0 * 30 / 400) * 40 * 30;
        CHECK(prev > hex && prev < 1.05 * hex);
    }
}

static void testHilbert() {
    // 4 位：256 个格子一一对应，相邻编号的格子相邻
    std::vector<int> seen(256, 0);
    std::vector<std::pair<int, int>> at(256);
    for (uint32_t y = 0; y < 16; ++y)
        for (uint32_t x = 0; x < 16; ++x) {
            uint32_t d = hilbertIndex(x, y, 4);
            CHECK(d < 256);
            if (d < 256) ++seen[d], at[d] = {int(x), int(y)};
        }
    for (int d = 0; d < 256; ++d) CHECK(seen[d] == 1);
    for (int d = 0; d + 1 < 256; ++d) CHECK(std::abs(at[d].first - at[d + 1].first) + std::abs(at[d].second - at[d + 1].second) == 1);
    // 排序结果是一个排列
    Pcg32 rng{3};
    std::vector<Vec2> p = randomInts(rng, 1000, 5000);
    for (Order o : {Order::Random, Order::Hilbert, Order::SortX}) {
        std::vector<int> idx = insertionOrder(p, o, rng);
        std::sort(idx.begin(), idx.end());
        for (int i = 0; i < 1000; ++i) CHECK(idx[i] == i);
    }
}

static void testSuperTriangle() {
    // 超级三角形（精确谓词）：凸包附近会缺三角形
    Pcg32 rng{12};
    std::vector<Vec2> p = randomInts(rng, 500, 1000);
    std::set<Tri3> ref = canonSet(triangulate(p, Order::Hilbert, Precision::Exact).triangles());
    std::vector<Tri3> small = superTriangleDelaunay(p, 1, Precision::Exact);
    CHECK(int(small.size()) < int(ref.size()));
    std::vector<Tri3> big = superTriangleDelaunay(p, 1000, Precision::Exact);
    CHECK(int(big.size()) <= int(ref.size()));
    CHECK(int(big.size()) > int(small.size()));
    int notInRef = 0;
    for (const Tri3& t : small) notInRef += !ref.count(canon(t));
    // 留下的三角形外接圆里连超级顶点都没有，更没有真实的点，所以一定是真的 Delaunay 三角形：只会缺，不会错
    CHECK(notInRef == 0);
}

int main() {
    testPredicates();
    testSmallVsBrute();
    testDegenerate();
    testLarge();
    testCavityRecord();
    testVoronoi();
    testMoments();
    testLloyd();
    testHilbert();
    testSuperTriangle();
    return report("delaunay_test");
}
