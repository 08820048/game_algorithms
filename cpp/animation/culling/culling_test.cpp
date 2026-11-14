#include "culling.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <set>
#include <vector>

#include "check.h"
#include "scene.h"

using namespace ga;
using namespace ga::culling;

static View randomView(Pcg32& r, float span) {
    float a = r.nextFloat() * 2 * kPi;
    return {{r.nextFloat() * span, r.nextFloat() * span}, fromAngle(a), 0.2f + r.nextFloat() * 0.9f, 1 + r.nextFloat() * 10,
            100 + r.nextFloat() * 300};
}

// 参照：点在视锥内 ⇔ 视图空间 near ≤ z ≤ far 且 |x| ≤ z·tan(halfFov)
static bool insideRef(const View& v, Vec2 p) {
    Vec2 q = v.toView(p);
    return q.y >= v.near && q.y <= v.far && std::fabs(q.x) <= q.y * std::tan(v.halfFov);
}

// 参照：用盒子的四条边裁剪视锥梯形，剩下的面积 > 0 ⇔ 相交。
// 用 double 计算：擦边的盒子只切进去千分之几，切下的面积约 1e-5，float 的叉积相消后会变成 0（6.4 加上
// -ffp-contract=off 后暴露出来：参照说不相交，被测函数说相交，其实被测函数是对的）
static double clippedArea(const Frustum& f, const Aabb& b) {
    struct D {
        double x, y;
    };
    std::vector<D> poly, out;
    for (Vec2 c : f.corners) poly.push_back({c.x, c.y});
    auto clip = [&](auto inside, auto cut) {
        out.clear();
        for (size_t i = 0; i < poly.size(); ++i) {
            D a = poly[i], c = poly[(i + 1) % poly.size()];
            if (inside(a)) out.push_back(a);
            if (inside(a) != inside(c)) out.push_back(cut(a, c));
        }
        poly.swap(out);
    };
    auto atX = [](double x) { return [x](D a, D c) { double t = (x - a.x) / (c.x - a.x); return D{x, a.y + (c.y - a.y) * t}; }; };
    auto atY = [](double y) { return [y](D a, D c) { double t = (y - a.y) / (c.y - a.y); return D{a.x + (c.x - a.x) * t, y}; }; };
    clip([&](D p) { return p.x >= b.min.x; }, atX(b.min.x));
    clip([&](D p) { return p.x <= b.max.x; }, atX(b.max.x));
    clip([&](D p) { return p.y >= b.min.y; }, atY(b.min.y));
    clip([&](D p) { return p.y <= b.max.y; }, atY(b.max.y));
    double area = 0;
    for (size_t i = 0; i < poly.size(); ++i) {
        D a = poly[i], c = poly[(i + 1) % poly.size()];
        area += a.x * c.y - a.y * c.x;
    }
    return std::fabs(area) / 2;
}

static void testPlanes() {
    Pcg32 r{1};
    for (int trial = 0; trial < 200; ++trial) {
        View v = randomView(r, 500);
        Frustum f = makeFrustum(v);
        for (int k = 0; k < 200; ++k) {
            Vec2 p = v.eye + Vec2{(r.nextFloat() * 2 - 1) * 450, (r.nextFloat() * 2 - 1) * 450};
            bool in = true;
            for (const Plane& pl : f.planes) in &= pl.distance(p) >= 0;
            Vec2 q = v.toView(p);
            float slack = 1e-3f * (std::fabs(q.x) + std::fabs(q.y) + 1);
            bool nearEdge = std::fabs(q.y - v.near) < slack || std::fabs(q.y - v.far) < slack ||
                            std::fabs(std::fabs(q.x) - q.y * std::tan(v.halfFov)) < slack;
            if (!nearEdge) CHECK(in == insideRef(v, p));
        }
        for (Vec2 c : f.corners) CHECK(std::fabs(v.toView(c).y - v.near) < 1e-3f || std::fabs(v.toView(c).y - v.far) < 1e-2f);
    }
}

// 平面测试：不会把相交的盒子判为外侧；Inside 的盒子真的完全在里面；精确测试与裁剪参照一致；假阳性确实存在
static void testClassify() {
    Pcg32 r{2};
    long falsePos = 0, exactDisagree = 0, cases = 0, circleWrong = 0;
    for (int trial = 0; trial < 200; ++trial) {
        View v = randomView(r, 500);
        Frustum f = makeFrustum(v);
        for (const Aabb& b : randomBoxes(r, {v.eye - Vec2{450, 450}, v.eye + Vec2{450, 450}}, 300, 2, 120)) {
            ++cases;
            double area = clippedArea(f, b);
            Cull c = classify(f, b);
            if (area > 1e-2f) CHECK(c != Cull::Outside);
            if (c == Cull::Inside)
                for (Vec2 p : corners(b)) CHECK(insideRef(v, p) || std::fabs(v.toView(p).y - v.near) < 1e-2f);
            bool ex = overlapsExact(f, b);
            if (area > 1e-2 || area == 0) exactDisagree += ex != (area > 0);
            if (c != Cull::Outside && area == 0) ++falsePos;
            // 包围圆比盒子更松：圆判外侧 ⇒ 盒子也判外侧
            Vec2 centre = (b.min + b.max) * 0.5f;
            if (circleOutside(f, centre, length(b.max - centre)) && c != Cull::Outside) ++circleWrong;
        }
    }
    std::printf("classify: %ld boxes, %ld plane-test false positives, %ld exact disagreements\n", cases, falsePos, exactDisagree);
    CHECK(falsePos > 0);
    CHECK(exactDisagree == 0);
    CHECK(circleWrong == 0);
}

static bool containsBox(const Aabb& o, const Aabb& i) {
    return o.min.x <= i.min.x && o.min.y <= i.min.y && i.max.x <= o.max.x && i.max.y <= o.max.y;
}

// 层次剔除与逐个测试输出相同的集合，测试次数更少
static void testBvh() {
    Pcg32 r{3};
    for (int trial = 0; trial < 40; ++trial) {
        auto boxes = randomBoxes(r, {{0, 0}, {2000, 2000}}, 200 + trial * 100, 1, 20);
        Bvh t;
        t.build(boxes, 1 + trial % 6);
        // 每个物体恰好出现在一个叶子里
        std::vector<int> seen(boxes.size(), 0);
        for (const auto& n : t.nodes) {
            if (n.leaf())
                for (int i = n.first; i < n.first + n.count; ++i) ++seen[t.items[i]];
            for (int i = n.first; i < n.first + n.count; ++i) CHECK(containsBox(n.box, boxes[t.items[i]]));
        }
        for (int s : seen) CHECK(s == 1);
        for (int k = 0; k < 10; ++k) {
            View v = randomView(r, 2000);
            v.far = 200 + r.nextFloat() * 800;
            Frustum f = makeFrustum(v);
            std::set<int> a, b, c;
            CullStats sa, sb, sc;
            cullBrute(f, boxes, [&](int i) { a.insert(i); }, sa);
            cullBvh(t, f, boxes, true, [&](int i) { CHECK(b.insert(i).second); }, sb);
            cullBvh(t, f, boxes, false, [&](int i) { CHECK(c.insert(i).second); }, sc);
            CHECK(a == b);
            CHECK(a == c);
            CHECK(sb.planeTests <= sc.planeTests);
            CHECK(sb.planeTests < sa.planeTests);
        }
    }
}

// 遮挡：任何一条视线能看到的物体都不能被剔除（保守），四种查询方式都要满足；
// Hi-Z 细分与逐列结果完全相同；只看一层的 Hi-Z 剔除的是逐列剔除的子集；融合至少和单独判断剔除得一样多；
// 分辨率翻倍，剔除的集合只会变大
static void testOcclusion() {
    Pcg32 r{4};
    long total = 0, visible = 0, culled = 0, bigCulled = 0, bigIndiv = 0, culledHiZ1 = 0, culledIndiv = 0;
    for (int trial = 0; trial < 30; ++trial) {
        City city = makeCity(r, {{0, 0}, {1200, 800}}, 6 + trial % 4, 4 + trial % 3, 14 + float(trial % 5) * 4, 600);
        std::vector<std::vector<Vec2>> occ = city.occluders;
        if (trial % 3 == 0)  // 也测几段单独的墙（两个点的遮挡物）
            for (int k = 0; k < 6; ++k) occ.push_back({{r.nextFloat() * 1200, r.nextFloat() * 800}, {r.nextFloat() * 1200, r.nextFloat() * 800}});
        for (int k = 0; k < 4; ++k) {
            Vec2 eye = city.streets[r.next() % city.streets.size()];
            View v{eye, fromAngle(r.nextFloat() * 2 * kPi), 0.4f + r.nextFloat() * 0.6f, 2 + r.nextFloat() * 6, 1500};
            Frustum f = makeFrustum(v);
            int bins = 16 << (k + trial % 4);
            DepthBuffer db(bins), db2(bins * 2), scratch(bins);
            db.begin(v), db2.begin(v);
            for (const auto& o : occ) db.rasterize(o), db2.rasterize(o);
            db.buildHiZ();
            // 写进去的深度 ≥ 真实遮挡物在那一列的深度（保守）：在列内随机取视线核对
            for (int i = 0; i < bins; ++i) {
                if (db.depth(i) == kInf) continue;
                float s = db.binEdge(i) + (r.nextFloat()) * 2 / float(bins);
                Vec2 d = v.dir + v.right() * (s * std::tan(v.halfFov));
                Aabb probe{v.eye + d * (db.depth(i) * 1.0005f + 0.01f), v.eye + d * (db.depth(i) * 1.0005f + 0.01f)};
                CHECK(!visibleByRays(v, probe, occ, 1));
            }
            // 道具（小）+ 一些大物体（会同时被几栋楼挡住）
            std::vector<Aabb> items = city.props;
            for (const Aabb& b : randomBoxes(r, {{0, 0}, {1200, 800}}, 300, 15, 60)) items.push_back(b);
            std::vector<char> indiv;
            cullIndividually(scratch, v, occ, items, indiv);
            for (size_t i = 0; i < items.size(); ++i) {
                const Aabb& b = items[i];
                bool big = i >= city.props.size();
                if (!overlapsExact(f, b)) continue;
                ++total;
                bool vis = visibleByRays(v, b, occ, 64);
                visible += vis;
                bool o = db.occluded(b);
                long t1 = 0, t2 = 0;
                bool h = db.occludedHiZ(b, true, &t1), h1 = db.occludedHiZ(b, false, &t2);
                if (vis) CHECK(!o && !h && !h1 && !indiv[i]);
                CHECK(h == o);
                CHECK(!h1 || o);
                CHECK(!indiv[i] || o);
                CHECK(!o || db2.occluded(b));
                culled += o, culledHiZ1 += h1, culledIndiv += indiv[i];
                if (big) bigCulled += o, bigIndiv += indiv[i];
            }
        }
    }
    std::printf("occlusion: %ld in frustum, %ld visible by rays, culled %ld (hi-z one level %ld, individually %ld; big %ld vs %ld)\n",
                total, visible, culled, culledHiZ1, culledIndiv, bigCulled, bigIndiv);
    CHECK(bigCulled > bigIndiv);  // 融合对大物体确实多剔除了
    CHECK(culled > culledHiZ1);
    CHECK(culledHiZ1 > culled * 95 / 100);  // 只看一层（覆盖 1～2 格）损失不大
    CHECK(culled > (total - visible) / 2);
}

// 近平面：横穿视点前方的墙也要被裁剪后写入；视点背后的墙不写
static void testNearClip() {
    View v{{0, 0}, {1, 0}, 0.6f, 5, 1000};
    DepthBuffer db(64);
    db.begin(v);
    CHECK(db.rasterize({{-10, -50}, {-10, 50}}) == 0);  // 在背后
    CHECK(db.rasterize({{2, -400}, {60, 400}}) > 0);    // 一端在近平面之前
    // 视点站在楼里：近平面在楼内，整屏都应在 near 处被挡住
    db.begin(v);
    db.rasterize(corners({{-20, -20}, {40, 20}}));
    int full = 0;
    for (int i = 0; i < 64; ++i) full += db.depth(i) <= 40.01f;
    CHECK(full >= 60);
    CHECK(db.occluded({{100, -5}, {110, 5}}));
}

// 单个凸遮挡物：每列存的深度 = 列内所有视线碰到前表面的最大深度（既保守，又不比需要的更远）。
// 参照：逐条视线和四条边求交，取最近的交点
static float entryRef(const View& v, const std::vector<Vec2>& poly, float s) {
    Vec2 d = v.dir + v.right() * (s * std::tan(v.halfFov));
    float best = kInf;
    for (size_t j = 0; j < poly.size(); ++j) {
        Vec2 a = poly[j] - v.eye, e = poly[(j + 1) % poly.size()] - poly[j];
        float den = cross(d, e);
        if (std::fabs(den) < 1e-12f) continue;
        float t = cross(a, e) / den, u = cross(a, d) / den;
        if (u >= 0 && u <= 1 && t > 0) best = std::min(best, t);
    }
    return best;
}
static void testTight() {
    Pcg32 r{6};
    int bins = 0;
    for (int trial = 0; trial < 300; ++trial) {
        View v{{0, 0}, fromAngle(r.nextFloat() * 2 * kPi), 0.3f + r.nextFloat() * 0.8f, 1, 2000};
        Vec2 c = v.eye + rotate(v.dir, (r.nextFloat() * 2 - 1) * v.halfFov) * (40 + r.nextFloat() * 400);
        Vec2 h{3 + r.nextFloat() * 60, 3 + r.nextFloat() * 60};
        std::vector<Vec2> poly = corners({c - h, c + h});
        if (contains(Aabb{c - h, c + h}, v.eye)) continue;
        DepthBuffer db(64 << (trial % 4));
        db.begin(v);
        db.rasterize(poly);
        for (int i = 0; i < db.bins(); ++i) {
            if (db.depth(i) == kInf) continue;
            ++bins;
            float worst = 0;
            for (int k = 0; k <= 40; ++k) worst = std::max(worst, entryRef(v, poly, db.binEdge(i) + float(k) / 40 * 2 / float(db.bins())));
            CHECK(worst <= db.depth(i) * (1 + 1e-4f));           // 保守
            CHECK(db.depth(i) <= worst * (1 + 1e-4f) + 1e-3f);   // 紧
        }
    }
    CHECK(bins > 1000);
}

int main() {
    testTight();
    testPlanes();
    testClassify();
    testBvh();
    testOcclusion();
    testNearClip();
    return report("culling_test");
}
