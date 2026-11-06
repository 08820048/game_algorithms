// SAT 性能测试（不由 CMake 构建）：
//   c++ -std=c++20 -O2 -Icpp/common cpp/collision/sat/sat_bench.cpp -o sat_bench && ./sat_bench
#include <chrono>
#include <cstdio>

#include "../../foundations/random/random.h"
#include "sat.h"

using namespace ga;
using Clock = std::chrono::steady_clock;

static float rnd(Pcg32& r, float lo, float hi) { return lo + r.nextFloat() * (hi - lo); }

// 布尔判断不需要单位法线：投影区间同比例缩放，比较结果不变
static bool overlapsRaw(const ConvexPolygon& a, const ConvexPolygon& b) {
    auto sep = [&](Vec2 e) { return separatedOn(a, b, Vec2{e.y, -e.x}); };
    for (int i = 0; i < a.size(); ++i)
        if (sep(a.edge(i))) return false;
    for (int i = 0; i < b.size(); ++i)
        if (sep(b.edge(i))) return false;
    return true;
}

template <class F>
static double bestOf(int runs, F&& f) {
    double best = 1e30;
    for (int r = 0; r < runs; ++r) {
        auto t0 = Clock::now();
        f();
        best = std::min(best, std::chrono::duration<double, std::nano>(Clock::now() - t0).count());
    }
    return best;
}

static volatile int sink;

int main() {
    Pcg32 rng(7);
    const int kPairs = 20000;
    std::printf("%-8s %-10s %10s %10s %10s\n", "verts", "pairs", "normalized", "raw axis", "contact");
    for (int n : {4, 8, 16}) {
        for (int kind = 0; kind < 2; ++kind) {  // 0：大多分离 1：全部重叠
            std::vector<ConvexPolygon> as, bs;
            while (static_cast<int>(as.size()) < kPairs) {
                ConvexPolygon a = regularPolygon({0, 0}, 10, n, rnd(rng, 0, 6.3f));
                float d = kind == 0 ? rnd(rng, 0, 60) : rnd(rng, 0, 15);
                ConvexPolygon b = regularPolygon(fromAngle(rnd(rng, 0, 6.3f)) * d, 10, n, rnd(rng, 0, 6.3f));
                if (kind == 1 && !overlaps(a, b)) continue;
                as.push_back(a), bs.push_back(b);
            }
            int hits = 0;
            for (int i = 0; i < kPairs; ++i) hits += overlaps(as[i], bs[i]);
            double t1 = bestOf(15, [&] { int s = 0; for (int i = 0; i < kPairs; ++i) s += overlaps(as[i], bs[i]); sink = s; });
            double t2 = bestOf(15, [&] { int s = 0; for (int i = 0; i < kPairs; ++i) s += overlapsRaw(as[i], bs[i]); sink = s; });
            double t3 = bestOf(15, [&] { int s = 0; for (int i = 0; i < kPairs; ++i) s += contact(as[i], bs[i]).has_value(); sink = s; });
            std::printf("%-8d %3d%% hit   %8.1fns %8.1fns %8.1fns\n", n, hits * 100 / kPairs, t1 / kPairs, t2 / kPairs, t3 / kPairs);
        }
    }

    // 时间相干性：物体缓慢移动、旋转，比较每帧从头检查与先试上一帧的分离轴
    std::printf("\ncoherent motion (8-gons, 2000 pairs x 300 frames)\n");
    const int kN = 2000, kFrames = 300;
    std::vector<ConvexPolygon> local(kN);
    std::vector<Vec2> pa(kN), pb(kN), vb(kN);
    std::vector<float> rb(kN);
    for (int i = 0; i < kN; ++i) {
        local[i] = regularPolygon({0, 0}, 10, 8, 0);
        pa[i] = {0, 0};
        pb[i] = fromAngle(rnd(rng, 0, 6.3f)) * rnd(rng, 10, 50);
        vb[i] = fromAngle(rnd(rng, 0, 6.3f)) * 0.3f;
        rb[i] = rnd(rng, -0.02f, 0.02f);
    }
    auto frameShapes = [&](int f, int i) {
        Vec2 p = pb[i] + vb[i] * float(f % 100 < 50 ? f % 100 : 100 - f % 100);
        return std::pair{transformed(local[i], pa[i], 0), transformed(local[i], p, rb[i] * f)};
    };
    // 宽相之后才轮到 SAT：只保留包围盒重叠的物体对（它们彼此很近，分离轴不容易一下找到）。
    // 被剔除的那一帧用一对远离的形状占位，保持"每个物体对每帧一次"的访问顺序
    auto bounds = [](const ConvexPolygon& p) {
        Aabb b{p.v[0], p.v[0]};
        for (Vec2 q : p.v) b.min = {std::min(b.min.x, q.x), std::min(b.min.y, q.y)}, b.max = {std::max(b.max.x, q.x), std::max(b.max.y, q.y)};
        return b;
    };
    std::vector<std::pair<ConvexPolygon, ConvexPolygon>> frames;
    std::vector<char> live;
    frames.reserve(size_t(kN) * kFrames);
    for (int f = 0; f < kFrames; ++f)
        for (int i = 0; i < kN; ++i) {
            auto pr = frameShapes(f, i);
            live.push_back(overlaps(bounds(pr.first), bounds(pr.second)));
            frames.push_back(std::move(pr));
        }
    long hits = 0, liveCount = 0;
    for (size_t k = 0; k < frames.size(); ++k)
        if (live[k]) ++liveCount, hits += overlaps(frames[k].first, frames[k].second);
    double plain = bestOf(15, [&] {
        int s = 0;
        for (size_t k = 0; k < frames.size(); ++k)
            if (live[k]) s += overlaps(frames[k].first, frames[k].second);
        sink = s;
    });
    std::vector<SatCache> caches(kN);
    long cacheHits = 0, separated = 0;
    for (int f = 0; f < kFrames; ++f)
        for (int i = 0; i < kN; ++i) {
            size_t k = size_t(f) * kN + i;
            if (!live[k]) continue;
            auto& [a, b] = frames[k];
            int before = caches[i].axis;
            bool o = overlapsCached(a, b, caches[i]);
            if (!o) ++separated, cacheHits += (before >= 0 && caches[i].axis == before);
        }
    double cached = bestOf(15, [&] {
        std::vector<SatCache> c(kN);
        int s = 0;
        for (int f = 0; f < kFrames; ++f)
            for (int i = 0; i < kN; ++i) {
                size_t k = size_t(f) * kN + i;
                if (!live[k]) continue;
                auto& [a, b] = frames[k];
                s += overlapsCached(a, b, c[i]);
            }
        sink = s;
    });
    double total = double(liveCount);
    std::printf("%ld tests after AABB filter, overlapping %.0f%%, cached axis still separating %.0f%% of separated tests\n",
                liveCount, hits * 100 / total, cacheHits * 100.0 / separated);
    // 分离的那部分测试里，平均要试几条轴才找到分离轴
    long plainAxes = 0, cachedAxes = 0;
    std::vector<SatCache> c2(kN);
    for (int f = 0; f < kFrames; ++f)
        for (int i = 0; i < kN; ++i) {
            size_t k = size_t(f) * kN + i;
            if (!live[k]) continue;
            auto& [a, b] = frames[k];
            int n = a.size() + b.size();
            auto axisAt = [&](int j) { return j < a.size() ? a.normal(j) : b.normal(j - a.size()); };
            int t = 0;
            bool sep = false;
            for (int j = 0; j < n && !sep; ++j) ++t, sep = separatedOn(a, b, axisAt(j));
            if (!sep) continue;
            plainAxes += t;
            int prev = c2[i].axis;
            overlapsCached(a, b, c2[i]);
            int now = c2[i].axis;
            if (prev >= 0 && now == prev) cachedAxes += 1;                         // 缓存的轴直接命中
            else cachedAxes += (prev >= 0) + now + 1 - (prev >= 0 && prev < now);  // 先试缓存，再从头扫（跳过它）
        }
    std::printf("axes tried per separated test: plain %.2f, cached %.2f\n", double(plainAxes) / separated,
                double(cachedAxes) / separated);
    std::printf("plain  %.1f ns/test\ncached %.1f ns/test\n", plain / total, cached / total);
}
