#include "noise.h"

#include <cstdio>
#include <vector>

#include "check.h"

using namespace ga;
using namespace ga::noise;

static const FadeCurve kFades[] = {FadeCurve::Linear, FadeCurve::Smooth, FadeCurve::Quintic};

// ---- 暴力参照：核函数求和 ----
// 噪声 = Σ 每个格点的贡献 × K(x - i) × K(y - j)，K(t) = 1 - fade(|t|)（|t| < 1），否则为 0。
// 不分格子、不嵌套插值，直接对周围 4×4 个格点求和
static double kernel(FadeCurve f, double t) {
    t = std::abs(t);
    return t < 1 ? 1 - fade(f, t) : 0;
}
template <class Lattice>
static double valueBrute(const Lattice& lat, double x, double y, FadeCurve f) {
    double s = 0;
    for (int j = int(std::floor(y)) - 1; j <= int(std::floor(y)) + 2; ++j)
        for (int i = int(std::floor(x)) - 1; i <= int(std::floor(x)) + 2; ++i)
            s += lat.template value<double>(i, j) * kernel(f, x - i) * kernel(f, y - j);
    return s;
}
template <class Lattice>
static double perlinBrute(const Lattice& lat, double x, double y, FadeCurve f) {
    double s = 0;
    for (int j = int(std::floor(y)) - 1; j <= int(std::floor(y)) + 2; ++j)
        for (int i = int(std::floor(x)) - 1; i <= int(std::floor(x)) + 2; ++i) {
            double gx, gy;
            gradientVector(lat.gradient(i, j), gx, gy);
            s += (gx * (x - i) + gy * (y - j)) * kernel(f, x - i) * kernel(f, y - j);
        }
    return s;
}

static void testFade() {
    for (FadeCurve f : kFades) {
        CHECK(fade(f, 0.0) == 0 && fade(f, 1.0) == 1);
        for (double t = 0; t <= 1; t += 0.03125) {
            CHECK(std::abs(fade(f, 1 - t) - (1 - fade(f, t))) < 1e-12);  // 关于 (1/2, 1/2) 对称，核函数写法依赖这一点
            const double h = 1e-5;
            CHECK(std::abs((fade(f, t + h) - fade(f, t - h)) / (2 * h) - fadeD(f, t)) < 1e-6);
            CHECK(std::abs((fadeD(f, t + h) - fadeD(f, t - h)) / (2 * h) - fadeDD(f, t)) < 1e-5);
            if (t > 0 && t < 1) CHECK(fadeD(f, t) > 0);  // 单调
        }
    }
    // 两端的导数：linear 斜率为 1（格点处有折角），smoothstep 斜率为 0 但二阶导数为 ±6，quintic 两者都为 0
    CHECK(fadeD(FadeCurve::Linear, 0.0) == 1);
    CHECK(fadeD(FadeCurve::Smooth, 0.0) == 0 && fadeD(FadeCurve::Smooth, 1.0) == 0);
    CHECK(fadeDD(FadeCurve::Smooth, 0.0) == 6 && fadeDD(FadeCurve::Smooth, 1.0) == -6);
    CHECK(fadeD(FadeCurve::Quintic, 0.0) == 0 && fadeD(FadeCurve::Quintic, 1.0) == 0);
    CHECK(fadeDD(FadeCurve::Quintic, 0.0) == 0 && fadeDD(FadeCurve::Quintic, 1.0) == 0);
}

template <class Lattice>
static void testAgainstBrute(const Lattice& lat, uint64_t seed) {
    Pcg32 rng(seed);
    double worstV = 0, worstP = 0;
    for (int k = 0; k < 20000; ++k) {
        double x = (rng.nextDouble() - 0.5) * 600, y = (rng.nextDouble() - 0.5) * 600;
        FadeCurve f = kFades[k % 3];
        worstV = std::max(worstV, std::abs(valueNoise(lat, x, y, f) - valueBrute(lat, x, y, f)));
        worstP = std::max(worstP, std::abs(perlinNoise(lat, x, y, f) - perlinBrute(lat, x, y, f)));
        // 带导数的版本，值必须与不带导数的版本一致
        CHECK(std::abs(valueNoiseD(lat, x, y, f).value - valueNoise(lat, x, y, f)) < 1e-12);
        CHECK(std::abs(perlinNoiseD(lat, x, y, f).value - perlinNoise(lat, x, y, f)) < 1e-12);
    }
    CHECK(worstV < 1e-12);
    CHECK(worstP < 1e-12);
    // float 版本与 double 版本只差舍入误差
    for (int k = 0; k < 2000; ++k) {
        float x = float((rng.nextDouble() - 0.5) * 60), y = float((rng.nextDouble() - 0.5) * 60);
        CHECK(std::abs(perlinNoise(lat, x, y) - perlinNoise(lat, double(x), double(y))) < 1e-5);
        CHECK(std::abs(valueNoise(lat, x, y) - valueNoise(lat, double(x), double(y))) < 1e-5);
    }
}

template <class Lattice>
static void testLatticePoints(const Lattice& lat) {
    for (int y = -20; y <= 20; ++y)
        for (int x = -20; x <= 20; ++x)
            for (FadeCurve f : kFades) {
                CHECK(valueNoise(lat, double(x), double(y), f) == lat.template value<double>(x, y));
                CHECK(perlinNoise(lat, double(x), double(y), f) == 0);  // Perlin 噪声在每个格点上都是 0
            }
}

template <class Lattice>
static void testDerivatives(const Lattice& lat) {
    Pcg32 rng(9);
    const double h = 1e-6;
    for (int k = 0; k < 5000; ++k) {
        double x = (rng.nextDouble() - 0.5) * 100, y = (rng.nextDouble() - 0.5) * 100;
        // 离格线太近时中心差分会跨过折角（linear），跳过
        double fx = x - std::floor(x), fy = y - std::floor(y);
        if (fx < 1e-4 || fx > 1 - 1e-4 || fy < 1e-4 || fy > 1 - 1e-4) continue;
        FadeCurve f = kFades[k % 3];
        Sample<double> v = valueNoiseD(lat, x, y, f), p = perlinNoiseD(lat, x, y, f);
        CHECK(std::abs((valueNoise(lat, x + h, y, f) - valueNoise(lat, x - h, y, f)) / (2 * h) - v.dx) < 1e-6);
        CHECK(std::abs((valueNoise(lat, x, y + h, f) - valueNoise(lat, x, y - h, f)) / (2 * h) - v.dy) < 1e-6);
        CHECK(std::abs((perlinNoise(lat, x + h, y, f) - perlinNoise(lat, x - h, y, f)) / (2 * h) - p.dx) < 1e-6);
        CHECK(std::abs((perlinNoise(lat, x, y + h, f) - perlinNoise(lat, x, y - h, f)) / (2 * h) - p.dy) < 1e-6);
    }
    // 在格点上，Perlin 噪声的梯度正好等于该格点的梯度向量（smoothstep 与 quintic）
    for (int y = -5; y <= 5; ++y)
        for (int x = -5; x <= 5; ++x) {
            double gx, gy;
            gradientVector(lat.gradient(x, y), gx, gy);
            for (FadeCurve f : {FadeCurve::Smooth, FadeCurve::Quintic}) {
                Sample<double> s = perlinNoiseD(lat, double(x), double(y), f);
                CHECK(std::abs(s.dx - gx) < 1e-12 && std::abs(s.dy - gy) < 1e-12);
            }
        }
}

// 跨过格线时的连续性：值总是连续的；一阶导数只有 linear 会跳变；二阶导数只有 quintic 连续
template <class Lattice>
static void testContinuity(const Lattice& lat) {
    const double e = 1e-7;
    double jumpV[3] = {}, jumpD1[3] = {}, jumpD2[3] = {};
    for (int k = 0; k < 400; ++k) {
        double x = k % 40 - 20.0, y = (k / 40) * 0.37 + 0.11;  // 竖直格线 x = 整数 上的点
        for (int fi = 0; fi < 3; ++fi) {
            FadeCurve f = kFades[fi];
            auto val = [&](double xx) { return valueNoise(lat, xx, y, f); };
            auto d1 = [&](double xx) { return valueNoiseD(lat, xx, y, f).dx; };
            const double h = 1e-4;
            auto d2 = [&](double xx) { return (d1(xx + h) - d1(xx - h)) / (2 * h); };
            jumpV[fi] = std::max(jumpV[fi], std::abs(val(x + e) - val(x - e)));
            jumpD1[fi] = std::max(jumpD1[fi], std::abs(d1(x + e) - d1(x - e)));
            jumpD2[fi] = std::max(jumpD2[fi], std::abs(d2(x + 2 * h) - d2(x - 2 * h)));
        }
    }
    for (int fi = 0; fi < 3; ++fi) CHECK(jumpV[fi] < 1e-5);
    CHECK(jumpD1[0] > 0.5);  // linear：一阶导数跳变
    CHECK(jumpD1[1] < 1e-5 && jumpD1[2] < 1e-5);
    CHECK(jumpD2[1] > 1.0);  // smoothstep：二阶导数跳变
    CHECK(jumpD2[2] < 0.05);
}

template <class Lattice>
static void testRange(const Lattice& lat) {
    Pcg32 rng(77);
    double maxP = 0;
    for (int k = 0; k < 200000; ++k) {
        double x = (rng.nextDouble() - 0.5) * 1000, y = (rng.nextDouble() - 0.5) * 1000;
        // Value Noise 是四个角的值的凸组合，不会超出它们的范围
        int ix = int(std::floor(x)), iy = int(std::floor(y));
        double lo = 1e9, hi = -1e9;
        for (int dy = 0; dy < 2; ++dy)
            for (int dx = 0; dx < 2; ++dx) {
                double c = lat.template value<double>(ix + dx, iy + dy);
                lo = std::min(lo, c), hi = std::max(hi, c);
            }
        double v = valueNoise(lat, x, y, kFades[k % 3]);
        CHECK(v >= lo - 1e-12 && v <= hi + 1e-12);
        maxP = std::max(maxP, std::abs(perlinNoise(lat, x, y, kFades[k % 3])));
    }
    CHECK(maxP <= std::sqrt(0.5) + 1e-12);
}

// 不依赖哈希：对四个角的 8^4 种梯度组合逐一检查，|n| 不超过 √2/2，并且在格子中心能取到
struct FixedCorners {  // 直接指定四个角的梯度（只用于 [0, 1]² 这一个格子）
    int g[4];
    int gradient(int x, int y) const { return g[(y & 1) * 2 + (x & 1)]; }
};
static void testRangeExhaustive() {
    using Fixed = FixedCorners;
    double best = 0;
    for (int combo = 0; combo < 4096; ++combo) {
        Fixed lat{{combo & 7, (combo >> 3) & 7, (combo >> 6) & 7, (combo >> 9) & 7}};
        for (int j = 0; j <= 32; ++j)
            for (int i = 0; i <= 32; ++i)
                for (FadeCurve f : kFades) best = std::max(best, std::abs(perlinNoise(lat, i / 32.0, j / 32.0, f)));
    }
    CHECK(best <= std::sqrt(0.5) + 1e-12);
    CHECK(best >= std::sqrt(0.5) - 1e-12);
    // 四个角的梯度都朝向中心：左下角 (s, s) = 1，右下角 (-s, s) = 3，左上角 (s, -s) = 7，右上角 (-s, -s) = 5
    Fixed in{{1, 3, 7, 5}};
    CHECK(std::abs(perlinNoise(in, 0.5, 0.5) - std::sqrt(0.5)) < 1e-12);
}

static void testPeriodAndSeeds() {
    PermLattice perm(3);
    HashLattice hash{3}, tiled{3, 7};
    Pcg32 rng(5);
    int hashRepeats = 0;
    for (int k = 0; k < 2000; ++k) {
        double x = rng.nextDouble() * 50, y = rng.nextDouble() * 50;
        CHECK(perlinNoise(perm, x + 256, y) == perlinNoise(perm, x, y));  // 置换表：周期 256
        CHECK(perlinNoise(perm, x, y - 256) == perlinNoise(perm, x, y));
        CHECK(std::abs(perlinNoise(tiled, x + 7, y) - perlinNoise(tiled, x, y)) < 1e-12);  // 指定周期，无缝平铺
        CHECK(std::abs(valueNoise(tiled, x, y + 14) - valueNoise(tiled, x, y)) < 1e-12);
        hashRepeats += perlinNoise(hash, x + 256, y) == perlinNoise(hash, x, y);
    }
    CHECK(hashRepeats < 20);  // 整数哈希没有 256 的周期
    // 平铺必须跨过 0 也成立：负坐标取模后要落回 [0, period)
    for (int k = 0; k < 2000; ++k) {
        double x = (rng.nextDouble() - 0.5) * 60, y = (rng.nextDouble() - 0.5) * 60;
        CHECK(std::abs(perlinNoise(tiled, x - 7, y) - perlinNoise(tiled, x, y)) < 1e-12);
        CHECK(std::abs(valueNoise(tiled, x, y - 7) - valueNoise(tiled, x, y)) < 1e-12);
    }
    // 同一个种子结果完全相同；不同种子之间几乎不相关
    HashLattice a{1}, b{1}, c{2};
    double sac = 0, saa = 0, scc = 0;
    for (int k = 0; k < 20000; ++k) {
        double x = rng.nextDouble() * 200, y = rng.nextDouble() * 200;
        double va = perlinNoise(a, x, y), vc = perlinNoise(c, x, y);
        CHECK(va == perlinNoise(b, x, y));
        sac += va * vc, saa += va * va, scc += vc * vc;
    }
    CHECK(std::abs(sac / std::sqrt(saa * scc)) < 0.05);
}

// 格点哈希的质量：梯度编号均匀，相邻格点的值不相关
template <class Lattice>
static void testHashQuality(const Lattice& lat) {
    long long count[8] = {};
    double sxy = 0, sxx = 0, sdiag = 0, santi = 0;
    const int n = 300;
    for (int y = 0; y < n; ++y)
        for (int x = 0; x < n; ++x) {
            ++count[lat.gradient(x, y)];
            double v = lat.template value<double>(x, y);
            sxx += v * v, sxy += v * lat.template value<double>(x + 1, y), sdiag += v * lat.template value<double>(x + 1, y + 1);
            santi += v * lat.template value<double>(x + 1, y - 1);
        }
    double expect = n * n / 8.0, chi = 0;
    for (long long c : count) chi += (c - expect) * (c - expect) / expect;
    CHECK(chi < 24.3);  // 自由度 7，p = 0.001 的临界值
    CHECK(std::abs(sxy / sxx) < 0.02);
    CHECK(std::abs(sdiag / sxx) < 0.02);
    CHECK(std::abs(santi / sxx) < 0.02);
    CHECK(std::abs(sxx / (n * n) - 1.0 / 3) < 0.01);  // [-1, 1] 上均匀分布的方差是 1/3
}

static void testOneD() {
    for (FadeCurve f : kFades)
        for (double t = 0; t <= 1; t += 0.125) {
            // 一维梯度噪声在两端为 0，端点斜率等于格点斜率（linear 除外）
            CHECK(gradientSegment(0.7, -0.3, 0.0, f) == 0);
            CHECK(std::abs(gradientSegment(0.7, -0.3, 1.0, f)) < 1e-15);
            CHECK(valueSegment(0.2, 0.9, t, f) >= 0.2 - 1e-15 && valueSegment(0.2, 0.9, t, f) <= 0.9 + 1e-15);
        }
    const double h = 1e-6;
    for (FadeCurve f : {FadeCurve::Smooth, FadeCurve::Quintic}) {
        CHECK(std::abs(gradientSegment(0.7, -0.3, h, f) / h - 0.7) < 1e-4);
        CHECK(std::abs(-gradientSegment(0.7, -0.3, 1 - h, f) / h - -0.3) < 1e-4);
    }
}

int main() {
    testFade();
    testAgainstBrute(HashLattice{42}, 1);
    testAgainstBrute(PermLattice(42), 2);
    testAgainstBrute(HashLattice{42, 16}, 3);
    testLatticePoints(HashLattice{7});
    testLatticePoints(PermLattice(7));
    testDerivatives(HashLattice{8});
    testDerivatives(PermLattice(8));
    testContinuity(HashLattice{10});
    testRange(HashLattice{11});
    testRange(PermLattice(11));
    testRangeExhaustive();
    testPeriodAndSeeds();
    testHashQuality(HashLattice{12345});
    testHashQuality(PermLattice(12345));
    testOneD();
    return report("noise_test");
}
