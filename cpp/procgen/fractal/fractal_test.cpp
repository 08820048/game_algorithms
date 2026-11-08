#include "fractal.h"

#include <cstdio>
#include <vector>

#include "check.h"

using namespace ga;
using namespace ga::noise;

// ---- Simplex 的暴力参照：枚举附近所有三角形网格顶点，凡是在影响半径内的都算贡献 ----
// 不判断"在哪个三角形里"，也不假设只有 3 个角有贡献
template <class Lattice>
static double simplexBrute(const Lattice& lat, double x, double y, double r2 = 0.5) {
    double s = (x + y) * kSkew;
    int i0 = int(std::floor(x + s)), j0 = int(std::floor(y + s));
    double sum = 0;
    for (int j = j0 - 3; j <= j0 + 3; ++j)
        for (int i = i0 - 3; i <= i0 + 3; ++i) {
            double t = (i + j) * kUnskew, vx = i - t, vy = j - t;  // 顶点在原平面上的位置
            double dx = x - vx, dy = y - vy, a = r2 - dx * dx - dy * dy;
            if (a <= 0) continue;
            double gx, gy;
            gradientVector(lat.gradient(i, j), gx, gy);
            sum += a * a * a * a * (gx * dx + gy * dy);
        }
    return sum * kSimplexScale;
}

// ---- Worley 的暴力参照：9×9 个格子里的全部特征点 ----
template <class Lattice>
static void worleyBrute(const Lattice& lat, double x, double y, double& f1, double& f2) {
    f1 = f2 = 1e30;
    int ix = int(std::floor(x)), iy = int(std::floor(y));
    for (int j = iy - 4; j <= iy + 4; ++j)
        for (int i = ix - 4; i <= ix + 4; ++i) {
            double px, py;
            featurePoint(lat, i, j, px, py);
            double d = std::sqrt((px - x) * (px - x) + (py - y) * (py - y));
            if (d < f1) f2 = f1, f1 = d;
            else if (d < f2) f2 = d;
        }
}

static void testSimplexBrute() {
    HashLattice lat{5};
    PermLattice perm(5);
    Pcg32 rng(1);
    double worst = 0, worstP = 0;
    for (int k = 0; k < 30000; ++k) {
        double x = (rng.nextDouble() - 0.5) * 400, y = (rng.nextDouble() - 0.5) * 400;
        worst = std::max(worst, std::abs(simplexNoise(lat, x, y) - simplexBrute(lat, x, y)));
        worstP = std::max(worstP, std::abs(simplexNoise(perm, x, y) - simplexBrute(perm, x, y)));
    }
    CHECK(worst < 1e-10);
    CHECK(worstP < 1e-10);
    // 影响半径放大到 0.6：三角形外的顶点也进入半径，只看 3 个角的写法就和完整求和不一致了
    int differ = 0;
    for (int k = 0; k < 5000; ++k) {
        double x = (rng.nextDouble() - 0.5) * 400, y = (rng.nextDouble() - 0.5) * 400;
        differ += std::abs(simplexNoiseD(lat, x, y, 0.6).value - simplexBrute(lat, x, y, 0.6)) > 1e-9;
    }
    CHECK(differ > 500);
}

static void testSimplexProperties() {
    HashLattice lat{6};
    // 三角形网格的顶点上为 0
    for (int j = -10; j <= 10; ++j)
        for (int i = -10; i <= 10; ++i) {
            double t = (i + j) * kUnskew;
            CHECK(std::abs(simplexNoise(lat, i - t, j - t)) < 1e-12);
        }
    // 解析导数与中心差分一致
    Pcg32 rng(2);
    const double h = 1e-6;
    for (int k = 0; k < 5000; ++k) {
        double x = (rng.nextDouble() - 0.5) * 100, y = (rng.nextDouble() - 0.5) * 100;
        Sample<double> s = simplexNoiseD(lat, x, y);
        CHECK(std::abs((simplexNoise(lat, x + h, y) - simplexNoise(lat, x - h, y)) / (2 * h) - s.dx) < 1e-4);
        CHECK(std::abs((simplexNoise(lat, x, y + h) - simplexNoise(lat, x, y - h)) / (2 * h) - s.dy) < 1e-4);
    }
    // 跨过三角形的边时值与梯度都连续：沿对角线 x0 = y0 两侧（斜切坐标里 u = v）
    double jumpV = 0, jumpG = 0;
    for (int k = 0; k < 2000; ++k) {
        double u = rng.below(200) + rng.nextDouble(), v = u - std::floor(u) + rng.below(200);
        double e = 1e-9, t1 = (u + e + v) * kUnskew, t2 = (u + v + e) * kUnskew;
        Sample<double> a = simplexNoiseD(lat, u + e - t1, v - t1), b = simplexNoiseD(lat, u - t2, v + e - t2);
        jumpV = std::max(jumpV, std::abs(a.value - b.value));
        jumpG = std::max(jumpG, std::abs(a.dx - b.dx) + std::abs(a.dy - b.dy));
    }
    CHECK(jumpV < 1e-6);
    CHECK(jumpG < 1e-5);
}

struct FixedCorners {
    int g[4];
    uint32_t hash(int, int) const { return 0; }
    int gradient(int x, int y) const { return g[(y & 1) * 2 + (x & 1)]; }
};

// 范围：斜切格子 4 个角的梯度共 8⁴ 种组合，逐一在格子内的网格点上求值，绝对值不超过 1，并且能接近 1
static void testSimplexRange() {
    double best = 0;
    for (int c = 0; c < 4096; ++c) {
        FixedCorners lat{{c & 7, (c >> 3) & 7, (c >> 6) & 7, (c >> 9) & 7}};
        for (int a = 0; a <= 32; ++a)
            for (int b = 0; b <= 32; ++b) {
                double u = a / 32.0 * 0.9999999, v = b / 32.0 * 0.9999999, t = (u + v) * kUnskew;
                best = std::max(best, std::abs(simplexNoise(lat, u - t, v - t)));
            }
    }
    CHECK(best <= 1 + 1e-9);
    CHECK(best > 0.999);
    HashLattice lat{7};
    Pcg32 rng(3);
    for (int k = 0; k < 200000; ++k) CHECK(std::abs(simplexNoise(lat, rng.nextDouble() * 1e4, rng.nextDouble() * 1e4)) <= 1 + 1e-9);
}

static void testWorley() {
    HashLattice lat{8};
    Pcg32 rng(4);
    int wrong3x3 = 0;
    for (int k = 0; k < 200000; ++k) {
        double x = (rng.nextDouble() - 0.5) * 2000, y = (rng.nextDouble() - 0.5) * 2000;
        double f1, f2;
        worleyBrute(lat, x, y, f1, f2);
        CellSample<double> e = worley(lat, x, y);
        CHECK(e.f1 == f1 && e.f2 == f2);  // 精确搜索与暴力完全相同
        CHECK(e.f1 <= e.f2);
        double px, py;
        featurePoint(lat, e.cellX, e.cellY, px, py);
        CHECK(std::sqrt((px - x) * (px - x) + (py - y) * (py - y)) == e.f1);  // 记录的格子确实是最近点所在的格子
        featurePoint(lat, e.cell2X, e.cell2Y, px, py);
        CHECK(std::sqrt((px - x) * (px - x) + (py - y) * (py - y)) == e.f2);
        CellSample<double> c = worley(lat, x, y, 1);
        CHECK(c.f1 >= f1 && c.f2 >= f2);  // 3×3 只会漏掉点，不会找到更近的
        wrong3x3 += c.f1 != f1 || c.f2 != f2;
    }
    CHECK(wrong3x3 > 0);  // 3×3 搜索确实会出错（比例见 fractal_bench）
    // 特征点在格子内均匀分布，x、y 两个偏移互不相关
    double su = 0, sv = 0, suu = 0, svv = 0, suv = 0;
    const int n = 200 * 200;
    for (int j = 0; j < 200; ++j)
        for (int i = 0; i < 200; ++i) {
            double px, py;
            featurePoint(lat, i, j, px, py);
            double u = px - i, v = py - j;
            su += u, sv += v, suu += u * u, svv += v * v, suv += u * v;
        }
    double mu = su / n, mv = sv / n;
    CHECK(std::abs(mu - 0.5) < 0.01 && std::abs(mv - 0.5) < 0.01);
    CHECK(std::abs(suu / n - mu * mu - 1.0 / 12) < 0.003 && std::abs(svv / n - mv * mv - 1.0 / 12) < 0.003);
    CHECK(std::abs((suv / n - mu * mv) / (1.0 / 12)) < 0.02);
    // 特征点上 F1 = 0；特征点都在自己的格子内
    for (int j = -5; j <= 5; ++j)
        for (int i = -5; i <= 5; ++i) {
            double px, py;
            featurePoint(lat, i, j, px, py);
            CHECK(px >= i && px < i + 1 && py >= j && py < j + 1);
            CellSample<double> s = worley(lat, px, py);
            CHECK(s.f1 == 0 && s.cellX == i && s.cellY == j);
        }
}

static void testFbm() {
    HashLattice lat{9};
    auto perlin = [&](double x, double y) { return perlinNoise(lat, x, y); };
    Pcg32 rng(5);
    FbmParams p;
    p.octaves = 5, p.lacunarity = 2.1, p.gain = 0.45;
    for (int k = 0; k < 2000; ++k) {
        double x = rng.nextDouble() * 100, y = rng.nextDouble() * 100;
        // 与逐层手算的结果一致
        double sum = 0, norm = 0, amp = 1, freq = 1;
        for (int o = 0; o < 5; ++o) {
            double ox, oy;
            octaveOffset(o, ox, oy);
            sum += amp * perlinNoise(lat, x * freq + ox, y * freq + oy), norm += amp, amp *= 0.45, freq *= 2.1;
        }
        CHECK(std::abs(fbm(perlin, x, y, p) - sum / norm) < 1e-12);
        CHECK(std::abs(fbm(perlin, x, y, p)) <= std::sqrt(0.5) + 1e-12);  // 归一化后不超过单层的范围
        double r = ridged(perlin, x, y, p);
        CHECK(r >= 0 && r <= 1);
        // ridged 与逐层手算的 (1 - |n|)² 一致
        double rs = 0, rn = 0, ra = 1, rf = 1;
        for (int o = 0; o < 5; ++o) {
            double ox, oy;
            octaveOffset(o, ox, oy);
            double q = 1 - std::abs(perlinNoise(lat, x * rf + ox, y * rf + oy));
            rs += ra * q * q, rn += ra, ra *= 0.45, rf *= 2.1;
        }
        CHECK(std::abs(r - rs / rn) < 1e-12);
    }
    // 不加偏移、频率倍率为 2：每一层的格点都包含整数点，Perlin 在格点上为 0，所以 fBm 在每个整数点上都是 0
    FbmParams aligned;
    aligned.offsets = false;
    FbmParams shifted;
    int nonzero = 0;
    for (int y = -20; y <= 20; ++y)
        for (int x = -20; x <= 20; ++x) {
            CHECK(fbm(perlin, double(x), double(y), aligned) == 0);
            nonzero += fbm(perlin, double(x), double(y), shifted) != 0;
        }
    CHECK(nonzero == 41 * 41);
    // 周期：置换表的噪声周期 256。频率倍率为 2 时，每层的周期 256/2^k 都整除 256，加了偏移也一样，fBm 仍以 256 为周期；
    // 倍率改成 1.98 就不再是
    PermLattice perm(9);
    auto pp = [&](double x, double y) { return perlinNoise(perm, x, y); };
    FbmParams odd = shifted;
    odd.lacunarity = 1.98;
    int periodicOdd = 0;
    for (int k = 0; k < 2000; ++k) {
        double x = rng.nextDouble() * 50, y = rng.nextDouble() * 50;
        CHECK(std::abs(fbm(pp, x + 256, y, shifted) - fbm(pp, x, y, shifted)) < 1e-9);
        CHECK(std::abs(fbm(pp, x, y + 256, aligned) - fbm(pp, x, y, aligned)) < 1e-9);
        periodicOdd += std::abs(fbm(pp, x + 256, y, odd) - fbm(pp, x, y, odd)) < 1e-9;
    }
    CHECK(periodicOdd < 20);
    // 一层时就是 base 本身
    FbmParams one;
    one.octaves = 1, one.offsets = false;
    for (int k = 0; k < 100; ++k) {
        double x = rng.nextDouble() * 100, y = rng.nextDouble() * 100;
        CHECK(fbm(perlin, x, y, one) == perlinNoise(lat, x, y));
    }
}

int main() {
    testSimplexBrute();
    testSimplexProperties();
    testSimplexRange();
    testWorley();
    testFbm();
    return report("fractal_test");
}
