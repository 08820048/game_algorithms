#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <utility>
#include <vector>

namespace ga {

// #region pcg32
// PCG32（pcg-random.org）：小、快、统计质量好，并且可以用种子精确复现。
// 游戏里需要"同一个种子生成同一张地图""回放录像"时，就不能用 rand() 或 std::random_device
class Pcg32 {
public:
    explicit Pcg32(uint64_t seed = 0x853c49e6748fea9bULL, uint64_t stream = 0xda3e39cb94b95bdbULL) {
        state_ = 0;
        inc_ = (stream << 1u) | 1u;
        next();
        state_ += seed;
        next();
    }

    uint32_t next() {
        uint64_t old = state_;
        state_ = old * 6364136223846793005ULL + inc_;
        auto xorshifted = static_cast<uint32_t>(((old >> 18u) ^ old) >> 27u);
        auto rot = static_cast<uint32_t>(old >> 59u);
        return (xorshifted >> rot) | (xorshifted << ((~rot + 1u) & 31u));
    }

    // [0, bound) 内的均匀整数。直接 next() % bound 会有偏差，这里拒绝掉"凑不满一轮"的那部分
    uint32_t below(uint32_t bound) {
        uint32_t threshold = (~bound + 1u) % bound;  // = 2^32 mod bound
        for (;;) {
            uint32_t r = next();
            if (r >= threshold) return r % bound;
        }
    }

    int range(int lo, int hi) { return lo + static_cast<int>(below(static_cast<uint32_t>(hi - lo + 1))); }  // 闭区间
    float nextFloat() { return static_cast<float>(next() >> 8) * (1.0f / 16777216.0f); }      // [0, 1)
    double nextDouble() { return static_cast<double>(next()) * (1.0 / 4294967296.0); }       // [0, 1)
    bool chance(double p) { return nextDouble() < p; }

private:
    uint64_t state_;
    uint64_t inc_;
};
// #endregion

// #region shuffle
// Fisher–Yates 洗牌：从后往前，每个位置只和"它自己以及它前面"的元素交换。O(n)，且每种排列概率相等
template <class T>
void shuffle(std::vector<T>& items, Pcg32& rng) {
    for (size_t i = items.size(); i > 1; --i) {
        size_t j = rng.below(static_cast<uint32_t>(i));  // j ∈ [0, i-1]
        std::swap(items[i - 1], items[j]);
    }
}
// #endregion

// #region naive-shuffle
// 错误示范：每个位置都和"任意位置"交换。看起来很随机，但 n^n 种执行路径无法均分到 n! 种排列上
template <class T>
void naiveShuffle(std::vector<T>& items, Pcg32& rng) {
    for (size_t i = 0; i < items.size(); ++i) {
        size_t j = rng.below(static_cast<uint32_t>(items.size()));
        std::swap(items[i], items[j]);
    }
}
// #endregion

// #region weighted
// 加权随机（掉落表）：预先算好前缀和，抽取时二分查找，O(log n)
class WeightedTable {
public:
    explicit WeightedTable(const std::vector<double>& weights) {
        double sum = 0;
        for (double w : weights) cumulative_.push_back(sum += w);
    }

    int pick(Pcg32& rng) const {
        double r = rng.nextDouble() * cumulative_.back();
        auto it = std::upper_bound(cumulative_.begin(), cumulative_.end(), r);
        return static_cast<int>(it - cumulative_.begin());
    }

    double probability(int i) const {
        double prev = i == 0 ? 0 : cumulative_[i - 1];
        return (cumulative_[i] - prev) / cumulative_.back();
    }

    int size() const { return static_cast<int>(cumulative_.size()); }

private:
    std::vector<double> cumulative_;
};
// #endregion

// #region shuffle-bag
// 洗牌袋：把一组结果装进袋子，打乱后依次取出，取完再重新装。保证每一轮里每种结果恰好出现一次
template <class T>
class ShuffleBag {
public:
    explicit ShuffleBag(std::vector<T> items) : items_(std::move(items)), next_(items_.size()) {}

    const T& next(Pcg32& rng) {
        if (next_ >= items_.size()) {
            shuffle(items_, rng);
            next_ = 0;
        }
        return items_[next_++];
    }

private:
    std::vector<T> items_;
    size_t next_;
};
// #endregion

// #region prd
// 伪随机分布（PRD）：第 N 次尝试的触发概率是 N·C，触发后 N 归零。
// 平均概率仍是 p，但"连续不触发"和"连续触发"都变少了
inline double prdAverageProbability(double c) {
    double expectedAttempts = 0;  // 两次触发之间的期望尝试次数
    double notYet = 1;            // 前 n-1 次都没触发的概率
    for (int n = 1; notYet > 0; ++n) {
        double pn = std::min(1.0, n * c);
        expectedAttempts += n * notYet * pn;
        notYet *= 1 - pn;
    }
    return 1 / expectedAttempts;
}

// 由名义概率 p 反求常数 C：平均概率随 C 单调递增，用二分查找
inline double prdConstant(double p) {
    double lo = 0, hi = p;
    for (int i = 0; i < 64; ++i) {
        double mid = (lo + hi) / 2;
        (prdAverageProbability(mid) < p ? lo : hi) = mid;
    }
    return (lo + hi) / 2;
}

class PrdRoll {
public:
    explicit PrdRoll(double p) : c_(prdConstant(p)) {}

    bool roll(Pcg32& rng) {
        ++attempts_;
        if (rng.chance(attempts_ * c_)) {
            attempts_ = 0;
            return true;
        }
        return false;
    }

    double constant() const { return c_; }

private:
    double c_;
    int attempts_ = 0;
};
// #endregion

}  // namespace ga
