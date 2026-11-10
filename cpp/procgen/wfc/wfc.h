#pragma once

// 波函数坍缩（Wave Function Collapse，Maxim Gumin 2016）
//   - 每个格子有一个"可能的图块"集合（位集）。一开始什么都可能
//   - 观察：挑一个还没确定的格子（缺省挑熵最小的），按权重随机定成一个图块
//   - 传播：一个格子的集合变小后，邻居里和它的任何候选都不相容的图块被删掉，再从邻居继续向外传（弧相容）
//   - 某个格子的集合变空就是矛盾：要么整张图重来，要么回溯（撤销最近的选择，换一个）
// 规则只有一张表：图块 a 的方向 d 上允许出现哪些图块 b。两种来源：
//   - 图块模型：每条边有一个"接口"标签，两边标签相同才能拼
//   - 重叠模型：从样图里取出所有 N×N 的图案，两个图案错开一格后重叠部分完全一样才能相邻

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "../../foundations/random/random.h"

namespace ga::wfc {

// 方向：0 右（+x），1 下（+y），2 左，3 上
inline constexpr int kDx[4] = {1, 0, -1, 0}, kDy[4] = {0, 1, 0, -1};
inline constexpr int opposite(int d) { return (d + 2) & 3; }

inline int popcount(uint64_t v) { return __builtin_popcountll(v); }
inline int lowestBit(uint64_t v) { return __builtin_ctzll(v); }

// #region rules
// compat[d][a] 是位集：图块 a 的 d 方向上可以出现的图块。allow() 同时写入反方向，保证对称
struct Rules {
    int T = 0, words = 0;
    std::vector<double> weight;
    std::vector<uint64_t> compat;

    void init(int t) {
        T = t, words = (t + 63) / 64;
        weight.assign(t, 1.0);
        compat.assign(size_t(4) * T * words, 0);
    }
    const uint64_t* allowed(int d, int a) const { return &compat[(size_t(d) * T + a) * words]; }
    bool allows(int a, int d, int b) const { return (allowed(d, a)[b >> 6] >> (b & 63)) & 1; }
    void allow(int a, int d, int b) {
        compat[(size_t(d) * T + a) * words + (b >> 6)] |= uint64_t(1) << (b & 63);
        compat[(size_t(opposite(d)) * T + b) * words + (a >> 6)] |= uint64_t(1) << (a & 63);
    }
};
// #endregion

// #region sockets
// 图块模型：edge[d] 是 d 方向那条边的接口标签。a 的 d 边和 b 的反方向边标签相同，b 就能放在 a 的 d 方向
struct SocketTile {
    std::array<int, 4> edge;
    double weight = 1;
};

inline Rules fromSockets(const std::vector<SocketTile>& tiles) {
    Rules r;
    r.init(int(tiles.size()));
    for (int a = 0; a < r.T; ++a) {
        r.weight[a] = tiles[a].weight;
        for (int d = 0; d < 4; ++d)
            for (int b = 0; b < r.T; ++b)
                if (tiles[a].edge[d] == tiles[b].edge[opposite(d)]) r.allow(a, d, b);
    }
    return r;
}
// #endregion

// #region overlap
// 重叠模型：样图是一张小的索引色图。取出所有 N×N 的窗口（可选旋转、镜像，共 8 种），相同的合并，出现次数就是权重。
// 图案 b 能放在 a 的 d 方向：把 b 沿 d 错开一格，两者重叠的 N×(N−1) 个像素完全相同
struct Sample {
    int w = 0, h = 0;
    std::vector<uint8_t> px;
    uint8_t at(int x, int y) const { return px[size_t(y) * w + x]; }
};

inline Rules fromSample(const Sample& s, int N, bool symmetry, bool periodicInput, std::vector<std::vector<uint8_t>>* patternsOut = nullptr) {
    std::map<std::vector<uint8_t>, int> index;
    std::vector<std::vector<uint8_t>> pats;
    std::vector<double> count;
    auto add = [&](const std::vector<uint8_t>& p) {
        auto [it, fresh] = index.try_emplace(p, int(pats.size()));
        if (fresh) pats.push_back(p), count.push_back(0);
        count[it->second] += 1;
    };
    int xs = periodicInput ? s.w : s.w - N + 1, ys = periodicInput ? s.h : s.h - N + 1;
    for (int y = 0; y < ys; ++y)
        for (int x = 0; x < xs; ++x) {
            std::vector<uint8_t> p(size_t(N) * N);
            for (int j = 0; j < N; ++j)
                for (int i = 0; i < N; ++i) p[size_t(j) * N + i] = s.at((x + i) % s.w, (y + j) % s.h);
            for (int v = 0; v < (symmetry ? 8 : 1); ++v) {
                std::vector<uint8_t> q(p.size());
                for (int j = 0; j < N; ++j)
                    for (int i = 0; i < N; ++i) {
                        int si = (v & 1) ? N - 1 - i : i, sj = j;  // 镜像
                        for (int k = 0; k < (v >> 1); ++k) std::swap(si, sj), si = N - 1 - si;  // 旋转 90° 若干次
                        q[size_t(j) * N + i] = p[size_t(sj) * N + si];
                    }
                add(q);
            }
        }
    Rules r;
    r.init(int(pats.size()));
    r.weight = count;
    for (int a = 0; a < r.T; ++a)
        for (int d = 0; d < 2; ++d)  // 只算右、下两个方向，allow() 自动补上反方向
            for (int b = 0; b < r.T; ++b) {
                bool ok = true;
                for (int j = 0; j < N && ok; ++j)
                    for (int i = 0; i < N && ok; ++i) {
                        int bi = i - kDx[d], bj = j - kDy[d];  // a 的 (i, j) 在 b 里的位置
                        if (bi < 0 || bj < 0 || bi >= N || bj >= N) continue;
                        ok = pats[a][size_t(j) * N + i] == pats[b][size_t(bj) * N + bi];
                    }
                if (ok) r.allow(a, d, b);
            }
    if (patternsOut) *patternsOut = pats;
    return r;
}
// #endregion

enum class Heuristic { Entropy, MinCount, Scanline, Random };
enum class Status { Running, Done, Contradiction };

struct Options {
    Heuristic heuristic = Heuristic::Entropy;
    bool periodic = false;          // 输出是否首尾相接
    bool backtrack = false;         // 矛盾时回溯，而不是直接失败
    long long maxBacktracks = 1 << 20;
};

struct SolveStats {
    long long observations = 0;  // 观察（随机选择）的次数
    long long revisions = 0;     // 传播时检查"邻居 × 方向"的次数
    long long removals = 0;      // 被删掉的候选总数
    long long scanned = 0;       // 挑格子时扫描过的格子数
    long long backtracks = 0;
};

class Solver {
public:
    Solver(const Rules& rules, int w, int h, Options opt = {}, uint64_t seed = 1) : r_(&rules), w_(w), h_(h), W_(rules.words), opt_(opt), rng_(seed) {
        wlogw_.resize(r_->T);
        for (int t = 0; t < r_->T; ++t) wlogw_[t] = r_->weight[t] > 0 ? r_->weight[t] * std::log(r_->weight[t]) : 0;
        reset();
    }

    int width() const { return w_; }
    int height() const { return h_; }
    Status status() const { return st_; }
    const SolveStats& stats() const { return stats_; }
    const uint64_t* domain(int c) const { return &dom_[size_t(c) * W_]; }
    int count(int c) const { return count_[c]; }
    int tileAt(int c) const { return count_[c] == 1 ? firstTile(c) : -1; }
    int lastObserved() const { return last_; }
    int contradictionCell() const { return bad_; }
    double entropy(int c) const { return std::log(sumW_[c]) - sumWL_[c] / sumW_[c]; }

    void reset() {
        int cells = w_ * h_;
        dom_.assign(size_t(cells) * W_, 0);
        for (int c = 0; c < cells; ++c)
            for (int t = 0; t < r_->T; ++t) dom_[size_t(c) * W_ + (t >> 6)] |= uint64_t(1) << (t & 63);
        double sw = 0, swl = 0;
        for (int t = 0; t < r_->T; ++t) sw += r_->weight[t], swl += wlogw_[t];
        count_.assign(cells, r_->T), sumW_.assign(cells, sw), sumWL_.assign(cells, swl);
        noise_.resize(cells);
        for (double& v : noise_) v = rng_.nextDouble() * 1e-6;  // 熵相同时随机打破平局
        undecided_ = r_->T > 1 ? cells : 0;
        inQueue_.assign(cells, 0), queue_.clear(), head_ = 0;
        trail_.clear(), trailWords_.clear(), decisions_.clear();
        scan_ = 0, last_ = -1, bad_ = -1, stats_ = {};
        st_ = Status::Running;
        // 有的图块在某个方向上根本没有能相邻的图块：先整体传播一遍把它们从内部格子里删掉
        for (int c = 0; c < cells; ++c) push(c);
        if (!propagate()) st_ = Status::Contradiction;
        else if (undecided_ == 0) st_ = Status::Done;
        trail_.clear(), trailWords_.clear();
    }

    // 外部约束：把格子 c 的集合限制为 mask（与原集合求交）并传播。用于测试和预设边界
    bool restrict(int c, const std::vector<uint64_t>& mask) {
        std::vector<uint64_t> nw(W_);
        for (int k = 0; k < W_; ++k) nw[k] = dom_[size_t(c) * W_ + k] & mask[k];
        if (std::all_of(nw.begin(), nw.end(), [](uint64_t v) { return v == 0; })) {
            bad_ = c, st_ = Status::Contradiction;
            return false;
        }
        setDomain(c, nw.data());
        push(c);
        if (!propagate()) {
            st_ = Status::Contradiction;
            return false;
        }
        if (undecided_ == 0) st_ = Status::Done;
        return true;
    }

    // #region step
    // 一步 = 一次观察 + 传播；开启回溯时，矛盾会撤销最近的选择并禁止那个图块
    Status step() {
        if (st_ != Status::Running) return st_;
        int c = choose();
        if (c < 0) return st_ = Status::Done;
        int t = pickTile(c);
        ++stats_.observations;
        last_ = c;
        if (opt_.backtrack) decisions_.push_back({trail_.size(), c, t});
        std::vector<uint64_t> one(W_, 0);
        one[t >> 6] = uint64_t(1) << (t & 63);
        setDomain(c, one.data());
        push(c);
        bool ok = propagate();
        while (!ok && opt_.backtrack && !decisions_.empty() && stats_.backtracks < opt_.maxBacktracks) {
            Decision d = decisions_.back();
            decisions_.pop_back();
            undo(d.trailSize);
            ++stats_.backtracks;
            // 这个选择导致矛盾：从格子里删掉它。这次删除记在上一个选择名下，再往回退时会一起撤销
            std::vector<uint64_t> nw(domain(d.cell), domain(d.cell) + W_);
            nw[d.tile >> 6] &= ~(uint64_t(1) << (d.tile & 63));
            if (std::all_of(nw.begin(), nw.end(), [](uint64_t v) { return v == 0; })) {
                bad_ = d.cell;
                continue;  // 这个格子已经没有别的选择，继续往回退
            }
            setDomain(d.cell, nw.data());
            push(d.cell);
            ok = propagate();
        }
        if (!ok) return st_ = Status::Contradiction;
        if (undecided_ == 0) st_ = Status::Done;
        return st_;
    }
    // #endregion

    Status run() {
        while (step() == Status::Running) {
        }
        return st_;
    }

    std::vector<int> result() const {
        std::vector<int> out(size_t(w_) * h_);
        for (int c = 0; c < w_ * h_; ++c) out[c] = tileAt(c);
        return out;
    }

private:
    struct Change {
        int cell, count;
        double sumW, sumWL;
    };
    struct Decision {
        size_t trailSize;
        int cell, tile;
    };

    int firstTile(int c) const {
        for (int k = 0; k < W_; ++k)
            if (dom_[size_t(c) * W_ + k]) return k * 64 + lowestBit(dom_[size_t(c) * W_ + k]);
        return -1;
    }
    int neighbor(int c, int d) const {
        int x = c % w_ + kDx[d], y = c / w_ + kDy[d];
        if (opt_.periodic) x = (x + w_) % w_, y = (y + h_) % h_;
        else if (x < 0 || y < 0 || x >= w_ || y >= h_) return -1;
        return y * w_ + x;
    }
    void push(int c) {
        if (!inQueue_[c]) inQueue_[c] = 1, queue_.push_back(c);
    }

    // 改写一个格子的集合，同时更新候选数和熵用的两个累加量；开启回溯时把旧值记进 trail
    void setDomain(int c, const uint64_t* nw) {
        uint64_t* d = &dom_[size_t(c) * W_];
        if (opt_.backtrack) {
            trail_.push_back({c, count_[c], sumW_[c], sumWL_[c]});
            trailWords_.insert(trailWords_.end(), d, d + W_);
        }
        int before = count_[c];
        for (int k = 0; k < W_; ++k) {
            uint64_t removed = d[k] & ~nw[k];
            while (removed) {
                int t = k * 64 + lowestBit(removed);
                removed &= removed - 1;
                --count_[c], sumW_[c] -= r_->weight[t], sumWL_[c] -= wlogw_[t];
                ++stats_.removals;
            }
            d[k] = nw[k];
        }
        undecided_ += (count_[c] > 1) - (before > 1);
    }

    void undo(size_t to) {
        while (trail_.size() > to) {
            Change ch = trail_.back();
            trail_.pop_back();
            int before = count_[ch.cell];
            std::copy(trailWords_.end() - W_, trailWords_.end(), &dom_[size_t(ch.cell) * W_]);
            trailWords_.resize(trailWords_.size() - W_);
            count_[ch.cell] = ch.count, sumW_[ch.cell] = ch.sumW, sumWL_[ch.cell] = ch.sumWL;
            undecided_ += (ch.count > 1) - (before > 1);
        }
        scan_ = 0;
    }

    // #region propagate
    // 队列里的格子 c 变小了：对每个方向 d，邻居只能保留"至少和 c 的一个候选相容"的图块
    bool propagate() {
        std::vector<uint64_t> mask(W_), nw(W_);
        while (head_ < queue_.size()) {
            int c = queue_[head_++];
            inQueue_[c] = 0;
            for (int d = 0; d < 4; ++d) {
                int n = neighbor(c, d);
                if (n < 0) continue;
                ++stats_.revisions;
                std::fill(mask.begin(), mask.end(), 0);
                for (int k = 0; k < W_; ++k)
                    for (uint64_t b = dom_[size_t(c) * W_ + k]; b; b &= b - 1) {
                        const uint64_t* a = r_->allowed(d, k * 64 + lowestBit(b));
                        for (int j = 0; j < W_; ++j) mask[j] |= a[j];
                    }
                bool changed = false, empty = true;
                for (int k = 0; k < W_; ++k) {
                    nw[k] = dom_[size_t(n) * W_ + k] & mask[k];
                    changed |= nw[k] != dom_[size_t(n) * W_ + k];
                    empty &= nw[k] == 0;
                }
                if (!changed) continue;
                if (empty) {
                    bad_ = n;
                    for (size_t q = head_; q < queue_.size(); ++q) inQueue_[queue_[q]] = 0;
                    queue_.clear(), head_ = 0;
                    return false;
                }
                setDomain(n, nw.data());
                push(n);
            }
        }
        queue_.clear(), head_ = 0;
        return true;
    }
    // #endregion

    // #region choose
    // 挑下一个要观察的格子。熵 H = log Σw − Σ(w log w)/Σw，两个累加量在删候选时增量维护
    int choose() {
        int cells = w_ * h_;
        if (opt_.heuristic == Heuristic::Scanline) {
            while (scan_ < cells && count_[scan_] <= 1) ++scan_, ++stats_.scanned;
            return scan_ < cells ? scan_ : -1;
        }
        stats_.scanned += cells;
        if (opt_.heuristic == Heuristic::Random) {
            if (undecided_ == 0) return -1;
            int k = int(rng_.below(uint32_t(undecided_)));
            for (int c = 0; c < cells; ++c)
                if (count_[c] > 1 && k-- == 0) return c;
            return -1;
        }
        int best = -1;
        double bv = INFINITY;
        for (int c = 0; c < cells; ++c) {
            if (count_[c] <= 1) continue;
            double v = (opt_.heuristic == Heuristic::Entropy ? entropy(c) : double(count_[c])) + noise_[c];
            if (v < bv) bv = v, best = c;
        }
        return best;
    }
    // #endregion

    int pickTile(int c) {
        double x = rng_.nextDouble() * sumW_[c];
        int lastT = -1;
        for (int k = 0; k < W_; ++k)
            for (uint64_t b = dom_[size_t(c) * W_ + k]; b; b &= b - 1) {
                int t = k * 64 + lowestBit(b);
                lastT = t;
                x -= r_->weight[t];
                if (x < 0) return t;
            }
        return lastT;  // 浮点误差兜底
    }

    const Rules* r_;
    int w_, h_, W_;
    Options opt_;
    Pcg32 rng_;
    std::vector<double> wlogw_;
    std::vector<uint64_t> dom_;
    std::vector<int> count_;
    std::vector<double> sumW_, sumWL_, noise_;
    std::vector<char> inQueue_;
    std::vector<int> queue_;
    size_t head_ = 0;
    std::vector<Change> trail_;
    std::vector<uint64_t> trailWords_;
    std::vector<Decision> decisions_;
    int undecided_ = 0, scan_ = 0, last_ = -1, bad_ = -1;
    Status st_ = Status::Running;
    SolveStats stats_;
};

// 结果是否合法：每对相邻格子都相容
inline bool validTiling(const Rules& r, int w, int h, bool periodic, const std::vector<int>& t) {
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) {
            int a = t[size_t(y) * w + x];
            if (a < 0) return false;
            for (int d = 0; d < 2; ++d) {
                int nx = x + kDx[d], ny = y + kDy[d];
                if (periodic) nx %= w, ny %= h;
                else if (nx >= w || ny >= h) continue;
                if (!r.allows(a, d, t[size_t(ny) * w + nx])) return false;
            }
        }
    return true;
}

}  // namespace ga::wfc
