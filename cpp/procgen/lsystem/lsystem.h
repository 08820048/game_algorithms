#pragma once

// L-System（Aristid Lindenmayer 1968）
//   - 一个字符串（公理）加一组改写规则。每一代把串里的每个字符同时换成它的规则右边（没有规则的字符原样保留）
//   - 再用"海龟"把最终的串画出来：F 前进并画线，+ - 转向，[ ] 保存 / 恢复海龟状态（分叉）
//   - 随机 L-System：一个字符可以有多条规则，按权重随机挑一条
// 串的长度随代数指数增长。这里有两种展开方式：
//   - 整串展开：一代一代地生成完整字符串，内存 = 最后一代的长度
//   - 流式展开：深度优先，直接把最后一代的字符一个个交给海龟，内存只和代数有关
// 随机选择用 (种子, 代, 在这一代里的位置) 的哈希，而不是按顺序取随机数，这样两种展开得到同一棵植物

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "../../foundations/random/random.h"

namespace ga::lsys {

// #region system
struct Production {
    double weight;
    std::string rhs;
};

struct LSystem {
    std::string axiom;
    std::array<std::vector<Production>, 128> rules{};  // 按字符索引；空 = 没有规则，原样保留

    LSystem& rule(char c, std::string rhs, double weight = 1) {
        rules[uint8_t(c)].push_back({weight, std::move(rhs)});
        return *this;
    }
    bool stochastic() const {
        for (const auto& r : rules)
            if (r.size() > 1) return true;
        return false;
    }
};

// splitmix64 的收尾混合：把相邻的整数打散成看起来无关的 64 位数
inline uint64_t mix64(uint64_t x) {
    x ^= x >> 30, x *= 0xbf58476d1ce4e5b9ULL;
    x ^= x >> 27, x *= 0x94d049bb133111ebULL;
    return x ^ (x >> 31);
}

// 第 level 代、位置 index 上的字符 c 改写成什么。nullptr = 没有规则，原样保留
inline const std::string* choose(const LSystem& s, char c, uint64_t seed, int level, uint64_t index) {
    const auto& rs = s.rules[uint8_t(c)];
    if (rs.empty()) return nullptr;
    if (rs.size() == 1) return &rs[0].rhs;
    double total = 0;
    for (const Production& p : rs) total += p.weight;
    uint64_t h = mix64(mix64(seed + uint64_t(level) * 0x9e3779b97f4a7c15ULL) ^ index);
    double u = double(h >> 11) * (1.0 / 9007199254740992.0) * total;
    for (const Production& p : rs) {
        if (u < p.weight) return &p.rhs;
        u -= p.weight;
    }
    return &rs.back().rhs;
}
// #endregion

// #region expand
// 整串展开：第 level 代的第 i 个字符换成它的右边，拼成第 level+1 代
inline std::string expand(const LSystem& s, int n, uint64_t seed = 0) {
    std::string cur = s.axiom, next;
    for (int level = 0; level < n; ++level) {
        next.clear();
        for (size_t i = 0; i < cur.size(); ++i) {
            const std::string* r = choose(s, cur[i], seed, level, i);
            if (r) next += *r;
            else next += cur[i];
        }
        cur.swap(next);
    }
    return cur;
}
// #endregion

// #region stream
struct StreamStats {
    int maxFrames = 0;        // 栈的最大深度（不超过 n + 1）
    uint64_t emitted = 0;     // 最后一代的长度
    uint64_t rewrites = 0;    // 查了多少次规则
};

// 流式展开：栈里的每一帧是"某一代的某条右边，读到了第几个字符"。读到的字符没到第 n 代就继续往下展开，
// 到了就交给 emit。深度优先恰好按从左到右的顺序访问每一代的字符，所以每一代一个计数器就是字符在那一代里的位置
template <class Emit>
void stream(const LSystem& s, int n, uint64_t seed, Emit&& emit, StreamStats* st = nullptr) {
    struct Frame {
        const std::string* str;
        size_t pos;
        int level;
    };
    std::vector<Frame> stack;
    std::vector<uint64_t> index(size_t(n) + 1, 0);
    StreamStats local;
    StreamStats& stats = st ? *st : local;
    stats = {};
    stack.push_back({&s.axiom, 0, 0});
    while (!stack.empty()) {
        Frame& f = stack.back();
        if (f.pos == f.str->size()) {
            stack.pop_back();
            continue;
        }
        char c = (*f.str)[f.pos++];
        int level = f.level;
        // 没有规则的字符原样进入下一代，不必为它压一帧
        while (level < n) {
            ++stats.rewrites;
            const std::string* r = choose(s, c, seed, level, index[size_t(level)]++);
            if (r) {
                stack.push_back({r, 0, level + 1});
                stats.maxFrames = std::max(stats.maxFrames, int(stack.size()));
                break;
            }
            ++level;
        }
        if (level == n) {
            ++index[size_t(n)];
            ++stats.emitted;
            emit(c);
        }
    }
}
// #endregion

// #region growth
// 每一代各字符的个数是一个向量 v，下一代 v' = v·M，M[a][b] = a 的右边里 b 的个数（随机规则取加权平均 = 期望）。
// 于是长度按 M 的最大特征值指数增长
inline std::array<double, 128> predictedCounts(const LSystem& s, int n) {
    std::array<double, 128> v{};
    for (char c : s.axiom) v[uint8_t(c)] += 1;
    for (int level = 0; level < n; ++level) {
        std::array<double, 128> nv{};
        for (int a = 0; a < 128; ++a) {
            if (v[a] == 0) continue;
            const auto& rs = s.rules[a];
            if (rs.empty()) {
                nv[a] += v[a];
                continue;
            }
            double total = 0;
            for (const Production& p : rs) total += p.weight;
            for (const Production& p : rs)
                for (char b : p.rhs) nv[uint8_t(b)] += v[a] * p.weight / total;
        }
        v = nv;
    }
    return v;
}
inline double predictedLength(const LSystem& s, int n) {
    double sum = 0;
    for (double x : predictedCounts(s, n)) sum += x;
    return sum;
}
// #endregion

// #region turtle
struct Segment {
    float x0, y0, x1, y1;
    float width;
    float dist;  // 从根部沿树枝走到这段起点的路程，用来按"生长"的顺序显示
    int depth;   // 括号深度
};
struct Leaf {
    float x, y, dist;
};

// Angle：航向存成一个角度，每走一步算一次 cos / sin。Rotate：航向存成单位向量，每转一次乘一个旋转矩阵
enum class Heading { Angle, Rotate };

struct TurtleOptions {
    double angle = 90;        // + - 转动的角度（度）
    double heading = 0;       // 初始朝向（度，0 = +x，90 = +y 向上）
    double step = 1;          // F 的长度
    double lengthScale = 0.8; // " 把步长乘上它
    double widthScale = 0.7;  // ! 把线宽乘上它
    double width = 1;
    double jitter = 0;        // 每次转向额外加 [-jitter, jitter] 度的随机量
    double tropism = 0;       // 每走一单位长度，航向朝 -y（重力方向）弯曲的程度
    uint64_t seed = 1;
    Heading mode = Heading::Angle;
};

// 字符的含义：F G 前进并画线；f 前进不画；+ 左转；- 右转；| 掉头；[ 压栈；] 出栈；! 线宽变细；" 步长变短。其余字符忽略
template <class Real = double>
class Turtle {
public:
    explicit Turtle(const TurtleOptions& o, std::vector<Segment>* segs = nullptr, std::vector<Leaf>* leaves = nullptr)
        : o_(o), segs_(segs), leaves_(leaves), rng_(o.seed) {
        const Real a = Real(o.heading * kDeg);
        st_.theta = a, st_.hx = std::cos(a), st_.hy = std::sin(a);
        st_.len = Real(o.step), st_.width = Real(o.width);
        turnC_ = std::cos(Real(o.angle * kDeg)), turnS_ = std::sin(Real(o.angle * kDeg));
    }

    void feed(char c) {
        switch (c) {
            case 'F':
            case 'G':
            case 'f': forward(c != 'f'); break;
            case '+': turn(+1); break;
            case '-': turn(-1); break;
            case '|': st_.theta += Real(M_PI), st_.hx = -st_.hx, st_.hy = -st_.hy; break;
            case '[':
                stack_.push_back(st_);
                maxStack_ = std::max(maxStack_, int(stack_.size()));
                ++st_.depth, lastDrew_ = false;
                break;
            case ']':
                if (stack_.empty()) break;
                if (lastDrew_ && leaves_) leaves_->push_back({float(st_.x), float(st_.y), float(st_.dist)});
                st_ = stack_.back(), stack_.pop_back(), lastDrew_ = false;
                break;
            case '!': st_.width *= Real(o_.widthScale); break;
            case '"': st_.len *= Real(o_.lengthScale); break;
            default: break;
        }
    }
    void feed(const std::string& s) {
        for (char c : s) feed(c);
    }
    // 串结束时最后一段树枝的末端也算一片叶子
    void finish() {
        if (lastDrew_ && leaves_) leaves_->push_back({float(st_.x), float(st_.y), float(st_.dist)});
        lastDrew_ = false;
    }

    Real x() const { return st_.x; }
    Real y() const { return st_.y; }
    uint64_t drawn() const { return drawn_; }
    int maxStack() const { return maxStack_; }
    float minX() const { return minX_; }
    float maxX() const { return maxX_; }
    float minY() const { return minY_; }
    float maxY() const { return maxY_; }

private:
    static constexpr double kDeg = M_PI / 180;
    struct State {
        Real x = 0, y = 0, theta = 0, hx = 1, hy = 0, len = 1, width = 1, dist = 0;
        int depth = 0;
    };

    void forward(bool draw) {
        Real dx, dy;
        if (o_.mode == Heading::Angle) dx = std::cos(st_.theta), dy = std::sin(st_.theta);
        else dx = st_.hx, dy = st_.hy;
        Real nx = st_.x + dx * st_.len, ny = st_.y + dy * st_.len;
        if (draw) {
            if (segs_) segs_->push_back({float(st_.x), float(st_.y), float(nx), float(ny), float(st_.width), float(st_.dist), st_.depth});
            ++drawn_;
            grow(float(st_.x), float(st_.y)), grow(float(nx), float(ny));
        }
        st_.x = nx, st_.y = ny, st_.dist += st_.len;
        lastDrew_ = draw;
        // 向性：朝重力方向弯，弯的程度正比于航向与重力的叉积（水平的树枝弯得最多）
        if (o_.tropism != 0) rotate(Real(o_.tropism) * st_.len * -dx);
    }
    void turn(int sign) {
        if (o_.jitter == 0) {
            if (o_.mode == Heading::Angle) st_.theta += Real(sign * o_.angle * kDeg);
            else {
                Real s = Real(sign) * turnS_;
                Real hx = st_.hx * turnC_ - st_.hy * s, hy = st_.hx * s + st_.hy * turnC_;
                st_.hx = hx, st_.hy = hy;
            }
            return;
        }
        double j = (rng_.nextDouble() * 2 - 1) * o_.jitter;
        rotate(Real((sign * o_.angle + j) * kDeg));
    }
    void rotate(Real a) {
        if (o_.mode == Heading::Angle) {
            st_.theta += a;
            return;
        }
        Real c = std::cos(a), s = std::sin(a);
        Real hx = st_.hx * c - st_.hy * s, hy = st_.hx * s + st_.hy * c;
        st_.hx = hx, st_.hy = hy;
    }
    void grow(float x, float y) {
        minX_ = std::min(minX_, x), maxX_ = std::max(maxX_, x);
        minY_ = std::min(minY_, y), maxY_ = std::max(maxY_, y);
    }

    TurtleOptions o_;
    std::vector<Segment>* segs_;
    std::vector<Leaf>* leaves_;
    Pcg32 rng_;
    State st_;
    std::vector<State> stack_;
    Real turnC_, turnS_;
    bool lastDrew_ = false;
    uint64_t drawn_ = 0;
    int maxStack_ = 0;
    float minX_ = 1e30f, maxX_ = -1e30f, minY_ = 1e30f, maxY_ = -1e30f;
};
// #endregion

// #region lattice
// 精确参照：转角是 360°/k（k = 4 或 6）时，海龟的每一步都落在整数格点上。
// k = 4 用 (x, y)；k = 6 用 a + b·ω（ω = 60° 方向的单位向量，ω² = ω − 1）
struct LatticeWalk {
    int k;
    long long a = 0, b = 0;
    int dir = 0;
    long long steps = 0;
    explicit LatticeWalk(int k_, int dir0 = 0) : k(k_), dir(dir0) {}
    void feed(char c) {
        if (c == 'F' || c == 'G' || c == 'f') {
            static constexpr int d4[4][2] = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}};
            static constexpr int d6[6][2] = {{1, 0}, {0, 1}, {-1, 1}, {-1, 0}, {0, -1}, {1, -1}};
            const int* d = k == 4 ? d4[dir] : d6[dir];
            a += d[0], b += d[1], ++steps;
        } else if (c == '+') dir = (dir + 1) % k;
        else if (c == '-') dir = (dir + k - 1) % k;
        else if (c == '|') dir = (dir + k / 2) % k;
    }
    double x() const { return k == 4 ? double(a) : double(a) + 0.5 * double(b); }
    double y() const { return k == 4 ? double(b) : double(b) * std::sqrt(3.0) / 2; }
};
// #endregion

}  // namespace ga::lsys
