#pragma once

// 命令模式、录像回放与撤销
//   - 把玩家输入变成"命令"：每帧一个 Input。模拟是确定性的：同样的初始状态 + 同样的输入序列 → 同样的结果。
//     录像只需要存初始种子和输入序列；回放就是重新模拟一遍
//   - 确定性很脆弱：这个模拟是混沌的（球与球碰撞），初始状态差 1e-6，几百帧后就完全不同。
//     所以回放必须逐位相同：固定步长、同样的运算顺序、随机数只来自模拟自己的生成器
//   - 拖动进度条：每隔 K 帧存一个关键帧（完整状态），跳转 = 读最近的关键帧 + 模拟剩下的帧
//   - 撤销：命令记住修改前的值（撤销 = 写回旧值）；或者每步存完整快照；或者只存命令，撤销 = 从检查点重放

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

#include "../../foundations/random/random.h"
#include "vec2.h"

namespace ga::command {

// #region sim
struct Input {
    int8_t dx = 0, dy = 0;  // 推杆的移动方向
    bool push = false;      // 按住时推杆变大
    friend bool operator==(Input, Input) = default;
};

struct Ball {
    Vec2 p, v;
};

// 确定性模拟：盒子里的球互相碰撞，玩家控制一个推杆。固定步长 1/60 秒
struct Sim {
    static constexpr float kW = 560, kH = 360, kR = 9, kDt = 1.0f / 60;
    std::vector<Ball> balls;
    Vec2 pusher{kW / 2, kH - 40};
    uint64_t rngState = 0;  // 模拟自己的随机数（这里用来让新加入的球随机出现）
    uint32_t frame = 0;

    static Sim make(uint64_t seed, int count) {
        Sim s;
        Pcg32 r{seed};
        for (int i = 0; i < count; ++i) {
            Vec2 p{kR + r.nextFloat() * (kW - 2 * kR), kR + r.nextFloat() * (kH * 0.6f)};
            s.balls.push_back({p, fromAngle(r.nextFloat() * 2 * kPi) * (60 + r.nextFloat() * 80)});
        }
        s.rngState = seed * 2654435761u + 1;
        return s;
    }
    float pusherRadius(bool push) const { return push ? 34.0f : 20.0f; }

    void step(Input in) {
        pusher.x = std::clamp(pusher.x + float(in.dx) * 220 * kDt, 0.0f, kW);
        pusher.y = std::clamp(pusher.y + float(in.dy) * 220 * kDt, 0.0f, kH);
        float pr = pusherRadius(in.push);
        for (Ball& b : balls) {
            b.v.y += 120 * kDt;  // 重力
            b.p += b.v * kDt;
            if (b.p.x < kR) b.p.x = kR, b.v.x = std::fabs(b.v.x);
            if (b.p.x > kW - kR) b.p.x = kW - kR, b.v.x = -std::fabs(b.v.x);
            if (b.p.y < kR) b.p.y = kR, b.v.y = std::fabs(b.v.y);
            if (b.p.y > kH - kR) b.p.y = kH - kR, b.v.y = -std::fabs(b.v.y) * 0.98f;
            Vec2 d = b.p - pusher;  // 推杆把球弹开
            float dist = length(d);
            if (dist < pr + kR && dist > 1e-4f) {
                Vec2 n = d / dist;
                b.p = pusher + n * (pr + kR);
                float vn = dot(b.v, n);
                if (vn < 0) b.v -= n * (2 * vn);
                b.v += n * 40;
            }
        }
        // 球与球：等质量弹性碰撞。按固定顺序 (i, j) 处理，保证确定性
        for (size_t i = 0; i < balls.size(); ++i)
            for (size_t j = i + 1; j < balls.size(); ++j) {
                Vec2 d = balls[j].p - balls[i].p;
                float d2 = lengthSq(d);
                if (d2 >= 4 * kR * kR || d2 < 1e-8f) continue;
                float dist = std::sqrt(d2);
                Vec2 n = d / dist;
                float overlap = 2 * kR - dist;
                balls[i].p -= n * (overlap / 2), balls[j].p += n * (overlap / 2);
                float rel = dot(balls[j].v - balls[i].v, n);
                if (rel < 0) balls[i].v += n * rel, balls[j].v -= n * rel;
            }
        // 每 4 秒随机挑一个球，给它一个随机的冲量（用模拟自己的随机数）
        if (frame % 240 == 239 && !balls.empty()) {
            Pcg32 r{rngState};
            Ball& b = balls[r.next() % balls.size()];
            b.v += fromAngle(r.nextFloat() * 2 * kPi) * 150;
            rngState = r.next() | (uint64_t(r.next()) << 32);
        }
        ++frame;
    }

    // 状态的哈希（FNV-1a，逐字节）：回放结束时比较，逐位相同才算正确
    uint64_t hash() const {
        uint64_t h = 1469598103934665603ull;
        auto mix = [&](const void* p, size_t n) {
            const auto* c = static_cast<const unsigned char*>(p);
            for (size_t i = 0; i < n; ++i) h = (h ^ c[i]) * 1099511628211ull;
        };
        for (const Ball& b : balls) mix(&b, sizeof b);
        mix(&pusher, sizeof pusher), mix(&rngState, sizeof rngState), mix(&frame, sizeof frame);
        return h;
    }
    size_t bytes() const { return balls.size() * sizeof(Ball) + sizeof pusher + sizeof rngState + sizeof frame; }
    float maxDistance(const Sim& o) const {
        float m = 0;
        for (size_t i = 0; i < balls.size(); ++i) m = std::max(m, length(balls[i].p - o.balls[i].p));
        return m;
    }
};
// #endregion

// #region record
// 录像：种子 + 每帧的输入。输入大多数时候不变，用游程编码（值，重复次数）压缩
struct Recording {
    uint64_t seed = 0;
    int balls = 0;
    std::vector<Input> inputs;

    struct Run {
        Input in;
        uint16_t count;
    };
    std::vector<Run> compress() const {
        std::vector<Run> runs;
        for (Input in : inputs) {
            if (!runs.empty() && runs.back().in == in && runs.back().count < UINT16_MAX) ++runs.back().count;
            else runs.push_back({in, 1});
        }
        return runs;
    }
    static std::vector<Input> decompress(const std::vector<Run>& runs) {
        std::vector<Input> out;
        for (const Run& r : runs) out.insert(out.end(), r.count, r.in);
        return out;
    }
    // 回放到第 n 帧（从头模拟）
    Sim replay(size_t n) const {
        Sim s = Sim::make(seed, balls);
        for (size_t i = 0; i < n && i < inputs.size(); ++i) s.step(inputs[i]);
        return s;
    }
};
// #endregion

// #region seek
// 关键帧：每 K 帧存一份完整状态。跳到第 n 帧 = 读第 ⌊n/K⌋ 个关键帧，再模拟 n mod K 帧
class Seeker {
public:
    Seeker(const Recording& rec, int every) : rec_(&rec), every_(every) {
        Sim s = Sim::make(rec.seed, rec.balls);
        for (size_t f = 0; f <= rec.inputs.size(); ++f) {
            if (f % size_t(every) == 0) keys_.push_back(s);
            if (f < rec.inputs.size()) s.step(rec.inputs[f]);
        }
    }
    // 返回第 n 帧的状态；steps 记下为此模拟了多少帧
    Sim seek(size_t n, long* steps = nullptr) const {
        n = std::min(n, rec_->inputs.size());
        size_t k = n / size_t(every_);
        Sim s = keys_[k];
        for (size_t f = k * size_t(every_); f < n; ++f) s.step(rec_->inputs[f]);
        if (steps) *steps += long(n - k * size_t(every_));
        return s;
    }
    size_t bytes() const {
        size_t b = 0;
        for (const Sim& s : keys_) b += s.bytes();
        return b;
    }
    size_t keyframes() const { return keys_.size(); }

private:
    const Recording* rec_;
    int every_;
    std::vector<Sim> keys_;
};
// #endregion

// #region grid
// 撤销的例子：一个瓦片编辑器。画一笔 = 一组 (格子, 新值)
struct Grid {
    int w, h;
    std::vector<uint8_t> cells;
    Grid(int w_, int h_) : w(w_), h(h_), cells(size_t(w_) * size_t(h_), 0) {}
    uint8_t& at(int x, int y) { return cells[size_t(y) * size_t(w) + size_t(x)]; }
    friend bool operator==(const Grid& a, const Grid& b) { return a.w == b.w && a.h == b.h && a.cells == b.cells; }
};
struct Edit {
    uint32_t cell;
    uint8_t value;
};
// #endregion

// #region undo
// 一条命令 = 一笔：记住每个格子修改前的值，撤销时写回去
struct Stroke {
    std::vector<uint32_t> cells;
    std::vector<uint8_t> before, after;
    size_t bytes() const { return cells.size() * (sizeof(uint32_t) + 2); }
};

class UndoStack {
public:
    // 执行一笔并入栈。merge = true：同一个格子在一笔里被画多次，只记第一次的旧值
    void apply(Grid& g, const std::vector<Edit>& edits, bool merge = true) {
        Stroke s;
        for (const Edit& e : edits) {
            uint8_t& c = g.cells[e.cell];
            if (merge) {
                auto it = std::find(s.cells.begin(), s.cells.end(), e.cell);
                if (it != s.cells.end()) {
                    c = e.value, s.after[size_t(it - s.cells.begin())] = e.value;
                    continue;
                }
            }
            s.cells.push_back(e.cell), s.before.push_back(c), s.after.push_back(e.value);
            c = e.value;
        }
        if (s.cells.empty()) return;
        done_.resize(top_);  // 新的修改让"重做"分支作废
        done_.push_back(std::move(s));
        ++top_;
    }
    bool undo(Grid& g) {
        if (top_ == 0) return false;
        const Stroke& s = done_[--top_];
        for (size_t i = s.cells.size(); i-- > 0;) g.cells[s.cells[i]] = s.before[i];  // 倒序：同一格画了多次时恢复最早的值
        return true;
    }
    bool redo(Grid& g) {
        if (top_ == done_.size()) return false;
        const Stroke& s = done_[top_++];
        for (size_t i = 0; i < s.cells.size(); ++i) g.cells[s.cells[i]] = s.after[i];
        return true;
    }
    size_t bytes() const {
        size_t b = 0;
        for (const Stroke& s : done_) b += s.bytes();
        return b;
    }
    size_t size() const { return top_; }
    size_t total() const { return done_.size(); }
    const std::vector<Stroke>& history() const { return done_; }

private:
    std::vector<Stroke> done_;
    size_t top_ = 0;  // done_[0, top_) 已执行，[top_, end) 可重做
};

// 对照一：每一步之后存完整快照
class SnapshotStack {
public:
    explicit SnapshotStack(const Grid& g) { snaps_.push_back(g.cells); }
    void apply(Grid& g, const std::vector<Edit>& edits) {
        for (const Edit& e : edits) g.cells[e.cell] = e.value;
        snaps_.resize(top_ + 1);
        snaps_.push_back(g.cells);
        ++top_;
    }
    bool undo(Grid& g) {
        if (top_ == 0) return false;
        g.cells = snaps_[--top_];
        return true;
    }
    size_t bytes() const { return snaps_.size() * (snaps_.empty() ? 0 : snaps_[0].size()); }

private:
    std::vector<std::vector<uint8_t>> snaps_;
    size_t top_ = 0;
};

// 对照二：只存命令，每 K 步存一个检查点。撤销 = 回到最近的检查点，重放到上一步
class ReplayStack {
public:
    ReplayStack(const Grid& g, int every) : every_(every) { checkpoints_.push_back(g.cells); }
    void apply(Grid& g, const std::vector<Edit>& edits) {
        for (const Edit& e : edits) g.cells[e.cell] = e.value;
        cmds_.resize(top_), checkpoints_.resize(top_ / size_t(every_) + 1);
        cmds_.push_back(edits);
        ++top_;
        if (top_ % size_t(every_) == 0) checkpoints_.push_back(g.cells);
    }
    bool undo(Grid& g, long* replayed = nullptr) {
        if (top_ == 0) return false;
        --top_;
        size_t k = top_ / size_t(every_);
        g.cells = checkpoints_[k];
        for (size_t i = k * size_t(every_); i < top_; ++i) {
            for (const Edit& e : cmds_[i]) g.cells[e.cell] = e.value;
            if (replayed) ++*replayed;
        }
        return true;
    }
    size_t bytes() const {
        size_t b = 0;
        for (auto& c : cmds_) b += c.size() * sizeof(Edit);
        for (auto& c : checkpoints_) b += c.size();
        return b;
    }

private:
    int every_;
    std::vector<std::vector<Edit>> cmds_;
    std::vector<std::vector<uint8_t>> checkpoints_;
    size_t top_ = 0;
};
// #endregion

}  // namespace ga::command
