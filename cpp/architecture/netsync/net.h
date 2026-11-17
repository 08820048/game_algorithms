#pragma once

// 网络同步：帧同步与状态同步
//   - 模拟的网络：每个包的到达时间 = 发送时间 + 单程延迟 + 抖动（均匀分布），按概率丢包，可能乱序。
//     全部由固定种子的随机数决定，同样的配置每次结果相同
//   - 帧同步（lockstep）：每台机器都运行完整的模拟，只交换输入。第 f 帧要等所有玩家第 f 帧的输入都到齐才能算。
//     本地输入安排在 D 帧之后执行（输入延迟），用来掩盖网络延迟；还没被对方确认的输入每个包都重发
//   - 状态同步：服务器运行唯一的权威模拟，客户端只发输入、收快照。快照可以完整发送、量化，
//     或者只发相对"对方已确认收到的那个快照"变化了的部分（差量）。客户端渲染时故意落后一段时间，在两个快照之间插值

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <map>
#include <vector>

#include "../../foundations/random/random.h"
#include "../fixed/fixed.h"

namespace ga::net {

using fixed::Fx;

// #region link
struct LinkConfig {
    double latencyMs = 50;  // 单程延迟
    double jitterMs = 0;    // 额外延迟在 [0, jitter] 内均匀分布，所以包会乱序
    double loss = 0;        // 丢包率
};
constexpr size_t kUdpIpHeader = 28;  // IPv4 20 字节 + UDP 8 字节

template <class Msg>
struct Link {
    LinkConfig cfg;
    Pcg32 rng{1};
    struct InFlight {
        double at;
        uint64_t seq;
        Msg msg;
    };
    std::vector<InFlight> flying;
    uint64_t seq = 0, packets = 0, bytes = 0, lost = 0;

    void send(double nowMs, Msg m, size_t payload) {
        ++packets, bytes += payload + kUdpIpHeader;
        if (rng.nextFloat() < cfg.loss) {
            ++lost;
            return;
        }
        double at = nowMs + cfg.latencyMs + cfg.jitterMs * rng.nextFloat();
        flying.push_back({at, seq++, std::move(m)});
    }
    // 交付所有到达时间 ≤ now 的包，按到达时间（相同时按发送顺序）
    template <class F>
    void deliver(double nowMs, F&& f) {
        auto mid = std::stable_partition(flying.begin(), flying.end(), [&](const InFlight& p) { return p.at > nowMs; });
        std::vector<InFlight> arrived(std::make_move_iterator(mid), std::make_move_iterator(flying.end()));
        flying.erase(mid, flying.end());
        std::sort(arrived.begin(), arrived.end(), [](const InFlight& a, const InFlight& b) { return a.at != b.at ? a.at < b.at : a.seq < b.seq; });
        for (auto& p : arrived) f(p.msg);
    }
};
// #endregion

// #region arena
struct Input {
    int8_t dx = 0, dy = 0;
    friend bool operator==(Input, Input) = default;
};

// 确定性的竞技场（定点数，见 6.4）：玩家四处移动，推开附近的单位；单位有摩擦，没人碰时会停下来
struct Arena {
    static constexpr int kW = 360, kH = 300;
    struct Player {
        Fx x, y;
    };
    struct Unit {
        Fx x, y, vx, vy;
    };
    std::vector<Player> players;
    std::vector<Unit> units;
    uint32_t frame = 0;

    static Arena make(uint64_t seed, int playerCount, int unitCount) {
        Arena a;
        Pcg32 r{seed};
        for (int i = 0; i < playerCount; ++i)
            a.players.push_back({Fx::fromInt(40 + int(r.next() % (kW - 80))), Fx::fromInt(40 + int(r.next() % (kH - 80)))});
        for (int i = 0; i < unitCount; ++i)
            a.units.push_back({Fx::fromRaw(int32_t(r.next() % (uint32_t(kW) << 16))), Fx::fromRaw(int32_t(r.next() % (uint32_t(kH) << 16))), {}, {}});
        return a;
    }

    void step(const Input* in) {
        const Fx speed = Fx::of(1.5), invR = Fx::of(1.0 / 36), k = Fx::of(0.6), one = Fx::fromInt(1);
        const Fx w = Fx::fromInt(kW), h = Fx::fromInt(kH), zero{};
        for (size_t i = 0; i < players.size(); ++i) {
            Player& p = players[i];
            p.x = std::clamp(p.x + speed * Fx::fromInt(in[i].dx), zero, w);
            p.y = std::clamp(p.y + speed * Fx::fromInt(in[i].dy), zero, h);
        }
        for (Unit& u : units) {
            for (const Player& p : players) {
                Fx ux = (u.x - p.x) * invR, uy = (u.y - p.y) * invR;
                if (ux >= one || ux <= -one || uy >= one || uy <= -one) continue;
                Fx q = one - (ux * ux + uy * uy);
                if (q <= zero) continue;
                u.vx += k * q * ux, u.vy += k * q * uy;
            }
            // 摩擦：raw × 15/16 向 0 截断，速度最终精确地变成 0
            u.vx = Fx::fromRaw(int32_t(int64_t(u.vx.raw) * 15 / 16));
            u.vy = Fx::fromRaw(int32_t(int64_t(u.vy.raw) * 15 / 16));
            u.x += u.vx, u.y += u.vy;
            if (u.x < zero) u.x = -u.x, u.vx = -u.vx;
            if (u.x > w) u.x = w + w - u.x, u.vx = -u.vx;
            if (u.y < zero) u.y = -u.y, u.vy = -u.vy;
            if (u.y > h) u.y = h + h - u.y, u.vy = -u.vy;
        }
        ++frame;
    }

    uint64_t hash() const {
        uint64_t x = 1469598103934665603ull;
        auto mix = [&](uint32_t v) { x = (x ^ v) * 1099511628211ull; };
        mix(frame);
        for (const Player& p : players) mix(uint32_t(p.x.raw)), mix(uint32_t(p.y.raw));
        for (const Unit& u : units) mix(uint32_t(u.x.raw)), mix(uint32_t(u.y.raw)), mix(uint32_t(u.vx.raw)), mix(uint32_t(u.vy.raw));
        return x;
    }
    size_t stateBytes() const { return 4 + players.size() * 8 + units.size() * 16; }
};

// 机器人玩家：每 40 帧换一个方向（由种子、玩家、时间段决定，所有机器算出的一样）
inline Input botInput(uint64_t seed, int player, uint32_t frame) {
    uint64_t h = (seed * 0x9e3779b97f4a7c15ull) ^ (uint64_t(player) << 32) ^ (frame / 40);
    h ^= h >> 31, h *= 0xbf58476d1ce4e5b9ull, h ^= h >> 29;
    return {int8_t(int(h % 3) - 1), int8_t(int((h >> 8) % 3) - 1)};
}
// 测量手感用：玩家 0 在 pressAt 帧之前不动，之后一直向右；pressAt < 0 时用机器人
inline Input inputFor(uint64_t seed, int player, uint32_t frame, int64_t pressAt) {
    if (pressAt < 0 || player != 0) return botInput(seed, player, frame);
    return int64_t(frame) >= pressAt ? Input{1, 0} : Input{};
}
// #endregion

// #region lockstep
struct LockstepConfig {
    int peers = 4, units = 200;
    int delay = 3;       // 输入延迟（帧）
    bool resend = true;  // 每个包带上所有还没被对方确认的输入；否则只带这一帧新产生的输入
    int hashEvery = 60;  // 每隔多少帧交换一次状态哈希
    int sendEvery = 1;   // 每隔几个 tick 发一次包（攒几帧输入一起发：包少了，但输入晚到）
    int64_t pressAt = -1;
    LinkConfig link;
    uint64_t seed = 1;
};

struct InputPacket {
    int from = 0;
    uint32_t first = 0;         // inputs[0] 是第几帧的输入
    std::vector<Input> inputs;  // 发送者自己的连续若干帧输入
    int64_t ack = -1;           // 发送者已经连续收到接收者的输入到第几帧
    int64_t hashFrame = -1;     // 发送者最近一次的状态哈希
    uint64_t hash = 0;
    size_t bytes() const { return 1 + 4 + 2 * inputs.size() + 4 + (hashFrame >= 0 ? 12 : 0); }
};

struct LockstepPeer {
    int id = 0;
    Arena sim;
    std::vector<std::vector<Input>> inputs;  // inputs[帧][玩家]
    std::vector<std::vector<bool>> have;
    std::vector<int64_t> contiguous;  // 每个玩家：已连续收到第 0 … contiguous 帧的输入
    std::vector<int64_t> ackedBy;     // 每个对方：已确认收到我第 0 … ackedBy 帧的输入
    std::map<uint32_t, uint64_t> myHashes, theirHashes;
    int64_t desyncAt = -1;            // 发现哈希不同的检查点
    long stallTicks = 0, lagSum = 0, maxLag = 0;
};

class LockstepSession {
public:
    explicit LockstepSession(const LockstepConfig& c) : cfg_(c) {
        int n = c.peers;
        for (int i = 0; i < n; ++i) {
            LockstepPeer p;
            p.id = i;
            p.sim = Arena::make(c.seed, n, c.units);
            p.contiguous.assign(size_t(n), c.delay - 1);  // 前 D 帧没有输入（全是 0），大家都知道
            p.ackedBy.assign(size_t(n), c.delay - 1);
            peers_.push_back(std::move(p));
        }
        links_.resize(size_t(n * n));
        for (int i = 0; i < n * n; ++i) links_[size_t(i)].cfg = c.link, links_[size_t(i)].rng = Pcg32{c.seed * 1000 + uint64_t(i)};
    }

    // 一个 tick = 1/60 秒的真实时间。每台机器：收包 → 尽量往前算（落后时一个 tick 最多算 3 帧追赶）→ 给每个对方发包
    void tick() {
        double now = double(tick_) * 1000.0 / 60;
        int n = cfg_.peers;
        for (int i = 0; i < n; ++i)
            for (int j = 0; j < n; ++j)
                if (i != j) link(j, i).deliver(now, [&](const InputPacket& pk) { receive(peers_[size_t(i)], pk); });
        for (auto& p : peers_) advance(p);
        if (tick_ % cfg_.sendEvery == 0)
            for (int i = 0; i < n; ++i)
                for (int j = 0; j < n; ++j)
                    if (i != j) sendTo(peers_[size_t(i)], j, now);
        ++tick_;
    }

    // 某台机器上某个单位的位置差一个最小单位（模拟一个不确定性 bug）
    void corrupt(int peer) {
        auto& u = peers_[size_t(peer)].sim.units;
        if (!u.empty()) u[0].x.raw += 1;
    }

    const LockstepConfig& config() const { return cfg_; }
    const std::vector<LockstepPeer>& peers() const { return peers_; }
    long ticks() const { return tick_; }
    uint64_t bytesSentBy(int peer) const {
        uint64_t b = 0;
        for (int j = 0; j < cfg_.peers; ++j)
            if (j != peer) b += links_[size_t(peer * cfg_.peers + j)].bytes;
        return b;
    }
    uint64_t packetsSentBy(int peer) const {
        uint64_t b = 0;
        for (int j = 0; j < cfg_.peers; ++j)
            if (j != peer) b += links_[size_t(peer * cfg_.peers + j)].packets;
        return b;
    }
    // 一台机器第 f 帧能不能算：所有玩家第 f 帧的输入都到了
    static bool ready(const LockstepPeer& p, uint32_t f) {
        for (int64_t c : p.contiguous)
            if (c < int64_t(f)) return false;
        return true;
    }

private:
    Link<InputPacket>& link(int from, int to) { return links_[size_t(from * cfg_.peers + to)]; }

    void store(LockstepPeer& p, int player, uint32_t f, Input in) {
        while (p.inputs.size() <= f) p.inputs.emplace_back(size_t(cfg_.peers)), p.have.emplace_back(size_t(cfg_.peers), false);
        p.inputs[f][size_t(player)] = in, p.have[f][size_t(player)] = true;
        int64_t& c = p.contiguous[size_t(player)];
        while (size_t(c + 1) < p.have.size() && p.have[size_t(c + 1)][size_t(player)]) ++c;
    }

    void receive(LockstepPeer& p, const InputPacket& pk) {
        for (size_t k = 0; k < pk.inputs.size(); ++k) store(p, pk.from, pk.first + uint32_t(k), pk.inputs[k]);
        p.ackedBy[size_t(pk.from)] = std::max(p.ackedBy[size_t(pk.from)], pk.ack);
        if (pk.hashFrame >= 0) {
            p.theirHashes[uint32_t(pk.hashFrame)] = pk.hash;
            check(p, uint32_t(pk.hashFrame));
        }
    }
    void check(LockstepPeer& p, uint32_t f) {
        auto a = p.myHashes.find(f), b = p.theirHashes.find(f);
        if (a != p.myHashes.end() && b != p.theirHashes.end() && a->second != b->second && p.desyncAt < 0) p.desyncAt = f;
    }

    void advance(LockstepPeer& p) {
        int steps = 0;
        while (p.sim.frame <= uint32_t(tick_) && steps < 3 && ready(p, p.sim.frame)) {
            uint32_t f = p.sim.frame;
            // 产生本地输入，安排在 D 帧之后执行
            // 输入在第 f 帧"按下"，在第 f + D 帧生效
            Input local = cfg_.pressAt >= 0 ? inputFor(cfg_.seed, p.id, f, cfg_.pressAt) : botInput(cfg_.seed, p.id, f + uint32_t(cfg_.delay));
            store(p, p.id, f + uint32_t(cfg_.delay), local);
            std::vector<Input> in(size_t(cfg_.peers));
            for (int k = 0; k < cfg_.peers; ++k) in[size_t(k)] = f < uint32_t(cfg_.delay) ? Input{} : p.inputs[f][size_t(k)];
            p.sim.step(in.data());
            ++steps;
            if (cfg_.hashEvery > 0 && p.sim.frame % uint32_t(cfg_.hashEvery) == 0) {
                p.myHashes[p.sim.frame] = p.sim.hash();
                check(p, p.sim.frame);
            }
        }
        long lag = long(tick_) + 1 - long(p.sim.frame);  // 落后于真实时间多少帧
        if (steps == 0 && lag > 0) ++p.stallTicks;
        p.lagSum += lag, p.maxLag = std::max(p.maxLag, lag);
    }

    void sendTo(LockstepPeer& p, int j, double now) {
        InputPacket pk;
        pk.from = p.id;
        int64_t last = p.contiguous[size_t(p.id)];
        // 重发：从对方确认的下一帧开始；不重发：每个输入只发一次（丢了就没了）
        int64_t first = cfg_.resend ? p.ackedBy[size_t(j)] + 1 : lastSent_[size_t(p.id * cfg_.peers + j)] + 1;
        first = std::max<int64_t>(first, cfg_.delay);
        pk.first = uint32_t(first);
        for (int64_t f = first; f <= last; ++f) pk.inputs.push_back(p.inputs[size_t(f)][size_t(p.id)]);
        if (!cfg_.resend) lastSent_[size_t(p.id * cfg_.peers + j)] = last;
        pk.ack = p.contiguous[size_t(j)];
        if (!p.myHashes.empty()) pk.hashFrame = p.myHashes.rbegin()->first, pk.hash = p.myHashes.rbegin()->second;
        size_t b = pk.bytes();
        link(p.id, j).send(now, std::move(pk), b);
    }

    LockstepConfig cfg_;
    std::vector<LockstepPeer> peers_;
    std::vector<Link<InputPacket>> links_;
    std::vector<int64_t> lastSent_ = std::vector<int64_t>(64, -1);
    long tick_ = 0;
};
// #endregion

// #region snapshot
// 量化：位置精确到 1/64 像素，存成 16 位（360 × 64 = 23040 < 65536）。渲染只需要位置，不发速度
struct QPos {
    uint16_t x = 0, y = 0;
    friend bool operator==(QPos, QPos) = default;
};
inline QPos quantize(Fx x, Fx y) { return {uint16_t((x.raw + 512) >> 10), uint16_t((y.raw + 512) >> 10)}; }
struct QState {
    uint32_t frame = 0;
    std::vector<QPos> players, units;
    friend bool operator==(const QState&, const QState&) = default;
};
inline QState quantize(const Arena& a) {
    QState s;
    s.frame = a.frame;
    for (auto& p : a.players) s.players.push_back(quantize(p.x, p.y));
    for (auto& u : a.units) s.units.push_back(quantize(u.x, u.y));
    return s;
}

enum class Encoding { Full, Quantized, Delta };

struct Writer {
    std::vector<uint8_t> b;
    void u8(uint8_t v) { b.push_back(v); }
    void u16(uint16_t v) { u8(uint8_t(v)), u8(uint8_t(v >> 8)); }
    void u32(uint32_t v) { u16(uint16_t(v)), u16(uint16_t(v >> 16)); }
    void pos(QPos p) { u16(p.x), u16(p.y); }
};
struct Reader {
    const std::vector<uint8_t>& b;
    size_t i = 0;
    uint8_t u8() { return b[i++]; }
    uint16_t u16() { uint16_t v = u8(); return uint16_t(v | (u8() << 8)); }
    uint32_t u32() { uint32_t v = u16(); return v | (uint32_t(u16()) << 16); }
    QPos pos() { QPos p; p.x = u16(), p.y = u16(); return p; }
};

// 完整：每个单位的位置和速度都用 float（16 字节）。只用来计算大小，客户端按量化后的位置使用
inline std::vector<uint8_t> encodeFull(const Arena& a) {
    Writer w;
    w.u32(a.frame), w.u8(0);
    auto f = [&](Fx v) { float x = float(v.toDouble()); uint32_t u; std::memcpy(&u, &x, 4); w.u32(u); };
    for (auto& p : a.players) f(p.x), f(p.y);
    for (auto& u : a.units) f(u.x), f(u.y), f(u.vx), f(u.vy);
    return w.b;
}
// 差量：相对基线（对方确认收到的某个快照），只发位置变了的单位。
// 变化的单位多时用位图（每个单位 1 位），少时用索引列表（每个 2 字节），取较小的一种。没有基线时发完整的量化状态
inline std::vector<uint8_t> encodeDelta(const QState& s, const QState* base);
inline std::vector<uint8_t> encodeDeltaRaw(const QState& s, const QState* base) {
    Writer w;
    w.u32(s.frame);
    if (!base) {
        w.u8(1);
        for (auto p : s.players) w.pos(p);
        for (auto p : s.units) w.pos(p);
        return w.b;
    }
    std::vector<uint16_t> changed;
    for (size_t i = 0; i < s.units.size(); ++i)
        if (!(s.units[i] == base->units[i])) changed.push_back(uint16_t(i));
    size_t bitmap = (s.units.size() + 7) / 8, list = 2 + 2 * changed.size();
    w.u8(bitmap <= list ? 2 : 3);
    w.u32(base->frame);
    for (auto p : s.players) w.pos(p);
    if (bitmap <= list) {
        std::vector<uint8_t> bits(bitmap, 0);
        for (uint16_t i : changed) bits[i / 8] |= uint8_t(1 << (i % 8));
        for (uint8_t v : bits) w.u8(v);
    } else {
        w.u16(uint16_t(changed.size()));
        for (uint16_t i : changed) w.u16(i);
    }
    for (uint16_t i : changed) w.pos(s.units[i]);
    return w.b;
}
// 几乎所有单位都变了时，差量（位图 + 基线帧号）反而比完整的量化状态大：取较小的一种
inline std::vector<uint8_t> encodeDelta(const QState& s, const QState* base) {
    std::vector<uint8_t> plain = encodeDeltaRaw(s, nullptr);
    if (!base) return plain;
    std::vector<uint8_t> d = encodeDeltaRaw(s, base);
    return d.size() < plain.size() ? d : plain;
}
// 解码：需要基线时从 baselines 里找；找不到（不应该发生）返回 false
inline bool decodeDelta(const std::vector<uint8_t>& data, size_t players, size_t units, const std::map<uint32_t, QState>& baselines, QState* out) {
    Reader r{data};
    out->frame = r.u32();
    uint8_t mode = r.u8();
    out->players.resize(players), out->units.resize(units);
    if (mode == 1) {
        for (auto& p : out->players) p = r.pos();
        for (auto& p : out->units) p = r.pos();
        return true;
    }
    auto it = baselines.find(r.u32());
    if (it == baselines.end()) return false;
    out->units = it->second.units;
    for (auto& p : out->players) p = r.pos();
    std::vector<uint16_t> changed;
    if (mode == 2) {
        size_t bitmap = (units + 7) / 8;
        for (size_t k = 0; k < bitmap; ++k) {
            uint8_t v = r.u8();
            for (int bit = 0; bit < 8; ++bit)
                if (v & (1 << bit)) changed.push_back(uint16_t(k * 8 + size_t(bit)));
        }
    } else {
        uint16_t n = r.u16();
        for (uint16_t k = 0; k < n; ++k) changed.push_back(r.u16());
    }
    for (uint16_t i : changed) out->units[i] = r.pos();
    return true;
}
// #endregion

// #region statesync
// 客户端在 r 时刻（服务器帧，可以是小数）看到的样子：r 两侧各有一个快照时插值；
// 没有比 r 更新的快照时，沿最后两个快照的方向外推（最多外推两个间隔）
struct Sampled {
    std::vector<std::pair<double, double>> players, units;  // 像素
    bool extrapolated = false;
};
inline bool sampleAt(const std::map<uint32_t, QState>& snaps, double r, Sampled* out, bool withUnits) {
    if (snaps.empty()) return false;
    auto b = snaps.lower_bound(uint32_t(std::max(0.0, std::ceil(r))));  // 第一个 ≥ r 的快照
    const QState *s0, *s1;
    bool extra = false;
    if (b != snaps.end() && b != snaps.begin()) s1 = &b->second, s0 = &std::prev(b)->second;
    else if (b == snaps.end() && snaps.size() >= 2) s1 = &std::prev(snaps.end())->second, s0 = &std::prev(snaps.end(), 2)->second, extra = true;
    else if (b == snaps.end()) s0 = s1 = &snaps.rbegin()->second, extra = true;
    else s0 = s1 = &b->second;  // 所有快照都比 r 新：停在最早的一个
    if (double(s1->frame) == r) s0 = s1, extra = false;
    double t = s1->frame == s0->frame ? 0 : (r - s0->frame) / double(s1->frame - s0->frame);
    if (extra) t = std::min(t, 3.0);
    auto lerp = [&](const std::vector<QPos>& a, const std::vector<QPos>& c, std::vector<std::pair<double, double>>& o) {
        o.resize(a.size());
        for (size_t i = 0; i < a.size(); ++i)
            o[i] = {(a[i].x + (c[i].x - a[i].x) * t) / 64.0, (a[i].y + (c[i].y - a[i].y) * t) / 64.0};
    };
    lerp(s0->players, s1->players, out->players);
    if (withUnits) lerp(s0->units, s1->units, out->units);
    out->extrapolated = extra;
    return true;
}

struct StateConfig {
    int clients = 4, units = 200;
    int sendEvery = 3;        // 每 3 帧发一次快照 = 20 Hz
    double interpMs = 100;    // 客户端渲染落后于服务器的额外时间
    Encoding encoding = Encoding::Delta;
    int64_t pressAt = -1;
    LinkConfig link;
    uint64_t seed = 1;
};
struct Snapshot {
    uint32_t frame = 0;
    std::vector<uint8_t> data;
};
struct ClientInput {
    uint32_t frame = 0;
    Input input;
    int64_t ackSnapshot = -1;  // 客户端收到的最新快照
};

struct StateClient {
    std::map<uint32_t, QState> snaps;  // 收到并解码的快照（也是差量的基线），只保留最近的
    int64_t newest = -1;
    long extrapolated = 0, rendered = 0, decodeFailures = 0;
    double errorSum = 0, errorMax = 0;
    struct View {
        double x, y;
        bool extrapolated;
    };
    std::vector<View> players;  // 本 tick 渲染出的玩家位置（像素）
};

class StateSession {
public:
    explicit StateSession(const StateConfig& c) : cfg_(c), server_(Arena::make(c.seed, c.clients, c.units)) {
        clients_.resize(size_t(c.clients));
        lastInput_.resize(size_t(c.clients));
        acked_.assign(size_t(c.clients), -1);
        sentStates_.resize(size_t(c.clients));
        for (int i = 0; i < c.clients; ++i) {
            down_.emplace_back(), up_.emplace_back();
            down_.back().cfg = up_.back().cfg = c.link;
            down_.back().rng = Pcg32{c.seed * 77 + uint64_t(i)}, up_.back().rng = Pcg32{c.seed * 91 + uint64_t(i)};
        }
        history_.push_back(server_.players);
    }

    void tick() {
        double now = double(tick_) * 1000.0 / 60;
        // 服务器：收输入（每个玩家用最新的一个），走一帧，按频率给每个客户端发快照
        for (int i = 0; i < cfg_.clients; ++i)
            up_[size_t(i)].deliver(now, [&](const ClientInput& m) {
                if (int64_t(m.frame) >= lastInputFrame_[size_t(i)]) lastInput_[size_t(i)] = m.input, lastInputFrame_[size_t(i)] = m.frame;
                acked_[size_t(i)] = std::max(acked_[size_t(i)], m.ackSnapshot);
            });
        server_.step(lastInput_.data());
        history_.push_back(server_.players);
        if (server_.frame % uint32_t(cfg_.sendEvery) == 0) sendSnapshots(now);
        // 客户端：收快照、渲染、发输入
        for (int i = 0; i < cfg_.clients; ++i) {
            StateClient& c = clients_[size_t(i)];
            down_[size_t(i)].deliver(now, [&](const Snapshot& s) { receive(c, s); });
            render(c, now);
            ClientInput in{uint32_t(tick_), inputFor(cfg_.seed, i, uint32_t(tick_), cfg_.pressAt), c.newest};
            up_[size_t(i)].send(now, in, 4 + 2 + 4);
        }
        ++tick_;
    }

    const StateConfig& config() const { return cfg_; }
    const Arena& server() const { return server_; }
    const std::vector<std::vector<Arena::Player>>& history() const { return history_; }
    const std::map<uint32_t, QState>& truth() const { return truth_; }  // 每个快照帧服务器的量化状态（测试用）
    const std::vector<StateClient>& clients() const { return clients_; }
    int64_t inputFrame(int c) const { return lastInputFrame_[size_t(c)]; }  // 服务器正在使用的、客户端 c 第几帧的输入
    uint64_t downBytes(int c) const { return down_[size_t(c)].bytes; }
    uint64_t upBytes(int c) const { return up_[size_t(c)].bytes; }
    // 三种编码在同一时刻各要多少字节（只统计，实际发送的是 config().encoding）
    uint64_t encodedBytes(Encoding e) const { return encoded_[int(e)]; }
    long ticks() const { return tick_; }
    // 渲染时刻（以服务器帧为单位）：现在 − 单程延迟 − 插值延迟（假设时钟已经同步）
    double renderFrame(double nowMs) const { return (nowMs - cfg_.link.latencyMs - cfg_.interpMs) * 60.0 / 1000; }

private:
    void sendSnapshots(double now) {
        QState q = quantize(server_);
        truth_[q.frame] = q;
        std::vector<uint8_t> full = encodeFull(server_);
        for (int i = 0; i < cfg_.clients; ++i) {
            auto& sent = sentStates_[size_t(i)];
            const QState* base = nullptr;
            if (acked_[size_t(i)] >= 0) {
                auto it = sent.find(uint32_t(acked_[size_t(i)]));
                if (it != sent.end()) base = &it->second;
            }
            std::vector<uint8_t> delta = encodeDelta(q, base), plain = encodeDelta(q, nullptr);
            encoded_[0] += full.size() + kUdpIpHeader, encoded_[1] += plain.size() + kUdpIpHeader, encoded_[2] += delta.size() + kUdpIpHeader;
            // 完整和量化编码客户端都按"无基线"处理；完整编码的包更大，按它的大小计
            Snapshot s{q.frame, cfg_.encoding == Encoding::Delta ? delta : plain};
            size_t size = cfg_.encoding == Encoding::Full ? full.size() : s.data.size();
            sent[q.frame] = q;
            while (sent.size() > 128) sent.erase(sent.begin());
            down_[size_t(i)].send(now, std::move(s), size);
        }
    }
    void receive(StateClient& c, const Snapshot& s) {
        if (int64_t(s.frame) <= c.newest && c.snaps.count(s.frame)) return;
        QState q;
        if (!decodeDelta(s.data, size_t(cfg_.clients), size_t(cfg_.units), c.snaps, &q)) {
            ++c.decodeFailures;
            return;
        }
        c.snaps[s.frame] = std::move(q);
        c.newest = std::max<int64_t>(c.newest, s.frame);
        while (c.snaps.size() > 64) c.snaps.erase(c.snaps.begin());
    }
    void render(StateClient& c, double now) {
        c.players.clear();
        double r = renderFrame(now);
        Sampled v;
        if (r < 0 || !sampleAt(c.snaps, r, &v, false)) return;
        // 真值：服务器在 r 时刻的位置（相邻两帧之间线性插值）
        size_t f0 = size_t(std::floor(r));
        double ft = r - double(f0);
        const auto &h0 = history_[std::min(f0, history_.size() - 1)], &h1 = history_[std::min(f0 + 1, history_.size() - 1)];
        ++c.rendered, c.extrapolated += v.extrapolated;
        for (size_t p = 0; p < v.players.size(); ++p) {
            auto [x, y] = v.players[p];
            c.players.push_back({x, y, v.extrapolated});
            double tx = h0[p].x.toDouble() * (1 - ft) + h1[p].x.toDouble() * ft;
            double ty = h0[p].y.toDouble() * (1 - ft) + h1[p].y.toDouble() * ft;
            double e = std::hypot(x - tx, y - ty);
            c.errorSum += e, c.errorMax = std::max(c.errorMax, e);
        }
    }

    StateConfig cfg_;
    Arena server_;
    std::vector<std::vector<Arena::Player>> history_;
    std::map<uint32_t, QState> truth_;
    std::vector<StateClient> clients_;
    std::vector<Input> lastInput_;
    std::vector<int64_t> lastInputFrame_ = std::vector<int64_t>(64, -1);
    std::vector<int64_t> acked_;
    std::vector<std::map<uint32_t, QState>> sentStates_;
    std::vector<Link<Snapshot>> down_;
    std::vector<Link<ClientInput>> up_;
    uint64_t encoded_[3] = {};
    long tick_ = 0;
};
// #endregion

}  // namespace ga::net
