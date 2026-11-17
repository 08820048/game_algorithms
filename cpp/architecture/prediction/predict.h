#pragma once

// 网络同步：客户端预测、回滚与延迟补偿（模拟网络和竞技场见 6.5 的 net.h）
//   - 客户端预测与和解：状态同步下，客户端不等服务器，立刻用自己的输入移动自己的角色。
//     快照带回"服务器已经处理到你第几号输入"，客户端从快照里自己的位置出发，把还没被处理的输入重新算一遍
//   - 回滚（rollback）：帧同步下，不等别人的输入，先猜（沿用对方最后一个已知输入）往前算；
//     真实输入到了、发现猜错了，就读回那一帧保存的状态，用正确的输入重算到现在
//   - 延迟补偿：射手看到的目标是过去的位置。服务器收到开枪消息后，把目标退回射手当时看到的时刻再判定命中

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <vector>

#include "../netsync/net.h"

namespace ga::net {

// #region field
// 有墙的场地：玩家是 16×16 的方块，每帧最多走 2 像素；撞到墙或其他玩家就停在贴着的位置（不推动别人）
struct Box {
    Fx x0, y0, x1, y1;
};
inline bool overlaps(const Box& a, const Box& b) { return a.x0 < b.x1 && b.x0 < a.x1 && a.y0 < b.y1 && b.y0 < a.y1; }

struct Field {
    static constexpr int kW = 360, kH = 300, kHalf = 8, kSpeed = 2;
    struct Player {
        Fx x, y;
        friend bool operator==(Player, Player) = default;
    };
    std::vector<Box> walls;
    std::vector<Player> players;

    static Field make(int playerCount) {
        Field f;
        auto box = [](int x0, int y0, int x1, int y1) { return Box{Fx::fromInt(x0), Fx::fromInt(y0), Fx::fromInt(x1), Fx::fromInt(y1)}; };
        f.walls = {box(80, 60, 120, 140), box(240, 160, 280, 240), box(140, 200, 210, 215), box(150, 70, 230, 85)};
        for (int i = 0; i < playerCount; ++i) f.players.push_back({Fx::fromInt(40 + (i % 8) * 40), Fx::fromInt(i % 16 < 8 ? 40 : 260)});
        return f;
    }
    static Box boxOf(Player p) {
        const Fx h = Fx::fromInt(kHalf);
        return {p.x - h, p.y - h, p.x + h, p.y + h};
    }
    // 障碍物：所有墙 + 除 skip 以外的玩家（位置由调用者给出：服务器用当前位置，客户端用它最近收到的快照）
    std::vector<Box> obstacles(const std::vector<Player>& ps, size_t skip) const {
        std::vector<Box> o = walls;
        for (size_t i = 0; i < ps.size(); ++i)
            if (i != skip) o.push_back(boxOf(ps[i]));
        return o;
    }
};

// 走一步：先 x 后 y。只处理"走之前不重叠、走之后重叠"的障碍物，退回到贴着它的位置；最后限制在场地内
inline Field::Player movePlayer(Field::Player me, Input in, const std::vector<Box>& obstacles) {
    const Fx h = Fx::fromInt(Field::kHalf);
    auto axis = [&](bool isX, int d) {
        if (d == 0) return;
        Field::Player np = me;
        (isX ? np.x : np.y) += Fx::fromInt(Field::kSpeed * d);
        Box before = Field::boxOf(me), after = Field::boxOf(np);
        for (const Box& o : obstacles) {
            if (overlaps(before, o) || !overlaps(after, o)) continue;
            if (isX) np.x = d > 0 ? std::min(np.x, o.x0 - h) : std::max(np.x, o.x1 + h);
            else np.y = d > 0 ? std::min(np.y, o.y0 - h) : std::max(np.y, o.y1 + h);
        }
        np.x = std::clamp(np.x, h, Fx::fromInt(Field::kW) - h);
        np.y = std::clamp(np.y, h, Fx::fromInt(Field::kH) - h);
        me = np;
    };
    axis(true, in.dx);
    axis(false, in.dy);
    return me;
}
// #endregion

// #region predict
enum class Mode {
    None,       // 不预测：自己的角色和别人一样，从快照插值（6.5 的做法）
    Predict,    // 预测但不和解：收到快照就把自己放到服务器给的位置，再接着用新输入走
    Reconcile,  // 预测 + 和解：从快照里自己的位置出发，重算所有服务器还没处理的输入
    Smooth,     // 和解 + 平滑：纠正量不立刻显示，而是在几帧内逐渐消失
};

struct PredictConfig {
    int players = 4;
    int sendEvery = 3;      // 20 Hz 快照
    double interpMs = 100;  // 其他玩家（以及不预测时的自己）的插值延迟
    Mode mode = Mode::Reconcile;
    int64_t pressAt = -1;   // 测手感：玩家 0 在这一帧之前不动，之后一直向右
    bool othersIdle = false;  // 其他玩家站着不动
    LinkConfig link;
    uint64_t seed = 1;
};

struct InputBatch {  // 客户端 → 服务器：从 first 号开始的连续输入（服务器还没确认的全部重发）
    uint32_t first = 0;
    std::vector<Input> inputs;
};
struct FieldSnapshot {  // 服务器 → 客户端
    uint32_t frame = 0;
    std::vector<Field::Player> players;  // 精确位置：和解要从和服务器逐位相同的状态出发
    int64_t lastProcessed = -1;          // 服务器已经处理到这个客户端第几号输入
};

struct PredictClient {
    std::vector<Input> inputs;  // 自己产生的输入，序号 = 下标 = 产生它的 tick
    std::map<uint32_t, FieldSnapshot> snaps;
    int64_t newest = -1;
    Field::Player predicted;    // 预测的自己
    double offX = 0, offY = 0;  // 平滑：显示位置 = 预测位置 + 偏移，偏移每帧乘 0.85
    double shownX = 0, shownY = 0;
    std::vector<std::pair<double, double>> shown;  // 本 tick 画出的所有玩家（自己是 shownX/Y）
    std::vector<std::pair<double, double>> shownHistory;  // 每个 tick 画出的自己，下标 = tick
    long corrections = 0;       // 收到快照后，重新算出的自己和之前预测的不一样
    double correctionSum = 0, correctionMax = 0;
};

class PredictSession {
public:
    explicit PredictSession(const PredictConfig& c) : cfg_(c), server_(Field::make(c.players)) {
        int n = c.players;
        clients_.resize(size_t(n));
        nextSeq_.assign(size_t(n), 0);
        serverHistory_.resize(size_t(n));
        for (int i = 0; i < n; ++i) {
            clients_[size_t(i)].predicted = server_.players[size_t(i)];
            up_.emplace_back(), down_.emplace_back();
            up_.back().cfg = down_.back().cfg = c.link;
            up_.back().rng = Pcg32{c.seed * 131 + uint64_t(i)}, down_.back().rng = Pcg32{c.seed * 171 + uint64_t(i)};
        }
    }

    void tick() {
        double now = double(tick_) * 1000.0 / 60;
        // 服务器：输入一到就按序号处理（每个只处理一次），每 sendEvery 帧给每个客户端发快照
        for (int i = 0; i < cfg_.players; ++i)
            up_[size_t(i)].deliver(now, [&](const InputBatch& b) { serverApply(i, b); });
        ++frame_;
        if (frame_ % uint32_t(cfg_.sendEvery) == 0)
            for (int i = 0; i < cfg_.players; ++i) {
                FieldSnapshot s{frame_, server_.players, int64_t(nextSeq_[size_t(i)]) - 1};
                down_[size_t(i)].send(now, std::move(s), 4 + 4 + 8 * size_t(cfg_.players));
            }
        for (int i = 0; i < cfg_.players; ++i) clientTick(i, now);
        ++tick_;
    }

    const PredictConfig& config() const { return cfg_; }
    const Field& server() const { return server_; }
    const std::vector<PredictClient>& clients() const { return clients_; }
    long ticks() const { return tick_; }
    // 服务器处理完客户端 c 第 seq 号输入后，c 的位置（还没处理到时返回 false）
    bool serverAfter(int c, uint32_t seq, Field::Player* out) const {
        const auto& h = serverHistory_[size_t(c)];
        if (seq >= h.size()) return false;
        *out = h[seq];
        return true;
    }
    uint64_t upBytes(int c) const { return up_[size_t(c)].bytes; }
    // 演示用：玩家 0 改由键盘控制
    void setManual(bool on, Input in) { manualOn_ = on, manual_ = in; }

private:
    void serverApply(int c, const InputBatch& b) {
        uint32_t& next = nextSeq_[size_t(c)];
        if (b.first > next) return;  // 不会发生：客户端从它知道的"已处理"之后开始发
        for (uint32_t s = next; s < b.first + b.inputs.size(); ++s) {
            server_.players[size_t(c)] = movePlayer(server_.players[size_t(c)], b.inputs[s - b.first], server_.obstacles(server_.players, size_t(c)));
            serverHistory_[size_t(c)].push_back(server_.players[size_t(c)]);
            next = s + 1;
        }
    }

    void clientTick(int i, double now) {
        PredictClient& c = clients_[size_t(i)];
        down_[size_t(i)].deliver(now, [&](const FieldSnapshot& s) { receive(i, c, s); });
        // 产生本 tick 的输入，立刻用来移动自己（预测）
        Input in = i == 0 && manualOn_ ? manual_ : i != 0 && cfg_.othersIdle ? Input{} : inputFor(cfg_.seed, i, uint32_t(tick_), cfg_.pressAt);
        c.inputs.push_back(in);
        if (cfg_.mode != Mode::None) c.predicted = movePlayer(c.predicted, in, server_.obstacles(latestPlayers(c), size_t(i)));
        InputBatch b;
        int64_t acked = c.newest >= 0 ? c.snaps.at(uint32_t(c.newest)).lastProcessed : -1;
        b.first = uint32_t(acked + 1);
        b.inputs.assign(c.inputs.begin() + b.first, c.inputs.end());
        size_t bytes = 4 + 2 * b.inputs.size();
        up_[size_t(i)].send(now, std::move(b), bytes);
        render(i, c, now);
    }

    std::vector<Field::Player> latestPlayers(const PredictClient& c) const {
        return c.newest >= 0 ? c.snaps.at(uint32_t(c.newest)).players : Field::make(cfg_.players).players;
    }

    void receive(int i, PredictClient& c, const FieldSnapshot& s) {
        c.snaps[s.frame] = s;
        while (c.snaps.size() > 64) c.snaps.erase(c.snaps.begin());
        if (int64_t(s.frame) <= c.newest) return;  // 乱序到达的旧快照只用来插值
        c.newest = s.frame;
        if (cfg_.mode == Mode::None) return;
        Field::Player p = s.players[size_t(i)];
        if (cfg_.mode != Mode::Predict) {
            // 和解：服务器处理到 lastProcessed 号，后面的输入它还没见过，在快照的基础上重新走一遍
            std::vector<Box> obs = server_.obstacles(s.players, size_t(i));
            for (size_t q = size_t(s.lastProcessed + 1); q < c.inputs.size(); ++q) p = movePlayer(p, c.inputs[q], obs);
        }
        if (!(p == c.predicted)) {
            double dx = (c.predicted.x - p.x).toDouble(), dy = (c.predicted.y - p.y).toDouble(), d = std::hypot(dx, dy);
            ++c.corrections, c.correctionSum += d, c.correctionMax = std::max(c.correctionMax, d);
            if (cfg_.mode == Mode::Smooth) c.offX += dx, c.offY += dy;
        }
        c.predicted = p;
    }

    void render(int i, PredictClient& c, double now) {
        // 其他玩家：渲染时刻 = 现在 − 单程延迟 − 插值延迟，在两侧的快照之间插值（没有更新的快照时停在最新的）
        double r = (now - cfg_.link.latencyMs - cfg_.interpMs) * 60.0 / 1000;
        c.shown.assign(size_t(cfg_.players), {0, 0});
        std::vector<Field::Player> init = Field::make(cfg_.players).players;
        for (size_t k = 0; k < c.shown.size(); ++k) c.shown[k] = {init[k].x.toDouble(), init[k].y.toDouble()};
        if (!c.snaps.empty()) {
            auto b = c.snaps.lower_bound(uint32_t(std::max(0.0, std::ceil(r))));
            const FieldSnapshot *s0, *s1;
            if (b == c.snaps.end()) s0 = s1 = &c.snaps.rbegin()->second;
            else if (b == c.snaps.begin()) s0 = s1 = &b->second;
            else s0 = &std::prev(b)->second, s1 = &b->second;
            double t = s1->frame == s0->frame ? 0 : std::clamp((r - s0->frame) / double(s1->frame - s0->frame), 0.0, 1.0);
            for (size_t k = 0; k < c.shown.size(); ++k)
                c.shown[k] = {s0->players[k].x.toDouble() + (s1->players[k].x - s0->players[k].x).toDouble() * t,
                              s0->players[k].y.toDouble() + (s1->players[k].y - s0->players[k].y).toDouble() * t};
        }
        if (cfg_.mode == Mode::None) {
            c.shownX = c.shown[size_t(i)].first, c.shownY = c.shown[size_t(i)].second;
        } else {
            if (std::hypot(c.offX, c.offY) > 64) c.offX = c.offY = 0;  // 纠正量太大（超过 4 个身位）：直接跳过去
            c.shownX = c.predicted.x.toDouble() + c.offX, c.shownY = c.predicted.y.toDouble() + c.offY;
            c.offX *= 0.85, c.offY *= 0.85;
            if (std::hypot(c.offX, c.offY) < 0.01) c.offX = c.offY = 0;
            c.shown[size_t(i)] = {c.shownX, c.shownY};
        }
        c.shownHistory.push_back({c.shownX, c.shownY});
    }

    PredictConfig cfg_;
    Field server_;
    uint32_t frame_ = 0;
    std::vector<PredictClient> clients_;
    std::vector<uint32_t> nextSeq_;
    std::vector<std::vector<Field::Player>> serverHistory_;
    std::vector<Link<InputBatch>> up_;
    std::vector<Link<FieldSnapshot>> down_;
    bool manualOn_ = false;
    Input manual_;
    long tick_ = 0;
};
// #endregion

// #region rollback
// 机器人：每 period 帧换一个方向（period = 40 时和 net.h 的 botInput 相同）
inline Input botInputEvery(uint64_t seed, int player, uint32_t frame, uint32_t period) {
    uint64_t h = (seed * 0x9e3779b97f4a7c15ull) ^ (uint64_t(player) << 32) ^ (frame / period);
    h ^= h >> 31, h *= 0xbf58476d1ce4e5b9ull, h ^= h >> 29;
    return {int8_t(int(h % 3) - 1), int8_t(int((h >> 8) % 3) - 1)};
}

struct RollbackConfig {
    int peers = 4, units = 200;
    int delay = 0;       // 输入延迟 D（帧）：可以和回滚一起用，D 越大猜的帧越少
    int window = 8;      // 最多比"所有输入都到齐"的帧超前多少帧；超过就停下来等（window = 0 就是帧同步）
    int botPeriod = 40;
    LinkConfig link;
    uint64_t seed = 1;
};

struct RollbackPeer {
    int id = 0;
    Arena sim;
    std::vector<std::vector<Input>> inputs, used;  // [帧][玩家]：收到的真实输入 / 算这一帧时用的（真实的或猜的）
    std::vector<std::vector<bool>> have;
    std::vector<int64_t> contiguous, ackedBy;      // 同帧同步
    std::vector<Arena> saved;                      // saved[f % 环长] = 第 f 帧开始时的状态
    std::vector<int64_t> savedFrame;
    int64_t rollbackFrom = -1;                     // 最早一个猜错的帧
    std::map<uint32_t, uint64_t> confirmedHashes;  // 所有输入都确认之后的状态哈希（每 60 帧）
    uint32_t nextHash = 60;
    long stallTicks = 0, rollbacks = 0, resimFrames = 0, maxStepsPerTick = 0, ringMisses = 0;
    long guessed = 0, wrongGuesses = 0;            // 猜过的输入（玩家 × 帧），其中猜错的
    double jumpSum = 0, jumpMax = 0;               // 回滚时其他玩家在画面上瞬移的距离
    long jumps = 0;
    int lastSteps = 0, lastResim = 0;
    struct Jump {
        int player;
        double x0, y0, x1, y1;
    };
    std::vector<Jump> lastJumps;
};

class RollbackSession {
public:
    explicit RollbackSession(const RollbackConfig& c) : cfg_(c) {
        int n = c.peers, ring = c.window + 3;
        for (int i = 0; i < n; ++i) {
            RollbackPeer p;
            p.id = i;
            p.sim = Arena::make(c.seed, n, c.units);
            p.contiguous.assign(size_t(n), c.delay - 1);
            p.ackedBy.assign(size_t(n), c.delay - 1);
            p.saved.resize(size_t(ring));
            p.savedFrame.assign(size_t(ring), -1);
            peers_.push_back(std::move(p));
        }
        links_.resize(size_t(n * n));
        for (int i = 0; i < n * n; ++i) links_[size_t(i)].cfg = c.link, links_[size_t(i)].rng = Pcg32{c.seed * 1000 + uint64_t(i)};
    }

    void tick() {
        double now = double(tick_) * 1000.0 / 60;
        int n = cfg_.peers;
        for (int i = 0; i < n; ++i)
            for (int j = 0; j < n; ++j)
                if (i != j) link(j, i).deliver(now, [&](const InputPacket& pk) { receive(peers_[size_t(i)], pk); });
        for (auto& p : peers_) advance(p);
        for (int i = 0; i < n; ++i)
            for (int j = 0; j < n; ++j)
                if (i != j) sendTo(peers_[size_t(i)], j, now);
        ++tick_;
    }

    const RollbackConfig& config() const { return cfg_; }
    const std::vector<RollbackPeer>& peers() const { return peers_; }
    long ticks() const { return tick_; }
    uint64_t bytesSentBy(int peer) const {
        uint64_t b = 0;
        for (int j = 0; j < cfg_.peers; ++j)
            if (j != peer) b += links_[size_t(peer * cfg_.peers + j)].bytes;
        return b;
    }
    static int64_t confirmed(const RollbackPeer& p) { return *std::min_element(p.contiguous.begin(), p.contiguous.end()); }

private:
    Link<InputPacket>& link(int from, int to) { return links_[size_t(from * cfg_.peers + to)]; }
    size_t ring() const { return size_t(cfg_.window + 3); }

    bool store(RollbackPeer& p, int player, uint32_t f, Input in) {
        while (p.inputs.size() <= f) {
            p.inputs.emplace_back(size_t(cfg_.peers)), p.used.emplace_back(size_t(cfg_.peers));
            p.have.emplace_back(size_t(cfg_.peers), false);
        }
        if (p.have[f][size_t(player)]) return false;
        p.inputs[f][size_t(player)] = in, p.have[f][size_t(player)] = true;
        int64_t& c = p.contiguous[size_t(player)];
        while (size_t(c + 1) < p.have.size() && p.have[size_t(c + 1)][size_t(player)]) ++c;
        return true;
    }

    void receive(RollbackPeer& p, const InputPacket& pk) {
        for (size_t k = 0; k < pk.inputs.size(); ++k) {
            uint32_t f = pk.first + uint32_t(k);
            if (!store(p, pk.from, f, pk.inputs[k]) || f >= p.sim.frame) continue;
            // 这一帧已经算过了，当时用的是猜的输入
            ++p.guessed;
            if (!(p.used[f][size_t(pk.from)] == pk.inputs[k])) {
                ++p.wrongGuesses;
                p.rollbackFrom = p.rollbackFrom < 0 ? int64_t(f) : std::min<int64_t>(p.rollbackFrom, f);
            }
        }
        p.ackedBy[size_t(pk.from)] = std::max(p.ackedBy[size_t(pk.from)], pk.ack);
    }

    // 算第 f 帧要用的输入：收到了就用真实的；没收到就猜：沿用这个玩家最后一个连续收到的输入
    Input best(const RollbackPeer& p, uint32_t f, int player) const {
        if (f < uint32_t(cfg_.delay)) return {};
        if (f < p.have.size() && p.have[f][size_t(player)]) return p.inputs[f][size_t(player)];
        int64_t c = p.contiguous[size_t(player)];
        return c < cfg_.delay ? Input{} : p.inputs[size_t(c)][size_t(player)];
    }

    void simulate(RollbackPeer& p) {
        uint32_t f = p.sim.frame;
        p.saved[f % ring()] = p.sim, p.savedFrame[f % ring()] = f;
        while (p.used.size() <= f) {
            p.inputs.emplace_back(size_t(cfg_.peers)), p.used.emplace_back(size_t(cfg_.peers));
            p.have.emplace_back(size_t(cfg_.peers), false);
        }
        for (int k = 0; k < cfg_.peers; ++k) p.used[f][size_t(k)] = best(p, f, k);
        p.sim.step(p.used[f].data());
    }

    void advance(RollbackPeer& p) {
        int steps = 0, resim = 0;
        p.lastJumps.clear();
        // 1. 回滚：读回最早猜错的那一帧开始时的状态，用现在知道的输入重算到当前帧
        if (p.rollbackFrom >= 0 && p.rollbackFrom < int64_t(p.sim.frame)) {
            uint32_t target = p.sim.frame, g = uint32_t(p.rollbackFrom);
            std::vector<Arena::Player> before = p.sim.players;
            if (p.savedFrame[g % ring()] != g) ++p.ringMisses;  // 不应该发生：窗口保证需要的状态都还在
            p.sim = p.saved[g % ring()];
            while (p.sim.frame < target) simulate(p), ++resim;
            ++p.rollbacks;
            for (size_t k = 0; k < before.size(); ++k) {
                double x0 = before[k].x.toDouble(), y0 = before[k].y.toDouble();
                double x1 = p.sim.players[k].x.toDouble(), y1 = p.sim.players[k].y.toDouble(), d = std::hypot(x1 - x0, y1 - y0);
                if (d == 0) continue;
                p.lastJumps.push_back({int(k), x0, y0, x1, y1});
                ++p.jumps, p.jumpSum += d, p.jumpMax = std::max(p.jumpMax, d);
            }
        }
        p.rollbackFrom = -1;
        recordHashes(p);
        // 2. 往前算：只要不超过窗口就不等（落后时一个 tick 最多算 3 帧追赶）
        while (int64_t(p.sim.frame) <= tick_ && steps < 3) {
            uint32_t f = p.sim.frame;
            store(p, p.id, f + uint32_t(cfg_.delay), botInputEvery(cfg_.seed, p.id, f + uint32_t(cfg_.delay), uint32_t(cfg_.botPeriod)));
            if (int64_t(f) > confirmed(p) + cfg_.window) break;
            simulate(p), ++steps;
        }
        recordHashes(p);
        if (steps == 0 && int64_t(p.sim.frame) <= tick_) ++p.stallTicks;
        p.resimFrames += resim, p.maxStepsPerTick = std::max<long>(p.maxStepsPerTick, steps + resim);
        p.lastSteps = steps, p.lastResim = resim;
    }

    // 第 k 帧开始时的状态只依赖第 k 帧之前的输入；它们都确认了，这个状态就不会再变
    void recordHashes(RollbackPeer& p) {
        while (int64_t(p.nextHash) <= confirmed(p) + 1 && p.nextHash <= p.sim.frame) {
            uint32_t k = p.nextHash;
            if (k == p.sim.frame) p.confirmedHashes[k] = p.sim.hash();
            else if (p.savedFrame[k % ring()] == k) p.confirmedHashes[k] = p.saved[k % ring()].hash();
            else ++p.ringMisses;
            p.nextHash += 60;
        }
    }

    void sendTo(RollbackPeer& p, int j, double now) {
        InputPacket pk;
        pk.from = p.id;
        int64_t first = std::max<int64_t>(p.ackedBy[size_t(j)] + 1, cfg_.delay);
        pk.first = uint32_t(first);
        for (int64_t f = first; f <= p.contiguous[size_t(p.id)]; ++f) pk.inputs.push_back(p.inputs[size_t(f)][size_t(p.id)]);
        pk.ack = p.contiguous[size_t(j)];
        size_t b = pk.bytes();
        link(p.id, j).send(now, std::move(pk), b);
    }

    RollbackConfig cfg_;
    std::vector<RollbackPeer> peers_;
    std::vector<Link<InputPacket>> links_;
    long tick_ = 0;
};
// #endregion

// #region lagcomp
// 射击场：目标在上方左右移动，中间一道墙只留一个缺口；射手在下方，只有目标在缺口里时才看得见
struct Range {
    static constexpr int kW = 360, kH = 300, kHalf = 8;
    static constexpr double kTargetY = 60, kWallY0 = 140, kWallY1 = 150, kGap0 = 150, kGap1 = 210, kShooterX = 180, kShooterY = 270;
    // 从射手到 (x, y) 的视线是否穿过缺口（墙是一条横带，视线在墙的上下沿之间的部分都要落在缺口里）
    static bool visible(double x, double y) {
        auto at = [&](double wy) { return kShooterX + (x - kShooterX) * (kShooterY - wy) / (kShooterY - y); };
        double a = at(kWallY1), b = at(kWallY0);
        return std::min(a, b) >= kGap0 && std::max(a, b) <= kGap1;
    }
};

struct LagCompConfig {
    LinkConfig link{50, 0, 0};
    double interpMs = 100;
    int sendEvery = 3;
    double maxRewindMs = 1e9;  // 最多回退多久（防止作弊者谎报很早的时刻；也意味着延迟太高的玩家得不到补偿）
    double aimError = 3;       // 射手瞄准的误差（像素，正态分布的标准差）
    uint64_t seed = 1;
};

struct Shot {
    double x = 0, y = 0;     // 准星位置
    double renderFrame = 0;  // 射手开枪时画面显示的是服务器第几帧（可以是小数）
};
struct ShotResult {
    Shot shot;
    uint32_t serverFrame = 0;          // 服务器收到时的帧
    double nowX = 0, backX = 0;        // 目标此刻的位置 / 退回射手看到的时刻的位置
    bool hitNow = false, hitBack = false;
    bool behindCover = false;          // 补偿后命中，但目标此刻已经在墙后了
    double coverMs = 0;                // 已经在墙后多久
};

class LagCompSession {
public:
    explicit LagCompSession(const LagCompConfig& c) : cfg_(c), rng_(c.seed * 7 + 3) {
        up_.cfg = down_.cfg = c.link;
        up_.rng = Pcg32{c.seed * 11}, down_.rng = Pcg32{c.seed * 13};
        history_.push_back(x_);
    }

    void tick() {
        double now = double(tick_) * 1000.0 / 60;
        // 服务器：判定收到的开枪消息，移动目标，发快照
        up_.deliver(now, [&](const Shot& s) { judge(s); });
        if (--turn_ <= 0) dir_ = int(rng_.below(3)) - 1, turn_ = 10 + int(rng_.below(50));
        x_ += 2.0 * dir_;
        if (x_ < 40 || x_ > kRight) dir_ = -dir_, x_ = std::clamp(x_, 40.0, kRight);
        ++frame_;
        history_.push_back(x_);
        if (Range::visible(x_, Range::kTargetY)) lastVisible_ = frame_;
        if (frame_ % uint32_t(cfg_.sendEvery) == 0) down_.send(now, std::pair<uint32_t, double>{frame_, x_}, 4 + 4);
        // 射手：收快照，在渲染时刻插值，看得见就开枪
        down_.deliver(now, [&](const std::pair<uint32_t, double>& s) { snaps_[s.first] = s.second; });
        while (snaps_.size() > 64) snaps_.erase(snaps_.begin());
        double r = (now - cfg_.link.latencyMs - cfg_.interpMs) * 60.0 / 1000;
        seen_ = sample(r);
        if (--cooldown_ <= 0 && r > 0 && !snaps_.empty() && Range::visible(seen_, Range::kTargetY)) {
            Shot s{seen_ + gauss() * cfg_.aimError, Range::kTargetY + gauss() * cfg_.aimError, r};
            up_.send(now, s, 4 + 4 + 4);
            cooldown_ = 15 + int(rng_.below(30));
        }
        ++tick_;
    }

    const LagCompConfig& config() const { return cfg_; }
    double targetX() const { return x_; }
    double seenX() const { return seen_; }  // 射手画面里的目标
    const std::vector<ShotResult>& results() const { return results_; }
    long ticks() const { return tick_; }
    // 服务器在第 r 帧（可以是小数）时目标的位置
    double targetAt(double r) const {
        r = std::clamp(r, 0.0, double(history_.size() - 1));
        size_t f = size_t(std::floor(r));
        double t = r - double(f);
        return f + 1 < history_.size() ? history_[f] * (1 - t) + history_[f + 1] * t : history_[f];
    }
    static constexpr double kRight = 320;

private:
    double sample(double r) const {
        if (snaps_.empty()) return x_;
        auto b = snaps_.lower_bound(uint32_t(std::max(0.0, std::ceil(r))));
        if (b == snaps_.end()) return snaps_.rbegin()->second;
        if (b == snaps_.begin()) return b->second;
        auto a = std::prev(b);
        double t = (r - a->first) / double(b->first - a->first);
        return a->second + (b->second - a->second) * std::clamp(t, 0.0, 1.0);
    }
    double gauss() {
        double u = std::max(1e-12, rng_.nextDouble()), v = rng_.nextDouble();
        return std::sqrt(-2 * std::log(u)) * std::cos(6.283185307179586 * v);
    }
    static bool inside(const Shot& s, double tx) { return std::fabs(s.x - tx) < Range::kHalf && std::fabs(s.y - Range::kTargetY) < Range::kHalf; }

    void judge(const Shot& s) {
        ShotResult r;
        r.shot = s, r.serverFrame = frame_;
        r.nowX = x_;
        double earliest = double(frame_) - cfg_.maxRewindMs * 60 / 1000;
        r.backX = targetAt(std::max(s.renderFrame, earliest));
        r.hitNow = inside(s, r.nowX), r.hitBack = inside(s, r.backX);
        if (r.hitBack && !Range::visible(x_, Range::kTargetY)) r.behindCover = true, r.coverMs = double(frame_ - lastVisible_) * 1000 / 60;
        results_.push_back(r);
    }

    LagCompConfig cfg_;
    Pcg32 rng_;
    Link<Shot> up_;
    Link<std::pair<uint32_t, double>> down_;
    std::vector<double> history_;
    std::map<uint32_t, double> snaps_;
    std::vector<ShotResult> results_;
    double x_ = 180, seen_ = 180;
    int dir_ = 1, turn_ = 30, cooldown_ = 30;
    uint32_t frame_ = 0, lastVisible_ = 0;
    long tick_ = 0;
};
// #endregion

}  // namespace ga::net
