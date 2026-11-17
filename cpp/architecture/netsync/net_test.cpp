#include "net.h"

#include <cmath>
#include <cstdio>

#include "check.h"

using namespace ga;
using namespace ga::net;

// 模拟网络：没有抖动时按顺序、恰好晚 latency 到达；丢包率接近设定值；字节数含 28 字节包头
static void testLink() {
    Link<int> l;
    l.cfg = {40, 0, 0};
    for (int i = 0; i < 100; ++i) l.send(i * 10.0, i, 10);
    std::vector<int> got;
    for (double t = 0; t < 2000; t += 1) l.deliver(t, [&](int v) {
            CHECK(std::fabs(t - (v * 10.0 + 40)) < 1.0);  // 每毫秒检查一次，到达时刻误差 < 1 ms
            got.push_back(v);
        });
    CHECK(got.size() == 100);
    for (int i = 0; i < 100; ++i) CHECK(got[size_t(i)] == i);
    CHECK(l.bytes == 100 * (10 + kUdpIpHeader));

    Link<int> lossy;
    lossy.cfg = {10, 30, 0.1};
    for (int i = 0; i < 20000; ++i) lossy.send(i, i, 1);
    int n = 0, outOfOrder = 0, last = -1;
    lossy.deliver(1e9, [&](int v) { ++n, outOfOrder += v < last, last = std::max(last, v); });
    CHECK(std::abs(n - 18000) < 400);
    CHECK(lossy.lost == uint64_t(20000 - n));
    CHECK(outOfOrder > 1000);  // 抖动 30 ms > 发送间隔 1 ms：大量乱序
}

// 竞技场：确定；玩家不动时，远处单位的速度最终精确地变成 0（摩擦向 0 截断）
static void testArena() {
    Arena a = Arena::make(3, 4, 300), b = a;
    std::vector<Input> in(4);
    for (uint32_t f = 0; f < 500; ++f) {
        for (int p = 0; p < 4; ++p) in[size_t(p)] = botInput(3, p, f);
        a.step(in.data()), b.step(in.data());
    }
    CHECK(a.hash() == b.hash());
    std::vector<Input> still(4);
    for (int f = 0; f < 3000; ++f) a.step(still.data());
    int moving = 0;
    for (auto& u : a.units) moving += u.vx.raw != 0 || u.vy.raw != 0;
    CHECK(moving == 0);
}

// 帧同步：每台机器在自己算到的那一帧，状态必须和"在一台机器上直接模拟"的参照逐位相同
static Arena reference(const LockstepConfig& c, uint32_t frames) {
    Arena a = Arena::make(c.seed, c.peers, c.units);
    std::vector<Input> in(size_t(c.peers));
    for (uint32_t f = 0; f < frames; ++f) {
        for (int p = 0; p < c.peers; ++p) {
            if (f < uint32_t(c.delay)) in[size_t(p)] = {};
            else if (c.pressAt >= 0) in[size_t(p)] = inputFor(c.seed, p, f - uint32_t(c.delay), c.pressAt);
            else in[size_t(p)] = botInput(c.seed, p, f);
        }
        a.step(in.data());
    }
    return a;
}
static void testLockstepMatchesReference() {
    struct Case {
        LinkConfig link;
        int delay, sendEvery;
    };
    const Case cases[] = {{{20, 0, 0}, 2, 1}, {{30, 20, 0.05}, 4, 1}, {{80, 40, 0.1}, 3, 1}, {{50, 10, 0.02}, 6, 3}, {{120, 60, 0.2}, 12, 1}};
    for (const Case& k : cases) {
        LockstepConfig c;
        c.units = 120, c.delay = k.delay, c.sendEvery = k.sendEvery, c.link = k.link, c.seed = 5;
        LockstepSession s(c);
        for (int t = 0; t < 1500; ++t) s.tick();
        for (auto& p : s.peers()) {
            // contiguous 等于从 have 暴力数出来的值（乱序到达的输入也要被算进去）
            for (int pl = 0; pl < c.peers; ++pl) {
                int64_t f = c.delay - 1;
                while (size_t(f + 1) < p.have.size() && p.have[size_t(f + 1)][size_t(pl)]) ++f;
                CHECK(p.contiguous[size_t(pl)] == f);
            }
            CHECK(p.sim.frame > 100);
            CHECK(p.sim.hash() == reference(c, p.sim.frame).hash());
            CHECK(p.desyncAt < 0);
        }
        // 只重发对方还没确认的输入：包的大小取决于往返时间，而不是游戏进行了多久
        for (int i = 0; i < c.peers; ++i) {
            double perPacket = double(s.bytesSentBy(i)) / double(s.packetsSentBy(i)) - double(kUdpIpHeader);
            double rttTicks = 2 * (k.link.latencyMs + k.link.jitterMs) * 60 / 1000 + 2 * k.sendEvery + 4;
            CHECK(perPacket < 21 + 2 * rttTicks * 1.5);
        }
    }
}

// 高丢包、低延迟：一个输入在被确认前只来得及发一两次。确认号必须准确，否则丢掉的输入永远不再重发
static void testHighLossKeepsGoing() {
    for (uint64_t seed : {1, 2, 3}) {
        LockstepConfig c;
        c.units = 20, c.delay = 3, c.link = {4, 4, 0.4}, c.seed = seed;
        LockstepSession s(c);
        for (int t = 0; t < 3000; ++t) s.tick();
        uint32_t f = s.peers()[0].sim.frame;
        for (int t = 0; t < 600; ++t) s.tick();
        CHECK(s.peers()[0].sim.frame > f + 100);
        for (auto& p : s.peers()) CHECK(p.sim.hash() == reference(c, p.sim.frame).hash());
    }
}

// 抖动大、包乱序时，每个 tick 结束后 contiguous 都要等于从 have 暴力数出来的值。
// 要在不重发的模式下测：重发时，补上空缺的那个包也带着后面所有的帧，按顺序存进去，"每次只前进一帧"的错误实现也看不出来；
// 不重发时，补上空缺的包只有一帧，后面早已到达的帧必须一并算进去
static void testContiguousEveryTick() {
    LockstepConfig c;
    c.units = 10, c.delay = 8, c.resend = false, c.link = {20, 60, 0};
    LockstepSession s(c);
    long checked = 0;
    for (int t = 0; t < 600; ++t) {
        s.tick();
        for (auto& p : s.peers())
            for (int pl = 0; pl < c.peers; ++pl) {
                int64_t f = c.delay - 1;
                while (size_t(f + 1) < p.have.size() && p.have[size_t(f + 1)][size_t(pl)]) ++f;
                CHECK(p.contiguous[size_t(pl)] == f);
                ++checked;
            }
    }
    CHECK(checked == 600 * 16);
}

// 哈希覆盖全部状态：只有速度不同的两个状态，哈希也不同（位置相同的一刻看不出来，之后才会分开）
static void testHashCoversVelocity() {
    Arena a = Arena::make(1, 2, 10), b = a;
    b.units[3].vx.raw += 100;
    CHECK(a.hash() != b.hash());
    b = a, b.players[1].y.raw += 1;
    CHECK(a.hash() != b.hash());
}

// 不重发 + 丢包：丢掉的输入永远补不回来，所有机器卡死；不丢包时不重发也能正常运行
static void testNoResendDeadlocks() {
    LockstepConfig c;
    c.units = 50, c.delay = 4, c.resend = false, c.link = {30, 5, 0.05};
    LockstepSession s(c);
    for (int t = 0; t < 600; ++t) s.tick();
    uint32_t f = s.peers()[0].sim.frame;
    for (int t = 0; t < 600; ++t) s.tick();
    CHECK(s.peers()[0].sim.frame == f);  // 600 tick 没有前进一帧
    CHECK(f < 600);
    c.link.loss = 0;
    LockstepSession ok(c);
    for (int t = 0; t < 600; ++t) ok.tick();
    CHECK(ok.peers()[0].sim.frame == 600);
    CHECK(ok.peers()[0].stallTicks == 0);
}

// 输入延迟 ≥ 单程延迟（含抖动，按 tick 向上取整）时不停顿；太小则停顿
static void testDelayThreshold() {
    LockstepConfig c;
    c.units = 50, c.link = {30, 10, 0};  // 最多 40 ms：tick 末发出，第 3 个 tick 开头到达
    c.delay = 3;
    LockstepSession good(c);
    for (int t = 0; t < 1200; ++t) good.tick();
    for (auto& p : good.peers()) CHECK(p.stallTicks == 0 && p.sim.frame == 1200);
    c.delay = 2;
    LockstepSession bad(c);
    for (int t = 0; t < 1200; ++t) bad.tick();
    CHECK(bad.peers()[0].stallTicks > 100);
}

// 按下按键后，自己的角色恰好在 D 帧之后开始动（不停顿时）
static void testPressLatency() {
    LockstepConfig c;
    c.units = 20, c.delay = 5, c.link = {20, 0, 0}, c.pressAt = 300;
    LockstepSession s(c);
    Fx x0;
    long moved = -1;
    for (int t = 0; t < 400; ++t) {
        s.tick();
        const Arena& a = s.peers()[0].sim;
        if (t == 299) x0 = a.players[0].x;
        if (t >= 300 && moved < 0 && a.players[0].x != x0) moved = t;
    }
    CHECK(moved == 300 + 5);
    CHECK(s.peers()[0].sim.hash() == reference(c, s.peers()[0].sim.frame).hash());
}

// 哈希检查：一台机器的一个单位差一个最小单位，下一个检查点就会被发现
static void testDesyncDetection() {
    LockstepConfig c;
    c.units = 80, c.delay = 4, c.link = {30, 10, 0.02}, c.hashEvery = 60;
    LockstepSession s(c);
    for (int t = 0; t < 1200; ++t) {
        if (t == 500) s.corrupt(2);
        s.tick();
    }
    for (auto& p : s.peers()) {
        CHECK(p.desyncAt >= 540 && p.desyncAt <= 600);  // 第 500 帧附近被改，第一个检查点是 540
    }
    // 没被改的机器之间哈希一致：从参照来看，被改的是 2 号
    CHECK(s.peers()[0].sim.hash() == reference(c, s.peers()[0].sim.frame).hash());
    CHECK(s.peers()[2].sim.hash() != reference(c, s.peers()[2].sim.frame).hash());
}

// 差量编码：位图和索引列表两种模式，解码后与原状态逐位相同
static void testDeltaRoundTrip() {
    Pcg32 r{9};
    for (int t = 0; t < 300; ++t) {
        size_t units = 1 + r.next() % 700;
        QState base, s;
        base.frame = 10, s.frame = 13;
        for (int p = 0; p < 3; ++p) base.players.push_back({uint16_t(r.next()), uint16_t(r.next())}), s.players.push_back({uint16_t(r.next()), uint16_t(r.next())});
        uint32_t changePct = r.next() % 101;
        for (size_t i = 0; i < units; ++i) {
            QPos b{uint16_t(r.next()), uint16_t(r.next())};
            base.units.push_back(b);
            s.units.push_back(r.next() % 100 < changePct ? QPos{uint16_t(r.next()), uint16_t(r.next())} : b);
        }
        std::map<uint32_t, QState> baselines{{10, base}};
        QState out;
        auto d = encodeDelta(s, &base);
        CHECK(decodeDelta(d, 3, units, baselines, &out));
        CHECK(out == s);
        auto plain = encodeDelta(s, nullptr);
        CHECK(decodeDelta(plain, 3, units, {}, &out) && out == s);
        CHECK(d.size() <= plain.size());  // 全变了时退回完整的量化状态
        CHECK(plain.size() == 5 + 4 * (3 + units));
    }
    // 索引列表超过 255 项（单位很多、只有少数变化）
    {
        QState base, s2;
        base.frame = 1, s2.frame = 2;
        base.players = s2.players = {{1, 1}};
        for (int i = 0; i < 8000; ++i) base.units.push_back({uint16_t(i), 7});
        s2.units = base.units;
        for (int i = 0; i < 300; ++i) s2.units[size_t(i * 26)].y = 9;
        QState out;
        auto d = encodeDelta(s2, &base);
        CHECK(d[4] == 3);  // 列表模式
        CHECK(decodeDelta(d, 1, 8000, {{1, base}}, &out) && out == s2);
    }
    // 缺少基线时解码失败，而不是给出错误的状态
    QState s;
    s.frame = 1, s.players = {{1, 2}}, s.units = {{3, 4}};
    QState out;
    CHECK(!decodeDelta(encodeDeltaRaw(s, &s), 1, 1, {}, &out));
}

// 状态同步：在丢包、抖动、乱序下，客户端解码出的每个快照都等于服务器那一帧的量化状态
static void testStateSyncDecodes() {
    StateConfig c;
    c.units = 300, c.link = {60, 40, 0.1}, c.seed = 4;
    StateSession s(c);
    std::vector<int64_t> lastUsed(size_t(c.clients), -1);
    for (int t = 0; t < 1800; ++t) {
        s.tick();
        // 输入会乱序到达：服务器只能用更新的输入，不能退回旧的
        for (int i = 0; i < c.clients; ++i) {
            CHECK(s.inputFrame(i) >= lastUsed[size_t(i)]);
            lastUsed[size_t(i)] = s.inputFrame(i);
        }
    }
    for (auto& cl : s.clients()) {
        CHECK(cl.decodeFailures == 0);
        CHECK(cl.snaps.size() > 30);
        for (auto& [f, q] : cl.snaps) CHECK(s.truth().at(f) == q);
    }
    // 差量 < 量化 < 完整
    CHECK(s.encodedBytes(Encoding::Delta) < s.encodedBytes(Encoding::Quantized));
    CHECK(s.encodedBytes(Encoding::Quantized) < s.encodedBytes(Encoding::Full));
    CHECK(s.downBytes(0) * 4 <= s.encodedBytes(Encoding::Delta) + 4 * 100);  // 实际发送的就是差量
}

// 插值：没有抖动和丢包、插值延迟 ≥ 快照间隔时从不外推，误差很小；插值延迟为 0 时几乎总在外推
static void testInterpolation() {
    StateConfig c;
    c.units = 50, c.link = {40, 0, 0}, c.interpMs = 50;
    StateSession s(c);
    for (int t = 0; t < 1200; ++t) s.tick();
    for (auto& cl : s.clients()) {
        CHECK(cl.extrapolated == 0);
        CHECK(cl.errorSum / double(cl.rendered * 4) < 0.1);
    }
    c.interpMs = 0;
    StateSession zero(c);
    for (int t = 0; t < 1200; ++t) zero.tick();
    CHECK(zero.clients()[0].extrapolated > zero.clients()[0].rendered / 2);
    // 不用预测时，按键到看见自己动：上行 + 等快照 + 下行 + 插值延迟
    c.interpMs = 50, c.pressAt = 300;
    StateSession press(c);
    long moved = -1;
    double x0 = 0;
    for (int t = 0; t < 600; ++t) {
        press.tick();
        auto& v = press.clients()[0].players;
        if (v.empty()) continue;
        if (t == 299) x0 = v[0].x;
        if (t >= 300 && moved < 0 && std::fabs(v[0].x - x0) > 1e-9) moved = t;
    }
    // 40 + 40 + 50 ms = 8.4 帧，加上 tick 的取整和最多 3 帧的快照等待
    CHECK(moved >= 300 + 8 && moved <= 300 + 14);
}

int main() {
    testLink();
    testArena();
    testLockstepMatchesReference();
    testNoResendDeadlocks();
    testHighLossKeepsGoing();
    testHashCoversVelocity();
    testContiguousEveryTick();
    testDelayThreshold();
    testPressLatency();
    testDesyncDetection();
    testDeltaRoundTrip();
    testStateSyncDecodes();
    testInterpolation();
    return report("net_test");
}
