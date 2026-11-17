#include "predict.h"

#include <cmath>
#include <cstdio>

#include "check.h"

using namespace ga;
using namespace ga::net;

// 走一步 vs 暴力：以 1/256 像素为步长一点点挪，挪到"新撞上"某个障碍物之前为止
static void testMoveMatchesSweep() {
    Pcg32 r{5};
    auto q = [&](int lo, int hi) { return Fx::fromRaw(int32_t((lo * 4 + int(r.below(uint32_t((hi - lo) * 4)))) << 14)); };  // 1/4 像素的倍数
    const Fx h = Fx::fromInt(Field::kHalf);
    int stopped = 0;
    for (int trial = 0; trial < 20000; ++trial) {
        std::vector<Box> obs;
        for (int k = 0; k < 4; ++k) {
            Fx x = q(0, 60), y = q(0, 60);
            obs.push_back({x, y, x + q(2, 20), y + q(2, 20)});
        }
        Field::Player me{q(10, 50), q(10, 50)};
        Input in{int8_t(int(r.below(3)) - 1), int8_t(int(r.below(3)) - 1)};
        Field::Player got = movePlayer(me, in, obs);
        Field::Player ref = me;
        auto sweep = [&](bool isX, int d) {
            if (!d) return;
            Field::Player start = ref;
            for (int k = 1; k <= 2 * 256; ++k) {
                Field::Player c = start;
                (isX ? c.x : c.y) += Fx::fromRaw(d * k * 256);
                bool blocked = false;
                for (auto& o : obs) blocked |= !overlaps(Field::boxOf(start), o) && overlaps(Field::boxOf(c), o);
                if (blocked) break;
                ref = c;
            }
            ref.x = std::clamp(ref.x, h, Fx::fromInt(Field::kW) - h), ref.y = std::clamp(ref.y, h, Fx::fromInt(Field::kH) - h);
        };
        sweep(true, in.dx), sweep(false, in.dy);
        CHECK(got == ref);
        stopped += !(got == me) && std::abs((got.x - me.x).raw) + std::abs((got.y - me.y).raw) < (Field::kSpeed << 16) * (std::abs(in.dx) + std::abs(in.dy));
    }
    CHECK(stopped > 1000);  // 确实有很多次撞上了障碍物
}

// 只有一个玩家（没有别人挡路）：和解后的预测和服务器逐位相同，从来不用纠正；即使有抖动和丢包
static void testReconcileIsExactAlone() {
    for (LinkConfig link : {LinkConfig{30, 10, 0.05}, LinkConfig{125, 50, 0.1}}) {
        PredictConfig c;
        c.players = 1, c.link = link, c.mode = Mode::Reconcile;
        PredictSession s(c);
        for (int t = 0; t < 3600; ++t) s.tick();
        const auto& cl = s.clients()[0];
        CHECK(cl.corrections == 0);
        long compared = 0;
        for (size_t t = 0; t < cl.shownHistory.size(); ++t) {
            Field::Player p;
            if (!s.serverAfter(0, uint32_t(t), &p)) continue;
            ++compared;
            CHECK(cl.shownHistory[t].first == p.x.toDouble() && cl.shownHistory[t].second == p.y.toDouble());
        }
        CHECK(compared > 3500);
        // 不和解：每个快照都把还没处理的输入"抹掉"
        c.mode = Mode::Predict;
        PredictSession s2(c);
        for (int t = 0; t < 3600; ++t) s2.tick();
        CHECK(s2.clients()[0].corrections > 100);
    }
}

// 服务器把每个输入恰好处理一次、按序号顺序：它算出的位置 = 从出生点开始依次走这些输入
static void testServerAppliesEachInputOnce() {
    PredictConfig c;
    c.players = 1, c.link = {75, 60, 0.2};
    PredictSession s(c);
    for (int t = 0; t < 2000; ++t) s.tick();
    Field f = Field::make(1);
    Field::Player p = f.players[0];
    const auto& in = s.clients()[0].inputs;
    uint32_t n = 0;
    Field::Player got;
    for (uint32_t q = 0; q < in.size(); ++q) {
        p = movePlayer(p, in[q], f.walls);
        if (!s.serverAfter(0, q, &got)) break;
        CHECK(got == p);
        ++n;
    }
    CHECK(n > 1900);
    CHECK(s.server().players[0] == got);
}

// 别人站着不动时，撞上他们也能预测得分毫不差（障碍物就是快照里的位置）
static void testIdleOthersPredictedExactly() {
    PredictConfig c;
    c.players = 16, c.othersIdle = true, c.link = {60, 20, 0.05};
    PredictSession s(c);
    long touching = 0;
    for (int t = 0; t < 7200; ++t) {
        s.tick();
        const auto& ps = s.server().players;
        for (size_t k = 1; k < ps.size(); ++k) {
            Box a = Field::boxOf(ps[0]), b = Field::boxOf(ps[k]);
            bool ox = a.x0 <= b.x1 && b.x0 <= a.x1, oy = a.y0 <= b.y1 && b.y0 <= a.y1;
            touching += ox && oy;  // 贴在一起（不会重叠）
        }
    }
    CHECK(touching > 100);
    CHECK(s.clients()[0].corrections == 0);
}

// 有别人挡路时会预测错（客户端只知道别人过去的位置），而且人越多错得越多
static void testOthersCauseCorrections() {
    long prev = -1;
    for (int players : {2, 8, 16}) {
        PredictConfig c;
        c.players = players, c.link = {30, 10, 0.01};
        PredictSession s(c);
        for (int t = 0; t < 3600; ++t) s.tick();
        long corr = 0;
        for (auto& cl : s.clients()) corr += cl.corrections;
        CHECK(corr > prev);
        prev = corr;
    }
}

// 按键到看见：预测时就在按下的那一帧；不预测时 = 上行 + 等快照 + 下行 + 插值延迟
static void testPressToSee() {
    for (Mode m : {Mode::None, Mode::Reconcile}) {
        PredictConfig c;
        c.mode = m, c.link = {40, 0, 0}, c.pressAt = 300;
        PredictSession s(c);
        double x0 = 0;
        long moved = -1;
        for (int t = 0; t < 600; ++t) {
            s.tick();
            if (t == 299) x0 = s.clients()[0].shownX;
            if (t >= 300 && moved < 0 && s.clients()[0].shownX != x0) moved = t;
        }
        if (m == Mode::Reconcile) CHECK(moved == 300);
        // 40 + 40 + 100 ms = 10.8 帧，加上 tick 的取整和最多 3 帧的快照等待
        else CHECK(moved >= 300 + 10 && moved <= 300 + 16);
    }
}

// 平滑：显示位置和预测位置的差在纠正后逐渐消失；一帧之内的最大跳动比不平滑时小
static void testSmoothing() {
    double maxStep[2] = {};
    for (int k = 0; k < 2; ++k) {
        PredictConfig c;
        c.players = 8, c.link = {30, 10, 0.01}, c.mode = k ? Mode::Smooth : Mode::Reconcile;
        PredictSession s(c);
        double settled = 0;
        for (int t = 0; t < 3600; ++t) {
            s.tick();
            const auto& cl = s.clients()[0];
            if (cl.shownHistory.size() >= 2) {
                auto a = cl.shownHistory[cl.shownHistory.size() - 2], b = cl.shownHistory.back();
                maxStep[k] = std::max(maxStep[k], std::hypot(b.first - a.first, b.second - a.second));
            }
            settled = std::hypot(cl.shownX - cl.predicted.x.toDouble(), cl.shownY - cl.predicted.y.toDouble());
            if (k == 0) CHECK(settled == 0);
        }
        CHECK(s.clients()[0].corrections > 10);
        // 最后 1 秒没有纠正时偏移已经消失；有的话也小于一个身位
        CHECK(settled < 16);
    }
    CHECK(maxStep[1] < maxStep[0]);
    CHECK(maxStep[1] < 8);
}

// 回滚的结果和单机逐帧计算完全相同：所有输入都确认后，每台机器第 60k 帧的状态哈希都等于单机的
// 输入每 7 帧就变一次时（和每 60 帧一次的哈希错开），大量的猜测是错的
static void testRollbackMatchesReference() {
    for (int period : {40, 7})
    for (int d : {0, 2})
        for (LinkConfig link : {LinkConfig{30, 10, 0.05}, LinkConfig{75, 30, 0.1}}) {
            RollbackConfig c;
            c.delay = d, c.link = link, c.window = 12, c.botPeriod = period;
            RollbackSession s(c);
            for (int t = 0; t < 1800; ++t) s.tick();
            Arena ref = Arena::make(c.seed, c.peers, c.units);
            std::map<uint32_t, uint64_t> want;
            std::vector<Input> in(size_t(c.peers));
            for (uint32_t f = 0; f < 1800; ++f) {
                for (int p = 0; p < c.peers; ++p) in[size_t(p)] = f < uint32_t(d) ? Input{} : botInputEvery(c.seed, p, f, uint32_t(period));
                ref.step(in.data());
                if (ref.frame % 60 == 0) want[ref.frame] = ref.hash();
            }
            for (auto& p : s.peers()) {
                CHECK(p.ringMisses == 0);
                CHECK(p.rollbacks > 10);
                CHECK(p.confirmedHashes.size() >= 25);
                for (auto [f, h] : p.confirmedHashes) CHECK(want.at(f) == h);
            }
        }
    CHECK(botInputEvery(3, 1, 777, 40) == botInput(3, 1, 777));
}

// 猜对了就不回滚：输入从不变化时，只有每个对方第一个输入到达前猜的 0 会错
static void testNoRollbackWhenGuessRight() {
    RollbackConfig c;
    c.botPeriod = 1 << 30, c.link = {50, 20, 0.05};
    RollbackSession s(c);
    for (int t = 0; t < 1200; ++t) s.tick();
    for (auto& p : s.peers()) {
        CHECK(p.guessed > 1000);
        CHECK(p.wrongGuesses <= 3 * 10);
        CHECK(p.rollbacks <= 3);
    }
}

// 窗口 0 就是帧同步：从不猜；D + 窗口小于单程延迟时会停顿，够大时不停顿
static void testWindow() {
    RollbackConfig c;
    c.window = 0, c.delay = 6, c.link = {30, 10, 0.01};
    RollbackSession s(c);
    for (int t = 0; t < 1200; ++t) s.tick();
    for (auto& p : s.peers()) CHECK(p.guessed == 0 && p.rollbacks == 0 && p.maxStepsPerTick <= 3);
    // 单程最多 70 ms = 4.2 帧
    for (auto [d, w, stalls] : {std::tuple{0, 2, true}, std::tuple{2, 1, true}, std::tuple{0, 8, false}, std::tuple{3, 3, false}}) {
        RollbackConfig k;
        k.delay = d, k.window = w, k.link = {50, 20, 0};
        RollbackSession r(k);
        for (int t = 0; t < 1200; ++t) r.tick();
        long st = 0;
        for (auto& p : r.peers()) st += p.stallTicks;
        CHECK((st > 100) == stalls);
        if (!stalls) CHECK(st == 0);
        // 一个 tick 里最多重算 窗口 帧，再往前算 3 帧
        for (auto& p : r.peers()) CHECK(p.maxStepsPerTick <= w + 3);
    }
}

// 视线 vs 暴力：沿视线密集取点，看有没有落在墙里
static void testVisibility() {
    Pcg32 r{9};
    int vis = 0;
    for (int i = 0; i < 20000; ++i) {
        double x = r.nextDouble() * Range::kW, y = Range::kTargetY + (r.nextDouble() - 0.5) * 40;
        bool blocked = false;
        for (int k = 0; k <= 20000; ++k) {
            double t = k / 20000.0, px = Range::kShooterX + (x - Range::kShooterX) * t, py = Range::kShooterY + (y - Range::kShooterY) * t;
            blocked |= py >= Range::kWallY0 && py <= Range::kWallY1 && (px < Range::kGap0 || px > Range::kGap1);
        }
        bool v = Range::visible(x, y);
        // 视线正好擦过墙角时取点可能漏掉，允许 0.05 像素内的分歧
        if (v == blocked) {
            bool nearCorner = false;
            for (double dx : {-0.05, 0.05}) nearCorner |= Range::visible(x + dx, y) != v;
            CHECK(nearCorner);
        }
        vis += v;
    }
    CHECK(vis > 3000 && vis < 12000);
}

// 延迟补偿：快照每帧都发、没有瞄准误差时，退回去的目标正好在射手看到的位置；回退不超过上限
static void testLagCompRewindsExactly() {
    LagCompConfig c;
    c.link = {60, 0, 0}, c.sendEvery = 1, c.aimError = 0;
    LagCompSession s(c);
    for (int t = 0; t < 36000; ++t) s.tick();
    long hitNow = 0;
    for (auto& r : s.results()) {
        CHECK(std::fabs(r.backX - r.shot.x) < 1e-9);
        CHECK(r.hitBack);
        hitNow += r.hitNow;
        if (r.behindCover) CHECK(!Range::visible(r.nowX, Range::kTargetY) && r.coverMs > 0);
    }
    CHECK(s.results().size() > 300);
    CHECK(hitNow < long(s.results().size()) / 2);  // 不补偿时一大半打不中

    c.maxRewindMs = 150;
    LagCompSession capped(c);
    for (int t = 0; t < 36000; ++t) capped.tick();
    long hit = 0;
    for (auto& r : capped.results()) {
        hit += r.hitBack;
        double earliest = r.serverFrame - 150.0 * 60 / 1000;
        CHECK(std::fabs(r.backX - capped.targetAt(std::max(r.shot.renderFrame, earliest))) < 1e-12);
    }
    CHECK(hit < long(capped.results().size()) * 9 / 10);  // 需要回退 60 + 60 + 100 ms > 150 ms
}

int main() {
    testMoveMatchesSweep();
    testReconcileIsExactAlone();
    testServerAppliesEachInputOnce();
    testIdleOthersPredictedExactly();
    testOthersCauseCorrections();
    testPressToSee();
    testSmoothing();
    testRollbackMatchesReference();
    testNoRollbackWhenGuessRight();
    testWindow();
    testVisibility();
    testLagCompRewindsExactly();
    return report("predict_test");
}
