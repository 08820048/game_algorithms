#include "command.h"

#include <cstdio>
#include <vector>

#include "check.h"

using namespace ga;
using namespace ga::command;

static Recording makeRecording(uint64_t seed, int balls, int frames) {
    Recording rec{seed, balls, {}};
    Pcg32 r{seed + 100};
    Input cur;
    for (int f = 0; f < frames; ++f) {
        if (r.next() % 20 == 0) cur = {int8_t(int(r.next() % 3) - 1), int8_t(int(r.next() % 3) - 1), r.next() % 4 == 0};
        rec.inputs.push_back(cur);
    }
    return rec;
}

// 回放逐位相同；游程编码可逆
static void testReplay() {
    for (uint64_t seed : {1, 2, 3}) {
        Recording rec = makeRecording(seed, 40, 3000);
        Sim a = rec.replay(3000), b = rec.replay(3000);
        CHECK(a.hash() == b.hash());
        auto runs = rec.compress();
        CHECK(Recording::decompress(runs) == rec.inputs);
        CHECK(runs.size() < rec.inputs.size() / 5);
        // 换掉一秒的输入，结果就不同（只改一帧不一定有影响：推杆可能正顶着墙）
        Recording other = rec;
        for (int f = 1500; f < 1560; ++f) other.inputs[f] = {int8_t(f % 2 ? 1 : -1), 1, true};
        CHECK(other.replay(3000).hash() != a.hash());
        // 少走一帧，结果不同（帧号也在哈希里）
        CHECK(rec.replay(2999).hash() != a.hash());
    }
}

// 关键帧跳转与从头回放逐位相同，模拟的帧数不超过 K − 1
static void testSeek() {
    Recording rec = makeRecording(4, 30, 2000);
    Pcg32 r{5};
    for (int every : {1, 7, 60, 500}) {
        Seeker sk(rec, every);
        CHECK(sk.keyframes() == 2000 / size_t(every) + 1);
        for (int k = 0; k < 30; ++k) {
            size_t n = r.next() % 2001;
            long steps = 0;
            Sim s = sk.seek(n, &steps);
            CHECK(s.hash() == rec.replay(n).hash());
            CHECK(steps == long(n % size_t(every)));
        }
    }
}

// 混沌：初始位置差 1e-4 像素，几百帧后差距超过一个球的半径
static void testChaos() {
    Recording rec = makeRecording(6, 40, 3000);
    Sim a = Sim::make(6, 40), b = a;
    b.balls[0].p.x += 1e-4f;
    int diverged = -1;
    for (int f = 0; f < 3000 && diverged < 0; ++f) {
        a.step(rec.inputs[f]), b.step(rec.inputs[f]);
        if (a.maxDistance(b) > Sim::kR) diverged = f;
    }
    std::printf("perturbation 1e-4 px: positions differ by more than a ball radius after %d frames\n", diverged);
    CHECK(diverged > 0 && diverged < 3000);
}

// 撤销：三种做法与参照（每一步的完整副本）对照，随机执行 / 撤销 / 重做 / 在撤销后执行新的修改
static void testUndo() {
    Pcg32 r{7};
    Grid g(64, 48), gs(64, 48), gr(64, 48);
    UndoStack us;
    SnapshotStack ss(gs);
    ReplayStack rs(gr, 8);
    std::vector<Grid> ref{g};  // ref[0..top] 是已执行的历史，top 之后是可重做的
    size_t top = 0;
    long undos = 0, redos = 0;
    for (int step = 0; step < 4000; ++step) {
        uint32_t op = r.next() % 10;
        if (op < 6) {
            std::vector<Edit> edits;
            int n = 1 + int(r.next() % 40);
            uint32_t c = r.next() % uint32_t(g.cells.size());
            for (int i = 0; i < n; ++i) {
                c = (c + r.next() % 3) % uint32_t(g.cells.size());  // 一笔里经常画到同一个格子
                edits.push_back({c, uint8_t(1 + r.next() % 7)});
            }
            us.apply(g, edits, true), ss.apply(gs, edits), rs.apply(gr, edits);
            ref.erase(ref.begin() + long(top) + 1, ref.end()), ref.push_back(g), ++top;
        } else if (op < 9) {
            bool a = us.undo(g), b = ss.undo(gs), c = rs.undo(gr);
            CHECK(a == (top > 0) && b == a && c == a);
            if (top > 0) --top, ++undos;
        } else {
            bool a = us.redo(g);
            CHECK(a == (top + 1 < ref.size()));
            if (a) {
                ++top, ++redos;
                // 另外两种不支持重做：把重做的这一笔当作新的修改执行，它们的历史仍然等于 ref[0..top]
                const Stroke& st = us.history()[top - 1];
                std::vector<Edit> edits;
                for (size_t i = 0; i < st.cells.size(); ++i) edits.push_back({st.cells[i], st.after[i]});
                ss.apply(gs, edits), rs.apply(gr, edits);
            }
        }
        CHECK(g == ref[top]);
        CHECK(gs == g && gr == g);
    }
    CHECK(undos > 500 && redos > 50);
}

// 合并：一笔里同一个格子画了多次，只存一条记录，撤销恢复到画之前的值
static void testMerge() {
    Grid g(8, 8);
    g.at(3, 3) = 5;
    UndoStack us;
    std::vector<Edit> e;
    for (int i = 0; i < 10; ++i) e.push_back({3 * 8 + 3, uint8_t(i + 1)});
    us.apply(g, e, true);
    CHECK(us.history()[0].cells.size() == 1 && g.at(3, 3) == 10);
    us.undo(g);
    CHECK(g.at(3, 3) == 5);
    us.redo(g);
    CHECK(g.at(3, 3) == 10);
    UndoStack raw;
    raw.apply(g, e, false);
    CHECK(raw.history()[0].cells.size() == 10);
    raw.undo(g);
    CHECK(g.at(3, 3) == 10);  // 撤销恢复到这一笔之前（之前是 10）
    // 撤销后执行新的修改：重做分支作废
    us = UndoStack{};
    us.apply(g, {{0, 1}}), us.apply(g, {{1, 1}}), us.undo(g), us.apply(g, {{2, 1}});
    CHECK(us.total() == 2 && !us.redo(g));
}

// 哈希覆盖全部状态：只有帧号不同的两个状态，哈希也不同（帧号决定下一次随机冲量的时刻）。
// 模拟自己的随机数每次使用后都前进
static void testStateCoverage() {
    Sim a = Sim::make(8, 0), b = a;
    a.step({}), b.step({}), b.step({});
    CHECK(a.balls.empty() && a.pusher == b.pusher && a.rngState == b.rngState);
    CHECK(a.hash() != b.hash());
    Sim s = Sim::make(9, 10);
    std::vector<uint64_t> states{s.rngState};
    for (int f = 0; f < 240 * 4; ++f) {
        s.step({});
        if (f % 240 == 239) states.push_back(s.rngState);
    }
    for (size_t i = 0; i < states.size(); ++i)
        for (size_t j = i + 1; j < states.size(); ++j) CHECK(states[i] != states[j]);
}

int main() {
    testStateCoverage();
    testReplay();
    testSeek();
    testChaos();
    testUndo();
    testMerge();
    return report("command_test");
}
