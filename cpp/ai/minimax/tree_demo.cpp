// Alpha-Beta 剪枝逐步演示：一棵 3 叉、3 层的博弈树，27 个叶子的分数站在 MAX 一方
//   - 上三角是 MAX（取最大），下三角是 MIN（取最小），方块是叶子
//   - 每一步：进入一个节点 / 叶子估值 / 子节点的值返回给父节点 / 剪掉剩下的子节点
//   - 左侧显示当前路径上每一层的 alpha（MAX 已有把握的分数）和 beta（MIN 已有把握的分数）
//   - minimax：不剪枝，看完全部 27 个叶子；best first / worst first：把每个节点的子节点按真实值重新排序

#include <algorithm>
#include <climits>
#include <cstdio>
#include <string>
#include <vector>

#include "../../foundations/random/random.h"
#include "demo.h"
#include "ui.h"

namespace {

using namespace ga;

constexpr int kWidth = 800, kHeight = 480;
constexpr float kTop = 52;
constexpr int kB = 3, kDepth = 3, kLeaves = 27, kNodes = 40;
constexpr float kTreeX0 = 78, kTreeX1 = 790;
constexpr float kLevelY[4] = {96, 184, 272, 356};
constexpr int kNegInf = INT_MIN / 2, kPosInf = INT_MAX / 2;

int levelOf(int n) { return n == 0 ? 0 : n < 4 ? 1 : n < 13 ? 2 : 3; }
int firstChild(int n) { return n * kB + 1; }
bool isMax(int n) { return levelOf(n) % 2 == 0; }

enum class Ev { Enter, Leaf, Update, Prune };
struct Event {
    Ev type;
    int node, value, alpha, beta, child;  // Update：child 是刚返回的子节点；Prune：child 是第一个被剪掉的子节点
};

struct NodeView {
    int state = 0;  // 0 未访问，1 在当前路径上，2 已完成，3 被剪掉
    int value = 0;
    bool hasValue = false;
    int alpha = kNegInf, beta = kPosInf;
};

std::string fmt(int v) { return v <= kNegInf ? "-inf" : v >= kPosInf ? "+inf" : std::to_string(v); }

class TreeDemo final : public Demo {
public:
    TreeDemo() {
        newTree();
        started_ = true;  // 之后的修改才需要通知框架（构造时框架还没有运行）
    }

    void reset() override { shown_ = 0; }
    bool step() override {
        if (shown_ >= int(events_.size())) return false;
        ++shown_;
        return shown_ < int(events_.size());
    }

    void draw() const override {
        const Palette& pal = palette();
        const float hl = hairline();
        std::vector<NodeView> v = replay();
        const Event* last = shown_ > 0 ? &events_[shown_ - 1] : nullptr;
        int current = currentNode(v);

        // 边：从父节点画到子节点
        for (int n = 1; n < kNodes; ++n) {
            int p = (n - 1) / kB;
            Vector2 a = pos(p), b = pos(n);
            a.y += isMax(p) ? 14 : 4, b.y -= isMax(n) ? 12 : (levelOf(n) == 3 ? 11 : 14);
            if (v[n].state == 3) {
                drawDashedLine(a, b, hl * 1.5f, Fade(pal.textMuted, 0.6f), 4, 4);
            } else {
                bool active = v[n].state == 1;
                DrawLineEx(a, b, active ? 2.5f : (v[n].state == 2 ? 1.5f : hl * 1.5f), active ? pal.path : v[n].state == 2 ? pal.text : pal.gridLine);
            }
        }
        // 剪刀记号：被剪掉的第一棵子树的边上画两道短线
        for (int n = 1; n < kNodes; ++n) {
            int p = (n - 1) / kB;
            if (v[n].state != 3 || (v[p].state == 3)) continue;
            Vector2 a = pos(p), b = pos(n), m{(a.x + b.x) / 2, (a.y + b.y) / 2};
            for (int k = -1; k <= 1; k += 2)
                DrawLineEx({m.x - 7, m.y + k * 3 + 3}, {m.x + 7, m.y + k * 3 - 3}, 2, pal.path);
        }

        for (int n = 0; n < kNodes; ++n) drawNode(n, v[n], n == current);

        // 左侧：层标签与当前路径上的 alpha / beta
        const char* names[4] = {"MAX", "MIN", "MAX", "leaf"};
        int path[4] = {-1, -1, -1, -1};
        for (int n = 0; n < kNodes; ++n)
            if (v[n].state == 1) path[levelOf(n)] = n;
        for (int l = 0; l < 4; ++l) {
            float y = kLevelY[l];
            drawText(names[l], {12, y - 20}, 12, pal.text);
            if (l < 3 && path[l] >= 0) {
                drawText(("a " + fmt(v[path[l]].alpha)).c_str(), {12, y - 3}, 11, pal.textMuted);
                drawText(("b " + fmt(v[path[l]].beta)).c_str(), {12, y + 11}, 11, pal.textMuted);
            }
        }

        // 底部：这一步发生了什么，以及计数
        DrawLineEx({12, 398}, {kWidth - 12.0f, 398}, hl, pal.gridLine);
        drawText(describe(last, v).c_str(), {12, 408}, 13, pal.text);
        int evaluated = 0, pruned = 0;
        for (int n = 13; n < kNodes; ++n) evaluated += v[n].state == 2 || v[n].state == 1, pruned += v[n].state == 3;
        char buf[160];
        std::snprintf(buf, sizeof buf, "leaves evaluated %d / %d   pruned %d   (best possible with perfect ordering: 11)", evaluated, kLeaves, pruned);
        drawText(buf, {12, 432}, 12, pal.textMuted);
        drawText("a = alpha: MAX is already sure of at least this    b = beta: MIN can already hold MAX to at most this", {12, 452}, 11,
                 pal.textMuted);
        DrawLineEx({0, kTop}, {float(kWidth), kTop}, hl, pal.gridLine);
    }

    void drawUi() override {
        if (ui::button({12, 13, 92, 26}, "alpha-beta", prune_)) setMode(true, order_);
        if (ui::button({108, 13, 80, 26}, "minimax", !prune_)) setMode(false, order_);
        const char* orders[] = {"as generated", "best first", "worst first"};
        const float xs[] = {232, 352, 456}, ws[] = {116, 100, 108};
        for (int k = 0; k < 3; ++k)
            if (ui::button({xs[k], 13, ws[k], 26}, orders[k], order_ == k)) setMode(prune_, k);
        if (ui::button({696, 13, 92, 26}, "new tree")) newTree();
    }

    std::string status() const override {
        char buf[96];
        std::snprintf(buf, sizeof buf, "step %d / %d", shown_, int(events_.size()));
        return buf;
    }

private:
    void newTree() {
        rng_ = Pcg32(seed_++);
        for (int& x : base_) x = int(rng_.below(21));
        setMode(prune_, order_);
    }

    void setMode(bool prune, int order) {
        prune_ = prune, order_ = order;
        leaves_ = base_;
        if (order_ > 0) reorder(0, 0);
        events_.clear();
        search(0, kNegInf, kPosInf);
        if (started_) notifyProblemChanged();
    }

    // 子树 n 的真实 minimax 值（对 leaves_ 当前的排列）
    int value(int n) const {
        if (levelOf(n) == kDepth) return leaves_[n - 13];
        int best = isMax(n) ? kNegInf : kPosInf;
        for (int k = 0; k < kB; ++k) {
            int c = value(firstChild(n) + k);
            best = isMax(n) ? std::max(best, c) : std::min(best, c);
        }
        return best;
    }

    // 把节点 n 的子树按真实值排序（best first：对轮到走的一方最好的在前），叶子跟着整棵子树移动
    void reorder(int n, int) {
        if (levelOf(n) >= kDepth) return;
        for (int k = 0; k < kB; ++k) reorder(firstChild(n) + k, 0);
        int span = 1;
        for (int l = levelOf(n) + 1; l < kDepth; ++l) span *= kB;
        std::vector<std::pair<int, std::vector<int>>> kids;
        for (int k = 0; k < kB; ++k) {
            int c = firstChild(n) + k, lo = leftmostLeaf(c);
            kids.push_back({value(c), std::vector<int>(leaves_.begin() + lo, leaves_.begin() + lo + span)});
        }
        bool bestFirst = order_ == 1;
        std::stable_sort(kids.begin(), kids.end(), [&](auto& a, auto& b) {
            return (isMax(n) == bestFirst) ? a.first > b.first : a.first < b.first;
        });
        int lo = leftmostLeaf(firstChild(n));
        for (auto& [val, ls] : kids) std::copy(ls.begin(), ls.end(), leaves_.begin() + lo), lo += span;
    }
    static int leftmostLeaf(int n) {
        while (levelOf(n) < kDepth) n = firstChild(n);
        return n - 13;
    }

    // 两方写法的 Alpha-Beta，记录每一步供逐步播放
    int search(int n, int alpha, int beta) {
        events_.push_back({Ev::Enter, n, 0, alpha, beta, -1});
        if (levelOf(n) == kDepth) {
            int v = leaves_[n - 13];
            events_.push_back({Ev::Leaf, n, v, alpha, beta, -1});
            return v;
        }
        int best = isMax(n) ? kNegInf : kPosInf;
        for (int k = 0; k < kB; ++k) {
            int c = firstChild(n) + k;
            int v = search(c, alpha, beta);
            if (isMax(n)) best = std::max(best, v), alpha = std::max(alpha, best);
            else best = std::min(best, v), beta = std::min(beta, best);
            events_.push_back({Ev::Update, n, best, alpha, beta, c});
            if (prune_ && alpha >= beta && k + 1 < kB) {
                events_.push_back({Ev::Prune, n, best, alpha, beta, c + 1});
                break;
            }
        }
        return best;
    }

    std::vector<NodeView> replay() const {
        std::vector<NodeView> v(kNodes);
        for (int i = 0; i < shown_; ++i) {
            const Event& e = events_[i];
            NodeView& nv = v[e.node];
            switch (e.type) {
                case Ev::Enter: nv.state = 1, nv.alpha = e.alpha, nv.beta = e.beta; break;
                case Ev::Leaf: nv.value = e.value, nv.hasValue = true; break;
                case Ev::Update:
                    v[e.child].state = 2;
                    nv.value = e.value, nv.hasValue = true, nv.alpha = e.alpha, nv.beta = e.beta;
                    break;
                case Ev::Prune:
                    for (int c = e.child; c < firstChild(e.node) + kB; ++c) markPruned(v, c);
                    break;
            }
        }
        if (shown_ == int(events_.size()) && shown_ > 0) v[0].state = 2;
        return v;
    }
    static void markPruned(std::vector<NodeView>& v, int n) {
        v[n].state = 3;
        if (levelOf(n) < kDepth)
            for (int k = 0; k < kB; ++k) markPruned(v, firstChild(n) + k);
    }
    // 当前节点：路径上最深的节点
    static int currentNode(const std::vector<NodeView>& v) {
        int cur = -1;
        for (int n = 0; n < kNodes; ++n)
            if (v[n].state == 1 && (cur < 0 || levelOf(n) > levelOf(cur))) cur = n;
        return cur;
    }

    static Vector2 pos(int n) {
        int l = levelOf(n);
        if (l == kDepth) {
            float w = (kTreeX1 - kTreeX0) / kLeaves;
            return {kTreeX0 + (n - 13 + 0.5f) * w, kLevelY[3]};
        }
        float x = 0;
        for (int k = 0; k < kB; ++k) x += pos(firstChild(n) + k).x;
        return {x / kB, kLevelY[l]};
    }

    void drawNode(int n, const NodeView& v, bool current) const {
        const Palette& pal = palette();
        Vector2 c = pos(n);
        bool pruned = v.state == 3, done = v.state == 2;
        Color ink = pruned ? Fade(pal.textMuted, 0.5f) : pal.text;
        Color fill = current ? pal.frontier : done ? pal.text : pal.background;
        Color label = done && !current ? pal.background : pal.text;
        if (levelOf(n) == kDepth) {
            Rectangle r{c.x - 10, c.y - 10, 20, 20};
            DrawRectangleRec(r, pruned ? pal.background : fill);
            DrawRectangleLinesEx(r, 1.2f, ink);
            Color t = pruned ? Fade(pal.textMuted, 0.6f) : v.hasValue ? label : Fade(pal.textMuted, 0.8f);
            drawTextCentered(std::to_string(leaves_[n - 13]).c_str(), c, 11, t);
            if (pruned) DrawLineEx({c.x - 8, c.y + 8}, {c.x + 8, c.y - 8}, 1, Fade(pal.textMuted, 0.6f));
            return;
        }
        float hw = levelOf(n) == 2 ? 15 : 18, h = levelOf(n) == 2 ? 24 : 28;
        Vector2 p1, p2, p3;
        if (isMax(n)) p1 = {c.x, c.y - h * 0.55f}, p2 = {c.x - hw, c.y + h * 0.45f}, p3 = {c.x + hw, c.y + h * 0.45f};
        else p1 = {c.x - hw, c.y - h * 0.45f}, p2 = {c.x, c.y + h * 0.55f}, p3 = {c.x + hw, c.y - h * 0.45f};
        if (!pruned) DrawTriangle(p1, p2, p3, fill);
        DrawTriangleLines(p1, p2, p3, ink);
        DrawLineEx(p1, p2, 1.2f, ink), DrawLineEx(p2, p3, 1.2f, ink), DrawLineEx(p3, p1, 1.2f, ink);
        if (v.hasValue && !pruned) {
            Vector2 tc{c.x, c.y + (isMax(n) ? 4.0f : -4.0f)};
            drawTextCentered(fmt(v.value).c_str(), tc, 11, label);
        }
    }

    std::string describe(const Event* e, const std::vector<NodeView>&) const {
        if (!e) return prune_ ? "press play or step: alpha-beta searches the tree left to right"
                              : "press play or step: minimax visits every node";
        const char* who = isMax(e->node) ? "MAX" : "MIN";
        char buf[200];
        switch (e->type) {
            case Ev::Enter:
                if (levelOf(e->node) == kDepth) std::snprintf(buf, sizeof buf, "visit a leaf (window a %s, b %s)", fmt(e->alpha).c_str(), fmt(e->beta).c_str());
                else std::snprintf(buf, sizeof buf, "enter a %s node with window a %s, b %s", who, fmt(e->alpha).c_str(), fmt(e->beta).c_str());
                break;
            case Ev::Leaf: std::snprintf(buf, sizeof buf, "leaf evaluates to %d", e->value); break;
            case Ev::Update: {
                int childValue = value(e->child);
                if (isMax(e->node))
                    std::snprintf(buf, sizeof buf, "child returns %d -> MAX keeps the larger: %s, a = %s", childValue, fmt(e->value).c_str(), fmt(e->alpha).c_str());
                else
                    std::snprintf(buf, sizeof buf, "child returns %d -> MIN keeps the smaller: %s, b = %s", childValue, fmt(e->value).c_str(), fmt(e->beta).c_str());
                if (e->node == 0 && shown_ == int(events_.size())) std::snprintf(buf, sizeof buf, "done: the root's value is %d", e->value);
                break;
            }
            case Ev::Prune:
                if (isMax(e->node))
                    std::snprintf(buf, sizeof buf, "a %s >= b %s: MIN above will never let play reach here. prune the rest", fmt(e->alpha).c_str(), fmt(e->beta).c_str());
                else
                    std::snprintf(buf, sizeof buf, "b %s <= a %s: MAX above already has better. prune the rest", fmt(e->beta).c_str(), fmt(e->alpha).c_str());
                break;
        }
        return buf;
    }

    std::vector<int> base_ = std::vector<int>(kLeaves), leaves_ = std::vector<int>(kLeaves);
    std::vector<Event> events_;
    int shown_ = 0, order_ = 0;
    bool prune_ = true, started_ = false;
    uint64_t seed_ = 3;
    Pcg32 rng_{3};
};

}  // namespace

int main() {
    static TreeDemo demo;
    ga::DemoConfig config;
    config.title = "Alpha-beta pruning";
    config.width = kWidth;
    config.height = kHeight;
    config.stepsPerSecond = 2.5f;
    return ga::runDemo(demo, config);
}
