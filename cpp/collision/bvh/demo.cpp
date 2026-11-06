// 动态 BVH 与 Sweep and Prune 演示
//   bvh：盒子运动，树增量更新。虚线是叶子的胖盒子，物体跑出胖盒子才重新插入（黄色闪一下）；
//        下方是树的结构图。悬停在物体或结构图的节点上，高亮对应的节点及其包围盒
//   sap：下方是每个物体在 x 轴上的投影区间。鼠标的竖线就是"扫描线"，与它相交的区间（活动列表）
//        对应的物体被高亮；只有 x 区间重叠的物体之间才需要测试

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "../../foundations/random/random.h"
#include "bvh.h"
#include "demo.h"
#include "ui.h"

namespace {

using namespace ga;

constexpr int kWidth = 800;
constexpr int kHeight = 500;
constexpr float kTop = 52;
constexpr Vec2 kOrigin{16, kTop + 12};
constexpr Vec2 kWorld{544, 288};
constexpr float kBandY = kOrigin.y + kWorld.y + 14;  // 下方的结构图 / 区间图
constexpr float kBandH = kHeight - kBandY - 10;
constexpr float kPanelX = kOrigin.x + kWorld.x + 22;
constexpr float kDt = 1.0f / 60.0f;

inline Vector2 screen(Vec2 w) { return {w.x + kOrigin.x, w.y + kOrigin.y}; }
inline Rectangle rect(const Aabb& b) { return {b.min.x + kOrigin.x, b.min.y + kOrigin.y, b.max.x - b.min.x, b.max.y - b.min.y}; }

struct Body {
    Vec2 pos, size, vel;
    Aabb box() const { return {pos, pos + size}; }
};

class BvhDemo final : public Demo {
public:
    BvhDemo() { reset(); }

    void reset() override {
        Pcg32 r(9);
        auto rnd = [&](float lo, float hi) { return lo + r.nextFloat() * (hi - lo); };
        bodies_.clear();
        for (int i = 0; i < static_cast<int>(count_); ++i) {
            Vec2 size{rnd(10, 34), rnd(10, 34)};
            bodies_.push_back({{rnd(0, kWorld.x - size.x), rnd(0, kWorld.y - size.y)}, size, fromAngle(rnd(0, 2 * kPi)) * rnd(12, 45)});
        }
        rebuildTree();
    }

    bool step() override {
        for (Body& b : bodies_) {
            b.pos += b.vel * kDt;
            if (b.pos.x < 0) b.pos.x = 0, b.vel.x = std::abs(b.vel.x);
            if (b.pos.y < 0) b.pos.y = 0, b.vel.y = std::abs(b.vel.y);
            if (b.pos.x + b.size.x > kWorld.x) b.pos.x = kWorld.x - b.size.x, b.vel.x = -std::abs(b.vel.x);
            if (b.pos.y + b.size.y > kWorld.y) b.pos.y = kWorld.y - b.size.y, b.vel.y = -std::abs(b.vel.y);
        }
        refreshBoxes();
        reinserted_ = 0;
        for (size_t i = 0; i < bodies_.size(); ++i) {
            if (tree_.move(leaf_[i], boxes_[i])) ++reinserted_, flash_[i] = 1;
            else flash_[i] = std::max(0.0f, flash_[i] - 0.34f);
        }
        swaps_ = sap_.update(boxes_);
        findPairs();
        layoutDiagram();  // 树的结构变了，结构图要重新排版
        return true;
    }

    void update() override {
        Vector2 m = GetMousePosition();
        mouse_ = {m.x, m.y};
        layoutDiagram();
    }

    void draw() const override {
        const Palette& pal = palette();
        const float hl = hairline();
        DrawRectangleLinesEx({kOrigin.x, kOrigin.y, kWorld.x, kWorld.y}, hl * 1.5f, pal.textMuted);
        BeginScissorMode(int(kOrigin.x), int(kOrigin.y), int(kWorld.x), int(kWorld.y));
        if (sapMode_) drawSapWorld();
        else drawBvhWorld();
        EndScissorMode();
        if (sapMode_) drawIntervals();
        else drawDiagram();
        drawPanel();
        DrawRectangleRec({0, 0, float(kWidth), kTop}, pal.background);
        DrawLineEx({0, kTop}, {float(kWidth), kTop}, hl, pal.gridLine);
    }

    void drawUi() override {
        if (ui::button({16, 13, 52, 26}, "bvh", !sapMode_) && sapMode_) sapMode_ = false;
        if (ui::button({74, 13, 52, 26}, "sap", sapMode_) && !sapMode_) sapMode_ = true;
        if (ui::slider({140, 13, 180, 26}, "count", &count_, 4, 120, "%.0f", 2)) reset();
        if (!sapMode_) {
            if (ui::slider({332, 13, 180, 26}, "margin", &margin_, 0, 16, "%.0f", 1)) rebuildTree();
            if (ui::toggle({524, 13, 70, 26}, "rotate", &balance_)) rebuildTree();
        }
    }

    std::string status() const override {
        char buf[96];
        if (sapMode_) std::snprintf(buf, sizeof buf, "%ld swaps · %ld tests · %zu pairs", swaps_, sapTests_, pairs_.size());
        else std::snprintf(buf, sizeof buf, "height %d · %d reinserted · %zu pairs", tree_.height(), reinserted_, pairs_.size());
        return buf;
    }

private:
    void refreshBoxes() {
        boxes_.clear();
        for (const Body& b : bodies_) boxes_.push_back(b.box());
    }

    void rebuildTree() {
        refreshBoxes();
        tree_ = DynamicTree(margin_, balance_);
        leaf_.clear();
        for (int i = 0; i < static_cast<int>(boxes_.size()); ++i) leaf_.push_back(tree_.insert(boxes_[i], i));
        flash_.assign(boxes_.size(), 0);
        reinserted_ = 0;
        sap_ = SweepAndPrune();
        sap_.sortFromScratch(boxes_);
        swaps_ = 0;
        findPairs();
        layoutDiagram();
    }

    void findPairs() {
        std::vector<Pair> tmp;
        bvhTests_ = tree_.findPairs(boxes_, pairs_);
        sapTests_ = sap_.findPairs(tmp);
    }

    // ---- 树的结构图：叶子按中序从左到右排开，内部节点在两个孩子中间，深度决定高度 ----
    void layoutDiagram() {
        const auto& nodes = tree_.nodes();
        pos_.assign(nodes.size(), {0, 0});
        depth_.assign(nodes.size(), 0);
        if (tree_.root() < 0) return;
        int leaves = static_cast<int>(bodies_.size());
        int height = std::max(1, tree_.height());
        float levelH = std::min(18.0f, (kBandH - 16) / height);
        float slot = (kWidth - 32.0f) / std::max(1, leaves);
        int next = 0;
        auto place = [&](auto&& self, int n, int d) -> float {
            depth_[n] = d;
            float x;
            if (nodes[n].leaf()) x = 16 + slot * (next++ + 0.5f);
            else {
                float l = self(self, nodes[n].left, d + 1);  // 先左后右：保证叶子按中序编号
                float r = self(self, nodes[n].right, d + 1);
                x = (l + r) / 2;
            }
            pos_[n] = {x, kBandY + 8 + d * levelH};
            return x;
        };
        place(place, tree_.root(), 0);

        // 悬停：结构图上最近的节点，或者鼠标下的物体对应的叶子
        hoverNode_ = -1;
        float best = 9;
        for (size_t n = 0; n < nodes.size(); ++n)
            if (nodes[n].height >= 0 && distance(pos_[n], mouse_) < best) best = distance(pos_[n], mouse_), hoverNode_ = int(n);
        if (hoverNode_ < 0) {
            Vec2 w = mouse_ - kOrigin;
            for (int i = int(bodies_.size()) - 1; i >= 0; --i)
                if (contains(boxes_[i], w)) {
                    hoverNode_ = leaf_[i];
                    break;
                }
        }
    }

    bool inSubtree(int n, int root) const {
        for (; n >= 0; n = tree_.nodes()[n].parent)
            if (n == root) return true;
        return false;
    }
    bool isAncestor(int a, int n) const { return inSubtree(n, a); }

    void drawBvhWorld() const {
        const Palette& pal = palette();
        const float hl = hairline();
        const auto& nodes = tree_.nodes();
        std::vector<char> hot(bodies_.size(), 0);
        for (Pair p : pairs_) hot[p.a] = hot[p.b] = 1;
        // 内部节点的包围盒：越靠近根越深
        int height = std::max(1, tree_.height());
        for (size_t n = 0; n < nodes.size(); ++n) {
            if (nodes[n].height <= 0) continue;
            float t = float(nodes[n].height) / height;
            DrawRectangleLinesEx(rect(nodes[n].box), 1.0f, Fade(pal.textMuted, 0.15f + 0.45f * t));
        }
        for (size_t i = 0; i < bodies_.size(); ++i) {
            Rectangle r = rect(boxes_[i]);
            Color ink = hot[i] ? pal.path : pal.text;
            DrawRectangleRec(r, Fade(ink, hot[i] ? 0.16f : 0.07f));
            if (flash_[i] > 0) DrawRectangleRec(r, Fade(pal.frontier, 0.7f * flash_[i]));
            DrawRectangleLinesEx(r, hl * 1.5f, ink);
            if (margin_ > 0) dashedRect(rect(nodes[leaf_[i]].box), Fade(pal.textMuted, 0.8f));
        }
        if (hoverNode_ >= 0) {
            // 悬停的节点：自己和所有祖先的包围盒
            for (int n = hoverNode_; n >= 0; n = nodes[n].parent)
                DrawRectangleLinesEx(rect(nodes[n].box), n == hoverNode_ ? 2.5f : 1.5f, n == hoverNode_ ? pal.frontier : Fade(pal.frontier, 0.6f));
        }
    }

    void drawDiagram() const {
        const Palette& pal = palette();
        const float hl = hairline();
        const auto& nodes = tree_.nodes();
        if (tree_.root() < 0) return;
        for (size_t n = 0; n < nodes.size(); ++n) {
            if (nodes[n].height <= 0) continue;
            for (int c : {nodes[n].left, nodes[n].right}) {
                bool lit = hoverNode_ >= 0 && (inSubtree(c, hoverNode_) || isAncestor(c, hoverNode_));
                DrawLineEx(rl(pos_[n]), rl(pos_[c]), lit ? 1.6f : hl * 1.5f, lit ? pal.frontier : Fade(pal.textMuted, 0.7f));
            }
        }
        for (size_t n = 0; n < nodes.size(); ++n) {
            if (nodes[n].height < 0) continue;
            bool lit = hoverNode_ >= 0 && (inSubtree(int(n), hoverNode_) || isAncestor(int(n), hoverNode_));
            bool leaf = nodes[n].leaf();
            Color c = lit ? pal.frontier : leaf ? pal.text : pal.textMuted;
            DrawCircleV(rl(pos_[n]), int(n) == hoverNode_ ? 4.5f : leaf ? 2.5f : 2.0f, c);
        }
        drawText("tree", {16, kBandY - 2}, 11, pal.textMuted);
    }

    static Vector2 rl(Vec2 v) { return {v.x, v.y}; }

    // ---- SAP ----
    float sweepX() const { return mouse_.x - kOrigin.x; }
    bool sweeping() const { return mouse_.x >= kOrigin.x && mouse_.x < kOrigin.x + kWorld.x && mouse_.y > kTop && !ui::wantsMouse(); }

    void drawSapWorld() const {
        const Palette& pal = palette();
        const float hl = hairline();
        std::vector<char> hot(bodies_.size(), 0);
        for (Pair p : pairs_) hot[p.a] = hot[p.b] = 1;
        float sx = sweepX();
        bool sw = sweeping();
        for (size_t i = 0; i < bodies_.size(); ++i) {
            Rectangle r = rect(boxes_[i]);
            Color ink = hot[i] ? pal.path : pal.text;
            bool active = sw && boxes_[i].min.x <= sx && sx <= boxes_[i].max.x;
            DrawRectangleRec(r, Fade(ink, hot[i] ? 0.16f : 0.07f));
            if (active) DrawRectangleRec(r, Fade(pal.frontier, 0.4f));
            DrawRectangleLinesEx(r, active ? 2.0f : hl * 1.5f, active ? pal.frontier : ink);
        }
        if (sw) DrawLineEx(screen({sx, 0}), screen({sx, kWorld.y}), 1.5f, pal.frontier);
    }

    void drawIntervals() const {
        const Palette& pal = palette();
        // 贪心分行：按起点顺序，放进第一个空出来的行
        const auto& es = sap_.entries();
        std::vector<float> rowEnd;
        std::vector<int> rowOf(es.size());
        for (size_t k = 0; k < es.size(); ++k) {
            size_t row = 0;
            while (row < rowEnd.size() && rowEnd[row] > es[k].lo) ++row;
            if (row == rowEnd.size()) rowEnd.push_back(0);
            rowEnd[row] = es[k].hi + 2;
            rowOf[k] = int(row);
        }
        float rowH = std::min(10.0f, (kBandH - 18) / std::max<size_t>(1, rowEnd.size()));
        float sx = sweepX();
        bool sw = sweeping();
        int active = 0;
        for (size_t k = 0; k < es.size(); ++k) {
            bool on = sw && es[k].lo <= sx && sx <= es[k].hi;
            active += on;
            float y = kBandY + 14 + rowOf[k] * rowH;
            DrawRectangleRec({kOrigin.x + es[k].lo, y, es[k].hi - es[k].lo, std::max(2.0f, rowH - 3)},
                             on ? pal.frontier : Fade(pal.text, 0.35f));
        }
        if (sw) DrawLineEx({kOrigin.x + sx, kBandY + 10}, {kOrigin.x + sx, kHeight - 8.0f}, 1.5f, pal.frontier);
        char buf[64];
        std::snprintf(buf, sizeof buf, sw ? "x intervals, sorted by start · %d active at the sweep line" : "x intervals, sorted by start", active);
        drawText(buf, {16, kBandY - 2}, 11, pal.textMuted);
    }

    static void dashedRect(Rectangle r, Color c) {
        drawDashedLine({r.x, r.y}, {r.x + r.width, r.y}, 1.0f, c, 4, 3);
        drawDashedLine({r.x + r.width, r.y}, {r.x + r.width, r.y + r.height}, 1.0f, c, 4, 3);
        drawDashedLine({r.x + r.width, r.y + r.height}, {r.x, r.y + r.height}, 1.0f, c, 4, 3);
        drawDashedLine({r.x, r.y + r.height}, {r.x, r.y}, 1.0f, c, 4, 3);
    }

    void drawPanel() const {
        const Palette& pal = palette();
        DrawLineEx({kPanelX - 10, kTop}, {kPanelX - 10, kBandY - 8}, hairline(), pal.gridLine);
        float x = kPanelX, y = kTop + 16, right = kWidth - 16.0f;
        char buf[32];
        auto row = [&](const char* label, long v, Color c) {
            drawText(label, {x, y}, 12, pal.textMuted);
            std::snprintf(buf, sizeof buf, "%ld", v);
            drawText(buf, {right - measureText(buf, 12).x, y}, 12, c);
            y += 20;
        };
        long n = static_cast<long>(bodies_.size());
        row("objects", n, pal.text);
        row("brute-force tests", n * (n - 1) / 2, pal.text);
        if (sapMode_) {
            row("x-overlap tests", sapTests_, pal.frontier);
            row("swaps this frame", swaps_, pal.text);
        } else {
            row("tree tests", bvhTests_, pal.frontier);
            row("height", tree_.height(), pal.text);
            row("reinserted", reinserted_, reinserted_ ? pal.frontier : pal.text);
        }
        row("overlapping pairs", static_cast<long>(pairs_.size()), pal.path);
    }

    std::vector<Body> bodies_;
    std::vector<Aabb> boxes_;
    std::vector<Pair> pairs_;
    DynamicTree tree_;
    std::vector<int> leaf_;
    std::vector<float> flash_;
    SweepAndPrune sap_;
    long swaps_ = 0, bvhTests_ = 0, sapTests_ = 0;
    int reinserted_ = 0;
    bool sapMode_ = false;
    bool balance_ = true;
    float margin_ = 8;
    float count_ = 40;
    Vec2 mouse_{-1000, -1000};
    std::vector<Vec2> pos_;
    std::vector<int> depth_;
    int hoverNode_ = -1;
};

}  // namespace

int main() {
    static BvhDemo demo;
    ga::DemoConfig config;
    config.title = "Dynamic BVH & Sweep and Prune";
    config.width = kWidth;
    config.height = kHeight;
    config.stepsPerSecond = 60.0f;
    return ga::runDemo(demo, config);
}
