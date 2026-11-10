// BSP 地牢
//   - 动画分三个阶段：逐个切割节点（广度优先）→ 每个叶子放一个房间 → 自底向上连接兄弟子树
//   - min leaf：叶子的最小边长；depth：最多切几层（max 表示切到不能再切）
//   - long axis：沿长边切；关掉后在允许的方向里随机选。限制层数时差别很明显，切到底时几乎没有差别
//   - closest：连接两棵子树时选距离最近的一对房间；关掉后各随机选一个，走廊更长，也更常穿过别的房间
//   - 右侧是分割树：当前处理的节点标红

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "demo.h"
#include "dungeon.h"
#include "ui.h"

namespace {

using namespace ga;
using namespace ga::dungeon;

constexpr int kWidth = 800, kHeight = 480;
constexpr float kTop = 52;
constexpr int kW = 80, kH = 50;
constexpr float kCell = 7;
constexpr float kMx = 16, kMy = kTop + 12;
constexpr float kTx = 598, kTw = 190, kTy = kTop + 30, kTh = 260;

class BspDemo final : public Demo {
public:
    BspDemo() { generate(); }

    void reset() override { tick_ = 0, rebuild(); }

    bool step() override {
        if (tick_ >= total()) return false;
        ++tick_;
        rebuild();
        return tick_ < total();
    }

    void draw() const override {
        const Palette& pal = palette();
        const float hl = hairline();
        // 地图：墙是墨色，房间是纸色，走廊略深
        DrawRectangleRec({kMx, kMy, kW * kCell, kH * kCell}, pal.wall);
        for (int y = 0; y < kH; ++y)
            for (int x = 0; x < kW; ++x) {
                uint8_t t = view_.at(x, y);
                if (t == Wall) continue;
                DrawRectangleRec({kMx + x * kCell, kMy + y * kCell, kCell, kCell}, t == Room ? pal.background : pal.visitedNear);
            }
        // 分割线：已创建的节点的边框，越深越细
        int created = createdCount();
        for (int i = 0; i < created; ++i) {
            const Rect& a = res_.nodes[i].area;
            float th = std::max(1.0f, 3.0f - res_.nodes[i].depth * 0.4f);
            DrawRectangleLinesEx({kMx + a.x * kCell, kMy + a.y * kCell, a.w * kCell, a.h * kCell}, th, Fade(pal.frontier, 0.95f));
        }
        int cur = currentNode();
        if (cur >= 0) {
            const Rect& a = res_.nodes[cur].area;
            DrawRectangleLinesEx({kMx + a.x * kCell, kMy + a.y * kCell, a.w * kCell, a.h * kCell}, 2.5f, pal.path);
        }
        if (phase() == 2 && tick_ > splitSteps() + int(res_.leaves.size())) {  // 刚连上的走廊
            const Link& l = res_.corridors[tick_ - splitSteps() - res_.leaves.size() - 1];
            DrawCircleV({kMx + (l.ax + 0.5f) * kCell, kMy + (l.ay + 0.5f) * kCell}, 4, pal.path);
            DrawCircleV({kMx + (l.bx + 0.5f) * kCell, kMy + (l.by + 0.5f) * kCell}, 4, pal.path);
        }

        // 分割树
        drawText("split tree", {kTx, kTop + 10}, 11, pal.textMuted);
        for (int i = 0; i < int(res_.nodes.size()); ++i) {
            const BspNode& n = res_.nodes[i];
            if (n.left < 0 || i >= created) continue;
            for (int c : {n.left, n.right})
                if (c < created) DrawLineEx(treePos_[i], treePos_[c], hl * 1.5f, pal.textMuted);
        }
        for (int i = 0; i < created; ++i) {
            bool leaf = res_.nodes[i].left < 0;
            Color c = i == cur ? pal.path : leaf ? pal.frontier : pal.text;
            DrawCircleV(treePos_[i], leaf ? 2.6f : 2.2f, c);
        }

        char buf[128];
        float y = kTy + kTh + 16;
        const char* phaseName[] = {"1/3 splitting", "2/3 placing rooms", "3/3 connecting", "done"};
        drawText(phaseName[phase()], {kTx, y}, 12, pal.text);
        y += 20;
        std::snprintf(buf, sizeof buf, "leaves %d  max depth %d", int(res_.leaves.size()), maxDepth_);
        drawText(buf, {kTx, y}, 11, pal.textMuted);
        y += 17;
        std::snprintf(buf, sizeof buf, "leaf aspect mean %.2f max %.2f", aspMean_, aspMax_);
        drawText(buf, {kTx, y}, 11, pal.textMuted);
        y += 17;
        if (phase() == 3) {
            std::snprintf(buf, sizeof buf, "corridor tiles %d", corridorTiles_);
            drawText(buf, {kTx, y}, 11, pal.textMuted);
            y += 17;
            std::snprintf(buf, sizeof buf, "detour ratio %.2f", detour_);
            drawText(buf, {kTx, y}, 11, pal.textMuted);
        }
        DrawLineEx({0, kTop}, {float(kWidth), kTop}, hl, pal.gridLine);
    }

    void drawUi() override {
        bool ch = false;
        ch |= ui::slider({12, 13, 150, 26}, "seed", &seed_, 1, 50, "%.0f", 1);
        ch |= ui::slider({172, 13, 170, 26}, "min leaf", &minLeaf_, 5, 14, "%.0f", 1);
        ch |= ui::slider({352, 13, 150, 26}, "depth", &depth_, 1, 9, depth_ >= 9 ? "max" : "%.0f", 1);
        ch |= ui::toggle({512, 13, 104, 26}, "long axis", &longAxis_);
        ch |= ui::toggle({622, 13, 90, 26}, "closest", &closest_);
        if (ch) generate(), notifyProblemChanged();
    }

    std::string status() const override {
        char buf[96];
        std::snprintf(buf, sizeof buf, "%d leaves · %d corridors · %s", int(res_.leaves.size()), int(res_.corridors.size()),
                      phase() == 3 ? "done" : "building");
        return buf;
    }

private:
    int splitSteps() const { return internal_; }
    int total() const { return internal_ + int(res_.leaves.size()) + int(res_.corridors.size()); }
    int phase() const {
        if (tick_ < internal_) return 0;
        if (tick_ < internal_ + int(res_.leaves.size())) return 1;
        if (tick_ < total()) return 2;
        return 3;
    }
    // 已经切开的内部节点：按创建顺序排在前面的 tick_ 个内部节点
    int createdCount() const {
        if (tick_ >= internal_) return int(res_.nodes.size());
        int seen = 0;
        for (int i = 0; i < int(res_.nodes.size()); ++i)
            if (res_.nodes[i].left >= 0 && seen++ == tick_) return res_.nodes[i].left;  // 第 tick_ 个内部节点的孩子还没出现
        return int(res_.nodes.size());
    }
    int currentNode() const {
        if (tick_ == 0) return -1;
        if (phase() == 0 || tick_ == internal_) {  // 刚切开的节点
            int seen = 0;
            for (int i = 0; i < int(res_.nodes.size()); ++i)
                if (res_.nodes[i].left >= 0 && ++seen == tick_) return i;
        }
        if (phase() == 1 || (phase() == 2 && tick_ == internal_ + int(res_.leaves.size()))) return res_.leaves[tick_ - internal_ - 1];
        if (phase() == 2) {  // 走廊按内部节点倒序连接
            int k = tick_ - internal_ - int(res_.leaves.size()) - 1, seen = 0;
            for (int i = int(res_.nodes.size()) - 1; i >= 0; --i)
                if (res_.nodes[i].left >= 0 && seen++ == k) return i;
        }
        return -1;
    }

    void generate() {
        Map m(kW, kH);
        Pcg32 rng{uint64_t(seed_)};
        BspParams p;
        p.minLeaf = int(minLeaf_), p.longAxis = longAxis_, p.closest = closest_, p.maxDepth = depth_ >= 9 ? 99 : int(depth_);
        res_ = generateBsp(m, rng, p);
        final_ = m;
        internal_ = int(res_.nodes.size() - res_.leaves.size());
        maxDepth_ = 0, aspMean_ = 0, aspMax_ = 0;
        for (int li : res_.leaves) {
            const Rect& a = res_.nodes[li].area;
            float asp = float(std::max(a.w, a.h)) / std::min(a.w, a.h);
            aspMean_ += asp / res_.leaves.size(), aspMax_ = std::max(aspMax_, asp);
            maxDepth_ = std::max(maxDepth_, res_.nodes[li].depth);
        }
        corridorTiles_ = 0;
        for (uint8_t t : m.t) corridorTiles_ += t == Corridor;
        Pcg32 r2(1);
        detour_ = float(detourRatio(m, r2, 30, 30, 10));
        // 树的布局：叶子按深度优先顺序等距排开，内部节点在两个孩子中间
        treePos_.assign(res_.nodes.size(), {0, 0});
        int leafIndex = 0, nLeaves = int(res_.leaves.size());
        float rowH = std::min(28.0f, (kTh - 10) / float(std::max(1, maxDepth_)));
        auto layout = [&](auto&& self, int n) -> float {
            const BspNode& node = res_.nodes[n];
            float x = node.left < 0 ? kTx + 6 + (leafIndex++ + 0.5f) * (kTw - 12) / nLeaves : (self(self, node.left) + self(self, node.right)) * 0.5f;
            treePos_[n] = {x, kTy + 6 + node.depth * rowH};
            return x;
        };
        layout(layout, 0);
        tick_ = 0;
        rebuild();
    }

    // 按当前进度重建显示用的地图：已放下的房间 + 已连上的走廊
    void rebuild() {
        view_ = Map(kW, kH);
        int rooms = std::clamp(tick_ - internal_, 0, int(res_.leaves.size()));
        for (int k = 0; k < rooms; ++k) carveRoom(view_, res_.nodes[res_.leaves[k]].room);
        int links = std::clamp(tick_ - internal_ - int(res_.leaves.size()), 0, int(res_.corridors.size()));
        for (int k = 0; k < links; ++k) {
            const Link& l = res_.corridors[k];
            carveCorridor(view_, l.ax, l.ay, l.bx, l.by, l.horizontalFirst);
        }
    }

    float seed_ = 3, minLeaf_ = 8, depth_ = 9;
    bool longAxis_ = true, closest_ = true;
    BspResult res_;
    Map final_{kW, kH}, view_{kW, kH};
    std::vector<Vector2> treePos_;
    int tick_ = 0, internal_ = 0, maxDepth_ = 0, corridorTiles_ = 0;
    float aspMean_ = 0, aspMax_ = 0, detour_ = 0;
};

}  // namespace

int main() {
    static BspDemo demo;
    ga::DemoConfig config;
    config.title = "BSP dungeon";
    config.width = kWidth;
    config.height = kHeight;
    config.stepsPerSecond = 6.0f;
    return ga::runDemo(demo, config);
}
