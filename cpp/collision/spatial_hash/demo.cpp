// 宽相检测演示：均匀网格与空间哈希
//   - 一群盒子在场地里运动，每一步重建网格并找出所有重叠的物体对（描红）
//   - 格子的底色越深，里面的物体越多；只有同一个格子里的物体之间才需要测试
//   - 鼠标悬停在一个格子上：列出它里面的物体，细线是要做的测试，红线是在这个格子里报告的重叠
//   - hash 模式：格子坐标经过哈希落进有限个桶，与悬停格子同桶的其他格子用虚框标出，
//     它们的物体也会混进同一个列表，带来"冒名"的测试
//   - big：加入几个大物体，看它们如何同时占据许多格子

#include <cmath>
#include <cstdio>
#include <optional>
#include <string>
#include <vector>

#include "../../foundations/random/random.h"
#include "demo.h"
#include "spatial_hash.h"
#include "ui.h"

namespace {

using namespace ga;

constexpr int kWidth = 800;
constexpr int kHeight = 500;
constexpr float kTop = 52;
constexpr Vec2 kOrigin{16, kTop + 12};     // 场地左上角（屏幕坐标）
constexpr Vec2 kWorld{560, kHeight - kTop - 24};  // 场地大小；物体使用以场地左上角为原点的坐标
constexpr float kPanelX = 592;
constexpr float kDt = 1.0f / 60.0f;

inline Vector2 rl(Vec2 v) { return {v.x, v.y}; }
inline Vector2 screen(Vec2 w) { return rl(w + kOrigin); }

struct Body {
    Vec2 pos, size, vel;
    Aabb box() const { return {pos, pos + size}; }
};

class SpatialHashDemo final : public Demo {
public:
    SpatialHashDemo() { reset(); }

    void reset() override {
        Pcg32 r(11);
        auto rnd = [&](float lo, float hi) { return lo + r.nextFloat() * (hi - lo); };
        bodies_.clear();
        for (int i = 0; i < static_cast<int>(count_); ++i) {
            Vec2 size{rnd(8, 24), rnd(8, 24)};
            Vec2 pos{rnd(0, kWorld.x - size.x), rnd(0, kWorld.y - size.y)};
            bodies_.push_back({pos, size, fromAngle(rnd(0, 2 * kPi)) * rnd(15, 60)});
        }
        if (big_) {
            bodies_.push_back({{60, 50}, {130, 90}, {18, 11}});
            bodies_.push_back({{330, 250}, {100, 120}, {-14, -16}});
            bodies_.push_back({{380, 40}, {150, 70}, {-10, 20}});
        }
        rebuild();
    }

    bool step() override {
        for (Body& b : bodies_) {
            b.pos += b.vel * kDt;
            // 碰到场地边缘就反弹
            if (b.pos.x < 0) b.pos.x = 0, b.vel.x = std::abs(b.vel.x);
            if (b.pos.y < 0) b.pos.y = 0, b.vel.y = std::abs(b.vel.y);
            if (b.pos.x + b.size.x > kWorld.x) b.pos.x = kWorld.x - b.size.x, b.vel.x = -std::abs(b.vel.x);
            if (b.pos.y + b.size.y > kWorld.y) b.pos.y = kWorld.y - b.size.y, b.vel.y = -std::abs(b.vel.y);
        }
        rebuild();
        return true;
    }

    void update() override {
        Vector2 m = GetMousePosition();
        Vec2 w = Vec2{m.x, m.y} - kOrigin;
        hover_.reset();
        if (!ui::wantsMouse() && w.x >= 0 && w.y >= 0 && w.x < kWorld.x && w.y < kWorld.y)
            hover_ = Vec2{std::floor(w.x / cell_), std::floor(w.y / cell_)};
    }

    void draw() const override {
        const Palette& pal = palette();
        const float hl = hairline();
        int cols = static_cast<int>(std::ceil(kWorld.x / cell_)), rows = static_cast<int>(std::ceil(kWorld.y / cell_));

        // 格子底色：所在存储位置（网格的格子 / 哈希的桶）里有几个物体
        BeginScissorMode(int(kOrigin.x), int(kOrigin.y), int(kWorld.x), int(kWorld.y));
        for (int y = 0; y < rows; ++y)
            for (int x = 0; x < cols; ++x) {
                int s = slotOf(x, y), n = slotEnd(s) - slotBegin(s);
                if (n == 0) continue;
                DrawRectangleRec(cellRect(x, y), Fade(pal.frontier, 0.07f + 0.05f * std::min(n, 6)));
            }
        for (int x = 0; x <= cols; ++x)
            DrawLineEx(screen({x * cell_, 0}), screen({x * cell_, kWorld.y}), hl, pal.gridLine);
        for (int y = 0; y <= rows; ++y)
            DrawLineEx(screen({0, y * cell_}), screen({kWorld.x, y * cell_}), hl, pal.gridLine);

        // 悬停的格子；哈希模式下，同一个桶里的其他格子用虚框标出
        int hs = -1;
        if (hover_) {
            int hx = int(hover_->x), hy = int(hover_->y);
            hs = slotOf(hx, hy);
            if (hash_)
                for (int y = 0; y < rows; ++y)
                    for (int x = 0; x < cols; ++x)
                        if ((x != hx || y != hy) && slotOf(x, y) == hs) drawDashedRect(cellRect(x, y), pal.frontier);
            Rectangle r = cellRect(hx, hy);
            DrawRectangleLinesEx(r, 2, pal.text);
        }

        // 物体
        std::vector<char> hot(bodies_.size(), 0), inSlot(bodies_.size(), 0);
        for (Pair p : pairs_) hot[p.a] = hot[p.b] = 1;
        if (hs >= 0)
            for (int k = slotBegin(hs); k < slotEnd(hs); ++k) inSlot[entry(k)] = 1;
        for (size_t i = 0; i < bodies_.size(); ++i) {
            Aabb b = boxes_[i];
            Rectangle r{b.min.x + kOrigin.x, b.min.y + kOrigin.y, b.max.x - b.min.x, b.max.y - b.min.y};
            Color ink = hot[i] ? pal.path : pal.text;
            DrawRectangleRec(r, Fade(ink, hot[i] ? 0.16f : 0.07f));
            DrawRectangleLinesEx(r, inSlot[i] ? 2.0f : hl * 1.5f, Fade(ink, inSlot[i] || hs < 0 ? 1.0f : 0.55f));
        }

        // 悬停格子里要做的测试：每两个物体之间一条线
        if (hs >= 0) {
            for (int k = slotBegin(hs); k < slotEnd(hs); ++k)
                for (int l = k + 1; l < slotEnd(hs); ++l) {
                    int i = entry(k), j = entry(l);
                    bool reported = overlaps(boxes_[i], boxes_[j]) && refSlot(i, j) == hs;
                    Vector2 a = screen(boxes_[i].center()), c = screen(boxes_[j].center());
                    if (reported) DrawLineEx(a, c, 2.5f, pal.path);
                    else DrawLineEx(a, c, hl * 1.5f, Fade(pal.text, 0.5f));
                }
        }
        EndScissorMode();
        DrawRectangleLinesEx({kOrigin.x, kOrigin.y, kWorld.x, kWorld.y}, hl * 1.5f, pal.textMuted);

        drawPanel(hs);
        DrawRectangleRec({0, 0, float(kWidth), kTop}, pal.background);
        DrawLineEx({0, kTop}, {float(kWidth), kTop}, hl, pal.gridLine);
    }

    void drawUi() override {
        bool changed = false;
        if (ui::button({16, 13, 56, 26}, "grid", !hash_) && hash_) hash_ = false, changed = true;
        if (ui::button({78, 13, 56, 26}, "hash", hash_) && !hash_) hash_ = true, changed = true;
        changed |= ui::slider({148, 13, 190, 26}, "cell", &cell_, 16, 112, "%.0f", 4);
        if (ui::slider({350, 13, 190, 26}, "count", &count_, 10, 400, "%.0f", 10)) reset();
        if (ui::toggle({552, 13, 52, 26}, "big", &big_)) reset();
        if (hash_) changed |= ui::slider({kPanelX + 4, kTop + 12, kWidth - kPanelX - 16, 26}, "buckets", &buckets_, 1, 160, "%.0f", 1);
        if (changed) rebuild();
    }

    std::string status() const override {
        char buf[96];
        std::snprintf(buf, sizeof buf, "%ld tests vs %ld brute force · %zu pairs", tests_, bruteTests(), pairs_.size());
        return buf;
    }

private:
    void rebuild() {
        boxes_.clear();
        for (const Body& b : bodies_) boxes_.push_back(b.box());
        if (hash_) {
            grid_.reset();
            hash_bp_.emplace(HashIndexer(cell_, static_cast<int>(buckets_)));
            hash_bp_->build(boxes_);
            tests_ = hash_bp_->findPairs(pairs_);
        } else {
            hash_bp_.reset();
            grid_.emplace(GridIndexer({{0, 0}, kWorld}, cell_));
            grid_->build(boxes_);
            tests_ = grid_->findPairs(pairs_);
        }
    }

    // 两种结构的统一访问
    int slotOf(int x, int y) const { return hash_ ? hash_bp_->indexer().slot(x, y) : grid_->indexer().slot(x, y); }
    int slotBegin(int s) const { return hash_ ? hash_bp_->slotBegin(s) : grid_->slotBegin(s); }
    int slotEnd(int s) const { return hash_ ? hash_bp_->slotEnd(s) : grid_->slotEnd(s); }
    int entry(int k) const { return hash_ ? hash_bp_->entry(k) : grid_->entry(k); }
    int refSlot(int i, int j) const {
        Vec2 p{std::max(boxes_[i].min.x, boxes_[j].min.x), std::max(boxes_[i].min.y, boxes_[j].min.y)};
        CellRange c = hash_ ? hash_bp_->range({p, p}) : grid_->range({p, p});
        return slotOf(c.x0, c.y0);
    }
    long bruteTests() const { long n = static_cast<long>(bodies_.size()); return n * (n - 1) / 2; }

    Rectangle cellRect(int x, int y) const { return {kOrigin.x + x * cell_, kOrigin.y + y * cell_, cell_, cell_}; }

    static void drawDashedRect(Rectangle r, Color c) {
        float t = 1.5f;
        drawDashedLine({r.x, r.y}, {r.x + r.width, r.y}, t, c, 5, 4);
        drawDashedLine({r.x + r.width, r.y}, {r.x + r.width, r.y + r.height}, t, c, 5, 4);
        drawDashedLine({r.x + r.width, r.y + r.height}, {r.x, r.y + r.height}, t, c, 5, 4);
        drawDashedLine({r.x, r.y + r.height}, {r.x, r.y}, t, c, 5, 4);
    }

    void drawPanel(int hs) const {
        const Palette& pal = palette();
        DrawLineEx({kPanelX - 8, kTop}, {kPanelX - 8, float(kHeight)}, hairline(), pal.gridLine);
        float x = kPanelX + 8, y = kTop + (hash_ ? 52 : 16);
        char buf[64];
        auto row = [&](const char* label, long v, Color c) {
            drawText(label, {x, y}, 12, pal.textMuted);
            std::snprintf(buf, sizeof buf, "%ld", v);
            Vector2 sz = measureText(buf, 12);
            drawText(buf, {kWidth - 16 - sz.x, y}, 12, c);
            y += 20;
        };
        row("objects", static_cast<long>(bodies_.size()), pal.text);
        int entries = hash_ ? hash_bp_->entryCount() : grid_->entryCount();
        std::snprintf(buf, sizeof buf, "%.2f", bodies_.empty() ? 0.0 : double(entries) / bodies_.size());
        drawText("cells / object", {x, y}, 12, pal.textMuted);
        drawText(buf, {kWidth - 16 - measureText(buf, 12).x, y}, 12, pal.text);
        y += 28;

        // 测试次数：两两测试 vs 只测同格的物体
        float barW = kWidth - 16 - x;
        long brute = std::max(1L, bruteTests());
        row("brute-force tests", brute, pal.text);
        DrawRectangleRec({x, y - 4, barW, 6}, Fade(pal.text, 0.25f));
        y += 12;
        row(hash_ ? "hash tests" : "grid tests", tests_, pal.frontier);
        DrawRectangleRec({x, y - 4, std::max(1.0f, barW * float(tests_) / brute), 6}, pal.frontier);
        y += 12;
        row("overlapping pairs", static_cast<long>(pairs_.size()), pal.path);
        y += 12;

        // 悬停格子的内容
        DrawLineEx({x, y}, {kWidth - 16.0f, y}, hairline(), pal.gridLine);
        y += 12;
        if (hs < 0) {
            drawText("hover a cell", {x, y}, 12, pal.textMuted);
            return;
        }
        int hx = int(hover_->x), hy = int(hover_->y);
        if (hash_) std::snprintf(buf, sizeof buf, "cell (%d, %d) -> bucket %d", hx, hy, hs);
        else std::snprintf(buf, sizeof buf, "cell (%d, %d)", hx, hy);
        drawText(buf, {x, y}, 12, pal.text);
        y += 20;
        // 哈希模式下，桶里有些物体其实不覆盖这个格子
        Aabb cellBox{{hx * cell_, hy * cell_}, {(hx + 1) * cell_, (hy + 1) * cell_}};
        int n = slotEnd(hs) - slotBegin(hs), strangers = 0, reported = 0;
        for (int k = slotBegin(hs); k < slotEnd(hs); ++k) {
            const Aabb& b = boxes_[entry(k)];
            if (!(b.min.x <= cellBox.max.x && cellBox.min.x <= b.max.x && b.min.y <= cellBox.max.y && cellBox.min.y <= b.max.y)) ++strangers;
            for (int l = k + 1; l < slotEnd(hs); ++l)
                if (overlaps(b, boxes_[entry(l)]) && refSlot(entry(k), entry(l)) == hs) ++reported;
        }
        row("objects here", n, pal.text);
        if (hash_) row("from other cells", strangers, strangers ? pal.frontier : pal.text);
        row("tests here", static_cast<long>(n) * (n - 1) / 2, pal.text);
        row("reported here", reported, pal.path);
    }

    std::vector<Body> bodies_;
    std::vector<Aabb> boxes_;
    std::vector<Pair> pairs_;
    std::optional<UniformGrid> grid_;
    std::optional<SpatialHash> hash_bp_;
    long tests_ = 0;
    bool hash_ = false;
    bool big_ = false;
    float cell_ = 48;
    float count_ = 120;
    float buckets_ = 31;
    std::optional<Vec2> hover_;
};

}  // namespace

int main() {
    static SpatialHashDemo demo;
    ga::DemoConfig config;
    config.title = "Uniform Grid & Spatial Hash";
    config.width = kWidth;
    config.height = kHeight;
    config.stepsPerSecond = 60.0f;
    return ga::runDemo(demo, config);
}
