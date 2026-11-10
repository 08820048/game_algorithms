// Voronoi 图与 Lloyd 松弛
//   - 每个单元是离对应点最近的区域；每一步把所有点移到自己单元的质心，再重新计算
//   - 颜色（只看不碰边框的内部单元）：五边形黄、七边形红、其他（四边形、八边形……）深色，六边形不着色
//   - 白噪声起点几次迭代就变得均匀，但要几百次才接近蜂窝；五边形和七边形总是成对出现（晶格的位错）
//   - 右侧：能量（Σ∫|x − 点|²，以同样点数的完美六边形为 1）随迭代下降的曲线，以及邻居数的分布

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "../poisson/poisson.h"
#include "delaunay.h"
#include "demo.h"
#include "ui.h"

namespace {

using namespace ga;
using namespace ga::delaunay;

constexpr int kWidth = 800, kHeight = 480;
constexpr float kTop = 80;
constexpr float kMx = 12, kMy = kTop + 10, kDw = 520, kDh = 380;
constexpr float kPx = 550;
constexpr int kMaxIter = 400;

class VoronoiDemo final : public Demo {
public:
    VoronoiDemo() { generate(); }
    void reset() override {
        pts_ = start_;
        iter_ = 0;
        energy_.clear();
        rebuild();
    }
    bool step() override {
        if (iter_ >= kMaxIter) return false;
        LloydStep r = lloyd(pts_, kDw, kDh);
        lastMove_ = r.meanMove;
        ++iter_;
        rebuild();
        return iter_ < kMaxIter;
    }

    void draw() const override {
        const Palette& pal = palette();
        const float hl = hairline();
        if (colour_)
            for (size_t i = 0; i < cells_.size(); ++i) {
                Color c = BLANK;
                int k = int(nb_[i].size());
                if (boundary_[i]) c = Fade(pal.gridLine, 0.25f);
                else if (k == 5) c = Fade(pal.frontier, 0.55f);
                else if (k == 7) c = Fade(pal.path, 0.38f);
                else if (k != 6) c = Fade(pal.text, 0.55f);
                if (c.a) fillPoly(cells_[i], c);
            }
        if (showDelaunay_)
            for (size_t i = 0; i < nb_.size(); ++i)
                for (int j : nb_[i])
                    if (int(i) < j) DrawLineEx(sp(pts_[i]), sp(pts_[j]), hl * 1.2f, Fade(pal.textMuted, 0.8f));
        for (const Poly& c : cells_)
            for (size_t k = 0; k < c.size(); ++k) {
                P2 a = c[k], b = c[(k + 1) % c.size()];
                DrawLineEx({kMx + float(a.x), kMy + float(a.y)}, {kMx + float(b.x), kMy + float(b.y)}, 1.1f, pal.text);
            }
        for (const Vec2& p : pts_) DrawCircleV(sp(p), 2.0f, pal.text);
        DrawRectangleLinesEx({kMx - 1, kMy - 1, kDw + 2, kDh + 2}, hl, pal.gridLine);

        char buf[128];
        float y = kMy;
        auto line = [&](const char* s, Color col, float size = 12) {
            drawText(s, {kPx, y}, size, col);
            y += size + 7;
        };
        int counts[10] = {}, interior = 0;
        for (size_t i = 0; i < nb_.size(); ++i)
            if (!boundary_[i]) ++counts[std::min<size_t>(9, nb_[i].size())], ++interior;
        std::snprintf(buf, sizeof buf, "iteration %d", iter_);
        line(buf, pal.text, 13);
        std::snprintf(buf, sizeof buf, "energy %.4f x hexagons", energy_.empty() ? 0.0 : energy_.back());
        line(buf, pal.text);
        std::snprintf(buf, sizeof buf, "hexagons %.1f%% of interior", interior ? 100.0 * counts[6] / interior : 0.0);
        line(buf, pal.text);
        std::snprintf(buf, sizeof buf, "last move %.4f x spacing", lastMove_ / spacing());
        line(buf, iter_ ? pal.text : pal.textMuted);

        // 能量曲线：纵轴是 energy − 1（对数），横轴是迭代次数（对数）
        const float cx = kPx, cy = y + 16, cw = 206, ch = 110;
        drawText("energy - 1  (log-log)", {cx, cy - 15}, 10, pal.textMuted);
        DrawRectangleLinesEx({cx, cy, cw, ch}, hl, pal.gridLine);
        auto px = [&](int it) { return cx + cw * float(std::log10(1.0 + it) / std::log10(1.0 + kMaxIter)); };
        auto py = [&](double e) { return cy + ch * float(std::clamp((0.5 - std::log10(std::max(e - 1, 1e-3))) / 3.5, 0.0, 1.0)); };
        for (double g : {1.0, 0.1, 0.01}) {
            float yy = py(1 + g);
            DrawLineEx({cx, yy}, {cx + cw, yy}, hl, Fade(pal.gridLine, 0.6f));
            std::snprintf(buf, sizeof buf, "%g", g);
            drawText(buf, {cx + cw + 4, yy - 6}, 9, pal.textMuted);
        }
        for (size_t k = 1; k < energy_.size(); ++k) DrawLineEx({px(int(k) - 1), py(energy_[k - 1])}, {px(int(k)), py(energy_[k])}, 1.8f, pal.path);

        // 邻居数分布
        float hy = cy + ch + 34;
        drawText("neighbours (interior cells)", {cx, hy - 16}, 10, pal.textMuted);
        for (int k = 4; k <= 8; ++k) {
            float frac = interior ? float(counts[k]) / interior : 0;
            float bx = cx + (k - 4) * 46, bh = 70 * frac;
            Color c = k == 5 ? pal.frontier : k == 7 ? pal.path : k == 6 ? pal.textMuted : pal.text;
            DrawRectangleRec({bx, hy + 70 - bh, 34, bh}, c);
            std::snprintf(buf, sizeof buf, "%d", k);
            drawText(buf, {bx + 13, hy + 74}, 10, pal.textMuted);
        }
        DrawLineEx({0, kTop}, {float(kWidth), kTop}, hl, pal.gridLine);
    }

    void drawUi() override {
        bool ch = false;
        const char* names[] = {"white", "bridson"};
        for (int i = 0; i < 2; ++i)
            if (ui::button({12.0f + i * 80, 10, 76, 26}, names[i], start_kind_ == i)) start_kind_ = i, ch = true;
        ch |= ui::slider({180, 10, 150, 26}, "seed", &seed_, 1, 50, "%.0f", 1);
        ch |= ui::slider({340, 10, 190, 26}, "points", &n_, 30, 600, "%.0f", 1);
        ui::toggle({12, 44, 100, 26}, "delaunay", &showDelaunay_);
        ui::toggle({120, 44, 100, 26}, "colour", &colour_);
        if (ch) generate(), notifyProblemChanged();
    }

    std::string status() const override {
        char buf[96];
        std::snprintf(buf, sizeof buf, "iteration %d · energy %.4f x hexagons", iter_, energy_.empty() ? 0.0 : energy_.back());
        return buf;
    }

private:
    Vector2 sp(Vec2 p) const { return {kMx + p.x, kMy + p.y}; }
    double spacing() const { return std::sqrt(double(kDw) * kDh / std::max<size_t>(1, pts_.size())); }
    void fillPoly(const Poly& c, Color col) const {
        // 单元的顶点顺序继承自边界框，在屏幕上是顺时针；raylib 要求逆时针，所以反过来画
        for (size_t k = 1; k + 1 < c.size(); ++k)
            DrawTriangle({kMx + float(c[0].x), kMy + float(c[0].y)}, {kMx + float(c[k + 1].x), kMy + float(c[k + 1].y)}, {kMx + float(c[k].x), kMy + float(c[k].y)},
                         col);
    }
    void rebuild() {
        Triangulation tr = triangulate(pts_);
        nb_ = tr.neighbors();
        cells_ = voronoiCells(pts_, nb_, kDw, kDh);
        boundary_.assign(pts_.size(), 0);
        double e = 0;
        for (size_t i = 0; i < pts_.size(); ++i) {
            for (P2 v : cells_[i]) boundary_[i] |= v.x < 1e-6 || v.y < 1e-6 || v.x > kDw - 1e-6 || v.y > kDh - 1e-6;
            e += secondMoment(cells_[i], pts_[i]);
        }
        double hex = 5 * std::sqrt(3.0) / 54 * (double(kDw) * kDh / pts_.size()) * kDw * kDh;
        energy_.push_back(e / hex);
    }
    void generate() {
        Pcg32 rng{uint64_t(seed_)};
        int n = int(n_);
        if (start_kind_ == 0) {
            start_ = poisson::whiteNoise(kDw, kDh, n, rng);
        } else {
            // 半径取得让点数接近 n（Bridson 覆盖率约 0.48），再截到 n 个以内
            float r = std::sqrt(0.48f * 4 * kDw * kDh / (3.14159f * n));
            poisson::BridsonParams prm;
            start_ = poisson::bridson(kDw, kDh, r, rng, prm);
            if (int(start_.size()) > n) start_.resize(n);
        }
        reset();
    }

    int start_kind_ = 0;
    float seed_ = 2, n_ = 260;
    bool showDelaunay_ = false, colour_ = true;
    std::vector<Vec2> start_, pts_;
    std::vector<std::vector<int>> nb_;
    std::vector<Poly> cells_;
    std::vector<char> boundary_;
    std::vector<double> energy_;
    int iter_ = 0;
    double lastMove_ = 0;
};

}  // namespace

int main() {
    static VoronoiDemo demo;
    ga::DemoConfig config;
    config.title = "Voronoi diagram and Lloyd relaxation";
    config.width = kWidth;
    config.height = kHeight;
    config.stepsPerSecond = 3.0f;
    return ga::runDemo(demo, config);
}
