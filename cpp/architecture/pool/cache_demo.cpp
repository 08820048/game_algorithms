// 缓存行：遍历 64 个对象，只读每个对象的 16 字节热数据（位置、速度）
//   - 一格 = 8 字节；粗竖线是缓存行的边界（64 或 128 字节）
//   - 朱红：真正用到的字节；灰底：读进来的缓存行（用不到的部分也一起读了）
//   - AoS：64 字节的对象连续存放；hot/cold：热数据单独一个数组；pointers：对象散落在堆上，
//     按指针的顺序访问；pool 50%：128 个槽位、一半是空的，遍历时每个槽位都要看一眼 alive
//   - 右上角：读进的缓存行、有效字节占读进字节的比例、不连续的跳转次数（硬件预取难以预测）

#include <algorithm>
#include <cstdio>
#include <numeric>
#include <set>
#include <string>
#include <vector>

#include "../../foundations/random/random.h"
#include "demo.h"
#include "ui.h"

namespace {

using namespace ga;

constexpr int kWidth = 800, kHeight = 480;
constexpr float kTop = 80;
constexpr int kBytes = 8192, kRowBytes = 512, kCell = 8;
constexpr int kObjects = 64;

struct Access {
    int addr, bytes;
    bool useful;
};

class CacheDemo final : public Demo {
public:
    CacheDemo() { build(); }
    void build() {
        trace_.clear();
        objectAt_.assign(kBytes / 64, -1);
        Pcg32 r{4};
        switch (layout_) {
            case 0:  // AoS：对象 i 在 64·i，热数据是前 16 字节
                for (int i = 0; i < kObjects; ++i) trace_.push_back({{i * 64, 16, true}}), objectAt_[i] = i;
                break;
            case 1:  // 热数据数组：16·i
                for (int i = 0; i < kObjects; ++i) trace_.push_back({{i * 16, 16, true}});
                break;
            case 2: {  // 堆上随机的 64 个位置
                std::vector<int> slots(kBytes / 64);
                std::iota(slots.begin(), slots.end(), 0);
                for (int i = int(slots.size()) - 1; i > 0; --i) std::swap(slots[i], slots[r.next() % (i + 1)]);
                for (int i = 0; i < kObjects; ++i) trace_.push_back({{slots[i] * 64, 16, true}}), objectAt_[slots[i]] = i;
                break;
            }
            case 3: {  // 池：128 个 64 字节的槽位，随机一半存活；alive 标志在槽位末尾
                std::vector<int> alive(128, 0);
                for (int k = 0; k < kObjects;) {
                    int s = int(r.next() % 128);
                    if (!alive[s]) alive[s] = 1, ++k;
                }
                for (int s = 0; s < 128; ++s) {
                    std::vector<Access> a{{s * 64 + 56, 8, false}};  // 先读 alive
                    if (alive[s]) a.push_back({s * 64, 16, true}), objectAt_[s] = s;
                    trace_.push_back(a);
                }
                break;
            }
        }
        reset();
    }
    void reset() override {
        pos_ = 0;
        touched_.clear(), useful_.clear();
        jumps_ = 0, lastLine_ = -1, done_ = false;
    }
    bool step() override {
        if (done_) {
            if (++pause_ > 40) pause_ = 0, reset();
            return true;
        }
        int line = int(lineBytes_);
        for (const Access& a : trace_[pos_]) {
            for (int b = a.addr; b < a.addr + a.bytes; b += kCell) {
                int l = b / line;
                if (lastLine_ >= 0 && l != lastLine_ && l != lastLine_ + 1) ++jumps_;
                lastLine_ = l;
                touched_.insert(l);
                if (a.useful) useful_.insert(b / kCell);
            }
        }
        if (++pos_ >= int(trace_.size())) done_ = true;
        return true;
    }

    void draw() const override {
        const Palette& pal = palette();
        const float hl = hairline();
        const int line = int(lineBytes_);
        const float x0 = 16, y0 = kTop + 52, cw = (kWidth - 32.0f) / (kRowBytes / kCell), rh = 15;
        auto cellRect = [&](int byte) {
            int row = byte / kRowBytes, col = (byte % kRowBytes) / kCell;
            return Rectangle{x0 + col * cw, y0 + row * (rh + 4), cw, rh};
        };
        // 读进来的缓存行
        for (int l : touched_) {
            Rectangle a = cellRect(l * line), b = cellRect(l * line + line - kCell);
            DrawRectangleRec({a.x, a.y, b.x + b.width - a.x, rh}, Fade(pal.textMuted, 0.28f));
        }
        // 对象的轮廓（64 字节一个）
        if (layout_ != 1)
            for (int s = 0; s < kBytes / 64; ++s) {
                if (objectAt_[s] < 0 && layout_ != 3) continue;
                Rectangle a = cellRect(s * 64);
                DrawRectangleLinesEx({a.x + 1, a.y + 1, cw * 8 - 2, rh - 2}, hl, objectAt_[s] >= 0 ? pal.text : pal.gridLine);
            }
        for (int c : useful_) DrawRectangleRec(cellRect(c * kCell), pal.path);
        // 当前访问
        if (!done_ && pos_ < int(trace_.size()))
            for (const Access& a : trace_[pos_]) {
                Rectangle r = cellRect(a.addr);
                DrawRectangleLinesEx({r.x - 1, r.y - 1, cw * float(a.bytes / kCell) + 2, rh + 2}, 1.6f, pal.text);
            }
        // 格线与缓存行边界
        for (int row = 0; row < kBytes / kRowBytes; ++row) {
            float y = y0 + row * (rh + 4);
            for (int b = 0; b <= kRowBytes; b += kCell) {
                float x = x0 + float(b / kCell) * cw;
                bool edge = b % line == 0;
                DrawLineEx({x, y}, {x, y + rh}, edge ? 1.4f : hl, edge ? pal.text : Fade(pal.gridLine, 0.6f));
            }
        }
        int loaded = int(touched_.size()) * line, useful = int(useful_.size()) * kCell;
        char buf[200];
        std::snprintf(buf, sizeof buf, "lines loaded %zu (%d B)   useful %d B (%.0f%%)   jumps %d   objects %d / %d", touched_.size(),
                      loaded, useful, loaded ? 100.0 * useful / loaded : 0.0, jumps_, done_ ? kObjects : countDone(), kObjects);
        drawText(buf, {12, kTop + 8}, 11, pal.textMuted);
        drawText("one cell = 8 bytes, one row = 512 bytes; thick lines = cache line boundaries", {12, kTop + 26}, 11, pal.textMuted);
        DrawLineEx({0, kTop}, {float(kWidth), kTop}, hl, pal.gridLine);
    }

    void drawUi() override {
        const char* names[4] = {"AoS", "hot/cold", "pointers", "pool 50%"};
        float x = 12;
        bool ch = false;
        for (int i = 0; i < 4; ++i) {
            if (ui::button({x, 10, 84, 26}, names[i], layout_ == i)) layout_ = i, ch = true;
            x += 88;
        }
        if (ui::button({x + 12, 10, 70, 26}, "64 B", lineBytes_ == 64)) lineBytes_ = 64, ch = true;
        if (ui::button({x + 86, 10, 70, 26}, "128 B", lineBytes_ == 128)) lineBytes_ = 128, ch = true;
        if (ch) build(), notifyProblemChanged();
    }

    std::string status() const override {
        char buf[64];
        std::snprintf(buf, sizeof buf, "%zu lines · %d jumps", touched_.size(), jumps_);
        return buf;
    }

private:
    int countDone() const {
        int n = 0;
        for (int i = 0; i < pos_; ++i)
            for (const Access& a : trace_[i]) n += a.useful;
        return n;
    }

    std::vector<std::vector<Access>> trace_;
    std::vector<int> objectAt_;
    std::set<int> touched_, useful_;
    int pos_ = 0, jumps_ = 0, lastLine_ = -1, pause_ = 0, layout_ = 0;
    float lineBytes_ = 64;
    bool done_ = false;
};

}  // namespace

int main() {
    static CacheDemo demo;
    ga::DemoConfig config;
    config.title = "Cache lines";
    config.width = kWidth;
    config.height = kHeight;
    config.stepsPerSecond = 16.0f;
    return ga::runDemo(demo, config);
}
