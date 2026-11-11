// 遍历中删除
//   - 32 个实体排成一个数组。系统每一遍从头到尾处理每个实体：年龄加一，寿命到了就删除
//   - forward + erase：删除后后面的元素整体前移，游标照常前进 → 跳过了下一个
//   - forward + swap：交换删除把最后一个搬到当前位置，游标前进 → 搬来的那个被跳过
//   - reverse + swap：倒序遍历，搬来的元素已经处理过 → 正确
//   - deferred：遍历时只记下要删的，遍历结束后再删（命令缓冲）→ 正确
//   - 一遍结束时，没有被处理的实体闪红：它这一帧没有更新（少扣一次血、少走一步）

#include <algorithm>
#include <cstdio>
#include <string>
#include <vector>

#include "../../foundations/random/random.h"
#include "demo.h"
#include "ui.h"

namespace {

using namespace ga;

constexpr int kWidth = 800, kHeight = 480;
constexpr float kTop = 80;
constexpr int kCount = 32;

struct Item {
    int id, age, life;
    bool updated = false;
};

class IterateDemo final : public Demo {
public:
    IterateDemo() { reset(); }
    void reset() override {
        rng_ = Pcg32{7};
        items_.clear();
        next_ = 0, passes_ = 0, skipped_ = 0, updates_ = 0, killed_ = 0;
        for (int i = 0; i < kCount; ++i) {
            items_.push_back(spawn());
            items_.back().age = int(rng_.next() % uint32_t(items_.back().life));  // 一开始年龄随机，第一遍就有实体死亡
        }
        beginPass();
    }
    bool step() override {
        if (flash_ > 0) {  // 一遍结束后停一会儿，让被跳过的实体闪一下
            if (--flash_ == 0) refill(), beginPass();
            return true;
        }
        bool reverse = mode_ == 2;
        if (reverse ? cursor_ < 0 : cursor_ >= int(items_.size())) {
            endPass();
            return true;
        }
        Item& it = items_[cursor_];
        it.updated = true, ++it.age, ++updates_;
        lastMoved_ = -1;
        if (it.age >= it.life) {
            ++killed_;
            if (mode_ == 3) {
                pendingKill_.push_back(it.id);
            } else if (mode_ == 0) {
                items_.erase(items_.begin() + cursor_);
                lastMoved_ = cursor_;  // 后面的整体前移了一格
            } else {
                if (cursor_ != int(items_.size()) - 1) items_[cursor_] = items_.back(), lastMoved_ = cursor_;
                items_.pop_back();
            }
        }
        cursor_ += reverse ? -1 : 1;
        return true;
    }

    void draw() const override {
        const Palette& pal = palette();
        const float hl = hairline();
        const float x0 = 24, y0 = kTop + 70, w = 92, h = 64;
        for (int i = 0; i < kCount; ++i) {
            float x = x0 + float(i % 8) * w, y = y0 + float(i / 8) * (h + 16);
            Rectangle r{x, y, w - 8, h};
            DrawRectangleLinesEx(r, hl, pal.gridLine);
            if (i >= int(items_.size())) continue;
            const Item& it = items_[i];
            bool skipped = flash_ > 0 && !it.updated && it.id < passStartNext_;
            bool pending = std::find(pendingKill_.begin(), pendingKill_.end(), it.id) != pendingKill_.end();
            Color fill = skipped ? pal.path : it.updated ? Fade(pal.text, 0.85f) : Fade(pal.textMuted, 0.18f);
            DrawRectangleRec(r, fill);
            Color tc = (skipped || it.updated) ? pal.background : pal.text;
            char b[32];
            std::snprintf(b, sizeof b, "#%d", it.id);
            drawText(b, {x + 6, y + 6}, 12, tc);
            std::snprintf(b, sizeof b, "age %d/%d", it.age, it.life);
            drawText(b, {x + 6, y + 26}, 11, tc);
            if (pending) drawText("delete", {x + 6, y + 44}, 10, pal.path);
            if (i == lastMoved_ && flash_ == 0) DrawRectangleLinesEx({x - 2, y - 2, w - 4, h + 4}, 1.6f, pal.path);
        }
        if (flash_ == 0 && cursor_ >= 0 && cursor_ < kCount) {
            float x = x0 + float(cursor_ % 8) * w, y = y0 + float(cursor_ / 8) * (h + 16);
            drawArrow({x + (w - 8) / 2, y - 18}, {x + (w - 8) / 2, y - 3}, 2, pal.text, 7);
        }
        char buf[200];
        std::snprintf(buf, sizeof buf, "passes %d   updates %ld   deleted %ld   skipped (not updated in a pass) %ld", passes_, updates_, killed_,
                      skipped_);
        drawText(buf, {12, kTop + 8}, 11, pal.textMuted);
        const char* explain[4] = {"erase shifts the rest left; the cursor moves on and misses the element that slid in",
                                  "swap-remove moves the last element here; the cursor moves on and misses it",
                                  "reverse order: the element moved in from the end was already processed",
                                  "deletions are queued and applied after the pass: the array never changes mid-pass"};
        drawText(explain[mode_], {12, kTop + 28}, 11, mode_ < 2 ? pal.path : pal.textMuted);
        DrawLineEx({0, kTop}, {float(kWidth), kTop}, hl, pal.gridLine);
    }

    void drawUi() override {
        const char* names[4] = {"forward + erase", "forward + swap", "reverse + swap", "deferred"};
        float x = 12;
        for (int i = 0; i < 4; ++i) {
            float w = i == 3 ? 90 : 136;
            if (ui::button({x, 10, w, 26}, names[i], mode_ == i) && mode_ != i) mode_ = i, reset(), notifyProblemChanged();
            x += w + 6;
        }
    }

    std::string status() const override {
        char buf[64];
        std::snprintf(buf, sizeof buf, "skipped %ld of %ld updates", skipped_, updates_ + skipped_);
        return buf;
    }

private:
    Item spawn() { return {next_++, 0, 2 + int(rng_.next() % 5)}; }
    void beginPass() {
        for (Item& it : items_) it.updated = false;
        passStartNext_ = next_;
        pendingKill_.clear();
        cursor_ = mode_ == 2 ? int(items_.size()) - 1 : 0;
        lastMoved_ = -1;
    }
    void endPass() {
        for (const Item& it : items_)
            if (!it.updated) ++skipped_;
        if (mode_ == 3)
            for (int id : pendingKill_)
                std::erase_if(items_, [id](const Item& it) { return it.id == id; });
        ++passes_;
        flash_ = 12;
    }
    void refill() {
        pendingKill_.clear();
        while (int(items_.size()) < kCount) items_.push_back(spawn());
    }

    std::vector<Item> items_;
    std::vector<int> pendingKill_;
    Pcg32 rng_{7};
    int next_ = 0, cursor_ = 0, mode_ = 1, passes_ = 0, flash_ = 0, lastMoved_ = -1, passStartNext_ = 0;
    long skipped_ = 0, updates_ = 0, killed_ = 0;
};

}  // namespace

int main() {
    static IterateDemo demo;
    ga::DemoConfig config;
    config.title = "Deleting while iterating";
    config.width = kWidth;
    config.height = kHeight;
    config.stepsPerSecond = 16.0f;
    return ga::runDemo(demo, config);
}
