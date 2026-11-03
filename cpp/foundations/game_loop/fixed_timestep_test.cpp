#include "fixed_timestep.h"

#include <cmath>
#include <vector>

#include "check.h"

using namespace ga;

int main() {
    const double dt = 1.0 / 60.0;

    // 一帧 55ms，按 1/60s 一步：走 3 步，剩下 0.3 步
    {
        FixedTimestep ts(dt);
        int calls = 0;
        int steps = ts.advance(0.055, [&](double) { ++calls; });
        CHECK(steps == 3);
        CHECK(calls == 3);
        CHECK(ts.alpha() >= 0.0 && ts.alpha() < 1.0);
    }
    // 帧时间不足一步：一步都不走，时间留在蓄水池里
    {
        FixedTimestep ts(dt);
        CHECK(ts.advance(0.01, [](double) {}) == 0);
        CHECK(std::abs(ts.alpha() - 0.6) < 1e-9);
        CHECK(ts.advance(0.01, [](double) {}) == 1);  // 累计 20ms，够一步了
    }
    // 死亡螺旋保护：卡顿 2 秒，最多只补 maxFrameTime 的时间
    {
        FixedTimestep ts(dt, 0.26);
        int steps = ts.advance(2.0, [](double) {});
        CHECK(steps == 15);  // 0.26s 只够 15 步，而不是 120 步
    }
    // 确定性：同样的总时长，无论怎么切分成帧，固定步长的结果完全一致（逐位相同）
    {
        const Vec2 g{0, 1500};
        auto runFixed = [&](const std::vector<double>& frames) {
            FixedTimestep ts(dt);
            Body b{{0, 0}, {160, -560}};
            for (double f : frames) ts.advance(f, [&](double h) { integrate(b, g, static_cast<float>(h)); });
            return b;
        };
        // 总时长都是 1.005s（避开恰好整除步长的边界），一个匀速 60 帧，一个快慢交替
        std::vector<double> even(60, 1.0 / 60.0);
        even.push_back(0.005);
        std::vector<double> uneven;
        for (int i = 0; i < 30; ++i) {
            uneven.push_back(1.0 / 40.0);
            uneven.push_back(1.0 / 120.0);
        }
        uneven.push_back(0.005);
        Body a = runFixed(even);
        Body b = runFixed(uneven);
        CHECK(a.position == b.position);
        CHECK(a.velocity == b.velocity);
    }
    // 对照：可变步长下，不同帧率得到的跳跃最高点不同
    {
        auto peak = [](double frameTime) {
            Body b{{0, 0}, {0, -560}};
            float best = 0;
            for (int i = 0; i < 1000 && b.velocity.y < 0; ++i) {
                variableStep(b, {0, 1500}, frameTime);
                best = std::min(best, b.position.y);
            }
            return -best;
        };
        float at15 = peak(1.0 / 15.0);
        float at144 = peak(1.0 / 144.0);
        CHECK(at15 > at144 + 10.0f);  // 低帧率跳得更高
    }
    // 插值：alpha = 0 / 1 时分别等于上一个 / 当前状态
    {
        Vec2 a{0, 0}, b{10, 20};
        CHECK(interpolate(a, b, 0.0) == a);
        CHECK(interpolate(a, b, 1.0) == b);
        CHECK(interpolate(a, b, 0.5) == (Vec2{5, 10}));
    }

    // 游戏循环骨架：用假时钟驱动 10 帧、每帧 26ms，共 260ms -> 15 次更新、10 次渲染
    {
        double clock = 0;
        int frames = 0, updates = 0, renders = 0;
        runGameLoop(
            1.0 / 60.0, [&] { return clock; }, [&] { clock += 0.026; return frames++ < 10; },
            [&](double) { ++updates; }, [&](double alpha) { ++renders; CHECK(alpha >= 0 && alpha < 1); });
        CHECK(renders == 10);
        CHECK(updates == 15);
    }

    return report("fixed_timestep_test");
}
