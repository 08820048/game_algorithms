#include "interpolation.h"

#include <cmath>

#include "check.h"

using namespace ga;

static bool near(float a, float b, float eps = 1e-4f) { return std::abs(a - b) <= eps; }

int main() {
    // lerp / inverseLerp / remap
    {
        CHECK(lerp(10, 20, 0.0f) == 10);
        CHECK(lerp(10, 20, 1.0f) == 20);
        CHECK(lerp(10, 20, 0.25f) == 12.5f);
        CHECK(inverseLerp(10, 20, 15) == 0.5f);
        CHECK(remap(50, 0, 100, 0, 200) == 100);  // 50% 血量 -> 100 像素宽
    }
    // 角度插值走短的一边：从 170° 到 -170° 只转 20°
    {
        float a = 170 * kPi / 180, b = -170 * kPi / 180;
        float mid = lerpAngle(a, b, 0.5f);
        CHECK(near(std::cos(mid), -1.0f, 1e-3f));  // 中点在 180°，而不是 0°
        Vec2 s = slerp({1, 0}, {0, 1}, 0.5f);
        CHECK(near(length(s), 1.0f));
        CHECK(near(s.x, s.y));
    }
    // 所有缓动函数的端点都是 0 和 1
    {
        EaseFn fns[] = {ease::linear, ease::inQuad, ease::outQuad, ease::inOutQuad, ease::inCubic,
                        ease::outCubic, ease::inOutCubic, ease::inOutSine, ease::smoothstep,
                        ease::outBack, ease::outElastic, ease::outBounce};
        for (EaseFn f : fns) {
            CHECK(near(f(0.0f), 0.0f, 1e-3f));
            CHECK(near(f(1.0f), 1.0f, 1e-3f));
        }
        CHECK(ease::outBack(0.7f) > 1.0f);  // outBack 会过冲
        CHECK(ease::inQuad(0.5f) < 0.5f && ease::outQuad(0.5f) > 0.5f);
    }
    // 朴素跟随依赖帧率：同样 1 秒，30fps 和 120fps 剩余距离不同
    {
        auto remaining = [](int fps) {
            Vec2 p{0, 0};
            for (int i = 0; i < fps; ++i) p = naiveFollow(p, {100, 0}, 0.1f);
            return 100 - p.x;
        };
        CHECK(remaining(30) > remaining(120) + 1.0f);
    }
    // 指数衰减与帧率无关：同样 1 秒，不同帧率结果几乎一致
    {
        auto remaining = [](int fps) {
            float p = 0;
            for (int i = 0; i < fps; ++i) p = expDecay(p, 100.0f, 6.0f, 1.0f / fps);
            return 100 - p;
        };
        CHECK(near(remaining(30), remaining(144), 1e-3f));
        CHECK(near(remaining(60), 100 * std::exp(-6.0f), 1e-3f));
    }
    // smoothDamp：最终到达目标，且从静止出发时不会越过目标
    {
        float p = 0, v = 0, maxP = 0;
        for (int i = 0; i < 600; ++i) {
            p = smoothDamp(p, 100, v, 0.3f, 1.0f / 60);
            maxP = std::max(maxP, p);
        }
        CHECK(near(p, 100, 1e-2f));
        CHECK(maxP <= 100.001f);
    }

    return report("interpolation_test");
}
