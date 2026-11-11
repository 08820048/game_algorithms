#pragma once

// 演示、测试、bench 共用的场景：街区（楼 = 遮挡物，道具 = 被剔除的小物体）和随机盒子

#include <vector>

#include "../../foundations/random/random.h"
#include "culling.h"

namespace ga::culling {

struct City {
    std::vector<Aabb> buildings;               // 遮挡物
    std::vector<std::vector<Vec2>> occluders;  // 楼的四个角（顺时针）
    std::vector<Aabb> props;                   // 小物体：街上和院子里
    std::vector<Vec2> streets;                 // 街道交叉口，视点可以放在这里
};

inline std::vector<Vec2> corners(const Aabb& b) { return {b.min, {b.max.x, b.min.y}, b.max, {b.min.x, b.max.y}}; }

// cols × rows 个街区，每个街区 1～3 栋楼；街宽 street
inline City makeCity(Pcg32& rng, Aabb world, int cols, int rows, float street, int props) {
    City c;
    float bw = (world.max.x - world.min.x) / float(cols), bh = (world.max.y - world.min.y) / float(rows);
    for (int j = 0; j <= rows; ++j)
        for (int i = 0; i <= cols; ++i) c.streets.push_back({world.min.x + float(i) * bw, world.min.y + float(j) * bh});
    for (int j = 0; j < rows; ++j)
        for (int i = 0; i < cols; ++i) {
            Aabb lot{{world.min.x + float(i) * bw + street / 2, world.min.y + float(j) * bh + street / 2},
                     {world.min.x + float(i + 1) * bw - street / 2, world.min.y + float(j + 1) * bh - street / 2}};
            float w = lot.max.x - lot.min.x, h = lot.max.y - lot.min.y;
            int kind = int(rng.next() % 4);
            if (kind == 0) {  // 一栋整楼
                c.buildings.push_back(lot);
            } else if (kind == 1) {  // 左右两栋，中间一条窄巷
                float cut = w * (0.35f + 0.3f * rng.nextFloat()), gap = 0.06f * w + 2;
                c.buildings.push_back({lot.min, {lot.min.x + cut - gap / 2, lot.max.y}});
                c.buildings.push_back({{lot.min.x + cut + gap / 2, lot.min.y}, lot.max});
            } else if (kind == 2) {  // 上下两栋
                float cut = h * (0.35f + 0.3f * rng.nextFloat()), gap = 0.06f * h + 2;
                c.buildings.push_back({lot.min, {lot.max.x, lot.min.y + cut - gap / 2}});
                c.buildings.push_back({{lot.min.x, lot.min.y + cut + gap / 2}, lot.max});
            } else {  // 广场：只有一栋小楼
                Vec2 m{w * 0.25f, h * 0.25f};
                c.buildings.push_back({lot.min + m, lot.max - m});
            }
        }
    for (const Aabb& b : c.buildings) c.occluders.push_back(corners(b));
    while (int(c.props.size()) < props) {
        float s = 2 + 4 * rng.nextFloat();
        Vec2 p{world.min.x + (world.max.x - world.min.x) * rng.nextFloat(), world.min.y + (world.max.y - world.min.y) * rng.nextFloat()};
        Aabb box{p - Vec2{s, s}, p + Vec2{s, s}};
        bool clash = false;
        for (const Aabb& b : c.buildings)
            if (overlaps(b, box)) clash = true;
        if (!clash) c.props.push_back(box);
    }
    return c;
}

// 随机盒子：大小服从"多数小、少数大"
inline std::vector<Aabb> randomBoxes(Pcg32& rng, Aabb world, int n, float small, float large) {
    std::vector<Aabb> out;
    for (int i = 0; i < n; ++i) {
        float u = rng.nextFloat();
        float hw = small + (large - small) * u * u * u, hh = hw * (0.4f + 1.2f * rng.nextFloat());
        Vec2 p{world.min.x + (world.max.x - world.min.x) * rng.nextFloat(), world.min.y + (world.max.y - world.min.y) * rng.nextFloat()};
        out.push_back({p - Vec2{hw, hh}, p + Vec2{hw, hh}});
    }
    return out;
}

}  // namespace ga::culling
