#pragma once

// 演示、测试与性能对比共用的例子：一名士兵与一个敌人。
// 士兵每次决策时读取四个输入（血量、弹药、与敌人的距离、与医疗点的距离），
// 用效用评分在六个动作中选一个：攻击、换弹、追击、治疗、逃跑、巡逻。
// 敌人在场地里游荡，靠近时会持续造成伤害；士兵没有任何"状态转移"，每一刻都重新打分。

#include <array>
#include <vector>

#include "../../foundations/random/random.h"
#include "utility.h"
#include "vec2.h"

namespace ga::soldier {

using namespace ga::utility;

// #region inputs
enum Input { Health, Ammo, EnemyDist, MedkitDist, Exposure, kInputCount };  // Exposure：去医疗点的路上，有多长一段在敌人射程内
enum ActionId { Attack, Reload, Chase, Heal, Flee, Patrol, kActionCount };

constexpr float kArenaW = 440, kArenaH = 420;
constexpr float kMaxHealth = 100, kClip = 8;
constexpr float kAttackRange = 200;  // 士兵的射程
constexpr float kEnemyRange = 160;   // 敌人的射程
const Vec2 kMedkits[] = {{56, 56}, {384, 364}};  // 两个医疗点，分别在对角
// #endregion

// #region actions
inline std::vector<Action> actions() {
    return {
        {"attack", 1, {
            {"has ammo",   Ammo,      0, kClip, power(0.3f)},             // 有一点子弹就够了
            {"in range",   EnemyDist, 0, 400,   logistic(0.5f, -20)},      // 200 像素以内
            {"healthy",    Health,    0, 100,   linear(0.6f, 0.4f)},       // 血少时略微犹豫
        }},
        {"reload", 1, {
            {"clip empty", Ammo,      kClip, 0, power(2)},                 // 越空越想换
            {"safe",       EnemyDist, 0, 400,   {Curve::Logistic, 12, 0.7f, 0.3f, 0.45f}},  // 远处更安全，但最低 0.3
        }},
        {"chase", 1, {
            {"out of range", EnemyDist, 0, 400, logistic(0.5f, 20)},
            {"in sight",     EnemyDist, 0, 400, logistic(0.85f, -25)},     // 340 像素以外就看不见了
            {"has ammo",     Ammo,      0, kClip, power(0.5f)},
            {"healthy",      Health,    0, 100, linear()},
        }},
        {"heal", 1, {
            {"wounded",      Health,     100, 0, power(2)},                // 血越少越急
            {"medkit near",  MedkitDist, 0, 600, linear(-0.7f, 1)},        // 近的更好，远的也不至于 0
            {"safe route",   Exposure,  0, 200, logistic(0.4f, -12)},      // 路上暴露在敌人射程内的长度
        }},
        {"flee", 1, {
            {"near death",   Health,    0, 100, logistic(0.35f, -20)},     // 35 血以下急剧上升
            {"enemy close",  EnemyDist, 0, 400, logistic(0.4f, -15)},   // 敌人射程（160）附近开始下降
            {"no medkit",    MedkitDist, 0, 100, linear(0.8f, 0.2f)},   // 已经站在医疗点上，就别跑了
        }},
        {"patrol", 0.12f, {}},  // 没有考量：得分就是权重，作为"什么都不想做"时的保底
    };
}
// #endregion

struct World {
    Vec2 agent{220, 300}, enemy{330, 120}, enemyGoal{330, 120};
    float health = kMaxHealth;
    int ammo = int(kClip);
    float enemyHealth = 150;
    float respawn = 0;     // 敌人被消灭后，过多久重新出现
    int action = -1;
    float fireTimer = 0, reloadTimer = 0;
    int patrolPoint = 0;
    float hitFlash = 0;    // 敌人刚被击中（仅用于绘制）
    int downs = 0, kills = 0;
    bool enemyAlive() const { return respawn <= 0; }
};

constexpr float kReloadTime = 1.2f;

// 线段 a→b 落在圆（圆心 c、半径 r）内的长度：沿这条路线走，有多远会暴露在敌人的射程内
inline float exposure(Vec2 a, Vec2 b, Vec2 c, float r) {
    Vec2 d = b - a, f = a - c;
    float A = dot(d, d), B = 2 * dot(f, d), C = dot(f, f) - r * r;
    if (A < 1e-6f) return 0;
    float disc = B * B - 4 * A * C;
    if (disc <= 0) return 0;
    float sq = std::sqrt(disc);
    float t1 = std::max((-B - sq) / (2 * A), 0.0f), t2 = std::min((-B + sq) / (2 * A), 1.0f);
    return t2 > t1 ? (t2 - t1) * std::sqrt(A) : 0;
}

// 去哪个医疗点：路上暴露得少的那个；都差不多时选近的
inline int bestMedkit(const World& w) {
    auto cost = [&](Vec2 m) {
        float e = w.enemyAlive() ? exposure(w.agent, m, w.enemy, kEnemyRange) : 0;
        return 2 * e + distance(w.agent, m);
    };
    return cost(kMedkits[1]) < cost(kMedkits[0]) ? 1 : 0;
}

// #region compute-inputs
// 有些输入直接读取（血量、弹药），有些需要计算：比如"去医疗点的路上有多长一段暴露在敌人射程内"
inline std::array<float, kInputCount> inputs(const World& w) {
    Vec2 medkit = kMedkits[bestMedkit(w)];
    if (!w.enemyAlive())  // 没有敌人：当作在很远的地方
        return {w.health, float(w.ammo), 1000, distance(w.agent, medkit), 0};
    return {w.health, float(w.ammo), distance(w.agent, w.enemy), distance(w.agent, medkit),
            exposure(w.agent, medkit, w.enemy, kEnemyRange)};
}
// #endregion

inline Vec2 clampToArena(Vec2 p, float margin = 14) {
    return {std::clamp(p.x, margin, kArenaW - margin), std::clamp(p.y, margin, kArenaH - margin)};
}

inline void moveToward(Vec2& p, Vec2 target, float speed, float dt) {
    Vec2 d = target - p;
    float len = length(d);
    if (len <= speed * dt) p = target;
    else p += d / len * (speed * dt);
}

// 执行选中的动作 dt 秒，并结算敌人的伤害。换弹需要连续执行 kReloadTime 秒，中途换成别的动作就前功尽弃
inline void act(World& w, int action, float dt, Pcg32& rng) {
    const Vec2 patrol[] = {{120, 330}, {330, 340}, {360, 200}, {200, 200}};
    bool alive = w.enemyAlive();
    float dist = alive ? distance(w.agent, w.enemy) : 1e9f;
    w.fireTimer = std::max(0.0f, w.fireTimer - dt);
    if (action != Reload) w.reloadTimer = 0;
    switch (action) {
    case Attack:
        if (dist < kAttackRange && w.ammo > 0 && w.fireTimer <= 0) {
            --w.ammo, w.fireTimer = 0.35f, w.hitFlash = 0.12f;
            w.enemyHealth -= 10;
        }
        break;
    case Reload:
        if ((w.reloadTimer += dt) >= kReloadTime) w.ammo = int(kClip), w.reloadTimer = 0;
        break;
    case Chase:
        if (dist > 140) moveToward(w.agent, w.enemy, 80, dt);
        break;
    case Heal:
    {
        Vec2 medkit = kMedkits[bestMedkit(w)];
        moveToward(w.agent, medkit, 90, dt);
        if (distance(w.agent, medkit) < 1) w.health = std::min(kMaxHealth, w.health + 30 * dt);
        break;
    }
    case Flee: {  // 在 16 个方向里选一个：离敌人远、不靠墙角，最好还能靠近安全的医疗点
        Vec2 medkit = kMedkits[bestMedkit(w)];
        auto safety = [&](Vec2 p) {
            float wall = std::min(std::min(p.x, kArenaW - p.x), std::min(p.y, kArenaH - p.y));
            return distance(p, w.enemy) + std::min(wall, 80.0f) - 0.6f * distance(p, medkit);
        };
        Vec2 best = w.agent;
        for (int k = 0; k < 16 && alive; ++k) {
            Vec2 p = clampToArena(w.agent + fromAngle(k * kPi / 8) * 60, 20);
            if (safety(p) > safety(best)) best = p;
        }
        moveToward(w.agent, best, 95, dt);
        break;
    }
    case Patrol:
        if (distance(w.agent, patrol[w.patrolPoint]) < 2) w.patrolPoint = (w.patrolPoint + 1) % 4;
        moveToward(w.agent, patrol[w.patrolPoint], 45, dt);
        break;
    }
    w.hitFlash = std::max(0.0f, w.hitFlash - dt);
    if (alive) {
        if (distance(w.agent, w.enemy) < kEnemyRange) w.health -= 16 * dt;
        if (w.enemyHealth <= 0) ++w.kills, w.respawn = 4;
    } else if ((w.respawn -= dt) <= 0) {  // 在离士兵较远的地方重新出现
        for (int tries = 0; tries < 50; ++tries) {
            w.enemy = {30 + rng.nextFloat() * (kArenaW - 60), 30 + rng.nextFloat() * (kArenaH - 60)};
            if (distance(w.enemy, w.agent) > 240) break;
        }
        w.enemyGoal = w.enemy, w.enemyHealth = 150;
    }
    if (w.health <= 0) {  // 被击倒：回到医疗点旁边重来
        ++w.downs;
        w.health = kMaxHealth, w.ammo = int(kClip), w.agent = kMedkits[0] + Vec2{30, 30};
    }
}

// 敌人：在场地里随机游荡
inline void moveEnemy(World& w, float dt, Pcg32& rng) {
    if (!w.enemyAlive()) return;
    if (distance(w.enemy, w.enemyGoal) < 2)
        w.enemyGoal = {40 + rng.nextFloat() * (kArenaW - 80), 40 + rng.nextFloat() * (kArenaH - 80)};
    moveToward(w.enemy, w.enemyGoal, 32, dt);
}

}  // namespace ga::soldier
