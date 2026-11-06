#pragma once

// 一个守卫的 AI，用三种方式实现同样的行为：
//   1. buildGuardHfsm   分层状态机：Alive { Active { Calm { Patrol, Investigate }, Combat { Chase, Attack }, Flee }, Stunned }, Dead
//   2. buildGuardFlat   扁平状态机：同样的行为，但每个状态都要自己写一遍"死亡""被击晕""血量低"的转移；
//                       被击晕后要回到原来的状态，只能为每个来源复制一个 Stunned
//   3. GuardSwitch      手写 switch：用一个 resume 变量记住被击晕前的状态
// 单元测试用随机输入驱动三者，要求每一步所处的状态完全相同

#include <string>

#include "fsm.h"

namespace ga {

// #region context
struct GuardCtx {
    // 感知输入（由游戏每帧写入）
    float t = 0;            // 当前时间（秒）
    bool seesPlayer = false;
    float playerDist = 10;  // 与玩家的距离（米）
    bool heardNoise = false;
    bool stunEvent = false;  // 这一帧被闪光弹击中
    float health = 100;
    float lastSeen = -1e9f;  // 最后一次看到玩家的时刻

    // 状态机自己维护的计时器
    float investigateUntil = 0;
    float stunUntil = 0;
};

// 所有转移条件集中在这里，三种实现共用
namespace guard {
inline bool dead(const GuardCtx& c) { return c.health <= 0; }
inline bool stunned(const GuardCtx& c) { return c.stunEvent; }
inline bool lowHealth(const GuardCtx& c) { return c.health < 30; }
inline bool recovered(const GuardCtx& c) { return c.health > 60; }
inline bool sees(const GuardCtx& c) { return c.seesPlayer; }
inline bool noise(const GuardCtx& c) { return c.heardNoise; }
inline bool investigateDone(const GuardCtx& c) { return c.t > c.investigateUntil; }
inline bool lostTrack(const GuardCtx& c) { return c.t - c.lastSeen > 2; }
inline bool inRange(const GuardCtx& c) { return c.playerDist < 1.5f; }
inline bool outOfRange(const GuardCtx& c) { return c.playerDist > 2; }
inline bool stunOver(const GuardCtx& c) { return c.t > c.stunUntil; }
inline void startInvestigating(GuardCtx& c) { c.investigateUntil = c.t + 3; }
inline void startStun(GuardCtx& c) { c.stunUntil = c.t + 1.5f; }
}  // namespace guard
// #endregion

// #region hfsm
struct GuardStates {
    int alive, active, calm, patrol, investigate, combat, chase, attack, flee, stunned, dead;
};

inline GuardStates buildGuardHfsm(HfsmDef<GuardCtx>& m) {
    using namespace guard;
    GuardStates s;
    s.alive = m.addState("Alive");
    s.active = m.addState("Active", s.alive);
    s.calm = m.addState("Calm", s.active);
    s.patrol = m.addState("Patrol", s.calm);
    s.investigate = m.addState("Investigate", s.calm, startInvestigating);
    s.combat = m.addState("Combat", s.active);
    s.chase = m.addState("Chase", s.combat);
    s.attack = m.addState("Attack", s.combat);
    s.flee = m.addState("Flee", s.active);
    s.stunned = m.addState("Stunned", s.alive, startStun);
    s.dead = m.addState("Dead");

    m.addTransition(s.alive, s.dead, dead);                 // 对所有活着的状态有效
    m.addTransition(s.active, s.stunned, stunned);           // 对 Calm、Combat、Flee 及其子状态有效
    m.addTransition(s.calm, s.flee, lowHealth);
    m.addTransition(s.calm, s.combat, sees);
    m.addTransition(s.patrol, s.investigate, noise);
    m.addTransition(s.investigate, s.patrol, investigateDone);
    m.addTransition(s.combat, s.flee, lowHealth);
    m.addTransition(s.combat, s.investigate, lostTrack);
    m.addTransition(s.chase, s.attack, inRange);
    m.addTransition(s.attack, s.chase, outOfRange);
    m.addTransition(s.flee, s.calm, recovered);
    m.addTransition(s.stunned, s.active, stunOver, /*resume=*/true);  // 历史转移：回到被击晕前的状态
    return s;
}
// #endregion

// #region flat
// 同样的行为写成扁平状态机：每个状态都要自己检查"死亡""击晕""血量低"，
// 而且为了"击晕结束后回到原来的状态"，每个来源都需要一个自己的 Stunned
inline int buildGuardFlat(HfsmDef<GuardCtx>& m) {
    using namespace guard;
    int patrol = m.addState("Patrol");
    int investigate = m.addState("Investigate", -1, startInvestigating);
    int chase = m.addState("Chase");
    int attack = m.addState("Attack");
    int flee = m.addState("Flee");
    int dead = m.addState("Dead");
    int normal[] = {patrol, investigate, chase, attack, flee};
    int stunnedFrom[5];
    for (int i = 0; i < 5; ++i) stunnedFrom[i] = m.addState("Stunned", -1, startStun);

    for (int i = 0; i < 5; ++i) {
        m.addTransition(normal[i], dead, guard::dead);
        m.addTransition(normal[i], stunnedFrom[i], stunned);
    }
    for (int x : {patrol, investigate}) {
        m.addTransition(x, flee, lowHealth);
        m.addTransition(x, chase, sees);
    }
    m.addTransition(patrol, investigate, noise);
    m.addTransition(investigate, patrol, investigateDone);
    for (int x : {chase, attack}) {
        m.addTransition(x, flee, lowHealth);
        m.addTransition(x, investigate, lostTrack);
    }
    m.addTransition(chase, attack, inRange);
    m.addTransition(attack, chase, outOfRange);
    m.addTransition(flee, patrol, recovered);
    for (int i = 0; i < 5; ++i) {
        m.addTransition(stunnedFrom[i], dead, guard::dead);
        m.addTransition(stunnedFrom[i], normal[i], stunOver);
    }
    return patrol;  // 初始状态
}
// #endregion

// #region switch
// 手写 switch：最常见的写法。一个枚举表示状态，resume 记住被击晕前在做什么
enum class Guard { Patrol, Investigate, Chase, Attack, Flee, Stunned, Dead };

inline const char* guardName(Guard g) {
    const char* names[] = {"Patrol", "Investigate", "Chase", "Attack", "Flee", "Stunned", "Dead"};
    return names[static_cast<int>(g)];
}

struct GuardSwitch {
    Guard state = Guard::Patrol;
    Guard resume = Guard::Patrol;

    void enter(Guard s, GuardCtx& c) {
        state = s;
        if (s == Guard::Investigate) guard::startInvestigating(c);
        if (s == Guard::Stunned) guard::startStun(c);
    }

    void update(GuardCtx& c) {
        using namespace guard;
        if (state == Guard::Dead) return;
        if (dead(c)) return enter(Guard::Dead, c);
        if (state != Guard::Stunned && stunned(c)) {
            resume = state;
            return enter(Guard::Stunned, c);
        }
        switch (state) {
            case Guard::Patrol:
                if (lowHealth(c)) enter(Guard::Flee, c);
                else if (sees(c)) enter(Guard::Chase, c);
                else if (noise(c)) enter(Guard::Investigate, c);
                break;
            case Guard::Investigate:
                if (lowHealth(c)) enter(Guard::Flee, c);
                else if (sees(c)) enter(Guard::Chase, c);
                else if (investigateDone(c)) enter(Guard::Patrol, c);
                break;
            case Guard::Chase:
                if (lowHealth(c)) enter(Guard::Flee, c);
                else if (lostTrack(c)) enter(Guard::Investigate, c);
                else if (inRange(c)) enter(Guard::Attack, c);
                break;
            case Guard::Attack:
                if (lowHealth(c)) enter(Guard::Flee, c);
                else if (lostTrack(c)) enter(Guard::Investigate, c);
                else if (outOfRange(c)) enter(Guard::Chase, c);
                break;
            case Guard::Flee:
                if (recovered(c)) enter(Guard::Patrol, c);
                break;
            case Guard::Stunned:
                if (stunOver(c)) enter(resume, c);
                break;
            case Guard::Dead: break;
        }
    }
};
// #endregion

}  // namespace ga
