#include "globals.hpp"
#include "autos.hpp"
#include "pros/screen.hpp"
#include "pros/misc.h"
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include "pros/apix.h"

Mode currentMode = AUTON_SELECT;
static bool screenDirty = true;
void markDirty() { screenDirty = true; }

std::vector<std::string> autonNames = {
    "Left 7","Right 7","Mid 3+4","Right Rush","Left Rush",
    "Sig SAWP","Skills","Unjam","Test1","Turn Test",
    "Back PID","Nigr Sawp","Ngr Skills","Test2"
};
int selectedAuton = 0;

enum BlockColor { BLOCK_RED, BLOCK_BLUE, BLOCK_NONE };
struct GoalSlot { BlockColor color; };

const int LONG_CAP   = 14;
const int CENTER_CAP = 7;

struct LongGoal {
    GoalSlot slots[14];
    int count = 0;
};

struct CenterGoal {
    GoalSlot slots[7];
    int count = 0;
};

LongGoal  longLeft, longRight;
CenterGoal centerUpper, centerLower;

struct FieldBlock { int x, y; BlockColor color; bool active; };
std::vector<FieldBlock> fieldBlocks;

struct Robot {
    int x, y;
    float velX = 0, velY = 0;
    int heldRed = 0, heldBlue = 0;
    const int maxHeld = 7;
};

struct AIBot {
    int x, y;
    float velX = 0, velY = 0;
    int held = 0;
    const int maxHeld = 7;
    int state = 0;
    int defenseTimer = 0;
    int stuckTimer   = 0;
    int prevX = 0, prevY = 0;
    bool wingPhase   = false;
    int wingTimer    = 0;
};

Robot player;
AIBot ai;
bool aiEnabled = true;

const int TOTAL_MS = 105000;
int  gameStart     = 0;

uint32_t intakeCooldown = 0;

enum AIState { AI_SEEK = 0, AI_DEFEND, AI_DEPOSIT, AI_WING, AI_PARK };

struct ParkZone { int x1, y1, x2, y2; };

ParkZone redPark  = { 95, 185, 145, 215 };
ParkZone bluePark = { 95,  25, 145,  55 };

struct Loader { int x1, y1, x2, y2; BlockColor color; int count; int maxCount; uint32_t refillTimer; };

Loader loaders[4] = {
    {  0,   0,  35,  35, BLOCK_BLUE, 12, 12, 0 },
    { 205,   0, 240,  35, BLOCK_BLUE, 12, 12, 0 },
    {  0, 205,  35, 240, BLOCK_RED,  12, 12, 0 },
    { 205, 205, 240, 240, BLOCK_RED,  12, 12, 0 },
};
const uint32_t REFILL_MS = 9000;

void clearGoals() {
    longLeft.count = longRight.count = 0;
    centerUpper.count = centerLower.count = 0;
}

void initGame() {
    fieldBlocks.clear();
    clearGoals();

    player.x    = 210;
    player.y    = 120;
    player.velX = player.velY = 0;
    player.heldRed = player.heldBlue = 0;

    ai.x    = 30;
    ai.y    = 120;
    ai.velX = ai.velY = 0;
    ai.held = 0;
    ai.state = AI_SEEK;
    ai.defenseTimer = 0;
    ai.stuckTimer   = 0;
    ai.prevX = ai.x; ai.prevY = ai.y;
    ai.wingPhase = false; ai.wingTimer = 0;

    for (int i = 0; i < 4; i++) {
        loaders[i].count = loaders[i].maxCount;
        loaders[i].refillTimer = 0;
    }

    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 6; c++) {
            fieldBlocks.push_back({ 60 + c * 22, 80  + r * 18, BLOCK_RED,  true });
            fieldBlocks.push_back({ 60 + c * 22, 140 + r * 18, BLOCK_BLUE, true });
        }
    }

    gameStart = pros::millis();
}

static int clampI(int v, int lo, int hi) { return v < lo ? lo : v > hi ? hi : v; }
static float clampF(float v, float lo, float hi) { return v < lo ? lo : v > hi ? hi : v; }
static int iabs(int v) { return v < 0 ? -v : v; }

void updatePhysics() {
    int rawLY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rawRX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    float fwd  = rawLY * (2.0f / 127.0f);
    float turn = rawRX * (1.0f / 127.0f);

    float leftPow  = fwd + turn;
    float rightPow = fwd - turn;

    float expo = 0.7f;
    leftPow  = leftPow  * expo + leftPow  * (1.0f - expo) * std::abs(leftPow);
    rightPow = rightPow * expo + rightPow * (1.0f - expo) * std::abs(rightPow);

    float vx = (leftPow + rightPow) * 0.5f * 1.6f;
    float vy = 0;

    player.velX = player.velX * 0.65f + vx * 0.35f;
    player.velY = player.velY * 0.65f + vy * 0.35f;

    player.x = clampI(player.x + (int)player.velX, 8, 232);
    player.y = clampI(player.y + (int)player.velY, 8, 232);

    if (iabs(player.x - ai.x) < 10 && iabs(player.y - ai.y) < 10) {
        player.velX *= -0.5f;
        player.velY *= -0.5f;
    }
}

void updateAI() {
    if (!aiEnabled) return;

    ai.stuckTimer++;
    if (ai.stuckTimer > 80) {
        if (iabs(ai.x - ai.prevX) < 2 && iabs(ai.y - ai.prevY) < 2) {
            ai.x += (pros::millis() % 3 == 0) ? 8 : -8;
            ai.y += (pros::millis() % 5 == 0) ? 8 : -8;
        }
        ai.prevX = ai.x; ai.prevY = ai.y;
        ai.stuckTimer = 0;
    }

    int elapsed = pros::millis() - gameStart;
    bool isEndgame = elapsed > (int)(TOTAL_MS * 0.82f);
    int playerCarrying = player.heldRed + player.heldBlue;
    int dx = player.x - ai.x, dy = player.y - ai.y;
    int distToPlayer = (int)std::sqrt(dx * dx + dy * dy);

    if (isEndgame && ai.held == 0) {
        ai.state = AI_PARK;
    }

    switch (ai.state) {

    case AI_SEEK: {
        if (playerCarrying >= 3 && distToPlayer < 80 && !isEndgame) {
            ai.state = AI_DEFEND;
            ai.defenseTimer = 100;
            break;
        }
        if (!ai.wingPhase && ai.held >= 3 && (pros::millis() % 400 < 4)) {
            ai.wingPhase = true;
            ai.wingTimer = 100;
        }
        if (ai.wingPhase) {
            ai.state = AI_WING;
            break;
        }
        if (ai.held >= ai.maxHeld) { ai.state = AI_DEPOSIT; break; }

        FieldBlock* target = nullptr;
        int bestDist = 9999;
        for (auto& b : fieldBlocks) {
            if (!b.active || b.color != BLOCK_BLUE) continue;
            int ddx = b.x - ai.x, ddy = b.y - ai.y;
            int d = (int)std::sqrt(ddx * ddx + ddy * ddy);
            if (d < bestDist) { bestDist = d; target = &b; }
        }

        if (!target) {
            for (auto& l : loaders) {
                if (l.color != BLOCK_BLUE || l.count <= 0) continue;
                int cx = (l.x1 + l.x2) / 2, cy = (l.y1 + l.y2) / 2;
                if (iabs(ai.x - cx) < 10 && iabs(ai.y - cy) < 10) {
                    int take = std::min(4, std::min(ai.maxHeld - ai.held, l.count));
                    ai.held += take;
                    l.count -= take;
                    ai.state = AI_DEPOSIT;
                } else {
                    if (ai.x < cx) ai.x++; else if (ai.x > cx) ai.x--;
                    if (ai.y < cy) ai.y++; else if (ai.y > cy) ai.y--;
                }
                break;
            }
        } else {
            if (ai.x < target->x) ai.x++; else if (ai.x > target->x) ai.x--;
            if (ai.y < target->y) ai.y++; else if (ai.y > target->y) ai.y--;
            if (iabs(ai.x - target->x) < 7 && iabs(ai.y - target->y) < 7) {
                target->active = false;
                ai.held++;
                if (ai.held >= ai.maxHeld) ai.state = AI_DEPOSIT;
            }
        }
        break;
    }

    case AI_DEFEND: {
        int goalCX = longLeft.count > 0 ? 8 : 8;
        int goalCY = 120;
        int interceptX = (player.x * 2 + goalCX) / 3;
        int interceptY = (player.y * 2 + goalCY) / 3;
        if (ai.x < interceptX) ai.x++; else if (ai.x > interceptX) ai.x--;
        if (ai.y < interceptY) ai.y++; else if (ai.y > interceptY) ai.y--;
        if (distToPlayer < 12) {
            ai.velX = (float)(ai.x - player.x) * 0.15f;
            ai.velY = (float)(ai.y - player.y) * 0.15f;
        }
        ai.defenseTimer--;
        if (ai.defenseTimer <= 0 || distToPlayer > 100 || playerCarrying < 1) {
            ai.state = AI_SEEK;
        }
        break;
    }

    case AI_DEPOSIT: {
        int tx = 8, ty = 120;
        if (iabs(ai.x - tx) > 12 || iabs(ai.y - ty) > 12) {
            if (ai.x > tx) ai.x--; else if (ai.x < tx) ai.x++;
            if (ai.y > ty) ai.y--; else if (ai.y < ty) ai.y++;
        } else {
            for (int i = 0; i < ai.held && longLeft.count < LONG_CAP; i++)
                longLeft.slots[longLeft.count++].color = BLOCK_BLUE;
            ai.held = 0;
            ai.state = AI_SEEK;
        }
        break;
    }

    case AI_WING: {
        int tx = 120, ty = 8;
        if (ai.x < tx) ai.x++; else if (ai.x > tx) ai.x--;
        if (ai.y < ty) ai.y++; else if (ai.y > ty) ai.y--;
        for (auto& b : fieldBlocks) {
            if (!b.active) continue;
            int ddx = b.x - ai.x, ddy = b.y - ai.y;
            if ((int)std::sqrt(ddx * ddx + ddy * ddy) < 14) {
                b.x = clampI(b.x + (int)ai.velX, 0, 240);
                b.y = clampI(b.y + (int)ai.velY, 0, 240);
            }
        }
        ai.wingTimer--;
        if (ai.wingTimer <= 0 || (iabs(ai.x - tx) < 10 && iabs(ai.y - ty) < 10)) {
            for (int i = 0; i < ai.held && longLeft.count < LONG_CAP; i++)
                longLeft.slots[longLeft.count++].color = BLOCK_BLUE;
            ai.held = 0;
            ai.wingPhase = false;
            ai.state = AI_SEEK;
        }
        break;
    }

    case AI_PARK: {
        int tx = (bluePark.x1 + bluePark.x2) / 2;
        int ty = (bluePark.y1 + bluePark.y2) / 2;
        if (ai.x < tx) ai.x++; else if (ai.x > tx) ai.x--;
        if (ai.y < ty) ai.y++; else if (ai.y > ty) ai.y--;
        break;
    }
    }

    ai.velX = clampF(ai.velX, -2.5f, 2.5f);
    ai.velY = clampF(ai.velY, -2.5f, 2.5f);
    ai.velX *= 0.8f; ai.velY *= 0.8f;
    ai.x = clampI(ai.x, 8, 232);
    ai.y = clampI(ai.y, 8, 232);
}

void pickup() {
    if (pros::millis() < intakeCooldown) return;
    if (player.heldRed + player.heldBlue >= player.maxHeld) return;

    for (auto& b : fieldBlocks) {
        if (!b.active) continue;
        if (iabs(player.x - b.x) < 12 && iabs(player.y - b.y) < 12) {
            b.active = false;
            b.color == BLOCK_RED ? player.heldRed++ : player.heldBlue++;
            intakeCooldown = pros::millis() + 220;
            return;
        }
    }
}

void extake() {
    if (player.heldRed > 0) player.heldRed--;
    else if (player.heldBlue > 0) player.heldBlue--;
}

void deposit() {
    if (player.x < 22) {
        int n = player.heldRed; player.heldRed = 0;
        for (int i = 0; i < n && longLeft.count < LONG_CAP; i++)
            longLeft.slots[longLeft.count++].color = BLOCK_RED;
        n = player.heldBlue; player.heldBlue = 0;
        for (int i = 0; i < n && longLeft.count < LONG_CAP; i++)
            longLeft.slots[longLeft.count++].color = BLOCK_BLUE;
    }
    if (player.x > 218) {
        int n = player.heldRed; player.heldRed = 0;
        for (int i = 0; i < n && longRight.count < LONG_CAP; i++)
            longRight.slots[longRight.count++].color = BLOCK_RED;
        n = player.heldBlue; player.heldBlue = 0;
        for (int i = 0; i < n && longRight.count < LONG_CAP; i++)
            longRight.slots[longRight.count++].color = BLOCK_BLUE;
    }
    if (player.y < 22 && player.x > 90 && player.x < 150) {
        int n = player.heldRed; player.heldRed = 0;
        for (int i = 0; i < n && centerUpper.count < CENTER_CAP; i++)
            centerUpper.slots[centerUpper.count++].color = BLOCK_RED;
        n = player.heldBlue; player.heldBlue = 0;
        for (int i = 0; i < n && centerUpper.count < CENTER_CAP; i++)
            centerUpper.slots[centerUpper.count++].color = BLOCK_BLUE;
    }
    if (player.y > 218 && player.x > 90 && player.x < 150) {
        int n = player.heldRed; player.heldRed = 0;
        for (int i = 0; i < n && centerLower.count < CENTER_CAP; i++)
            centerLower.slots[centerLower.count++].color = BLOCK_RED;
        n = player.heldBlue; player.heldBlue = 0;
        for (int i = 0; i < n && centerLower.count < CENTER_CAP; i++)
            centerLower.slots[centerLower.count++].color = BLOCK_BLUE;
    }
}

void tryMatchLoad() {
    for (auto& l : loaders) {
        int cx = (l.x1 + l.x2) / 2, cy = (l.y1 + l.y2) / 2;
        if (iabs(player.x - cx) < 20 && iabs(player.y - cy) < 20 && l.count > 0) {
            int space = player.maxHeld - player.heldRed - player.heldBlue;
            int take  = std::min(2, std::min(space, l.count));
            l.color == BLOCK_RED ? player.heldRed += take : player.heldBlue += take;
            l.count -= take;
            return;
        }
    }
}

void updateLoaders() {
    for (auto& l : loaders) {
        if (l.count < l.maxCount) {
            if (l.refillTimer == 0) l.refillTimer = pros::millis() + REFILL_MS;
            if (pros::millis() > l.refillTimer) {
                l.count = std::min(l.count + 4, l.maxCount);
                l.refillTimer = 0;
            }
        } else {
            l.refillTimer = 0;
        }
    }
}

static int countColor(GoalSlot* s, int c, BlockColor col) {
    int n = 0;
    for (int i = 0; i < c; i++) if (s[i].color == col) n++;
    return n;
}

struct Score { int r, b; };

Score calcScore() {
    Score s = {0, 0};
    s.r += countColor(longLeft.slots,  longLeft.count,  BLOCK_RED)  * 3;
    s.r += countColor(longRight.slots, longRight.count, BLOCK_RED)  * 3;
    s.b += countColor(longLeft.slots,  longLeft.count,  BLOCK_BLUE) * 3;
    s.b += countColor(longRight.slots, longRight.count, BLOCK_BLUE) * 3;

    int rC = countColor(centerUpper.slots, centerUpper.count, BLOCK_RED);
    int bC = countColor(centerUpper.slots, centerUpper.count, BLOCK_BLUE);
    if (rC > bC) s.r += 8; else if (bC > rC) s.b += 8;

    rC = countColor(centerLower.slots, centerLower.count, BLOCK_RED);
    bC = countColor(centerLower.slots, centerLower.count, BLOCK_BLUE);
    if (rC > bC) s.r += 6; else if (bC > rC) s.b += 6;

    bool rPark = (player.x > redPark.x1 && player.x < redPark.x2 &&
                  player.y > redPark.y1 && player.y < redPark.y2);
    bool bPark = (ai.x > bluePark.x1 && ai.x < bluePark.x2 &&
                  ai.y > bluePark.y1 && ai.y < bluePark.y2);
    if (rPark) s.r += 8;
    if (bPark) s.b += 8;

    auto czScore = [&](GoalSlot* slots, int cnt) {
        int lo = cnt / 3, hi = (cnt * 2) / 3;
        int rc = 0, bc = 0;
        for (int i = lo; i < hi; i++) {
            if (slots[i].color == BLOCK_RED)  rc++;
            if (slots[i].color == BLOCK_BLUE) bc++;
        }
        if (rc > bc) s.r += 10; else if (bc > rc) s.b += 10;
    };
    if (longLeft.count  > 0) czScore(longLeft.slots,  longLeft.count);
    if (longRight.count > 0) czScore(longRight.slots, longRight.count);

    return s;
}

static void fillRect(int x1, int y1, int x2, int y2, uint32_t col) {
    pros::screen::set_pen(col);
    pros::screen::fill_rect(x1, y1, x2, y2);
}

void drawGame() {
    pros::screen::erase();

    fillRect(0, 0, 240, 240, 0x1a1a1a);

    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            if ((r + c) % 2 == 0) fillRect(c * 80, r * 80, c * 80 + 80, r * 80 + 80, 0x222222);

    fillRect(0,   85, 14, 155, 0x1a4d1a);
    fillRect(226, 85, 240, 155, 0x1a4d1a);
    pros::screen::set_pen(0x00aa44);
    pros::screen::draw_rect(0, 85, 14, 155);
    pros::screen::draw_rect(226, 85, 240, 155);

    fillRect(80,  0,  160, 14, 0x1a4d1a);
    fillRect(80, 226, 160, 240, 0x1a4d1a);
    pros::screen::set_pen(0x00aa44);
    pros::screen::draw_rect(80, 0, 160, 14);
    pros::screen::draw_rect(80, 226, 160, 240);

    pros::screen::set_pen(0xf0c040);
    pros::screen::draw_rect(0, 105, 14, 135);
    pros::screen::draw_rect(226, 105, 240, 135);

    fillRect(100, 100, 140, 140, 0x2a2060);
    pros::screen::set_pen(0x8060ff);
    pros::screen::draw_rect(100, 100, 140, 140);
    pros::screen::draw_line(100, 120, 140, 120);

    fillRect(redPark.x1,  redPark.y1,  redPark.x2,  redPark.y2,  0x551010);
    fillRect(bluePark.x1, bluePark.y1, bluePark.x2, bluePark.y2, 0x101055);
    pros::screen::set_pen(0xff4444);
    pros::screen::draw_rect(redPark.x1,  redPark.y1,  redPark.x2,  redPark.y2);
    pros::screen::set_pen(0x4488ff);
    pros::screen::draw_rect(bluePark.x1, bluePark.y1, bluePark.x2, bluePark.y2);

    for (auto& l : loaders) {
        uint32_t bg  = l.color == BLOCK_RED ? 0x3a1010 : 0x10103a;
        uint32_t bdr = l.color == BLOCK_RED ? 0xcc3333 : 0x3366cc;
        fillRect(l.x1, l.y1, l.x2, l.y2, bg);
        pros::screen::set_pen(bdr);
        pros::screen::draw_rect(l.x1, l.y1, l.x2, l.y2);
    }

    for (auto& b : fieldBlocks) {
        if (!b.active) continue;
        uint32_t col = b.color == BLOCK_RED ? 0xE24B4A : 0x378ADD;
        fillRect(b.x - 4, b.y - 4, b.x + 4, b.y + 4, col);
    }

    for (int i = 0; i < longLeft.count && i < LONG_CAP; i++) {
        uint32_t col = longLeft.slots[i].color == BLOCK_RED ? 0xE24B4A : 0x378ADD;
        fillRect(1, 88 + i * 5, 12, 92 + i * 5, col);
    }
    for (int i = 0; i < longRight.count && i < LONG_CAP; i++) {
        uint32_t col = longRight.slots[i].color == BLOCK_RED ? 0xE24B4A : 0x378ADD;
        fillRect(228, 88 + i * 5, 239, 92 + i * 5, col);
    }
    for (int i = 0; i < centerUpper.count && i < CENTER_CAP; i++) {
        uint32_t col = centerUpper.slots[i].color == BLOCK_RED ? 0xE24B4A : 0x378ADD;
        int cx = 103 + i * 5;
        fillRect(cx, 102, cx + 4, 118, col);
    }
    for (int i = 0; i < centerLower.count && i < CENTER_CAP; i++) {
        uint32_t col = centerLower.slots[i].color == BLOCK_RED ? 0xE24B4A : 0x378ADD;
        int cx = 103 + i * 5;
        fillRect(cx, 122, cx + 4, 138, col);
    }

    int tot = player.heldRed + player.heldBlue;
    for (int i = 0; i < tot; i++) {
        uint32_t col = i < player.heldRed ? 0xE24B4A : 0x378ADD;
        fillRect(player.x - tot * 4 + i * 8,
                 player.y - 14,
                 player.x - tot * 4 + i * 8 + 6,
                 player.y - 8, col);
    }

    fillRect(player.x - 5, player.y - 5, player.x + 5, player.y + 5, 0xdddddd);
    pros::screen::set_pen(0x333333);
    pros::screen::draw_rect(player.x - 5, player.y - 5, player.x + 5, player.y + 5);

    if (aiEnabled) {
        uint32_t aiCol = ai.state == AI_DEFEND ? 0xE8A020 :
                         ai.state == AI_WING   ? 0x20c8a0 : 0x44cc44;
        fillRect(ai.x - 5, ai.y - 5, ai.x + 5, ai.y + 5, aiCol);
        if (ai.wingPhase) {
            fillRect(ai.x - 14, ai.y - 2, ai.x - 7,  ai.y + 2, 0x20c8a0);
            fillRect(ai.x + 7,  ai.y - 2, ai.x + 14, ai.y + 2, 0x20c8a0);
        }
    }

    Score sc = calcScore();
    int rem = std::max(0, TOTAL_MS - (int)(pros::millis() - gameStart));
    int mins = rem / 60000, secs = (rem % 60000) / 1000;

    pros::screen::set_pen(0xffffff);
    pros::screen::print(TEXT_SMALL, 0, "R:%-3d B:%-3d  %d:%02d  H:%d",
                        sc.r, sc.b, mins, secs, player.heldRed + player.heldBlue);
}

void handleGame() {
    if (currentMode != GAME) return;

    updateLoaders();
    updatePhysics();
    updateAI();
    pickup();

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) deposit();
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) extake();
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))  tryMatchLoad();
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y))  initGame();
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))  aiEnabled = !aiEnabled;

    drawGame();
}

void drawUI() {
    if (!screenDirty) return;
    if (currentMode == GAME) return;
    screenDirty = false;

    pros::screen::erase();

    if (currentMode == DRIVER) {
        pros::screen::print(TEXT_LARGE,  1, "DRIVER MODE");
        pros::screen::draw_rect(10,  60, 230, 100);
        pros::screen::print(TEXT_MEDIUM, 3, "  >> AUTON SELECT");
        pros::screen::draw_rect(10, 110, 230, 150);
        pros::screen::print(TEXT_MEDIUM, 5, "  >> DIAGNOSTICS");
        pros::screen::draw_rect(10, 160, 230, 200);
        pros::screen::print(TEXT_MEDIUM, 7, "  >> MINI GAME");
    }

    if (currentMode == AUTON_SELECT) {
        pros::screen::print(TEXT_LARGE, 0, "AUTON SELECT");
        for (int i = 0; i < (int)autonNames.size(); i++) {
            bool sel = (i == selectedAuton);
            pros::screen::set_pen(sel ? 0x00ff88 : 0xffffff);
            pros::screen::print(TEXT_SMALL, i + 1, "%s %s",
                                sel ? ">" : " ", autonNames[i].c_str());
        }
        pros::screen::set_pen(0xffffff);
        pros::screen::print(TEXT_SMALL, (int)autonNames.size() + 2,
                            "UP/DOWN select  A confirm");
    }

    if (currentMode == DIAG) {
        pros::screen::print(TEXT_LARGE, 0, "DIAGNOSTICS");
    }
}

void handleTouch() {
    if (!pros::screen::touch_status().touch_status == pros::E_TOUCH_PRESSED) return;
    int tx = pros::screen::touch_status().x;
    int ty = pros::screen::touch_status().y;

    if (currentMode == DRIVER) {
        if (ty > 55 && ty < 105)  { currentMode = AUTON_SELECT; markDirty(); }
        if (ty > 105 && ty < 155) { currentMode = DIAG;         markDirty(); }
        if (ty > 155 && ty < 205) { currentMode = GAME; initGame(); }
    } else if (currentMode == AUTON_SELECT) {
        int row = (ty - 30) / 18;
        if (row >= 0 && row < (int)autonNames.size()) {
            selectedAuton = row;
            markDirty();
        }
    }
    (void)tx;
}

void handleDiagnostics() {
    if (currentMode != DIAG) return;

    static uint32_t lastDraw = 0;
    if (pros::millis() - lastDraw < 200) return;
    lastDraw = pros::millis();

    pros::screen::erase();
    pros::screen::set_pen(0xffffff);
    pros::screen::print(TEXT_LARGE,  0, "DIAGNOSTICS");
    pros::screen::print(TEXT_MEDIUM, 1, "LY: %4d  RX: %4d",
        master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),
        master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
    pros::screen::print(TEXT_MEDIUM, 2, "R1:%d R2:%d L1:%d L2:%d",
        master.get_digital(pros::E_CONTROLLER_DIGITAL_R1),
        master.get_digital(pros::E_CONTROLLER_DIGITAL_R2),
        master.get_digital(pros::E_CONTROLLER_DIGITAL_L1),
        master.get_digital(pros::E_CONTROLLER_DIGITAL_L2));
    pros::screen::print(TEXT_MEDIUM, 3, "A:%d B:%d X:%d Y:%d",
        master.get_digital(pros::E_CONTROLLER_DIGITAL_A),
        master.get_digital(pros::E_CONTROLLER_DIGITAL_B),
        master.get_digital(pros::E_CONTROLLER_DIGITAL_X),
        master.get_digital(pros::E_CONTROLLER_DIGITAL_Y));
}

void updateMode() {
    if (currentMode == AUTON_SELECT) {
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            selectedAuton = std::min(selectedAuton + 1, (int)autonNames.size() - 1);
            markDirty();
        }
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
            selectedAuton = std::max(selectedAuton - 1, 0);
            markDirty();
        }
    }
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
        currentMode = DRIVER;
        markDirty();
    }
}

void runSelectedAuton() {
    switch (selectedAuton) {
        case 0:  left7();                   break;
        case 1:  right7();                  break;
        case 2:  midthreeplusfour();        break;
        case 3:  dheerarightfourballrush(); break;
        case 4:  dheeraleftfourballrush();  break;
        case 5:  sigSawp();                 break;
        case 6:  skillsProg();              break;
        case 11: nigrSawp();                break;
        case 12: ngrSkills();               break;
        default: break;
    }
}