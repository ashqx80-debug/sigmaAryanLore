#include "globals.hpp"
#include "autos.hpp"
#include "pros/screen.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "lemlib/api.hpp"
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include "pros/apix.h"
pros::Motor lf(1), lm(2), lb(3);
pros::Motor rf(-4), rm(-5), rb(-6);

Mode currentMode = AUTON_SELECT;
static bool screenDirty = true;
void markDirty() { screenDirty = true; }

// ─── Auton list ───────────────────────────────────────────────────────────────
std::vector<std::string> autonNames = {
    "Left 7","Right 7","Mid 3+4","Right Rush","Left Rush",
    "Sig SAWP","Skills","Unjam","Test1","Turn Test",
    "Back PID","Nigr Sawp","Ngr Skills","Test2"
};
int selectedAuton = 0;

// ─── Block / goal types ───────────────────────────────────────────────────────
enum BlockColor { BLOCK_RED, BLOCK_BLUE, BLOCK_NONE };
struct GoalSlot { BlockColor color; };

const int LONG_CAP   = 14;
const int CENTER_CAP = 7;

struct LongGoal   { GoalSlot slots[14]; int count = 0; };
struct CenterGoal { GoalSlot slots[7];  int count = 0; };

LongGoal   longLeft, longRight;
CenterGoal centerUpper, centerLower;

struct FieldBlock { int x, y; BlockColor color; bool active; };
std::vector<FieldBlock> fieldBlocks;

// ─── Player / AI ──────────────────────────────────────────────────────────────
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
    bool wingPhase = false;
    int wingTimer  = 0;
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

// ─── Driver state (mirrored from opcontrol) ───────────────────────────────────

static bool dih   = false;

// ─── Helpers ──────────────────────────────────────────────────────────────────
static int   clampI(int   v, int   lo, int   hi) { return v < lo ? lo : v > hi ? hi : v; }
static float clampF(float v, float lo, float hi) { return v < lo ? lo : v > hi ? hi : v; }
static int   iabs (int v) { return v < 0 ? -v : v; }

// ─── Goal / loader helpers ────────────────────────────────────────────────────
void clearGoals() {
    longLeft.count = longRight.count = 0;
    centerUpper.count = centerLower.count = 0;
}

// ─── Game init ────────────────────────────────────────────────────────────────
void initGame() {
    fieldBlocks.clear();
    clearGoals();

    player.x = player.y = 0;
    player.velX = player.velY = 0;
    player.x = 210; player.y = 120;
    player.heldRed = player.heldBlue = 0;

    ai.x = 30; ai.y = 120;
    ai.velX = ai.velY = 0;
    ai.held = 0; ai.state = AI_SEEK;
    ai.defenseTimer = 0; ai.stuckTimer = 0;
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

// ─── Physics (arcade using controller sticks) ─────────────────────────────────
void updatePhysics() {
    int rawLY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rawRX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    if (iabs(rawLY) < 8) rawLY = 0;
    if (iabs(rawRX) < 8) rawRX = 0;

    float throttle = rawLY * (2.0f / 127.0f);
    float steer    = rawRX * (1.0f / 127.0f);

    auto expo = [](float v) {
        return v * 0.6f + v * std::abs(v) * 0.4f;
    };

    throttle = expo(throttle);
    steer    = expo(steer);

    float vx = steer * 2.0f;
    float vy = -throttle * 2.0f;

    player.velX = player.velX * 0.6f + vx * 0.4f;
    player.velY = player.velY * 0.6f + vy * 0.4f;

    // friction
    player.velX *= 0.92f;
    player.velY *= 0.92f;

    player.x = clampI(player.x + (int)player.velX, 8, 232);
    player.y = clampI(player.y + (int)player.velY, 8, 232);

    // REAL pushback collision
    if (iabs(player.x - ai.x) < 10 && iabs(player.y - ai.y) < 10) {
        float dx = player.x - ai.x;
        float dy = player.y - ai.y;

        player.velX += dx * 0.12f;
        player.velY += dy * 0.12f;

        ai.velX -= dx * 0.12f;
        ai.velY -= dy * 0.12f;
    }

    // PLAYER pushes blocks
    for (auto& b : fieldBlocks) {
        if (!b.active) continue;

        if (iabs(player.x - b.x) < 10 && iabs(player.y - b.y) < 10) {
            b.x += (int)(player.velX * 1.6f);
            b.y += (int)(player.velY * 1.6f);

            b.x = clampI(b.x, 0, 240);
            b.y = clampI(b.y, 0, 240);
        }
    }
}

// ─── AI UPDATE (REWRITTEN CORE LOGIC) ────────────────────────────────────────
void updateAI() {
    if (!aiEnabled) return;

    int elapsed = pros::millis() - gameStart;
    bool isEndgame = elapsed > (int)(TOTAL_MS * 0.82f);

    int dx = player.x - ai.x;
    int dy = player.y - ai.y;
    float dist = std::sqrt(dx*dx + dy*dy);

    if (isEndgame && ai.held == 0) ai.state = AI_PARK;

    switch (ai.state) {

    case AI_SEEK: {
        if (dist < 80 && player.heldRed + player.heldBlue >= 3 && !isEndgame) {
            ai.state = AI_DEFEND;
            break;
        }

        if (ai.held >= ai.maxHeld) {
            ai.state = AI_DEPOSIT;
            break;
        }

        FieldBlock* target = nullptr;
        float best = 9999;

        for (auto& b : fieldBlocks) {
            if (!b.active || b.color != BLOCK_BLUE) continue;

            float d = std::sqrt((b.x - ai.x)*(b.x - ai.x) + (b.y - ai.y)*(b.y - ai.y));
            if (d < best) {
                best = d;
                target = &b;
            }
        }

        if (target) {
            float ang = atan2(target->y - ai.y, target->x - ai.x);
            ai.velX += cos(ang) * 0.3f;
            ai.velY += sin(ang) * 0.3f;

            if (best < 8) {
                target->active = false;
                ai.held++;
            }
        }

        break;
    }

    case AI_DEFEND: {
        float ang = atan2(player.y - ai.y, player.x - ai.x);
        ai.velX += cos(ang) * 0.4f;
        ai.velY += sin(ang) * 0.4f;

        if (dist < 10) {
            ai.velX += dx * 0.2f;
            ai.velY += dy * 0.2f;
        }

        if (dist > 100) ai.state = AI_SEEK;
        break;
    }

    case AI_DEPOSIT: {
        int tx = (pros::millis() % 2) ? 8 : 232;
        int ty = 120;

        float ang = atan2(ty - ai.y, tx - ai.x);
        ai.velX += cos(ang) * 0.4f;
        ai.velY += sin(ang) * 0.4f;

        if (iabs(ai.x - tx) < 12 && iabs(ai.y - ty) < 12) {
            for (int i = 0; i < ai.held && longLeft.count < LONG_CAP; i++)
                longLeft.slots[longLeft.count++].color = BLOCK_BLUE;

            ai.held = 0;
            ai.state = AI_SEEK;
        }
        break;
    }

    case AI_PARK: {
        int tx = (bluePark.x1 + bluePark.x2)/2;
        int ty = (bluePark.y1 + bluePark.y2)/2;

        float ang = atan2(ty - ai.y, tx - ai.x);
        ai.velX += cos(ang) * 0.4f;
        ai.velY += sin(ang) * 0.4f;
        break;
    }
    }

    ai.velX = clampF(ai.velX, -2.5f, 2.5f);
    ai.velY = clampF(ai.velY, -2.5f, 2.5f);

    ai.velX *= 0.85f;
    ai.velY *= 0.85f;

    // APPLY VELOCITY (CRITICAL FIX)
    ai.x += (int)ai.velX;
    ai.y += (int)ai.velY;

    ai.x = clampI(ai.x, 8, 232);
    ai.y = clampI(ai.y, 8, 232);
}

// ─── Game interactions ────────────────────────────────────────────────────────
void pickup() {
    if (pros::millis() < intakeCooldown) return;
    if (player.heldRed + player.heldBlue >= player.maxHeld) return;
    for (auto& b : fieldBlocks) {
        if (!b.active) continue;
        if (iabs(player.x-b.x)<12 && iabs(player.y-b.y)<12) {
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
        int cx = (l.x1+l.x2)/2, cy = (l.y1+l.y2)/2;
        if (iabs(player.x-cx)<20 && iabs(player.y-cy)<20 && l.count > 0) {
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
                l.count = std::min(l.count+4, l.maxCount);
                l.refillTimer = 0;
            }
        } else { l.refillTimer = 0; }
    }
}

// ─── Scoring ──────────────────────────────────────────────────────────────────
static int countColor(GoalSlot* s, int c, BlockColor col) {
    int n = 0;
    for (int i = 0; i < c; i++) if (s[i].color == col) n++;
    return n;
}

struct Score { int r, b; };
Score calcScore() {
    Score s = {0,0};
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

    bool rPark = (player.x > redPark.x1  && player.x < redPark.x2  &&
                  player.y > redPark.y1  && player.y < redPark.y2);
    bool bPark = (ai.x > bluePark.x1 && ai.x < bluePark.x2 &&
                  ai.y > bluePark.y1 && ai.y < bluePark.y2);
    if (rPark) s.r += 8;
    if (bPark) s.b += 8;

    auto czScore = [&](GoalSlot* slots, int cnt) {
        int lo = cnt/3, hi = (cnt*2)/3;
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

// ─── Draw helpers ─────────────────────────────────────────────────────────────
static void fillRect(int x1,int y1,int x2,int y2,uint32_t col) {
    pros::screen::set_pen(col);
    pros::screen::fill_rect(x1,y1,x2,y2);
}

// ─── Draw game ────────────────────────────────────────────────────────────────
void drawGame() {
    pros::screen::erase();
    fillRect(0,0,240,240,0x1a1a1a);

    for (int r=0;r<3;r++)
        for (int c=0;c<3;c++)
            if ((r+c)%2==0) fillRect(c*80,r*80,c*80+80,r*80+80,0x222222);

    // Goals
    fillRect(0,85,14,155,0x1a4d1a);  fillRect(226,85,240,155,0x1a4d1a);
    pros::screen::set_pen(0x00aa44);
    pros::screen::draw_rect(0,85,14,155);   pros::screen::draw_rect(226,85,240,155);
    fillRect(80,0,160,14,0x1a4d1a);  fillRect(80,226,160,240,0x1a4d1a);
    pros::screen::set_pen(0x00aa44);
    pros::screen::draw_rect(80,0,160,14);   pros::screen::draw_rect(80,226,160,240);

    // Center zone
    fillRect(100,100,140,140,0x2a2060);
    pros::screen::set_pen(0x8060ff);
    pros::screen::draw_rect(100,100,140,140);
    pros::screen::draw_line(100,120,140,120);

    // Park zones
    fillRect(redPark.x1, redPark.y1, redPark.x2, redPark.y2,  0x551010);
    fillRect(bluePark.x1,bluePark.y1,bluePark.x2,bluePark.y2,0x101055);
    pros::screen::set_pen(0xff4444); pros::screen::draw_rect(redPark.x1,redPark.y1,redPark.x2,redPark.y2);
    pros::screen::set_pen(0x4488ff); pros::screen::draw_rect(bluePark.x1,bluePark.y1,bluePark.x2,bluePark.y2);

    // Loaders
    for (auto& l : loaders) {
        uint32_t bg  = l.color==BLOCK_RED ? 0x3a1010 : 0x10103a;
        uint32_t bdr = l.color==BLOCK_RED ? 0xcc3333 : 0x3366cc;
        fillRect(l.x1,l.y1,l.x2,l.y2,bg);
        pros::screen::set_pen(bdr); pros::screen::draw_rect(l.x1,l.y1,l.x2,l.y2);
    }

    // Field blocks
    for (auto& b : fieldBlocks) {
        if (!b.active) continue;
        uint32_t col = b.color==BLOCK_RED ? 0xE24B4A : 0x378ADD;
        fillRect(b.x-4,b.y-4,b.x+4,b.y+4,col);
    }

    // Goal fills
    for (int i=0;i<longLeft.count&&i<LONG_CAP;i++) {
        uint32_t col=longLeft.slots[i].color==BLOCK_RED?0xE24B4A:0x378ADD;
        fillRect(1,88+i*5,12,92+i*5,col);
    }
    for (int i=0;i<longRight.count&&i<LONG_CAP;i++) {
        uint32_t col=longRight.slots[i].color==BLOCK_RED?0xE24B4A:0x378ADD;
        fillRect(228,88+i*5,239,92+i*5,col);
    }
    for (int i=0;i<centerUpper.count&&i<CENTER_CAP;i++) {
        uint32_t col=centerUpper.slots[i].color==BLOCK_RED?0xE24B4A:0x378ADD;
        int cx=103+i*5; fillRect(cx,102,cx+4,118,col);
    }
    for (int i=0;i<centerLower.count&&i<CENTER_CAP;i++) {
        uint32_t col=centerLower.slots[i].color==BLOCK_RED?0xE24B4A:0x378ADD;
        int cx=103+i*5; fillRect(cx,122,cx+4,138,col);
    }

    // Player held blocks bar
    int tot = player.heldRed+player.heldBlue;
    for (int i=0;i<tot;i++) {
        uint32_t col = i<player.heldRed ? 0xE24B4A : 0x378ADD;
        fillRect(player.x-tot*4+i*8, player.y-14,
                 player.x-tot*4+i*8+6, player.y-8, col);
    }

    // Player robot
    fillRect(player.x-5,player.y-5,player.x+5,player.y+5,0xdddddd);
    pros::screen::set_pen(0x333333);
    pros::screen::draw_rect(player.x-5,player.y-5,player.x+5,player.y+5);

    // AI robot
    if (aiEnabled) {
        uint32_t aiCol = ai.state==AI_DEFEND ? 0xE8A020 :
                         ai.state==AI_WING   ? 0x20c8a0 : 0x44cc44;
        fillRect(ai.x-5,ai.y-5,ai.x+5,ai.y+5,aiCol);
        if (ai.wingPhase) {
            fillRect(ai.x-14,ai.y-2,ai.x-7, ai.y+2,0x20c8a0);
            fillRect(ai.x+7, ai.y-2,ai.x+14,ai.y+2,0x20c8a0);
        }
    }

Score sc = calcScore();
int rem  = std::max(0, TOTAL_MS-(int)(pros::millis()-gameStart));

uint32_t timeColor = 0xffffff;
if (rem < 15000) timeColor = 0xff4444;
else if (rem < 30000) timeColor = 0xffaa00;

pros::screen::set_pen(timeColor);
pros::screen::print(TEXT_SMALL,0,
    "R:%-3d B:%-3d  %d:%02d  H:%d",
    sc.r, sc.b,
    rem/60000, (rem%60000)/1000,
    player.heldRed+player.heldBlue);

// ENDGAME TEXT
if (rem < 20000) {
    pros::screen::set_pen(0xff4444);
    pros::screen::print(TEXT_MEDIUM, 10, "ENDGAME");
}
}

// ─── Handle game frame ────────────────────────────────────────────────────────
void handleGame() {
    if (currentMode != GAME) return;

    int elapsed = pros::millis() - gameStart;
    bool isAuton = elapsed < 15000;

    updateLoaders();

    if (!isAuton) {
        updatePhysics();
    }

    updateAI();
    pickup();

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) deposit();
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) extake();

    // CONTINUOUS MATCH LOAD
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
        tryMatchLoad();
    }

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) initGame();
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) aiEnabled = !aiEnabled;

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
        currentMode = DRIVER;
        markDirty();
        return;
    }

    drawGame();
}
// ─── Diagnostics ──────────────────────────────────────────────────────────────
// Motor port groups – adjust port numbers to match globals.hpp
// These are referenced via the extern motors from globals.hpp.
// We list them by index for the heat/velocity readout.
extern pros::Motor intake_motor;
extern pros::Motor intake_hood_roller;

// LemLib chassis exposes its pose; pull it for x/y/θ display
extern lemlib::Chassis chassis;

void handleDiagnostics() {
    if (currentMode != DIAG) return;

    static uint32_t lastDraw = 0;
    if (pros::millis() - lastDraw < 150) return;   // ~7 fps – enough for live data
    lastDraw = pros::millis();

    pros::screen::erase();
    pros::screen::set_pen(0xffffff);
    pros::screen::print(TEXT_LARGE, 0, "DIAGNOSTICS");

    // ── Controller axes ──────────────────────────────────────────────────────
    int lY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    int lX = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
    int rY = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
    pros::screen::print(TEXT_MEDIUM, 1, "LY:%-4d LX:%-4d  RY:%-4d RX:%-4d", lY, lX, rY, rX);

    // ── Controller buttons ───────────────────────────────────────────────────
    pros::screen::print(TEXT_MEDIUM, 2, "R1:%d R2:%d L1:%d L2:%d  A:%d B:%d X:%d Y:%d",
        master.get_digital(pros::E_CONTROLLER_DIGITAL_R1),
        master.get_digital(pros::E_CONTROLLER_DIGITAL_R2),
        master.get_digital(pros::E_CONTROLLER_DIGITAL_L1),
        master.get_digital(pros::E_CONTROLLER_DIGITAL_L2),
        master.get_digital(pros::E_CONTROLLER_DIGITAL_A),
        master.get_digital(pros::E_CONTROLLER_DIGITAL_B),
        master.get_digital(pros::E_CONTROLLER_DIGITAL_X),
        master.get_digital(pros::E_CONTROLLER_DIGITAL_Y));

    // ── Odometry pose ────────────────────────────────────────────────────────
    lemlib::Pose pose = chassis.getPose();
    pros::screen::print(TEXT_MEDIUM, 3, "X: %6.2f  Y: %6.2f  Θ: %6.2f°",
                        pose.x, pose.y, pose.theta);

    // ── Drivetrain motor temps & velocities ──────────────────────────────────
    // We access the motor groups through chassis's internal motors.
    // If you have named extern motors, replace with those directly.
    // For a 6-motor drive (3 left / 3 right) using LemLib motor groups:
    //   chassis.leftMotors / chassis.rightMotors
    // Adapt port list below to match your globals.hpp.

    // Example using chassis motor groups if they are public:
    // pros::MotorGroup& lm = *(chassis.leftMotors);
    // pros::MotorGroup& rm = *(chassis.rightMotors);
    // Below uses direct port access for maximum compatibility.

    // Left motors (adjust port numbers to match your robot)
    pros::Motor lf(1), lm(2), lb(3);   // <- update ports
    pros::Motor rf(-4), rm(-5), rb(-6); // <- update ports (negative = reversed)

    auto tempStr = [](pros::Motor& m) -> int { return (int)m.get_temperature(); };
    auto velStr  = [](pros::Motor& m) -> int { return (int)m.get_actual_velocity(); };

    pros::screen::print(TEXT_MEDIUM, 4,
        "LF:%3dc %4drpm  LM:%3dc %4drpm  LB:%3dc %4drpm",
        tempStr(lf), velStr(lf),
        tempStr(lm), velStr(lm),
        tempStr(lb), velStr(lb));

    pros::screen::print(TEXT_MEDIUM, 5,
        "RF:%3dc %4drpm  RM:%3dc %4drpm  RB:%3dc %4drpm",
        tempStr(rf), velStr(rf),
        tempStr(rm), velStr(rm),
        tempStr(rb), velStr(rb));

    // ── Intake motors ────────────────────────────────────────────────────────
    pros::screen::print(TEXT_MEDIUM, 6,
        "Intake:%3dc %4drpm  Hood:%3dc %4drpm",
        (int)intake_motor.get_temperature(),      (int)intake_motor.get_actual_velocity(),
        (int)intake_hood_roller.get_temperature(),(int)intake_hood_roller.get_actual_velocity());

    // ── Battery ──────────────────────────────────────────────────────────────
    pros::screen::print(TEXT_MEDIUM, 7,
        "Batt: %.1fV  %.0f%%  Brain temp: N/A",
        pros::battery::get_voltage() / 1000.0,
        pros::battery::get_capacity());

    // ── Back button hint ─────────────────────────────────────────────────────
    pros::screen::set_pen(0xaaaaaa);
    pros::screen::print(TEXT_SMALL, 9, "Touch bottom bar or press LEFT to go back");
}

// ─── Static back-button bar (drawn on non-game screens) ──────────────────────
static void drawBackBar() {
    fillRect(0, 220, 240, 240, 0x1a1a2e);
    pros::screen::set_pen(0x4488ff);
    pros::screen::draw_rect(0, 220, 240, 240);
    pros::screen::set_pen(0xaaaaff);
    pros::screen::print(TEXT_SMALL, 13, "  < BACK");
}

// ─── Draw UI (non-game screens) ───────────────────────────────────────────────
void drawUI() {
    if (!screenDirty) return;
    if (currentMode == GAME) return;
    screenDirty = false;

    pros::screen::erase();

    // ── Driver hub ────────────────────────────────────────────────────────────
    if (currentMode == DRIVER) {
        fillRect(0,0,480,272,0x0d0d1a);

        pros::screen::set_pen(0x00ff88);
        pros::screen::print(TEXT_LARGE, 0, "DRIVER MODE");

        // Button: Auton Select
        fillRect(10,55,230,95,0x1a2a1a);
        pros::screen::set_pen(0x00cc66);
        pros::screen::draw_rect(10,55,230,95);
        pros::screen::set_pen(0xffffff);
        pros::screen::print(TEXT_MEDIUM, 3, "  [AUTON SELECT]");

        // Button: Diagnostics
        fillRect(10,105,230,145,0x1a1a2a);
        pros::screen::set_pen(0x4488ff);
        pros::screen::draw_rect(10,105,230,145);
        pros::screen::set_pen(0xffffff);
        pros::screen::print(TEXT_MEDIUM, 5, "  [DIAGNOSTICS]");

        // Button: Mini Game
        fillRect(10,155,230,195,0x2a1a1a);
        pros::screen::set_pen(0xee6644);
        pros::screen::draw_rect(10,155,230,195);
        pros::screen::set_pen(0xffffff);
        pros::screen::print(TEXT_MEDIUM, 7, "  [MINI GAME]");
    }

    // ── Auton select ──────────────────────────────────────────────────────────
    if (currentMode == AUTON_SELECT) {
        fillRect(0,0,480,272,0x0d0d1a);

        pros::screen::set_pen(0x00ff88);
        pros::screen::print(TEXT_LARGE, 0, "AUTON SELECT");

        int visStart = std::max(0, selectedAuton - 6);
        int visEnd   = std::min((int)autonNames.size(), visStart + 10);

        for (int i = visStart; i < visEnd; i++) {
            bool sel = (i == selectedAuton);
            int row = i - visStart + 1;

            if (sel) {
                fillRect(5, row*18+8, 235, row*18+26, 0x003322);
                pros::screen::set_pen(0x00ff88);
            } else {
                pros::screen::set_pen(0xcccccc);
            }
            pros::screen::print(TEXT_SMALL, row,
                "%s %d. %s",
                sel ? ">" : " ",
                i,
                autonNames[i].c_str());
        }

        pros::screen::set_pen(0x888888);
        pros::screen::print(TEXT_SMALL, 12,
            "UP/DN=scroll  A=confirm run  LEFT=back");

        drawBackBar();
    }

    // ── Diagnostics ───────────────────────────────────────────────────────────
    if (currentMode == DIAG) {
        // Live content drawn by handleDiagnostics(), but add back bar here too
        drawBackBar();
    }
}

// ─── Touch handler ────────────────────────────────────────────────────────────
void handleTouch() {
    auto ts = pros::screen::touch_status();
    if (ts.touch_status != pros::E_TOUCH_PRESSED) return;
    int tx = ts.x;
    int ty = ts.y;

    // Universal: back bar (bottom strip) on any non-DRIVER screen
    if (ty > 220 && currentMode != DRIVER && currentMode != GAME) {
        currentMode = DRIVER;
        markDirty();
        return;
    }

    if (currentMode == DRIVER) {
        if (ty > 55  && ty < 100) { currentMode = AUTON_SELECT; markDirty(); }
        if (ty > 100 && ty < 150) { currentMode = DIAG;         markDirty(); }
        if (ty > 150 && ty < 200) { currentMode = GAME; initGame(); }
    }
    else if (currentMode == AUTON_SELECT) {
        // Touch a name row to select it
        int visStart = std::max(0, selectedAuton - 6);
        int row = (ty - 8) / 18 - 1 + visStart;
        if (row >= 0 && row < (int)autonNames.size()) {
            selectedAuton = row;
            markDirty();
        }
    }

    (void)tx;
}

// ─── Mode / button navigation ─────────────────────────────────────────────────
void updateMode() {
    // Scroll auton list
    if (currentMode == AUTON_SELECT) {
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            selectedAuton = std::min(selectedAuton+1, (int)autonNames.size()-1);
            markDirty();
        }
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
            selectedAuton = std::max(selectedAuton-1, 0);
            markDirty();
        }
        // A = confirm & run auton immediately (useful during testing)
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            runSelectedAuton();
        }
    }

    // LEFT always goes back to DRIVER hub (except in GAME, handled separately)
    if (currentMode != GAME &&
        master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
        currentMode = DRIVER;
        markDirty();
    }
}

// ─── Run selected auton ───────────────────────────────────────────────────────
void runSelectedAuton() {
    switch (selectedAuton) {
        case  0: left7();                   break;
        case  1: right7();                  break;
        case  2: midthreeplusfour();        break;
        case  3: dheerarightfourballrush(); break;
        case  4: dheeraleftfourballrush();  break;
        case  5: sigSawp();                 break;
        case  6: skillsProg();              break;
        case 11: nigrSawp();                break;
        case 12: ngrSkills();               break;
        default: break;
    }
}