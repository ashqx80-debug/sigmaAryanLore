#ifndef SIGMA_HELPERS_HPP
#define SIGMA_HELPERS_HPP

#include "globals.hpp"
#include <vector>
#include "pros/rtos.hpp"

struct MechAction
{
    enum Type
    {
        MOTOR,
        DIGITAL
    } type;
    pros::Motor *motor;
    pros::adi::DigitalOut *digital;
    int value;
    int time;
};

extern std::vector<MechAction> mechQueue;
extern pros::Mutex mechMutex;

void enqueueMotor(pros::Motor &m, int v, int t = 0);
void enqueueDigital(pros::adi::DigitalOut &d, bool v, int t = 0);
void mechTask(void *);

void moveF(double distance, bool forwards, bool decreasing, int maxSpeed, int minSpeed, int timeOutMs);

// Commented-out MCL helpers (moved here for reference). If you want them
// enabled, remove the comment markers and implement/declare additional
// dependencies as needed.

#endif // SIGMA_HELPERS_HPP
