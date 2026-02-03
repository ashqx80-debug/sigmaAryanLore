#ifndef _TESTING_H_
#define _TESTING_H_

#include "main.h"
#include "lemlib/api.hpp"
#include "distance.h"

// Case 10-14 (PID / testing)
void pid_angular_auton();
void pid_lateral_auton();
void enqueue_test_left_auton();
void enqueue_test_right_auton();
void mcl_test_auton();

extern pros::Motor intake_motor;
extern pros::Motor intake_hood_roller;
extern pros::adi::DigitalOut hoodPiston;
extern pros::adi::DigitalOut rTongue;

extern lemlib::Chassis chassis;
extern pros::Imu imu;

// Queue helpers from main.cpp
void enqueueMotor(pros::Motor &m, int v, int t);
void enqueueDigital(pros::adi::DigitalOut &d, bool v, int t);

#endif
