#ifndef _SKILLS_H_
#define _SKILLS_H_

#include "main.h"
#include "lemlib/api.hpp"
#include "distance.h"

// Case 9
void skills_auton();

extern pros::Motor intake_motor;
extern pros::Motor intake_hood_roller;
extern pros::adi::DigitalOut hoodPiston;
extern pros::adi::DigitalOut rTongue;
extern pros::MotorGroup left_drive;
extern pros::MotorGroup right_drive;

extern lemlib::Chassis chassis;
extern pros::Imu imu;
extern pros::Rotation enc_vertical;

#endif
