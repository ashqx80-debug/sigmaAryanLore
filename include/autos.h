#ifndef _AUTOS_H_
#define _AUTOS_H_

#include "main.h"
#include "lemlib/api.hpp"

// Case 1-8 (main autos)
void left_safe_auton();
void right_safe_auton();
void mid_goal_left_auton();
void low_goal_right_auton();
void four_ball_rush_left_auton();
void four_ball_rush_right_auton();
void red_sig_swap_auton();
void blue_sig_swap_auton();
void elims_right_descore_auton();

// Extra existing auton (case 15)
void auton_case_15();

// Externs for hardware and chassis defined in main.cpp
extern pros::Motor intake_motor;
extern pros::Motor intake_hood_roller;
extern pros::adi::DigitalOut hoodPiston;
extern pros::adi::DigitalOut rTongue;
extern pros::adi::DigitalOut Snacky;
extern pros::adi::DigitalOut midgoalPiston;

extern pros::MotorGroup left_drive;
extern pros::MotorGroup right_drive;

extern lemlib::Chassis chassis;

#endif
