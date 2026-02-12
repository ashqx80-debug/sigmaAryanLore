#ifndef SIGMA_GLOBALS_HPP
#define SIGMA_GLOBALS_HPP

#include "main.h"
#include "lemlib/api.hpp"
#include "pros/adi.hpp"
#include "pros/motors.h"
#include "pros/distance.hpp"

//extern declarations for globals defined in src/main.cpp
extern pros::Motor intake_motor;
extern pros::Motor intake_hood_roller;

extern pros::adi::DigitalOut hoodPiston;
extern pros::adi::DigitalOut rTongue;
extern pros::adi::DigitalOut Snacky;
extern pros::adi::DigitalOut midgoalPiston;

extern pros::Controller master;

extern pros::MotorGroup left_drive;
extern pros::MotorGroup right_drive;

extern lemlib::Drivetrain drivetrain;
extern lemlib::Chassis chassis;

extern pros::Imu imu;
extern pros::Rotation enc_vertical;
extern pros::Rotation enc_horizontal;
extern pros::Distance intake_dist;

extern lemlib::TrackingWheel vertTW;
extern lemlib::TrackingWheel horzTW;
extern lemlib::OdomSensors odomSensors;

extern lemlib::ControllerSettings lateral;
extern lemlib::ControllerSettings angular;

extern bool hood;
extern bool snack;
extern bool tong;

#endif // SIGMA_GLOBALS_HPP
