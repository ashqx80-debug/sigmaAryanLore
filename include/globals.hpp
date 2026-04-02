#ifndef SIGMA_GLOBALS_HPP
#define SIGMA_GLOBALS_HPP

#include "pros/distance.hpp"
#include "lemlib/api.hpp"
#include "main.h"
#include "pros/adi.hpp"
#include "pros/motors.h"
#include "autos.hpp"

//extern declarations for globals defined in src/main.cpp
extern pros::Motor intake_motor;
extern pros::Motor intake_hood_roller;

extern pros::adi::DigitalOut hoodPiston;
extern pros::adi::DigitalOut Hoard;
extern pros::adi::DigitalOut rTongue;
extern pros::adi::DigitalOut Snacky;
extern pros::adi::DigitalOut midgoalPiston;
extern pros::adi::DigitalOut midgoalDescore;

extern pros::Controller master;

extern pros::MotorGroup left_drive;
extern pros::MotorGroup right_drive;

extern lemlib::Drivetrain drivetrain;
extern lemlib::Chassis chassis;

extern pros::Imu imu;
extern pros::Rotation enc_vertical;
extern pros::Rotation enc_horizontal;
extern pros::Distance intake_dist;
extern pros::Distance back_dist;
extern pros::Optical intake_optical;

extern lemlib::TrackingWheel vertTW;
extern lemlib::TrackingWheel horzTW;
extern lemlib::OdomSensors odomSensors;

extern lemlib::ControllerSettings lateral;
extern lemlib::ControllerSettings angular;
extern int selectedAuton;

void updateMode();
void handleTouch();
void handleDiagnostics();
void handleGame();
void drawUI();
void runSelectedAuton();
void moveB(double distance, bool forwards, bool decreasing, int maxSpeed, int minSpeed, int timeout);
void moveF(double distance, bool forwards, bool decreasing, int maxSpeed, int minSpeed, int timeout);
void unjamTest();
void markDirty();

extern bool hood;
extern bool snack;
extern bool tong;
enum Mode {
    DRIVER,
    AUTON_SELECT,
    DIAG,
    GAME
};

extern Mode currentMode;

#endif // SIGMA_GLOBALS_HPP
