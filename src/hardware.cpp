#include "globals.hpp"
#include "pros/optical.hpp"

pros::Motor intake_motor(-2, pros::MotorGears::blue);
pros::Motor intake_hood_roller(1, pros::MotorGears::blue);

pros::adi::DigitalOut hoodPiston('A');
pros::adi::DigitalOut rTongue('B');
pros::adi::DigitalOut Snacky('C');
pros::adi::DigitalOut midgoalPiston('D');

bool hood = false;
bool snack = false;
bool tong = false;
bool intakeSpin = false;
bool unjamEnabled = true;

pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::MotorGroup left_drive({10, -9, -8}, pros::MotorGears::blue);
pros::MotorGroup right_drive({-6, 7, 5}, pros::MotorGears::blue);
lemlib::Drivetrain drivetrain(&left_drive, &right_drive, 11, lemlib::Omniwheel::NEW_325, 450, 2);

pros::Imu imu(1);
pros::Rotation enc_vertical(-2);
pros::Rotation enc_horizontal(-22);
pros::Distance  intake_dist(11);
pros::Optical colorSensor(14);

lemlib::TrackingWheel vertTW(&enc_vertical, lemlib::Omniwheel::NEW_275, -0.5);
lemlib::TrackingWheel horzTW(&enc_horizontal, lemlib::Omniwheel::NEW_2, -5.75);

lemlib::OdomSensors odomSensors(&vertTW, nullptr, &horzTW, nullptr, &imu);

lemlib::ControllerSettings lateral(5.5, 0.0001, 22, 3, 3, 1, 3, 100, 20);
lemlib::ControllerSettings angular(2.91, 0.0001, 26, 3, 0, 0, 0, 0, 0);

lemlib::Chassis chassis(drivetrain, lateral, angular, odomSensors);
