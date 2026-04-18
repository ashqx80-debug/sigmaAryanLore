#include "globals.hpp"
#include "pros/optical.hpp"

// Hardware definitions (moved from main.cpp)
pros::Motor intake_motor(-2, pros::MotorGears::blue);
pros::Motor intake_hood_roller(1, pros::MotorGears::blue);

pros::adi::DigitalOut Hoard('A');
pros::adi::DigitalOut rTongue('B');
pros::adi::DigitalOut Snacky('D');
pros::adi::DigitalOut midgoalPiston('E');
pros::adi::DigitalOut midgoalDescore('C');
pros::adi::DigitalOut hoodPiston('F');

bool hood = false;
bool intakeSpin = false;
bool unjamEnabled = true;

pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::MotorGroup left_drive({7, -4, -6}, pros::MotorGears::blue);
pros::MotorGroup right_drive({-8, 3, 5}, pros::MotorGears::blue);

lemlib::Drivetrain drivetrain(&left_drive, &right_drive, 9.75, lemlib::Omniwheel::NEW_325, 450, 2);

pros::Imu imu(9);
pros::Rotation enc_vertical(-12);
pros::Rotation enc_horizontal(-22);
pros::Distance  intake_dist(19);
pros::Distance back_dist(20);
pros::Optical intake_optical(14);

lemlib::TrackingWheel vertTW(&enc_vertical, lemlib::Omniwheel::NEW_2, 0.75);
lemlib::TrackingWheel horzTW(&enc_horizontal, lemlib::Omniwheel::NEW_2, -5.75);

lemlib::OdomSensors odomSensors(&vertTW, nullptr, &horzTW, nullptr, &imu);

lemlib::ControllerSettings lateral(6.54, 0.0001, 0.4, 3, 3, 1, 3, 500, 20);
lemlib::ControllerSettings angular(2.04, 0.0001, 18, 3, 1, 100, 3, 500, 0);

lemlib::Chassis chassis(drivetrain, lateral, angular, odomSensors);
