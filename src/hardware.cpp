#include "globals.hpp"

// Hardware definitions (moved from main.cpp)
pros::Motor intake_motor(4, pros::MotorGears::blue);
pros::Motor intake_hood_roller(-12, pros::MotorGears::blue);

pros::adi::DigitalOut hoodPiston('A');
pros::adi::DigitalOut rTongue('B');
pros::adi::DigitalOut Snacky('C');
pros::adi::DigitalOut midgoalPiston('D');

bool hood = false;
bool snack = false;
bool tong = false;

pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::MotorGroup left_drive({9, -8, -10}, pros::MotorGears::blue);
pros::MotorGroup right_drive({-7, 6, 5}, pros::MotorGears::blue);

lemlib::Drivetrain drivetrain(&left_drive, &right_drive, 11, lemlib::Omniwheel::NEW_325, 450, 2);

pros::Imu imu(1);
pros::Rotation enc_vertical(-2);
pros::Rotation enc_horizontal(-22);
pros::Distance  intake_dist(11);

lemlib::TrackingWheel vertTW(&enc_vertical, lemlib::Omniwheel::NEW_275, -0.5);
lemlib::TrackingWheel horzTW(&enc_horizontal, lemlib::Omniwheel::NEW_2, -5.75);

lemlib::OdomSensors odomSensors(&vertTW, nullptr, &horzTW, nullptr, &imu);

lemlib::ControllerSettings lateral(5.5, 0.0001, 22, 3, 3, 1, 3, 100, 20);
lemlib::ControllerSettings angular(2.91, 0.0001, 26, 3, 0, 0, 0, 0, 0);

lemlib::Chassis chassis(drivetrain, lateral, angular, odomSensors);
