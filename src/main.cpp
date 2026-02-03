// #include "pros/llemu.hpp"
// #define MCL_OVERRIDE_CHASSIS 0
// #define ENABLE_MCL 0

#include "main.h"
#include "lemlib/api.hpp"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include <numeric>
#include <cmath>
#include <iostream>
#include "distance.h"
#include "autos.h"
#include "skills.h"
#include "testing.h"




// #include <vector>
// #include <random>
// #include <cmath>
// #include <algorithm>

pros::Motor intake_motor(4, pros::MotorGears::blue);
pros::Motor intake_hood_roller(3, pros::MotorGears::blue);

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
pros::Distance intake_dist(11);

lemlib::TrackingWheel vertTW(&enc_vertical, lemlib::Omniwheel::NEW_275, 0.5);
lemlib::TrackingWheel horzTW(&enc_horizontal, lemlib::Omniwheel::NEW_2, -5.75);

lemlib::OdomSensors odomSensors(&vertTW, nullptr, &horzTW, nullptr, &imu);

lemlib::ControllerSettings lateral(5, 0.0001, 21, 3, 3, 1, 3, 100, 20);
lemlib::ControllerSettings angular(2.9, 0.0001, 26, 0, 0, 0, 0, 0, 0);

lemlib::Chassis chassis(drivetrain, lateral, angular, odomSensors);

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

std::vector<MechAction> mechQueue;
pros::Mutex mechMutex;

void enqueueMotor(pros::Motor &m, int v, int t = 0)
{
    mechMutex.take();
    MechAction a;
    a.type = MechAction::MOTOR;
    a.motor = &m;
    a.digital = nullptr;
    a.value = v;
    a.time = t;
    mechQueue.push_back(a);
    mechMutex.give();
}

void enqueueDigital(pros::adi::DigitalOut &d, bool v, int t = 0)
{
    mechMutex.take();
    MechAction a;
    a.type = MechAction::DIGITAL;
    a.motor = nullptr;
    a.digital = &d;
    a.value = v ? 1 : 0;
    a.time = t;
    mechQueue.push_back(a);
    mechMutex.give();
}

void mechTask(void *)
{
    int dt = 20;
    std::vector<MechAction> active;

    while (true)
    {
        mechMutex.take();
        for (int i = 0; i < mechQueue.size(); i++)
        {
            active.push_back(mechQueue[i]);
        }
        mechQueue.clear();
        mechMutex.give();

        for (int i = active.size() - 1; i >= 0; i--)
        {
            if (active[i].type == MechAction::MOTOR)
            {
                active[i].motor->move(active[i].value);
            }
            else
            {
                active[i].digital->set_value(active[i].value);
            }

            if (active[i].time > 0)
            {
                active[i].time -= dt;

                if (active[i].time <= 0)
                {
                    if (active[i].type == MechAction::MOTOR)
                    {
                        active[i].motor->move(0);
                    }
                    else
                    {
                        active[i].digital->set_value(0);
                    }

                    active.erase(active.begin() + i);
                }
            }
        }

        pros::delay(dt);
    }
}
void initialize()
{
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    pros::lcd::initialize();
    chassis.calibrate();
    pros::delay(1000);

    // if (ENABLE_MCL)
    // {
    //     mclInit(0, 0, 0);
    //     pros::Task(mclTask, nullptr);
    // }

   //pros::Task(mechTask, nullptr);,
}

double expo(double normalizedInput) {
    normalizedInput = pow(normalizedInput, 3);
    return (normalizedInput / 15000);
}

void autonomous()
{
    /*
    1 - left safe | working
    2 - right safe | working
    3 - mid goal (left) | in dev
    5 - low goal (right) | not made
    4 - 4 ball rush (left) | not made
    6 - 4 ball rush (right) | not made
    7 - red sig sawp | not made
    8 - blue sig sawp | not made
    9 - skills | not made
    10 - pid angular | working
    11 - pid lateral | working
    12 - enqueue test left| not tested
    13 - enqueue test right| not tested
    14 - mcl test | not made
    */
    int autonSelector = 15;
    // chassis.setPose(estX, estY, estH * 180 / M_PI);
    Snacky.set_value(true);
    switch (autonSelector)
    {
        case 1:
            left_safe_auton();
            break;
        case 2:
            right_safe_auton();
            break;
        case 3:
            mid_goal_left_auton();
            break;
        case 4:
            four_ball_rush_left_auton();
            break;
        case 5:
            low_goal_right_auton();
            break;
        case 6:
            four_ball_rush_right_auton();
            break;
        case 7:
            red_sig_swap_auton();
            break;
        case 8:
            blue_sig_swap_auton();
            break;
        case 9:
            skills_auton();
            break;
        case 10:
            pid_angular_auton();
            break;
        case 11:
            pid_lateral_auton();
            break;
        case 12:
            enqueue_test_left_auton();
            break;
        case 13:
            enqueue_test_right_auton();
            break;
        case 14:
            mcl_test_auton();
            break;
        case 15:
            auton_case_15();
            break;
        default:
            break;
    }
}




void opcontrol() {
    while (true) {
        int leftY  = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / 1.1;

        int driveForward = expo(leftY);
        int driveTurn    = expo(rightX);

        left_drive.move(driveForward + driveTurn);
        right_drive.move(driveForward - driveTurn);

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            intake_motor.move(127);
            intake_hood_roller.move(127);
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            intake_motor.move(127);
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            intake_motor.move(-127);
            intake_hood_roller.move(-127);
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
            intake_motor.move(90);
            intake_hood_roller.move(-90);
        }
        else {
            intake_motor.move(0);
            intake_hood_roller.move(0);
        }

        hoodPiston.set_value(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2));
        midgoalPiston.set_value(master.get_digital(pros::E_CONTROLLER_DIGITAL_A));

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            tong = !tong;
            rTongue.set_value(tong);
        }

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
            snack = !snack;
            Snacky.set_value(snack);
        }

        pros::lcd::print(1, "X %.2lf Y %.2lf", chassis.getPose().x, chassis.getPose().y);
        pros::lcd::print(2, "H %.2lf", imu.get_heading());

        pros::delay(20);
    }
}
