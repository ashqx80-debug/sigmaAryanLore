// ninja Warning: ChatGPT copyright below
#include "pros/llemu.hpp"
//#define ENABLE_MCL 0
//#define MCL_OVERRIDE_CHASSIS 0

#include "main.h"
#include "lemlib/api.hpp"
#include "pros/adi.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include <vector>
#include <random>
#include <cmath>
#include <algorithm>

// -------------------- MOTORS & PNEUMATICS --------------------

pros::Motor intake_motor(5, pros::MotorGears::blue);
pros::Motor intake_hood_roller(10, pros::MotorGears::blue);

pros::adi::DigitalOut hoodPiston('A');
pros::adi::DigitalOut rTongue('C');
pros::adi::DigitalOut Snacky('B');

bool hood = false;

bool snack = false;
bool tong = false;

pros::Controller master(pros::E_CONTROLLER_MASTER);

// -------------------- DRIVE --------------------

pros::MotorGroup left_drive({8, -7, -6}, pros::MotorGears::blue);
pros::MotorGroup right_drive({1, 2, -3}, pros::MotorGears::blue);

lemlib::Drivetrain drivetrain(&left_drive, &right_drive, 11,
                              lemlib::Omniwheel::NEW_325, 450, 2);

pros::Imu imu(18);
pros::Rotation enc_vertical(-21);
pros::Rotation enc_horizontal(-13);

lemlib::TrackingWheel vertTW(&enc_vertical, lemlib::Omniwheel::NEW_275, +0.5);
lemlib::TrackingWheel horzTW(&enc_horizontal, lemlib::Omniwheel::NEW_2, -5.75);

lemlib::OdomSensors odomSensors(&vertTW, nullptr, &horzTW, nullptr, &imu);

lemlib::ControllerSettings lateral(5, 0.0001, 21, 0, 0, 0, 0, 0, 30);
lemlib::ControllerSettings angular(2.45, 0.0001, 22.5, 3, 1, 200, 3, 600, 90);

lemlib::Chassis chassis(drivetrain, lateral, angular, odomSensors);

// -------------------- MCL (UNCHANGED, VERBATIM) --------------------

// constexpr double FIELD_MIN = -72;
// constexpr double FIELD_MAX = 72;
// constexpr int NUM_PARTICLES = 300;

// pros::Distance distLeft(7);
// pros::Distance distRight(8);
// pros::Distance distFront(9);
// pros::Distance distBack(6);

// struct Particle
// {
//     double x;
//     double y;
//     double h;
//     double w;
// };

// std::vector<Particle> particles;

// std::default_random_engine rng(pros::millis());
// std::normal_distribution<double> linNoise(0, 0.25);
// std::normal_distribution<double> angNoise(0, 0.75 * M_PI / 180);

// double estX = 0;
// double estY = 0;
// double estH = 0;
// bool mclEnabled = ENABLE_MCL;

// double angleDiff(double a, double b)
// {
//     double d = a - b;
//     while (d > M_PI)
//     {
//         d -= 2 * M_PI;
//     }
//     while (d < -M_PI)
//     {
//         d += 2 * M_PI;
//     }
//     return d;
// }

// double eLeft(const Particle &p)
// {
//     return p.x - FIELD_MIN;
// }

// double eRight(const Particle &p)
// {
//     return FIELD_MAX - p.x;
// }

// double eFront(const Particle &p)
// {
//     return FIELD_MAX - p.y;
// }

// double eBack(const Particle &p)
// {
//     return p.y - FIELD_MIN;
// }

// void mclInit(double x, double y, double h)
// {
//     particles.clear();
//     for (int i = 0; i < NUM_PARTICLES; i++)
//     {
//         Particle p;
//         p.x = x + linNoise(rng) * 2;
//         p.y = y + linNoise(rng) * 2;
//         p.h = h + angNoise(rng);
//         p.w = 1.0 / NUM_PARTICLES;
//         particles.push_back(p);
//     }

//     estX = x;
//     estY = y;
//     estH = h;
// }

// -------------------- MECH ACTION SYSTEM --------------------

struct MechAction {
    enum Type { MOTOR, DIGITAL } type;
    pros::Motor* motor;
    pros::adi::DigitalOut* digital;
    int value;
    int time;
};

std::vector<MechAction> mechQueue;
pros::Mutex mechMutex;

void enqueueMotor(pros::Motor& m, int v, int t = 0) {
    mechMutex.take();
    mechQueue.push_back({MechAction::MOTOR, &m, nullptr, v, t});
    mechMutex.give();
}

void enqueueDigital(pros::adi::DigitalOut& d, bool v, int t = 0) {
    mechMutex.take();
    mechQueue.push_back({MechAction::DIGITAL, nullptr, &d, v ? 1 : 0, t});
    mechMutex.give();
}

void mechTask(void*) {
    while (true) {
        mechMutex.take();
        if (mechQueue.empty()) {
            mechMutex.give();
            pros::delay(10);
            continue;
        }

        MechAction a = mechQueue.front();
        mechQueue.erase(mechQueue.begin());
        mechMutex.give();

        if (a.type == MechAction::MOTOR) {
            a.motor->move(a.value);
            if (a.time > 0) {
                pros::delay(a.time);
                a.motor->move(0);
            }
        } else {
            a.digital->set_value(a.value);
            if (a.time > 0) {
                pros::delay(a.time);
                a.digital->set_value(0);
            }
        }
    }
}

// -------------------- INIT --------------------

void initialize() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    pros::lcd::initialize();
    chassis.calibrate();
    //pros::Task mech(mechTask, nullptr);
}

// -------------------- EXPO --------------------

double expo(double normalizedInput) {
    normalizedInput = pow(normalizedInput, 3);
    return (normalizedInput / 18500);
}

// -------------------- AUTONOMOUS (MOVEMENTS UNCHANGED) --------------------

void autonomous() {
     /*
    1 - left safe | working
    2 - right safe | working
    3 - mid goal (left) | in dev
    5 - low goal (right) | not made
    7 - red sig sawp | not made
    8 - blue sig sawp | not made
    9 - skills | not made
    10 - pid angular | working
    11 - pid lateral | working 
    12 - enqueue left | not tested
    13 - enqueue right | not tested
    14 - enqueue skills | not tested
    */

    int autonSelector = 2;
    Snacky.set_value(true);

    switch (autonSelector) {
         case 1:
            chassis.setPose(-12,26,0);
            chassis.moveToPoint(-27, 50, 1500, {.maxSpeed=75});
            intake_motor.move(127);
            chassis.waitUntil(20);
            rTongue.set_value(true);
            chassis.waitUntilDone();
            chassis.turnToPoint(-52,20, 800);
            chassis.moveToPoint(-52, 20, 1500);
            chassis.waitUntil(16);
            intake_motor.move(0);
            chassis.turnToHeading(180, 800);
            chassis.moveToPoint(-50,42, 800, {.forwards = false});
            chassis.waitUntilDone();
            intake_motor.move(127);
            intake_hood_roller.move(-127);
            hoodPiston.set_value(true);
            rTongue.set_value(true);
            pros::delay(1200);
            chassis.turnToHeading(180, 800);
            chassis.moveToPoint(-49, 0, 1200, {.maxSpeed=65});
            chassis.waitUntil(5);
            intake_hood_roller.move(0);
            hoodPiston.set_value(false);
            chassis.waitUntilDone();
            pros::delay(800);
            chassis.moveToPoint(-48,24, 800, {.forwards = false});
            chassis.turnToHeading(180, 800);
            chassis.moveToPoint(-48, 42, 800, {.forwards = false});
            rTongue.set_value(false);
            chassis.waitUntilDone();
            intake_motor.move(127);
            intake_hood_roller.move(-127);
            hoodPiston.set_value(true);
            pros::delay(750);
            chassis.moveToPoint(-48, 30, 200);
            hoodPiston.set_value(false);
            intake_hood_roller.move(0);
            intake_motor.move(0);
            chassis.moveToPoint(-48, 48, 500, {.minSpeed = 80});
            break;

        case 2:
            chassis.setPose(12,26,0);
            chassis.moveToPoint(27, 50, 1500, {.maxSpeed=75});
            intake_motor.move(127);
            chassis.waitUntil(20);
            rTongue.set_value(true);
            chassis.waitUntilDone();
            chassis.turnToPoint(52,20, 800);
            chassis.moveToPoint(52, 20, 1500);
            chassis.waitUntil(16);
            intake_motor.move(0);
            chassis.turnToHeading(180, 800);
            chassis.moveToPoint(50,42, 800, {.forwards = false});
            chassis.waitUntilDone();
            intake_motor.move(127);
            intake_hood_roller.move(-127);
            hoodPiston.set_value(true);
            rTongue.set_value(true);
            pros::delay(1200);
            chassis.turnToHeading(180, 800);
            chassis.moveToPoint(49, 0, 1200, {.maxSpeed=75});
            chassis.waitUntil(5);
            intake_hood_roller.move(0);
            hoodPiston.set_value(false);
            chassis.waitUntilDone();
            pros::delay(800);
            chassis.moveToPoint(48,24, 800, {.forwards = false});
            chassis.turnToHeading(180, 800);
            chassis.moveToPoint(48, 42, 800, {.forwards = false});
            rTongue.set_value(false);
            chassis.waitUntilDone();
            intake_motor.move(127);
            intake_hood_roller.move(-127);
            hoodPiston.set_value(true);
            pros::delay(1500);
            chassis.moveToPoint(48, 30, 1000);
            hoodPiston.set_value(false);
            intake_hood_roller.move(0);
            intake_motor.move(0);
            chassis.moveToPoint(48, 48, 1000, {.forwards=false, .minSpeed = 80});
            break;


}
}
// -------------------- OPCONTROL (UNCHANGED, VERBATIM) --------------------

void opcontrol() {
    while (true) {

        int leftY  = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / 1.1;

        int driveForward = expo(leftY);
        int driveTurn    = expo(rightX);

        left_drive.move(driveForward + driveTurn);
        right_drive.move(driveForward - driveTurn);

        bool intakeRunning = false;
        int intakePower = 0;
        int hoodRollerPower = 0;
        bool hooding = false;

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            intakePower = 127;
            hoodRollerPower = 127;
            intakeRunning = true;
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            intakePower = 127;
            intakeRunning = false;
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            intakePower = -127;
            hoodRollerPower = 127;
            intakeRunning = false;
            
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
            intakePower = 60;
            hoodRollerPower = -60;
            intakeRunning = false;
        }

        intake_motor.move(intakePower);
        intake_hood_roller.move(hoodRollerPower);
        hoodPiston.set_value(intakeRunning);
        

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
