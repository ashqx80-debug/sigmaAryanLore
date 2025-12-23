#include "main.h"
#include "lemlib/api.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include <vector>
#include <random>
#include <cmath>
#include <algorithm>

pros::Motor intake_motor(19, pros::MotorGears::blue);
pros::Motor intake_hood_roller(11, pros::MotorGears::blue);

pros::adi::DigitalOut hoodPiston('B');
pros::adi::DigitalOut rTongue('A');
pros::adi::DigitalOut midGoal('D');

pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::MotorGroup left_drive({-1, 3, -2}, pros::MotorGears::blue);
pros::MotorGroup right_drive({-6, 5, 4}, pros::MotorGears::blue);

bool tongue = false;
bool hood = false;
bool midgoal = false;

lemlib::Drivetrain drivetrain(
    &left_drive,
    &right_drive,
    11,
    lemlib::Omniwheel::NEW_325,
    450,
    2
);

pros::Imu imu(10);
pros::Rotation enc_vertical(21);
pros::Rotation enc_horizontal(-13);

lemlib::TrackingWheel vertTW(&enc_vertical, lemlib::Omniwheel::NEW_275, -0.25);
lemlib::TrackingWheel horzTW(&enc_horizontal, lemlib::Omniwheel::NEW_2, -5.75);

lemlib::OdomSensors odomSensors(
    &vertTW, nullptr,
    &horzTW, nullptr,
    &imu
);

lemlib::ControllerSettings lateral(5.7, 0, 27, 3, 1, 100, 3, 500, 20);
lemlib::ControllerSettings angular(4.05, 0.0001, 35, 3, 1, 100, 3, 500, 0);

lemlib::Chassis chassis(drivetrain, lateral, angular, odomSensors);

pros::Distance distLeft(7);
pros::Distance distRight(8);
pros::Distance distFront(9);
pros::Distance distBack(6);

constexpr double FIELD_MIN = -72;
constexpr double FIELD_MAX = 72;

constexpr int NUM_PARTICLES = 300;
constexpr double SENSOR_STDDEV = 2.0;
constexpr double LIN_NOISE = 0.3;
constexpr double ANG_NOISE = 1.0 * (M_PI / 180);

struct Particle {
    double x;
    double y;
    double h;
    double w;
};

std::vector<Particle> particles;

std::default_random_engine rng(pros::millis());
std::normal_distribution<double> linNoise(0, LIN_NOISE);
std::normal_distribution<double> angNoise(0, ANG_NOISE);

double estX = 0;
double estY = 0;
double estH = 0;

double eLeft(const Particle& p) { return p.x - FIELD_MIN; }
double eRight(const Particle& p) { return FIELD_MAX - p.x; }
double eFront(const Particle& p) { return FIELD_MAX - p.y; }
double eBack(const Particle& p) { return p.y - FIELD_MIN; }

void mclInit(double x, double y, double h) {
    particles.clear();
    particles.reserve(NUM_PARTICLES);
    enc_vertical.reset_position();
    enc_horizontal.reset_position();
    imu.tare_heading();
    for (int i = 0; i < NUM_PARTICLES; i++) {
        particles.push_back({
            x + linNoise(rng) * 2,
            y + linNoise(rng) * 2,
            h + angNoise(rng) * 2,
            1.0 / NUM_PARTICLES
        });
    }
    chassis.setPose(x, y, h * 180 / M_PI);
}

void mclMotion(double fwd, double str, double dtheta) {
    for (auto &p : particles) {
        double f = fwd + linNoise(rng);
        double s = str + linNoise(rng);
        double dt = dtheta + angNoise(rng);
        p.h = atan2(sin(p.h + dt), cos(p.h + dt));
        p.x += f * cos(p.h) - s * sin(p.h);
        p.y += f * sin(p.h) + s * cos(p.h);
        p.x = std::clamp(p.x, FIELD_MIN, FIELD_MAX);
        p.y = std::clamp(p.y, FIELD_MIN, FIELD_MAX);
    }
}

void mclSensor() {
    double dl = distLeft.get_distance() / 25.4;
    double dr = distRight.get_distance() / 25.4;
    double df = distFront.get_distance() / 25.4;
    double db = distBack.get_distance() / 25.4;
    if (dl < 2 || dr < 2 || df < 2 || db < 2) return;
    if (dl > 70 || dr > 70 || df > 70 || db > 70) return;
    double var = SENSOR_STDDEV * SENSOR_STDDEV;
    double sum = 0;
    for (auto &p : particles) {
        p.w =
            exp(-pow(dl - eLeft(p), 2) / (2 * var)) *
            exp(-pow(dr - eRight(p), 2) / (2 * var)) *
            exp(-pow(df - eFront(p), 2) / (2 * var)) *
            exp(-pow(db - eBack(p), 2) / (2 * var));
        sum += p.w;
    }
    if (sum == 0) return;
    for (auto &p : particles) p.w /= sum;
}

void mclResample() {
    std::vector<Particle> next;
    next.reserve(NUM_PARTICLES);
    std::vector<double> cdf(NUM_PARTICLES);
    cdf[0] = particles[0].w;
    for (int i = 1; i < NUM_PARTICLES; i++)
        cdf[i] = cdf[i - 1] + particles[i].w;
    std::uniform_real_distribution<double> u(0, 1.0 / NUM_PARTICLES);
    double r = u(rng);
    int i = 0;
    for (int m = 0; m < NUM_PARTICLES; m++) {
        double U = r + m * (1.0 / NUM_PARTICLES);
        while (U > cdf[i]) i++;
        next.push_back(particles[i]);
        next.back().w = 1.0 / NUM_PARTICLES;
    }
    particles = next;
}

void mclEstimate() {
    double sx = 0, sy = 0, sc = 0, ss = 0;
    for (auto &p : particles) {
        sx += p.x;
        sy += p.y;
        sc += cos(p.h);
        ss += sin(p.h);
    }
    estX = sx / NUM_PARTICLES;
    estY = sy / NUM_PARTICLES;
    estH = atan2(ss, sc);
}

void mclTask(void*) {
    double lastV = enc_vertical.get_position();
    double lastH = enc_horizontal.get_position();
    double lastHeading = imu.get_heading() * M_PI / 180;
    while (true) {
        double v = enc_vertical.get_position();
        double h = enc_horizontal.get_position();
        double heading = imu.get_heading() * M_PI / 180;
        mclMotion(
            (v - lastV) / 100.0,
            (h - lastH) / 100.0,
            heading - lastHeading
        );
        mclSensor();
        mclResample();
        mclEstimate();
        chassis.setPose(estX, estY, estH * 180 / M_PI);
        lastV = v;
        lastH = h;
        lastHeading = heading;
        pros::delay(20);
    }
}

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
    const int dt = 20;
    std::vector<MechAction> active;
    while (true) {
        mechMutex.take();
        for (auto &a : mechQueue) active.push_back(a);
        mechQueue.clear();
        mechMutex.give();
        for (int i = active.size() - 1; i >= 0; i--) {
            auto &a = active[i];
            if (a.type == MechAction::MOTOR) a.motor->move(a.value);
            else a.digital->set_value(a.value);
            if (a.time > 0) {
                a.time -= dt;
                if (a.time <= 0) {
                    if (a.type == MechAction::MOTOR) a.motor->move(0);
                    else a.digital->set_value(0);
                    active.erase(active.begin() + i);
                }
            }
        }
        pros::delay(dt);
    }
}

void autonomous() {
    int autonSelector = 2;

    chassis.setPose(estX, estY, estH * 180 / M_PI);

    switch (autonSelector) {

        case 0:
            pros::lcd::set_text(2, "Auton 0 selected");
            intake_motor.move(127);
            pros::delay(500);
            intake_motor.move(0);
            break;

        case 1:
            chassis.moveToPoint(-27, 50, 1700, {.maxSpeed = 50});
            intake_motor.move(127);
            pros::delay(50);
            intake_motor.move(-127);
            pros::delay(50);
            intake_motor.move(127);
            chassis.waitUntilDone();

            chassis.turnToPoint(-52, 20, 1000);
            chassis.moveToPoint(-52, 20, 1500);
            chassis.waitUntil(12);
            intake_motor.move(0);

            chassis.turnToHeading(180, 1000);
            chassis.moveToPoint(-50, 42, 1000, {.forwards = false});
            chassis.waitUntil(5);

            intake_motor.move(127);
            intake_hood_roller.move(-127);
            pros::delay(50);
            intake_motor.move(-127);
            pros::delay(50);
            intake_motor.move(127);

            rTongue.set_value(true);
            pros::delay(2000);

            chassis.moveToPoint(-50, 0, 1500, {.maxSpeed = 60});
            chassis.waitUntil(5);
            intake_hood_roller.move(0);
            chassis.waitUntilDone();

            pros::delay(2000);

            chassis.moveToPoint(-50, 24, 1000, {.forwards = false});
            chassis.turnToHeading(180, 1000);
            chassis.moveToPoint(-50, 42, 1000, {.forwards = false});
            rTongue.set_value(false);
            chassis.waitUntilDone();

            intake_motor.move(127);
            pros::delay(50);
            intake_motor.move(-127);
            pros::delay(50);
            intake_motor.move(127);

            intake_hood_roller.move(-127);
            pros::delay(2000);

            intake_hood_roller.move(0);
            intake_motor.move(0);

            chassis.moveToPoint(-50, 30, 1000);
            chassis.moveToPoint(-50, 42, 1000, {.forwards = false});
            break;

        case 2:
            chassis.moveToPoint(25, 52, 1500, {.maxSpeed = 50, .earlyExitRange = 10});
            intake_motor.move(127);
            pros::delay(1500);
            chassis.waitUntilDone();

            chassis.turnToPoint(52, 20, 500);
            chassis.waitUntilDone();
            chassis.moveToPoint(52, 20, 1000);

            pros::delay(1000);
            intake_motor.move(0);

            chassis.turnToHeading(180, 1000);
            chassis.moveToPoint(46, 42, 1000, {.forwards = false});
            chassis.waitUntilDone();

            intake_motor.move(127);
            intake_hood_roller.move(-127);
            pros::delay(200);

            rTongue.set_value(true);
            pros::delay(1700);

            chassis.moveToPoint(46, -3, 500, {.maxSpeed = 60});
            chassis.waitUntil(5);
            intake_hood_roller.move(0);

            chassis.turnToHeading(180, 500);
            pros::delay(200);

            chassis.moveToPoint(46, -10, 1000, {.minSpeed = 80});
            pros::delay(2000);

            chassis.moveToPoint(46, 24, 700, {.forwards = false});
            chassis.moveToPoint(46, 42, 700, {.forwards = false});

            rTongue.set_value(false);
            chassis.waitUntilDone();

            intake_motor.move(127);
            pros::delay(50);
            intake_motor.move(-127);
            pros::delay(50);
            intake_motor.move(127);

            intake_hood_roller.move(-127);
            pros::delay(1500);

            intake_hood_roller.move(0);
            intake_motor.move(0);

            chassis.moveToPoint(46, 30, 500);
            chassis.moveToPoint(46, 42, 400, {.forwards = false});
            break;

        case 3:
            chassis.moveToPoint(0, 24, 4000);
            break;
    }
}


void initialize() {
    pros::lcd::initialize();
    chassis.calibrate();
    pros::delay(500);
    mclInit(0, 0, 0);
    pros::Task mclBackground(mclTask, nullptr);
    pros::Task mechBackground(mechTask, nullptr);
}

double expo(int v) {
    double n = v / 127.0;
    return n * n * n * 127;
}

void opcontrol() {
    static bool lastA = false;
    static bool lastB = false;
    static bool lastL2 = false;
    while (true) {
        int drive = expo(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
        int turn = expo(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
        left_drive.move(drive + turn);
        right_drive.move(drive - turn);
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
            enqueueMotor(intake_motor, 127);
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
            enqueueMotor(intake_motor, 127);
            enqueueMotor(intake_hood_roller, -127);
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
            enqueueMotor(intake_motor, -127);
            enqueueMotor(intake_hood_roller, 127);
        }
        else
            enqueueMotor(intake_motor, 0);
        bool a = master.get_digital(pros::E_CONTROLLER_DIGITAL_A);
        if (a && !lastA) tongue = !tongue;
        lastA = a;
        enqueueDigital(rTongue, tongue);
        bool b = master.get_digital(pros::E_CONTROLLER_DIGITAL_B);
        if (b && !lastB) hood = !hood;
        lastB = b;
        enqueueDigital(hoodPiston, hood);
        bool l2 = master.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
        if (l2 && !lastL2) midgoal = !midgoal;
        lastL2 = l2;
        enqueueDigital(midGoal, midgoal);
        pros::lcd::print(0, "X %.1f Y %.1f H %.1f", estX, estY, estH * 180 / M_PI);
        pros::delay(20);
    }
}
