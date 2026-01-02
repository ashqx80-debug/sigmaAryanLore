#include <cmath>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define ENABLE_MCL 0
#define MCL_OVERRIDE_CHASSIS 0
#include "main.h"
#include "lemlib/api.hpp"
#include "pros/adi.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include <vector>
#include <random>
#include <algorithm>

pros::Motor intake_motor(19, pros::MotorGears::blue);
pros::Motor intake_hood_roller(11, pros::MotorGears::blue);

pros::adi::DigitalOut hoodPiston('B');
pros::adi::DigitalOut rTongue('A');
pros::adi::DigitalOut midGoal('D');

pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::MotorGroup left_drive({-1, 3, -2}, pros::MotorGears::blue);
pros::MotorGroup right_drive({-6, 5, 4}, pros::MotorGears::blue);

lemlib::Drivetrain drivetrain(&left_drive, &right_drive, 11, lemlib::Omniwheel::NEW_325, 450, 2);

pros::Imu imu(10);
pros::Rotation enc_vertical(21);
pros::Rotation enc_horizontal(-13);

lemlib::TrackingWheel vertTW(&enc_vertical, lemlib::Omniwheel::NEW_275, -0.25);
lemlib::TrackingWheel horzTW(&enc_horizontal, lemlib::Omniwheel::NEW_2, -5.75);

lemlib::OdomSensors odomSensors(&vertTW, nullptr, &horzTW, nullptr, &imu);

lemlib::ControllerSettings lateral(5.7, 0, 27, 3, 1, 100, 3, 500, 20);
lemlib::ControllerSettings angular(4.05, 0.0001, 35, 3, 1, 100, 3, 500, 0);

lemlib::Chassis chassis(drivetrain, lateral, angular, odomSensors);

bool tongue = false;
bool hood = false;
bool midgoal = false;

constexpr double FIELD_MIN = -72;
constexpr double FIELD_MAX = 72;
constexpr int NUM_PARTICLES = 300;

pros::Distance distLeft(7);
pros::Distance distRight(8);
pros::Distance distFront(9);
pros::Distance distBack(6);

struct Particle
{
    double x;
    double y;
    double h;
    double w;
};

std::vector<Particle> particles;

std::default_random_engine rng(pros::millis());
std::normal_distribution<double> linNoise(0, 0.25);
std::normal_distribution<double> angNoise(0, 0.75 * M_PI / 180);

double estX = 0;
double estY = 0;
double estH = 0;
bool mclEnabled = ENABLE_MCL;

double angleDiff(double a, double b)
{
    double d = a - b;
    while (d > M_PI)
    {
        d -= 2 * M_PI;
    }
    while (d < -M_PI)
    {
        d += 2 * M_PI;
    }
    return d;
}

double eLeft(const Particle &p)
{
    return p.x - FIELD_MIN;
}

double eRight(const Particle &p)
{
    return FIELD_MAX - p.x;
}

double eFront(const Particle &p)
{
    return FIELD_MAX - p.y;
}

double eBack(const Particle &p)
{
    return p.y - FIELD_MIN;
}

void mclInit(double x, double y, double h)
{
    particles.clear();
    for (int i = 0; i < NUM_PARTICLES; i++)
    {
        Particle p;
        p.x = x + linNoise(rng) * 2;
        p.y = y + linNoise(rng) * 2;
        p.h = h + angNoise(rng);
        p.w = 1.0 / NUM_PARTICLES;
        particles.push_back(p);
    }

    if (MCL_OVERRIDE_CHASSIS == 1)
    {
        chassis.setPose(x, y, h * 180 / M_PI);
    }

    estX = x;
    estY = y;
    estH = h;
}

void relocalizeFromWall()
{
    double dl = distLeft.get_distance() / 25.4;
    double dr = distRight.get_distance() / 25.4;
    double df = distFront.get_distance() / 25.4;
    double db = distBack.get_distance() / 25.4;

    particles.clear();

    for (int i = 0; i < NUM_PARTICLES; i++)
    {
        Particle p;

        p.h = estH + angNoise(rng);

        if (dl < 12)
        {
            p.x = FIELD_MIN + dl;
            p.y = FIELD_MIN + i * 144.0 / NUM_PARTICLES;
        }
        else if (dr < 12)
        {
            p.x = FIELD_MAX - dr;
            p.y = FIELD_MIN + i * 144.0 / NUM_PARTICLES;
        }
        else if (df < 12)
        {
            p.y = FIELD_MAX - df;
            p.x = FIELD_MIN + i * 144.0 / NUM_PARTICLES;
        }
        else
        {
            p.y = FIELD_MIN + db;
            p.x = FIELD_MIN + i * 144.0 / NUM_PARTICLES;
        }

        p.w = 1.0 / NUM_PARTICLES;
        particles.push_back(p);
    }
}

void mclTask(void *)
{
    double lastHeading = imu.get_heading() * M_PI / 180;

    while (true)
    {
        if (!mclEnabled)
        {
            pros::delay(20);
            continue;
        }

        double heading = imu.get_heading() * M_PI / 180;
        double dtheta = angleDiff(heading, lastHeading);

        double var = 2.0 * 2.0;
        double sum = 0;

        for (int i = 0; i < particles.size(); i++)
        {
            particles[i].h += dtheta + angNoise(rng);
            particles[i].x += linNoise(rng);
            particles[i].y += linNoise(rng);

            double dl = distLeft.get_distance() / 25.4;
            double dr = distRight.get_distance() / 25.4;
            double df = distFront.get_distance() / 25.4;
            double db = distBack.get_distance() / 25.4;

            if (dl < 2 || dr < 2 || df < 2 || db < 2 || dl > 70 || dr > 70 || df > 70 || db > 70)
            {
                continue;
            }

            double w = exp(-pow(dl - eLeft(particles[i]), 2) / (2 * var));
            w *= exp(-pow(dr - eRight(particles[i]), 2) / (2 * var));
            w *= exp(-pow(df - eFront(particles[i]), 2) / (2 * var));
            w *= exp(-pow(db - eBack(particles[i]), 2) / (2 * var));

            particles[i].w = w;
            sum += w;
        }

        if (sum < 1e-6)
        {
            relocalizeFromWall();
        }
        else
        {
            for (int i = 0; i < particles.size(); i++)
            {
                particles[i].w /= sum;
            }
        }

        double sx = 0;
        double sy = 0;
        double sc = 0;
        double ss = 0;

        for (int i = 0; i < particles.size(); i++)
        {
            sx += particles[i].x;
            sy += particles[i].y;
            sc += cos(particles[i].h);
            ss += sin(particles[i].h);
        }

        estX = sx / NUM_PARTICLES;
        estY = sy / NUM_PARTICLES;
        estH = atan2(ss, sc);

        if (MCL_OVERRIDE_CHASSIS == 1)
        {
            chassis.setPose(estX, estY, estH * 180 / M_PI);
        }

        for (int i = 0; i < std::min(50, (int)particles.size()); i++)
        {
            pros::lcd::print(i, "P%d: X %.1f Y %.1f", i, particles[i].x, particles[i].y);
        }

        lastHeading = heading;
        pros::delay(20);
    }
}

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

double expo(int v)
{
    double n = v / 127.0;
    return n * n * n * 127;
}

void autonomous()
{
    int autonSelector = 2;
    chassis.setPose(estX, estY, estH * 180 / M_PI);
    switch (autonSelector) {
        case 1:
            break;
        case 2:
        chassis.moveToPoint(25, 52, 1500, {.maxSpeed = 50});
        enqueueMotor(intake_motor, 127, 1500);
        chassis.waitUntilDone();

        chassis.turnToPoint(52, 20, 500);
        chassis.moveToPoint(52, 20, 1000);
        enqueueMotor(intake_motor, 0);

        chassis.turnToHeading(180, 1000);
        chassis.moveToPoint(46, 42, 1000, {.forwards = false});
        chassis.waitUntilDone();

        enqueueMotor(intake_motor, 127, 200);
        enqueueMotor(intake_hood_roller, -127, 200);
        enqueueDigital(rTongue, true, 1700);

        chassis.moveToPoint(46, -3, 500);
        enqueueMotor(intake_hood_roller, 0);

        chassis.moveToPoint(46, 24, 700, {.forwards = false});
        chassis.moveToPoint(46, 42, 700, {.forwards = false});

        enqueueDigital(rTongue, false, 0);

        enqueueMotor(intake_motor, 127, 50);
        enqueueMotor(intake_motor, -127, 50);
        enqueueMotor(intake_motor, 127, 50);

        enqueueMotor(intake_hood_roller, -127, 1500);
        enqueueMotor(intake_hood_roller, 0, 0);
        enqueueMotor(intake_motor, 0, 0);

        chassis.moveToPoint(46, 30, 500);
        chassis.moveToPoint(46, 42, 400, {.forwards = false});
        chassis.waitUntilDone();
        break;
    }
       
}

void opcontrol() {
    static bool lastA = false;
    static bool lastB = false;
    static bool lastL2 = false;

    while (true) {

        int drive = expo(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
        int turn  = expo(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));

        left_drive.move(drive + turn);
        right_drive.move(drive - turn);


        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            intake_motor.move(127);
            intake_hood_roller.move(0);
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            intake_motor.move(127);
            intake_hood_roller.move(-127);
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            intake_motor.move(-127);
            intake_hood_roller.move(127);
        }
        else {
            intake_motor.move(0);
            intake_hood_roller.move(0);
        }
        bool a = master.get_digital(pros::E_CONTROLLER_DIGITAL_A);
        if (a && !lastA) tongue = !tongue;
        lastA = a;
        rTongue.set_value(tongue);

        bool b = master.get_digital(pros::E_CONTROLLER_DIGITAL_B);
        if (b && !lastB) hood = !hood;
        lastB = b;
        hoodPiston.set_value(hood);

        bool l2 = master.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
        if (l2 && !lastL2) midgoal = !midgoal;
        lastL2 = l2;
        midGoal.set_value(midgoal);
        pros::lcd::print(0, "X %.1f Y %.1f H %.1f",
                         estX, estY, estH * 180 / M_PI);

        pros::delay(20);
    }
}


void initialize()
{
    pros::lcd::initialize();
    chassis.calibrate();
    pros::delay(500);

    if (ENABLE_MCL)
    {
        mclInit(0, 0, 0);
        pros::Task(mclTask, nullptr);
    }

    pros::Task(mechTask, nullptr);
}

}
