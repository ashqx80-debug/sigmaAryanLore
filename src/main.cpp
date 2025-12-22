#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include <vector>
#include <random>
#include <cmath>

/* ===================== HARDWARE ===================== */

pros::Motor intake_motor(19, pros::MotorGears::blue);
pros::Motor intake_hood_roller(11, pros::MotorGears::blue);

pros::adi::DigitalOut hoodPiston('B');
pros::adi::DigitalOut rTongue('A');
pros::adi::DigitalOut midGoal('D');

pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::MotorGroup left_motor_group({-1, 3, -2}, pros::MotorGears::blue);
pros::MotorGroup right_motor_group({-6, 5, 4}, pros::MotorGears::blue);

lemlib::Drivetrain drivetrain(
    &left_motor_group,
    &right_motor_group,
    11,
    lemlib::Omniwheel::NEW_325,
    450,
    2
);

pros::Imu imu(10);

pros::Rotation horizontal_encoder(21);
pros::Rotation vertical_encoder(-13);

lemlib::TrackingWheel horizontal_tracking_wheel(
    &horizontal_encoder, lemlib::Omniwheel::NEW_2, -5.75);

lemlib::TrackingWheel vertical_tracking_wheel(
    &vertical_encoder, lemlib::Omniwheel::NEW_275, -0.25);

lemlib::OdomSensors sensors(
    &vertical_tracking_wheel,
    nullptr,
    &horizontal_tracking_wheel,
    nullptr,
    &imu
);

lemlib::ControllerSettings lateral_controller(5.7, 0, 27, 3, 1, 100, 3, 500, 20);
lemlib::ControllerSettings angular_controller(4.05, 0.0001, 35, 3, 1, 100, 3, 500, 0);

lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors);

/* ===================== DISTANCE SENSORS ===================== */

pros::Distance distLeft(7);
pros::Distance distRight(8);
pros::Distance distFront(9);

/* ===================== FIELD ===================== */

constexpr double FIELD_X_MIN = -72;
constexpr double FIELD_X_MAX =  72;
constexpr double FIELD_Y_MIN = -72;
constexpr double FIELD_Y_MAX =  72;

/* ===================== MCL PARAMETERS ===================== */

constexpr int NUM_PARTICLES = 250;
constexpr double SENSOR_STD_DEV = 2.0;
constexpr double BASE_LINEAR_NOISE = 0.25;
constexpr double BASE_ANG_NOISE = 1.0 * (M_PI / 180);

/* ===================== PARTICLES ===================== */

struct Particle {
    double x;
    double y;
    double heading;
    double weight;
};

std::vector<Particle> particles;

/* ===================== RANDOM ===================== */

std::default_random_engine rng(pros::millis());
std::normal_distribution<double> lin_noise(0, BASE_LINEAR_NOISE);
std::normal_distribution<double> ang_noise(0, BASE_ANG_NOISE);

/* ===================== MCL OUTPUT ===================== */

double mcl_x = 0;
double mcl_y = 0;
double mcl_h = 0;

/* ===================== SCREEN MAPPING ===================== */

int fieldToScreenX(double x) {
    return (int)((x - FIELD_X_MIN) / (FIELD_X_MAX - FIELD_X_MIN) * 480);
}

int fieldToScreenY(double y) {
    return (int)(240 - (y - FIELD_Y_MIN) / (FIELD_Y_MAX - FIELD_Y_MIN) * 240);
}

/* ===================== PARTICLE DRAW ===================== */

void drawParticles() {
    pros::screen::erase();

    pros::screen::set_pen(0xFFFFFF);
    for (auto &p : particles) {
        int sx = fieldToScreenX(p.x);
        int sy = fieldToScreenY(p.y);
        if (sx >= 0 && sx < 480 && sy >= 0 && sy < 240)
            pros::screen::draw_pixel(sx, sy);
    }

    int mx = fieldToScreenX(mcl_x);
    int my = fieldToScreenY(mcl_y);

    pros::screen::set_pen(0xFF0000);
    pros::screen::fill_circle(mx, my, 3);

    int hx = mx + 10 * cos(mcl_h);
    int hy = my - 10 * sin(mcl_h);
    pros::screen::draw_line(mx, my, hx, hy);
}

/* ===================== MCL INIT ===================== */

void mcl_init(double x, double y, double h) {
    particles.clear();
    particles.reserve(NUM_PARTICLES);

    vertical_encoder.reset_position();
    horizontal_encoder.reset_position();
    imu.tare_heading();

    for (int i = 0; i < NUM_PARTICLES; i++) {
        particles.push_back({
            x + lin_noise(rng) * 2.0,
            y + lin_noise(rng) * 2.0,
            h + ang_noise(rng) * 2.0,
            1.0 / NUM_PARTICLES
        });
    }

    chassis.setPose(x, y, h * 180.0 / M_PI);
}

/* ===================== MOTION MODEL ===================== */

void mcl_motion_update(double forward, double sideways, double dtheta, bool pushing) {
    double linScale = pushing ? 3.0 : 1.0;
    double angScale = pushing ? 2.0 : 1.0;

    for (auto &p : particles) {
        double f = forward + lin_noise(rng) * linScale;
        double s = sideways + lin_noise(rng) * linScale;
        double dt = dtheta + ang_noise(rng) * angScale;

        p.heading += dt;
        if (p.heading > M_PI) p.heading -= 2 * M_PI;
        if (p.heading < -M_PI) p.heading += 2 * M_PI;

        p.x += f * cos(p.heading) - s * sin(p.heading);
        p.y += f * sin(p.heading) + s * cos(p.heading);
    }
}

/* ===================== SENSOR MODEL ===================== */

double expected_left(const Particle &p)  { return p.x - FIELD_X_MIN; }
double expected_right(const Particle &p) { return FIELD_X_MAX - p.x; }
double expected_front(const Particle &p) { return FIELD_Y_MAX - p.y; }

void mcl_sensor_update() {
    double dl = distLeft.get_distance() / 25.4;
    double dr = distRight.get_distance() / 25.4;
    double df = distFront.get_distance() / 25.4;

    if (dl < 1 || dr < 1 || df < 1) return;

    double sum = 0;
    for (auto &p : particles) {
        double w =
            exp(-pow(dl - expected_left(p), 2) / (2 * SENSOR_STD_DEV * SENSOR_STD_DEV)) *
            exp(-pow(dr - expected_right(p), 2) / (2 * SENSOR_STD_DEV * SENSOR_STD_DEV)) *
            exp(-pow(df - expected_front(p), 2) / (2 * SENSOR_STD_DEV * SENSOR_STD_DEV));
        p.weight = w;
        sum += w;
    }

    for (auto &p : particles)
        p.weight /= sum;
}

/* ===================== RESAMPLING ===================== */

void mcl_resample() {
    std::vector<Particle> newSet;
    newSet.reserve(NUM_PARTICLES);

    std::vector<double> cumulative(NUM_PARTICLES);
    cumulative[0] = particles[0].weight;
    for (int i = 1; i < NUM_PARTICLES; i++)
        cumulative[i] = cumulative[i - 1] + particles[i].weight;

    double step = 1.0 / NUM_PARTICLES;
    double r = ((double) rand() / RAND_MAX) * step;
    int idx = 0;

    for (int i = 0; i < NUM_PARTICLES; i++) {
        double u = r + i * step;
        while (u > cumulative[idx]) idx++;
        Particle p = particles[idx];
        p.weight = 1.0 / NUM_PARTICLES;
        newSet.push_back(p);
    }

    particles = newSet;
}

/* ===================== ESTIMATE ===================== */

void mcl_estimate() {
    double x = 0, y = 0, s = 0, c = 0;
    for (auto &p : particles) {
        x += p.x;
        y += p.y;
        s += sin(p.heading);
        c += cos(p.heading);
    }
    mcl_x = x / NUM_PARTICLES;
    mcl_y = y / NUM_PARTICLES;
    mcl_h = atan2(s, c);
}

/* ===================== MCL TASK ===================== */

void mcl_task(void *) {
    double lastV = vertical_encoder.get_position();
    double lastH = horizontal_encoder.get_position();
    double lastHeading = imu.get_heading() * M_PI / 180;

    int drawCounter = 0;

    while (true) {
        double v = vertical_encoder.get_position();
        double h = horizontal_encoder.get_position();

        double dv = (v - lastV) / 100.0;
        double dh = (h - lastH) / 100.0;

        double heading = imu.get_heading() * M_PI / 180;
        double dtheta = heading - lastHeading;

        bool pushing =
            fabs(left_motor_group.get_actual_velocity(0)) < 5 &&
            fabs(right_motor_group.get_actual_velocity(0)) < 5;

        mcl_motion_update(dv, dh, dtheta, pushing);
        mcl_sensor_update();
        mcl_resample();
        mcl_estimate();

        chassis.setPose(mcl_x, mcl_y, mcl_h * 180.0 / M_PI);

        if (++drawCounter >= 5) {
            drawParticles();
            drawCounter = 0;
        }

        lastV = v;
        lastH = h;
        lastHeading = heading;

        pros::delay(20);
    }
}

/* ===================== INIT ===================== */

void initialize() {
    pros::lcd::initialize();
    chassis.calibrate();
    pros::delay(500);

    mcl_init(0, 0, 0);
    pros::Task mclBackground(mcl_task, nullptr);
}
