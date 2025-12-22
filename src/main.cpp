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
#include <algorithm>
#include <cmath>


pros::Motor intake_motor(19, pros::MotorGears::blue);
pros::Motor intake_hood_roller(11, pros::MotorGears::blue);

// Pneumatics
pros::adi::DigitalOut hoodPiston('B');
pros::adi::DigitalOut rTongue('A');
pros::adi::DigitalOut midGoal('D');

// Pneumatic states
bool hoodState = false;
bool rTongueState = false;
bool midGoalState = false;

// Controller
pros::Controller master(pros::E_CONTROLLER_MASTER);


const int deadband = 5;

// Motor groups
pros::MotorGroup left_motor_group({-1, 3, -2}, pros::MotorGears::blue);
pros::MotorGroup right_motor_group({-6, 5, 4}, pros::MotorGears::blue);

// Drivetrain
lemlib::Drivetrain drivetrain(
    &left_motor_group,
    &right_motor_group,
    11,
    lemlib::Omniwheel::NEW_325,
    450,
    2
);

// IMU
pros::Imu imu(10);

// Tracking wheels
pros::Rotation horizontal_encoder(21);
pros::Rotation vertical_encoder(-13);

lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_2, -5.75);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_275, -0.25);

// Odometry sensors
lemlib::OdomSensors sensors(
    &vertical_tracking_wheel,
    nullptr,
    &horizontal_tracking_wheel,
    nullptr,
    &imu
);

// PID tuning
lemlib::ControllerSettings lateral_controller(
    5.7, 0, 27, 3, 1, 100, 3, 500, 20
);

lemlib::ControllerSettings angular_controller(
    4.05, 0.0001 , 35, 3, 1, 100, 3, 500, 0
);

// Chassis
lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors);

// Distance sensors
pros::Distance distLeft(7);
pros::Distance distRight(8);

// Particle settings
const int NUM_PARTICLES = 300;
const double SENSOR_STD_DEV = 2.0;
const double MOTION_NOISE_LINEAR = 0.2;
const double MOTION_NOISE_ANG = 1.0 * (M_PI/180);

const double FIELD_X_MIN = -72;
const double FIELD_X_MAX = 72;
const double FIELD_Y_MIN = -72;
const double FIELD_Y_MAX = 72;

struct Particle {
    double x;
    double y;
    double heading;
    double weight;
};

std::vector<Particle> particles;

std::default_random_engine rng(pros::millis());
std::normal_distribution<double> noise_linear(0, MOTION_NOISE_LINEAR);
std::normal_distribution<double> noise_angle(0, MOTION_NOISE_ANG);

// Final estimated pose
double mcl_x = 0;
double mcl_y = 0;
double mcl_h = 0;


void mcl_init(double x, double y, double heading) {
    particles.clear();
    particles.reserve(NUM_PARTICLES);

    for (int i = 0; i < NUM_PARTICLES; i++) {
        particles.push_back({
            x + ((rand() % 100) / 100.0 - 0.5) * 4,
            y + ((rand() % 100) / 100.0 - 0.5) * 4,
            heading + ((rand() % 100) / 100.0 - 0.5) * (10 * M_PI/180),
            1.0 / NUM_PARTICLES
        });
    }
}


void mcl_motion_update(double dx, double dy, double dtheta) {
    for (auto &p : particles) {
        p.x += dx + noise_linear(rng);
        p.y += dy + noise_linear(rng);
        p.heading += dtheta + noise_angle(rng);

        if (p.heading > M_PI) p.heading -= 2*M_PI;
        if (p.heading < -M_PI) p.heading += 2*M_PI;
    }
}


double predicted_left_distance(const Particle &p) {
    return fabs((FIELD_X_MIN - p.x) / cos(p.heading));
}

double predicted_right_distance(const Particle &p) {
    return fabs((FIELD_X_MAX - p.x) / cos(p.heading));
}

void mcl_sensor_update() {
    double dl = distLeft.get_distance() / 25.4;
    double dr = distRight.get_distance() / 25.4;

    for (auto &p : particles) {
        double pl = predicted_left_distance(p);
        double pr = predicted_right_distance(p);

        double wl = exp(-pow(dl - pl, 2) / (2 * SENSOR_STD_DEV * SENSOR_STD_DEV));
        double wr = exp(-pow(dr - pr, 2) / (2 * SENSOR_STD_DEV * SENSOR_STD_DEV));

        p.weight = wl * wr;
    }

    double sum = 0;
    for (auto &p : particles) sum += p.weight;
    for (auto &p : particles) p.weight /= sum;
}


void mcl_resample() {
    std::vector<Particle> new_particles;
    new_particles.reserve(NUM_PARTICLES);

    std::vector<double> cumulative(NUM_PARTICLES);
    cumulative[0] = particles[0].weight;
    for (int i = 1; i < NUM_PARTICLES; i++)
        cumulative[i] = cumulative[i - 1] + particles[i].weight;

    double step = 1.0 / NUM_PARTICLES;
    double r = ((double) rand() / RAND_MAX) * step;
    int index = 0;

    for (int i = 0; i < NUM_PARTICLES; i++) {
        double u = r + i * step;
        while (u > cumulative[index]) index++;
        new_particles.push_back(particles[index]);
    }

    particles = new_particles;
}


void mcl_compute_estimate() {
    double x = 0, y = 0;
    double sinSum = 0, cosSum = 0;

    for (auto &p : particles) {
        x += p.x;
        y += p.y;
        sinSum += sin(p.heading);
        cosSum += cos(p.heading);
    }

    mcl_x = x / NUM_PARTICLES;
    mcl_y = y / NUM_PARTICLES;
    mcl_h = atan2(sinSum, cosSum);
}


void mcl_task(void*) {
    double lastV = vertical_encoder.get_position();
    double lastH = horizontal_encoder.get_position();
    double lastHeading = imu.get_heading() * (M_PI/180);

    while (true) {
        double v = vertical_encoder.get_position();
        double h = horizontal_encoder.get_position();

        double dv = (v - lastV) / 100.0;
        double dh = (h - lastH) / 100.0;

        double heading = imu.get_heading() * (M_PI/180);
        double dtheta = heading - lastHeading;

        double dx = dv * cos(heading) - dh * sin(heading);
        double dy = dv * sin(heading) + dh * cos(heading);

        mcl_motion_update(dx, dy, dtheta);
        mcl_sensor_update();
        mcl_resample();
        mcl_compute_estimate();

        lastV = v;
        lastH = h;
        lastHeading = heading;

        pros::delay(20);
    }
}

double exponential(double normalizedInput) {
    normalizedInput = pow(normalizedInput, 3);
    return (normalizedInput / 18500);
}


void on_center_button() {
    static bool pressed = false;
    pressed = !pressed;
    if (pressed) {
        pros::lcd::set_text(2, "I was pressed!");
    } else {
        pros::lcd::clear_line(2);
    }
}

void initialize() {
    pros::lcd::initialize();
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

    chassis.calibrate();

    pros::lcd::register_btn1_cb(on_center_button);

    mcl_init(0, 0, 0);
    pros::Task mclBackground(mcl_task, (void*)"");
}

void disabled() {}
void competition_initialize() {}

void autonomous() {
    pros::lcd::set_text(1, "Running Autonomous");
    chassis.setPose(0, 0, 0);

    int autonSelector = 2;

    switch (autonSelector) {
        case 0:
            pros::lcd::set_text(2, "Auton 0 selected");
            intake_motor.move(127);
            pros::delay(500);
            intake_motor.move(0); 
            break;

        case 1:
            chassis.setPose(-12,26,0);
            chassis.moveToPoint(-27, 50, 1700, {.maxSpeed=50});
            intake_motor.move(127);
            pros::delay(50);
            intake_motor.move(-127);
            pros::delay(50);
            intake_motor.move(127);
            chassis.waitUntilDone();
            chassis.turnToPoint(-52,20, 1000);
            chassis.moveToPoint(-52, 20, 1500);
            chassis.waitUntil(12);
            intake_motor.move(0);
            chassis.turnToHeading(180, 1000);
            chassis.moveToPoint(-50,42, 1000, {.forwards = false});
            chassis.waitUntil(5);
            intake_motor.move(127);
            intake_hood_roller.move(-127);
            intake_motor.move(127);
            pros::delay(50);
            intake_motor.move(-127);
            pros::delay(50);
            intake_motor.move(127);
            rTongue.set_value(true);
            pros::delay(2000);
<<<<<<< HEAD

            chassis.moveToPoint(-50, 0, 1500,{.maxSpeed=70});
=======
           
            chassis.moveToPoint(-50, 0, 1500,{.maxSpeed=60});
>>>>>>> 3a983c57e7781368016fbb0da9bc01a36591dd4c
            chassis.waitUntil(5);
            intake_hood_roller.move(0);
            chassis.waitUntilDone();
            pros::delay(2000);
            chassis.moveToPoint(-50,24, 1000, {.forwards = false});
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
            chassis.moveToPoint(-50, 42, 1000, {.forwards= false});
            break;

        case 2:
            chassis.setPose(12,26,0);
            chassis.moveToPoint(25, 52, 1500, {.maxSpeed=50, .earlyExitRange = 10});
            intake_motor.move(127);
            //pros::delay(20);
            //intake_motor.move(-127);
            //pros::delay(20);
            //intake_motor.move(127);
            pros::delay(1500);
            chassis.waitUntilDone();
            chassis.turnToPoint(52, 20, 500);
            chassis.waitUntilDone();
            chassis.moveToPoint(52, 20, 1000);
            pros::delay(1000);
            intake_motor.move(0);
            chassis.turnToHeading(180, 1000);
            chassis.moveToPoint(46,42, 1000, {.forwards = false});
            chassis.waitUntilDone();
            intake_motor.move(127);
            intake_hood_roller.move(-127);
            pros::delay(200);
            rTongue.set_value(true);
<<<<<<< HEAD
            pros::delay(2000);

            chassis.moveToPoint(50, 0, 1500,{.maxSpeed=70});
=======
            pros::delay(1700);

            chassis.moveToPoint(46, -3, 500, {.maxSpeed=60});
>>>>>>> 3a983c57e7781368016fbb0da9bc01a36591dd4c
            chassis.waitUntil(5);
            intake_hood_roller.move(0);
            //chassis.waitUntilDone();
            //chassis.moveToPoint(46, 2, 500);
            chassis.turnToHeading(180, 500);
            pros::delay(200);
            chassis.moveToPoint(46, -10,  1000, {.minSpeed=80});
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
            chassis.moveToPoint(46, 42, 400, {.forwards= false});
            break;
<<<<<<< HEAD

        case 3:
            chassis.turnToHeading(90, 4000);
            intake_motor.move(127);
            pros::delay(500);
            intake_motor.move(0);
            break;
=======
           
            case 3:
            chassis.moveToPoint(0,24, 4000);

>>>>>>> 3a983c57e7781368016fbb0da9bc01a36591dd4c

        case 4:
            pros::lcd::set_text(2, "Case 4 With MCL");

            mcl_init(-12, 26, 0);

            for (int i = 0; i < 50; i++) {
                pros::lcd::print(5, "MCL: X=%.2f  Y=%.2f  H=%.2f",
                                 mcl_x, mcl_y, mcl_h * 180/M_PI);
                pros::delay(20);
            }

            chassis.moveToPoint(mcl_x + 12, mcl_y + 12, 2000);
            chassis.waitUntilDone();

            pros::lcd::print(6, "Final MCL: %.1f, %.1f, %.1f",
                             mcl_x, mcl_y, mcl_h * 180/M_PI);

            break;

        case 5:
            break;
    }
}
void opcontrol() {
    while (true) {
        // === DRIVE ===
        int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / 1.5;

        int leftyY = exponential(leftY);
        int rightxX = exponential(rightX);

        // Expo arcade drive
        left_motor_group.move(leftyY + rightX);
        right_motor_group.move(leftyY - rightX);

        // === INTAKE CONTROL ===
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            intake_motor.move(127);
            intake_hood_roller.move(-127);
            hoodState = true;
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            intake_motor.move(127);
            intake_hood_roller.move(0);
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            intake_hood_roller.move(127);
            intake_motor.move(-127);
        }
        else {
            intake_motor.move_voltage(0);
            intake_hood_roller.move_voltage(0);
            hoodState = false;
        }

        // === PNEUMATICS TOGGLE ===
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            rTongueState = !rTongueState;
            rTongue.set_value(rTongueState);
        }
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
            midGoalState = !midGoalState;
            midGoal.set_value(midGoalState);
        }

        // === DISPLAY DATA ===
        pros::lcd::print(3, "IMU Heading: %.2f", imu.get_heading());
        pros::lcd::print(4, "Vert Wheel: %.2f", vertical_tracking_wheel.getDistanceTraveled());
        pros::lcd::print(5, "MCL X: %.1f  Y: %.1f", mcl_x, mcl_y);
        pros::lcd::print(6, "MCL H: %.1f deg", mcl_h * 180.0 / M_PI);

        pros::delay(20);
    }
}

