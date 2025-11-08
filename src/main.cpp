#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"

// === DEVICE DECLARATIONS ===

// Intake motors
pros::Motor intake_motor(7, pros::MotorGears::blue);
pros::Motor intake_hood_roller(8, pros::MotorGears::blue);

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

// === DRIVE CONFIG ===
const int deadband = 5;

// Motor groups (check your directions!)
pros::MotorGroup left_motor_group({3, -2, -1}, pros::MotorGears::blue);
pros::MotorGroup right_motor_group({-11, 5, 4}, pros::MotorGears::blue);

// Drivetrain
lemlib::Drivetrain drivetrain(
    &left_motor_group,
    &right_motor_group,
    10, // track width in inches
    lemlib::Omniwheel::NEW_325,
    360, // rpm
    2    // drift
);

// IMU
pros::Imu imu(12);

// Tracking wheels
pros::Rotation horizontal_encoder(21);
pros::Rotation vertical_encoder(20);

lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_2, -5.75);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_2, -2.5);

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
    4, 0.005, 0, 0, 0, 0, 0, 0, 0
);

lemlib::ControllerSettings angular_controller(
    0.55, 0 , 10, 3, 1, 100, 3, 500, 0
);

// Chassis
lemlib::Chassis chassis(
    drivetrain,
    lateral_controller,
    angular_controller,
    sensors
);

// === HELPER FUNCTIONS ===
double exponential(double normalizedInput) {
    normalizedInput = pow(normalizedInput, 3);
    return (normalizedInput / 20000);
}

// === UNJAMMER TASK ===
// Runs in background; monitors intake for jams
void unjammer_task(void*) {
    while (true) {
        // If intake is running forward (R1 pressed) and jammed (low velocity)
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) &&
            !master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            
            // Detect jam: commanded forward but velocity too low
            if (fabs(intake_motor.get_actual_velocity()) < 5) {
                intake_motor.move(-127);  // Reverse to clear jam
                pros::delay(300);
                intake_motor.move(127);   // Resume intake
            }
        }
        pros::delay(50); // Check 20 times per second
    }
}

// === LCD CALLBACK ===
void on_center_button() {
    static bool pressed = false;
    pressed = !pressed;
    if (pressed) {
        pros::lcd::set_text(2, "I was pressed!");
    } else {
        pros::lcd::clear_line(2);
    }
}

// === INITIALIZE ===
void initialize() {
    pros::lcd::initialize();
    imu.reset();
    chassis.calibrate();

    pros::lcd::register_btn1_cb(on_center_button);

    // Start background unjammer task
    pros::Task unjammerBackground(unjammer_task, (void*)"");
}

// === DISABLED & COMP INIT ===
void disabled() {}
void competition_initialize() {}

// === AUTONOMOUS ===
void autonomous() {
    pros::lcd::set_text(1, "Running Autonomous");
    chassis.setPose(0, 0, 0);
    

    int autonSelector = 0;

    switch (autonSelector) {
        case 0:
            pros::lcd::set_text(2, "Auton 0 selected");
            chassis.moveToPoint(0,24, 4000);
            chassis.waitUntilDone();
            
            break;
        case 1:
            pros::lcd::set_text(2, "Auton 1 selected");
            break;
        case 2:
            pros::lcd::set_text(2, "Auton 2 selected");
            break;
        default:
            pros::lcd::set_text(2, "Unknown auton: none");
            break;
    }
}

// === DRIVER CONTROL ===
void opcontrol() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

    while (true) {
        // === DRIVE ===
        int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)/1.2;
        int leftyY = exponential(leftY);
        int rightxX = exponential(rightX);

        // Arcade drive
        left_motor_group.move(leftyY + rightxX);
        right_motor_group.move(leftyY - rightxX);

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

        // Display IMU heading
        pros::lcd::print(3, "Heading: %.2f", imu.get_heading());

        pros::delay(20);
    }
}
