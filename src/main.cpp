#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"


// === DEVICE DECLARATIONS ===


// Intake motors
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


// === DRIVE CONFIG ===
const int deadband = 5;


// Motor groups (check your directions!)
pros::MotorGroup left_motor_group({-1, 3, -2}, pros::MotorGears::blue);
pros::MotorGroup right_motor_group({-6, 5, 4}, pros::MotorGears::blue);


// Drivetrain
lemlib::Drivetrain drivetrain(
    &left_motor_group,
    &right_motor_group,
    11, // traxck width in inches
    lemlib::Omniwheel::NEW_325,
    450, // rpm
    2    // drift
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
    5.7, 0, 27, 0, 0, 0, 0, 0, 0
);


lemlib::ControllerSettings angular_controller(
    4.05, 0.0001 , 35, 0, 0, 0, 0, 0, 0
);




// input curve for steer i


// create the chassis
lemlib::Chassis chassis(drivetrain,
                        lateral_controller,
                        angular_controller,
                        sensors
);


// Chassis


// === HELPER FUNCTIONS ===
double exponential(double normalizedInput) {
    normalizedInput = pow(normalizedInput, 3);
    return (normalizedInput / 18500);
}




// === UNJAMMER TASK ===
// Runs in background; monitors intake for jams


//  void unjammer_task(void*) {
//      while (true) {
//          // If intake is running forward (R1 pressed) and jammed (low velocity)
//          if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) &&
//              !master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
           
//              // Detect jam: commanded forward but velocity too low
//              if (fabs(intake_motor.get_actual_velocity()) < 5) {
//                  intake_motor.move(-127);  // Reverse to clear jam
//                  pros::delay(300);
//                  intake_motor.move(127);   // Resume intake
//              }
//          }
//         pros::delay(50); // Check 20 times per second
//     }
// }


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
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

    //imu.reset();
    chassis.calibrate();


    pros::lcd::register_btn1_cb(on_center_button);


    // Start background unjammer task
    // pros::Task unjammerBackground(unjammer_task, (void*)"");
}


// === DISABLED & COMP INIT ===
void disabled() {}
void competition_initialize() {}


// === AUTONOMOUS ===
void autonomous() {
    pros::lcd::set_text(1, "Running Autonomous");
    chassis.setPose(0, 0, 0);
   


    int autonSelector = 3;


    switch (autonSelector) {
        case 0:
            pros::lcd::set_text(2, "Auton 0 selected");
            chassis.moveToPoint(0,24, 4000);
            chassis.waitUntilDone();
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
           
            chassis.moveToPoint(-50, 0, 1500,{.maxSpeed=70});
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
            chassis.setPose(12,24,0);
            chassis.moveToPoint(12, 48, 1000);
            chassis.waitUntilDone();
            chassis.turnToHeading(90, 1000);
            chassis.moveToPoint(36, 48, 1000);
            intake_motor.move(127);
            chassis.waitUntilDone();
            chassis.turnToPoint(48,24 , 1000);
            chassis.moveToPoint(48, 24, 1000);
            chassis.waitUntil(12);
            intake_motor.move(0);
            chassis.waitUntilDone();
            chassis.turnToHeading(0, 1000);
            chassis.moveToPoint(48,42, 1000, {.forwards = false});
            chassis.waitUntil(5);
            intake_motor.move(127);
            chassis.waitUntilDone();
            intake_hood_roller.move(-127);
            pros::delay(2000);
            rTongue.set_value(true);
            chassis.moveToPoint(48, 12, 1000);
            chassis.waitUntil(5);
            intake_hood_roller.move(0);
            chassis.waitUntilDone();
            pros::delay(2500);
            chassis.moveToPoint(48,24, 1000, {.forwards = false});
            chassis.turnToHeading(0, 1000);
            chassis.moveToPoint(48, 42, 1000, {.forwards = false});
            rTongue.set_value(false);
            chassis.waitUntilDone();
            intake_hood_roller.move(-127);
            pros::delay(2000);
            intake_hood_roller.move(0);
            intake_motor.move(0);
            break;
           
            case 3:
            chassis.turnToHeading(90, 4000);
           


            break;
            case 4:
            intake_motor.move(127);
            pros::delay(500);
            intake_motor.move(0);
            break;
    }
}


// === DRIVER CONTROL ===
void opcontrol() {


    while (true) {
        // === DRIVE ===
        int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)/1.5;
        int leftyY = exponential(leftY);
        int rightxX = exponential(rightX);




        // Our expo drive
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


        // Display IMU heading
        pros::lcd::print(3, "Heading: %.2f", imu.get_heading());
         pros::lcd::print(4, "Heading: %.2f", vertical_tracking_wheel.getDistanceTraveled());
        pros::delay(20);
    }
}



