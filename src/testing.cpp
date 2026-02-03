#include "testing.h"

void pid_angular_auton() {
    chassis.setPose(0,0,0);
    pros::lcd::print(2, "H %.2lf", imu.get_heading());
    chassis.turnToHeading(180, 2000);
    pros::lcd::print(2, "H %.2lf", imu.get_heading());
    chassis.waitUntilDone();
    pros::delay(1000);
    chassis.turnToHeading(-90, 2000);
    pros::lcd::print(2, "H %.2lf", imu.get_heading());
    chassis.waitUntilDone();
    pros::delay(1000);
    chassis.turnToHeading(90, 2000);
    pros::lcd::print(2, "H %.2lf", imu.get_heading());
    chassis.waitUntilDone();
    pros::delay(1000);
    chassis.turnToHeading(0, 2000);
    pros::lcd::print(2, "H %.2lf", imu.get_heading());
}

void pid_lateral_auton() {
    chassis.setPose(0,0,0);
    chassis.moveToPoint(0,24, 2000);
    chassis.waitUntilDone();
    pros::delay(1000);
    chassis.moveToPoint(0, 48, 2000);
    chassis.waitUntilDone();
    pros::delay(1000);
    chassis.moveToPoint(0, -24, 2000, {.forwards = false});
    chassis.waitUntilDone();
    pros::delay(1000);
    chassis.moveToPoint(0, 0, 2000);
}

void enqueue_test_left_auton() {
    chassis.setPose(-12, 26, 0);

    enqueueMotor(intake_motor, 127, 0);
    enqueueMotor(intake_hood_roller, 0, 0);
    enqueueDigital(rTongue, true, 0);

    chassis.moveToPoint(-27, 50, 1500, {.maxSpeed=75});
    chassis.waitUntil(20);

    enqueueMotor(intake_motor, 127, 0);
    enqueueDigital(rTongue, true, 0);

    chassis.turnToPoint(-52, 20, 800);
    chassis.moveToPoint(-52, 20, 1500);
    chassis.waitUntil(16);

    enqueueMotor(intake_motor, 127, 0);
    enqueueMotor(intake_hood_roller, -127, 0);
    enqueueDigital(hoodPiston, true, 0);
    enqueueDigital(rTongue, true, 0);
    pros::delay(1200);

    chassis.turnToHeading(180, 800);
    chassis.moveToPoint(-49, 0, 1200, {.maxSpeed=65});
    chassis.waitUntil(5);

    enqueueMotor(intake_hood_roller, 0, 0);
    enqueueDigital(hoodPiston, false, 0);

    chassis.moveToPoint(-48, 24, 800, {.forwards = false});
    chassis.turnToHeading(180, 800);
    chassis.moveToPoint(-48, 42, 800, {.forwards = false});

    enqueueDigital(rTongue, false, 0);
    enqueueMotor(intake_motor, 127, 0);
    enqueueMotor(intake_hood_roller, -127, 0);
    enqueueDigital(hoodPiston, true, 0);
    pros::delay(750);

    enqueueDigital(hoodPiston, false, 0);
    enqueueMotor(intake_hood_roller, 0, 0);
    enqueueMotor(intake_motor, 0, 0);

    chassis.moveToPoint(-48, 48, 500, {.minSpeed = 80});
}

void enqueue_test_right_auton() {
    chassis.setPose(12, 26, 0);

    enqueueMotor(intake_motor, 127, 0);
    enqueueMotor(intake_hood_roller, 0, 0);
    enqueueDigital(rTongue, true, 0);

    chassis.moveToPoint(27, 50, 1500, {.maxSpeed=75});
    chassis.waitUntil(20);

    enqueueMotor(intake_motor, 127, 0);
    enqueueDigital(rTongue, true, 0);

    chassis.turnToPoint(52, 20, 800);
    chassis.moveToPoint(52, 20, 1500);
    chassis.waitUntil(16);

    enqueueMotor(intake_motor, 127, 0);
    enqueueMotor(intake_hood_roller, -127, 0);
    enqueueDigital(hoodPiston, true, 0);
    enqueueDigital(rTongue, true, 0);
    pros::delay(1200);

    chassis.turnToHeading(180, 800);
    chassis.moveToPoint(49, 0, 1200, {.maxSpeed=75});
    chassis.waitUntil(5);

    enqueueMotor(intake_hood_roller, 0, 0);
    enqueueDigital(hoodPiston, false, 0);

    chassis.moveToPoint(48, 24, 800, {.forwards = false});
    chassis.turnToHeading(180, 800);
    chassis.moveToPoint(48, 42, 800, {.forwards = false});

    enqueueDigital(rTongue, false, 0);
    enqueueMotor(intake_motor, 127, 0);
    enqueueMotor(intake_hood_roller, -127, 0);
    enqueueDigital(hoodPiston, true, 0);
    pros::delay(1500);

    enqueueDigital(hoodPiston, false, 0);
    enqueueMotor(intake_hood_roller, 0, 0);
    enqueueMotor(intake_motor, 0, 0);

    chassis.moveToPoint(48, 48, 1000, {.forwards=false, .minSpeed = 80});
}

void mcl_test_auton() {
    // not made
}
