#include "skills.h"

void skills_auton() {
    pros::lcd::print(1, "X %.2lf Y %.2lf", chassis.getPose().x, chassis.getPose().y);
    pros::lcd::print(2, "H %.2lf", imu.get_heading());

    chassis.setPose(12, 26, 90);

    chassis.moveToPoint(49, 26, 1500);
    chassis.waitUntilDone();
    pros::delay(500);

    chassis.turnToHeading(180, 500);
    chassis.waitUntilDone();

    intake_motor.move(127);
    rTongue.set_value(true);

    chassis.moveToPoint(49, 0, 500, {.maxSpeed = 60});
    chassis.waitUntilDone();

    left_drive.move_velocity(127);
    right_drive.move_velocity(127);
    pros::delay(1600);

    chassis.moveToPoint(48, 15, 500);
    chassis.waitUntilDone();

    chassis.turnToPoint(57.5, 45, 500, {.forwards = false});
    rTongue.set_value(false);
    chassis.waitUntilDone();

    intake_motor.move(0);
    chassis.moveToPoint(57.5, 45, 1500, {.forwards = false, .maxSpeed = 65});
    chassis.waitUntilDone();
    chassis.turnToHeading(180, 500);

    moveF(50, false, true, 65, 15, 3000);

    imu.reset();
    pros::delay(2500);
    enc_vertical.reset_position();
    pros::delay(2500);

    chassis.setPose(58, 110, 180);
    chassis.turnToHeading(270, 1000);

    chassis.moveToPoint(40, 110, 1500);
    chassis.waitUntilDone();

    chassis.turnToHeading(0, 1000);
    pros::delay(1500);

    chassis.moveToPoint(45, 94, 1500, {.forwards = false});
    chassis.waitUntilDone();

    intake_motor.move(127);
    intake_hood_roller.move(127);
    hoodPiston.set_value(true);
    pros::delay(2500);

    rTongue.set_value(true);
    hoodPiston.set_value(false);
    intake_hood_roller.move(0);

    chassis.moveToPoint(40, 144, 500, {.maxSpeed = 75});
    chassis.waitUntilDone();

    left_drive.move_velocity(127);
    right_drive.move_velocity(127);
    pros::delay(1600);

    chassis.moveToPoint(42, 94, 1000, {.forwards = false});
    chassis.waitUntilDone();

    intake_hood_roller.move(127);
    hoodPiston.set_value(true);
    pros::delay(1500);
}
