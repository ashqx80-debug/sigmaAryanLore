#include "autos.h"

void left_safe_auton() {
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
}

void right_safe_auton() {
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
}

void mid_goal_left_auton() {
    chassis.setPose(-12,26,0);
    chassis.moveToPoint(-24, 45, 1300, {.maxSpeed=75});
    intake_motor.move(127);
    chassis.waitUntil(20);
    rTongue.set_value(true);
    chassis.waitUntilDone();
    chassis.turnToPoint(-15,54, 500, {.forwards = false});
    chassis.waitUntilDone();
    intake_motor.move(-70);
    chassis.moveToPoint(-15, 54, 800, {.forwards = false, .maxSpeed=70});
    chassis.waitUntilDone();
    intake_hood_roller.move(127);
    intake_motor.move(127);
    hoodPiston.set_value(true);
    pros::delay(1500);

    chassis.moveToPoint(-24, 48, 1200);
    chassis.turnToPoint(-52,20, 700);
    chassis.moveToPoint(-52, 20, 1000);
    chassis.waitUntil(5);
    intake_motor.move(0);
    hoodPiston.set_value(true);
    chassis.turnToHeading(180, 700);
    chassis.waitUntilDone();
    rTongue.set_value(true);
    intake_motor.move(127);
    chassis.moveToPoint(-48, 0, 1000, {.maxSpeed=70});
    chassis.waitUntil(5);
    intake_hood_roller.move(0);
    hoodPiston.set_value(false);
    chassis.waitUntilDone();
    pros::delay(700);
    chassis.moveToPoint(-48,24, 700, {.forwards = false});
    chassis.turnToHeading(180, 700);
    chassis.moveToPoint(-48, 42, 700, {.forwards = false});
    rTongue.set_value(false);
    chassis.waitUntilDone();
    intake_motor.move(127);
    intake_hood_roller.move(-127);
    hoodPiston.set_value(true);
    pros::delay(1200);
    hoodPiston.set_value(false);
    intake_hood_roller.move(0);
    intake_motor.move(0);
}

void low_goal_right_auton() {
    // not made
}

void four_ball_rush_left_auton() {
    // not made
}

void four_ball_rush_right_auton() {
    // not made
}

void red_sig_swap_auton() {
    chassis.setPose(-12,26,0);
    chassis.moveToPoint(-50, 26, 1500, {.maxSpeed=75});
}

void blue_sig_swap_auton() {
    chassis.setPose(12,26,90);
    chassis.moveToPoint(48, 26, 1000);
    chassis.waitUntilDone();

    chassis.turnToHeading(180, 350);
    chassis.waitUntilDone();

    intake_motor.move(127);
    rTongue.set_value(true);

    chassis.moveToPoint(48, -1, 500, {.maxSpeed = 80});
    chassis.waitUntilDone();
    left_drive.move_velocity(127);
    right_drive.move_velocity(127);
    pros::delay(500);
    chassis.turnToPoint(47, 43, 500, {.forwards = false});
    chassis.moveToPoint(47, 43, 750, {.forwards = false, .maxSpeed = 85});
    rTongue.set_value(false);
    chassis.waitUntilDone();
    intake_hood_roller.move(100);
    hoodPiston.set_value(true);
    pros::delay(1000);
    hoodPiston.set_value(false);
    intake_hood_roller.move(0);
    chassis.moveToPoint(49, 20, 500);
    chassis.waitUntilDone();
    chassis.turnToPoint(25, 51, 300);
    chassis.moveToPoint(25, 51, 1000, {.maxSpeed=85});
    chassis.waitUntilDone();
    rTongue.set_value(false);
    chassis.turnToPoint(-13, 58, 200);
    chassis.moveToPoint(-30, 51, 700, { .maxSpeed=85});
    chassis.waitUntilDone();
    rTongue.set_value(true);
    intake_motor.move(80);
    chassis.turnToPoint(-13, 58, 500, {.forwards = false});
    chassis.moveToPoint(-13, 58, 750, {.forwards = false});
    chassis.waitUntilDone();
    midgoalPiston.set_value(true);
    left_drive.move_velocity(-40);
    right_drive.move_velocity(-40);
    intake_hood_roller.move(-90);
    intake_motor.move(-80);
    pros::delay(250);
    left_drive.move_velocity(0);
    right_drive.move_velocity(0);
    intake_motor.move(80);
    pros::delay(500);
    intake_motor.move(80);
    pros::delay(1200);
    intake_motor.move(0);
    intake_hood_roller.move(0);
    midgoalPiston.set_value(false);
    chassis.turnToPoint(chassis.getPose().x-45, chassis.getPose().y - 31, 100);
    chassis.moveToPoint(chassis.getPose().x-45, chassis.getPose().y - 31, 750);
    chassis.waitUntilDone();
    chassis.turnToHeading(180, 200);
    chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y -35, 500, {.maxSpeed=75});
    chassis.waitUntilDone();
    intake_motor.move(127);
    left_drive.move_velocity(127);
    right_drive.move_velocity(127);
    pros::delay(300);
    chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y +42, 700, {.forwards = false, .maxSpeed=85 });
    chassis.waitUntilDone();
    hoodPiston.set_value(true);
    intake_hood_roller.move(127);
    pros::delay(1500);
}

void elims_right_descore_auton() {
    chassis.setPose(12, 26, 90);
    Snacky.set_value(true);
    chassis.moveToPoint(49, 26,950);
    chassis.turnToHeading(180,950);
    rTongue.set_value(true);
    chassis.moveToPoint(48, 0,900, {.maxSpeed=75});
    intake_motor.move(127);
    left_drive.move_velocity(127);
    right_drive.move_velocity(127);
    pros::delay(300);
    left_drive.move_velocity(0);
    right_drive.move_velocity(0);
    chassis.turnToPoint(47, 46,950, {.forwards = false});
    chassis.moveToPoint(47, 46,950, {.forwards = false});
    rTongue.set_value(false);
    chassis.waitUntilDone();
    intake_hood_roller.move(127);
    hoodPiston.set_value(true);
    pros::delay(1500);
    hoodPiston.set_value(false);
    intake_hood_roller.move(0);
    intake_motor.move(0);
    chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y - 10, 500);
    rTongue.set_value(false);
    chassis.turnToHeading(90,500);
    chassis.moveToPoint(64.3,34,1000);
    chassis.waitUntilDone();
    chassis.turnToHeading(-180,500);
    chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y + 20, 2000, {.forwards = false});
    Snacky.set_value(false);
}

void auton_case_15() {
    chassis.setPose(12, 26, 90);
    Snacky.set_value(true);
    intake_motor.move(127);

    chassis.moveToPoint(27, 50, 1250, {.maxSpeed=75});
    chassis.waitUntilDone();
    rTongue.set_value(true);
    chassis.moveToPoint(49.4, 26,950);
    chassis.turnToHeading(180,950);

    chassis.moveToPoint(48, 0,900, {.maxSpeed=75});
    intake_motor.move(127);
    left_drive.move_velocity(127);
    right_drive.move_velocity(127);
    pros::delay(300);
    left_drive.move_velocity(0);
    right_drive.move_velocity(0);
    chassis.turnToPoint(47, 46,950, {.forwards = false});
    chassis.moveToPoint(47, 46,950, {.forwards = false, .maxSpeed=85});
    rTongue.set_value(false);
    chassis.waitUntilDone();
    intake_hood_roller.move(100);
    hoodPiston.set_value(true);
    intake_motor.move(-127);
    pros::delay(250);
    intake_motor.move(127);
    pros::delay(1500);
    hoodPiston.set_value(false);
    intake_hood_roller.move(0);
    intake_motor.move(0);
    chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y - 7, 500);
    rTongue.set_value(false);
    chassis.turnToHeading(90,500);
    chassis.moveToPoint(60.3,34,1000);
    chassis.waitUntilDone();
    chassis.turnToHeading(-180,500);
    Snacky.set_value(false);
    chassis.moveToPoint(chassis.getPose().x-4, chassis.getPose().y + 10, 2000, {.forwards = false});
}
