#include "main.h"
#include "lemlib/api.hpp"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include <numeric>
#include <cmath>
#include <iostream>
#include "distance.h"
#include "skills.hpp"
#include "globals.hpp"

void skillsProg(){
    // -------- SKILLS --------
        pros::lcd::print(1, "X %.2lf Y %.2lf", chassis.getPose().x, chassis.getPose().y);
        pros::lcd::print(2, "H %.2lf", imu.get_heading());

        chassis.setPose(-12, 26, -90);
        moveF(620, true, true, 95, 0, 1300);
        chassis.turnToHeading(-180, 500);
        chassis.waitUntilDone();
        rTongue.set_value(true);
        pros::delay(500);
        intake_motor.move(127);
        chassis.moveToPoint(chassis.getPose().x, 0.5, 1000, {.maxSpeed = 55});
        chassis.waitUntilDone();
        left_drive.move_velocity(127);
        right_drive.move_velocity(127);
        pros::delay(1600);

        chassis.moveToPoint(chassis.getPose().x, 15, 500, {.forwards = false});
        chassis.waitUntilDone();
        chassis.turnToHeading(-90, 500);
        chassis.waitUntilDone();
        rTongue.set_value(false);
        pros::delay(500);
        moveF(350, true, true, 50, 0, 750);
        chassis.turnToHeading(0, 500);
        chassis.waitUntilDone();
        chassis.moveToPoint(chassis.getPose().x-1, chassis.getPose().y+50, 1500, {.maxSpeed = 85});
        chassis.waitUntilDone();
        moveF(800, true, true, 85, 0, 900);

        chassis.moveToPoint(chassis.getPose().x+5, chassis.getPose().y+5, 700);
        chassis.waitUntilDone();
        chassis.turnToHeading(-90, 600);
        chassis.waitUntilDone();
        moveF(600, false, false, 85, 0, 750);
       chassis.turnToHeading(0, 500);
       chassis.waitUntilDone();
       moveF(1231, false, false, 55, 0, 1250);
       chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y-3, 400, {.forwards = false, .maxSpeed = 85});
        intake_hood_roller.move(127);
        hoodPiston.set_value(true);
        rTongue.set_value(true);
        intake_motor.move(-127);
        pros::delay(250);
        intake_motor.move(127);
        pros::delay(1600);
        hoodPiston.set_value(false);
        intake_hood_roller.move(0);
        
        chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y+50, 1300, {.maxSpeed = 55});
        chassis.waitUntilDone();
        left_drive.move_velocity(127);
        right_drive.move_velocity(127);
        pros::delay(1600);

        chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y-47.5, 1500, {.forwards = false, .maxSpeed = 75});
        chassis.waitUntilDone();
        rTongue.set_value(false);
        chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y-2, 500, {.forwards = false, .maxSpeed = 85});
        intake_hood_roller.move(127);
        hoodPiston.set_value(true);
        pros::delay(1600);
        hoodPiston.set_value(true);

        chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y+10, 500);
        chassis.waitUntilDone();
        chassis.turnToHeading(90, 500);
        intake_hood_roller.move(0);
        hoodPiston.set_value(false);
        chassis.moveToPoint(chassis.getPose().x+72, chassis.getPose().y, 1750, {.maxSpeed = 95});
        chassis.waitUntilDone();
        moveF(665, true, true, 75, 0, 1250);

        chassis.turnToHeading(0, 500);
        rTongue.set_value(true);
        chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y + 35, 1200, {.maxSpeed = 55});
        left_drive.move_velocity(127);
        right_drive.move_velocity(127);
        pros::delay(1600);

        chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y - 15, 700, {.forwards = false});
        chassis.waitUntilDone();
        chassis.turnToHeading(90, 500);
        rTongue.set_value(false);
        chassis.waitUntilDone();
        moveF(350, true, true, 65, 0, 600);
        chassis.turnToHeading(180, 500);
        chassis.waitUntilDone();
        chassis.moveToPoint(chassis.getPose().x+1, chassis.getPose().y-55, 1300, {.maxSpeed = 85});
        chassis.waitUntilDone();
        moveF(800, true, true, 75, 0, 1000);
        chassis.moveToPoint(chassis.getPose().x-5, chassis.getPose().y-5, 750);
        chassis.waitUntilDone();
        chassis.turnToHeading(90, 500);
        chassis.waitUntilDone();
        moveF(600, false, false, 85, 0, 600);
        chassis.turnToHeading(180, 500);
        chassis.waitUntilDone();
        moveF(1131, false, false, 55, 0, 1250);
        intake_hood_roller.move(127);
        hoodPiston.set_value(true);
        rTongue.set_value(true);
        chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y+3, 500, {.forwards = false, .maxSpeed = 85});
        intake_motor.move(-127);
        pros::delay(250);
        intake_motor.move(127);
        pros::delay(1600);
        hoodPiston.set_value(false);
        intake_hood_roller.move(0);
        chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y-50, 1300, {.maxSpeed = 55});
        chassis.waitUntilDone();
        left_drive.move_velocity(127);
        right_drive.move_velocity(127);
        pros::delay(1600);
        chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y+47.5, 1600, {.forwards = false, .maxSpeed = 75});
        chassis.waitUntilDone();
        rTongue.set_value(false);
        chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y+3, 500, {.forwards = false, .maxSpeed = 85});
        intake_hood_roller.move(127);
        hoodPiston.set_value(true);
        pros::delay(1600);
        hoodPiston.set_value(true);
        chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y-15, 750);
        chassis.waitUntilDone();
        chassis.moveToPoint(chassis.getPose().x-40, chassis.getPose().y-28, 1000, {.maxSpeed = 75});
        chassis.waitUntilDone();
        chassis.turnToHeading(270, 500);
        chassis.waitUntilDone();
        chassis.moveToPoint(chassis.getPose().x-20, chassis.getPose().y, 1500, {.maxSpeed = 75});
}