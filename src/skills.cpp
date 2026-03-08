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
        moveF(660, true, true, 95, 0, 1000);
        chassis.turnToHeading(-180, 500);
        chassis.waitUntilDone();
        rTongue.set_value(true);
        pros::delay(300);
        intake_motor.move(127);
        chassis.moveToPoint(chassis.getPose().x, 1, 900, {.maxSpeed = 62});
        chassis.waitUntilDone();
        left_drive.move_velocity(127);
        right_drive.move_velocity(127);
        pros::delay(1600);

        chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y + 15, 500, {.forwards = false});
        chassis.waitUntilDone();
        chassis.turnToHeading(-90, 450);
        rTongue.set_value(false);
        chassis.waitUntilDone();
        moveF(350, true, true, 75, 0, 500);
        chassis.turnToHeading(180, 450);
        chassis.waitUntilDone();
        intake_motor.move(0);
        chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y+50, 1200, {.forwards = false,.maxSpeed = 85});
        chassis.waitUntilDone();
        moveB(800, false, true, 95, 0, 900);
        chassis.turnToHeading(90, 600);
        chassis.waitUntilDone();
        moveB(500, true, false, 95, 0, 500);
       chassis.turnToHeading(0, 400);
       chassis.waitUntilDone();
       moveF(1181, false, false, 65, 0, 750);
       intake_motor.move(127);
       chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y-5, 400, {.forwards = false, .maxSpeed = 85});
        intake_hood_roller.move(127);
        hoodPiston.set_value(true);
        rTongue.set_value(true);
        pros::delay(1600);
        hoodPiston.set_value(false);
        intake_hood_roller.move(0);
        
        chassis.moveToPoint(chassis.getPose().x+1, chassis.getPose().y+45.7, 1200, {.maxSpeed = 58});
        chassis.waitUntilDone();
        left_drive.move_velocity(127);
        right_drive.move_velocity(127);
        pros::delay(1800);

        chassis.moveToPoint(chassis.getPose().x-1, chassis.getPose().y-48, 1200, {.forwards = false, .maxSpeed = 75});
        chassis.waitUntilDone();
        rTongue.set_value(false);
        chassis.moveToPoint(chassis.getPose().x+3, chassis.getPose().y-7, 500, {.forwards = false, .maxSpeed = 85});
        intake_hood_roller.move(127);
        hoodPiston.set_value(true);
        pros::delay(1600);

        chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y+5, 500);
        hoodPiston.set_value(false);
        chassis.waitUntilDone();
        intake_hood_roller.move(0);
        chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y-5, 500, {.maxSpeed = 65});
        chassis.waitUntilDone();
        chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y+10, 500);
        chassis.waitUntilDone();
        chassis.turnToHeading(130, 500);
        intake_hood_roller.move(0);
        hoodPiston.set_value(false);
        chassis.moveToPoint(chassis.getPose().x+72, chassis.getPose().y-5, 1100, {.maxSpeed = 95});
        chassis.waitUntilDone();
        moveF(780, true, true, 95, 0, 900);

        chassis.turnToHeading(0, 500);
        chassis.waitUntilDone();
        rTongue.set_value(true);
        moveF(360, true, true, 65, 55, 500);

        chassis.waitUntilDone();
        left_drive.move_velocity(127);
        right_drive.move_velocity(127);
        pros::delay(1800);

        chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y - 15, 700, {.forwards = false});
        chassis.waitUntilDone();
        chassis.turnToHeading(90, 500);
        rTongue.set_value(false);
        chassis.waitUntilDone();
        moveF(350, true, true, 65, 0, 600);
        chassis.turnToHeading(0, 500);
        chassis.waitUntilDone();
        chassis.moveToPoint(chassis.getPose().x+1.3, chassis.getPose().y-55, 1000, {.forwards = false, .maxSpeed = 85});
        chassis.waitUntilDone();
        moveB(800, false, true, 85, 0, 1000);
        chassis.turnToHeading(-90, 500);
        chassis.waitUntilDone();
        moveB(500, true, false, 85, 0, 600);
        chassis.turnToHeading(180, 500);
        chassis.waitUntilDone();
        moveF(1161, false, false, 75, 0, 700);
        intake_hood_roller.move(127);
        hoodPiston.set_value(true);
        rTongue.set_value(true);
        chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y+5, 400, {.forwards = false, .maxSpeed = 85});
        pros::delay(1600);
        hoodPiston.set_value(false);
        intake_hood_roller.move(0);
        chassis.moveToPoint(chassis.getPose().x+2, chassis.getPose().y-48, 1250, {.maxSpeed = 60});
        chassis.waitUntilDone();

        left_drive.move_velocity(127);
        right_drive.move_velocity(127);
        pros::delay(1800);
        chassis.moveToPoint(chassis.getPose().x+1, chassis.getPose().y+47, 1000, {.forwards = false, .maxSpeed = 70});
        chassis.waitUntilDone();
        rTongue.set_value(false);
        chassis.moveToPoint(chassis.getPose().x-2, chassis.getPose().y+5, 400, {.forwards = false, .maxSpeed = 85});
        intake_hood_roller.move(127);
        hoodPiston.set_value(true);
        pros::delay(1600);
        chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y-5, 300, {.maxSpeed = 85});
        chassis.waitUntilDone();
        intake_hood_roller.move(0);
        hoodPiston.set_value(false);
        chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y+5, 500, {.maxSpeed = 65});
        chassis.waitUntilDone();
        
        chassis.moveToPoint(chassis.getPose().x-25, chassis.getPose().y-35, 800, {.maxSpeed = 95});
        chassis.waitUntilDone();
        chassis.turnToHeading(240, 500);
        chassis.waitUntilDone();
        chassis.moveToPoint(chassis.getPose().x-50, chassis.getPose().y-3, 1200, {.maxSpeed = 105, .minSpeed = 95});
        chassis.waitUntilDone();
        rTongue.set_value(true);
        pros::delay(500);
        rTongue.set_value(false);
        chassis.moveToPoint(chassis.getPose().x-30, chassis.getPose().y, 750, {.maxSpeed = 105, .minSpeed = 95});
        chassis.waitUntilDone();
        chassis.turnToHeading(-90, 500);
        moveF(700, true, true, 95, 0, 500);
        chassis.turnToPoint(chassis.getPose().x-35, chassis.getPose().y+30, 500);
}