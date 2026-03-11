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
        moveF(690, true, true, 95, 0, 1000);
        chassis.turnToHeading(180, 500);
        pros::delay(200);
        rTongue.set_value(true);
        intake_motor.move(127);
        chassis.waitUntilDone();
        chassis.moveToPoint(chassis.getPose().x, 0.5, 800, {.maxSpeed = 60});
        chassis.waitUntilDone();
        left_drive.move_velocity(127);
        right_drive.move_velocity(127);
        pros::delay(1700);

        chassis.moveToPoint(chassis.getPose().x-10.5, 42, 1000, {.forwards = false, .maxSpeed = 90});
        chassis.waitUntilDone();
        rTongue.set_value(false);
        chassis.waitUntilDone();
        chassis.turnToHeading(180, 500);
        chassis.waitUntilDone();
        intake_motor.move(0);
        moveB(800, false, true, 90, 0, 1500);
        chassis.turnToHeading(90, 850);
        chassis.waitUntilDone();
        moveB(485, true, false, 72, 0, 600);
       chassis.turnToHeading(0, 400);
       chassis.waitUntilDone();
       moveF(1155, false, false, 75, 64, 750);
       intake_motor.move(127);
       intake_hood_roller.move(100);
       chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y-5, 500);
       chassis.turnToHeading(0, 500);
       chassis.waitUntilDone();
       

        rTongue.set_value(true);
        intake_hood_roller.move(-127);
        intake_motor.move(-127);
        pros::delay(250);
        intake_motor.move(127);
        intake_hood_roller.move(127);
        pros::delay(1800);

        intake_hood_roller.move(0);
        
        chassis.moveToPoint(chassis.getPose().x-1.3, chassis.getPose().y+44.3, 1200, {.maxSpeed = 68});
        chassis.waitUntilDone();
        left_drive.move_velocity(127);
        right_drive.move_velocity(127);
        pros::delay(1800);
        left_drive.move_velocity(0);
        right_drive.move_velocity(0);
        chassis.turnToHeading(0, 650);
        chassis.waitUntilDone();
        chassis.moveToPoint(chassis.getPose().x-3.5, chassis.getPose().y-48.3, 800, {.forwards = false, .maxSpeed = 80});
        chassis.waitUntilDone();

        

        intake_hood_roller.move(100);
        chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y-5, 500);
        chassis.turnToHeading(0,500);
        chassis.waitUntilDone();
        intake_hood_roller.move(-127);
        intake_motor.move(-127);
        pros::delay(150);
        intake_motor.move(127);
        intake_hood_roller.move(127);
        pros::delay(2000);
        rTongue.set_value(false);
        moveF(840, true, true, 95, 0, 350);
        chassis.turnToHeading(90, 500);
        intake_hood_roller.move(0);
        chassis.waitUntilDone();
        moveB(2000, true, false, 95, 0, 900);
        chassis.waitUntilDone();
        chassis.turnToHeading(90, 500);
        chassis.waitUntilDone();
        moveF(755, true, true, 85, 0, 900);
        chassis.turnToHeading(90,500);
        chassis.waitUntilDone();

        chassis.turnToHeading(0, 650);
        pros::delay(200);
        rTongue.set_value(true);
        chassis.waitUntilDone();

        moveF(390, true, true, 65, 55, 500);

        chassis.waitUntilDone();
        left_drive.move_velocity(127);
        right_drive.move_velocity(127);
        pros::delay(1800);
        chassis.moveToPoint(chassis.getPose().x+14,  chassis.getPose().y-40, 1000, {.forwards=false, .maxSpeed=85});
        rTongue.set_value(false);
        chassis.waitUntilDone();
        chassis.turnToHeading(0, 500);
        chassis.waitUntilDone();
        moveB(790, false, true, 85, 0, 1200);
        chassis.turnToHeading(-90, 500);
        chassis.waitUntilDone();
        moveB(490, true, false, 85, 0, 600);
        chassis.turnToHeading(180, 500);
        chassis.waitUntilDone();
        moveF(1131, false, false, 75, 0, 700);
        intake_hood_roller.move(100);
        rTongue.set_value(true);
        chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y+6, 400, {.forwards = false, .maxSpeed = 85});
        intake_hood_roller.move(-127);
        intake_motor.move(-127);
        pros::delay(250);
        intake_motor.move(127);
        intake_hood_roller.move(127);
        pros::delay(1800);

        intake_hood_roller.move(0);
        chassis.moveToPoint(chassis.getPose().x-3.5, chassis.getPose().y-48, 1200, {.maxSpeed = 65});
        chassis.waitUntilDone();
        left_drive.move_velocity(127);
        right_drive.move_velocity(127);
        pros::delay(1800);
        chassis.moveToPoint(chassis.getPose().x+3, chassis.getPose().y+47, 1000, {.forwards = false, .maxSpeed = 70});
        chassis.waitUntilDone();
        chassis.moveToPoint(chassis.getPose().x-1, chassis.getPose().y+6, 400, {.forwards = false, .maxSpeed = 85});
        intake_hood_roller.move(127);
        intake_hood_roller.move(-127);
        intake_motor.move(-127);
        pros::delay(250);
        intake_motor.move(127);
        intake_hood_roller.move(127);
        pros::delay(2000);
        rTongue.set_value(false);
        
        chassis.moveToPoint(chassis.getPose().x-25, chassis.getPose().y-35, 800, {.maxSpeed = 95});
        chassis.waitUntilDone();
        chassis.turnToHeading(240, 500);
        chassis.waitUntilDone();
        chassis.moveToPoint(chassis.getPose().x-35, chassis.getPose().y-4, 3000, {.maxSpeed = 105, .minSpeed = 95});
        intake_hood_roller.move(127);
        pros::delay(350);
        rTongue.set_value(true);
        pros::delay(400);
        rTongue.set_value(false);
        chassis.waitUntilDone();
}