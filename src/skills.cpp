#include "globals.hpp"
#include "pros/adi.hpp"
#include "lemlib/api.hpp"
#include "main.h"
#include "pros/distance.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include <numeric>
#include <cmath>
#include <iostream>
#include "distance.h"
#include "skills.hpp"


void skillsProg(){
    // -------- SKILLS --------

        chassis.setPose(-12, 26, -90);
        moveF(687, true, true, 95, 0, 1000);
        chassis.turnToHeading(180, 500);
        pros::delay(200);
        rTongue.set_value(true);
        intake_motor.move(127);
        chassis.waitUntilDone();
        chassis.moveToPoint(chassis.getPose().x, 1.8, 900, {.maxSpeed = 60});
        chassis.waitUntilDone();
        left_drive.move_velocity(50);
        right_drive.move_velocity(50);
        pros::delay(1700);

        chassis.moveToPoint(chassis.getPose().x-10.6, 42, 1000, {.forwards = false, .maxSpeed = 90});
        chassis.waitUntilDone();

        chassis.waitUntilDone();
        chassis.turnToHeading(180, 500);
        chassis.waitUntilDone();
        chassis.waitUntilDone();
        rTongue.set_value(false);
        intake_motor.move(0);
        moveB(780, false, true, 86, 0, 1400);
        chassis.turnToHeading(90, 600);
        chassis.waitUntilDone();
        moveB(477, true, false, 72, 0, 600);
       chassis.turnToHeading(0, 500);
       chassis.waitUntilDone();
       intake_hood_roller.move(90);
       moveF(1155, false, false, 75, 64, 750);
       intake_motor.move(127);
       hoodPiston.set_value(true);

       chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y-5, 350, {.forwards = false});
       chassis.turnToHeading(0, 500);
       chassis.waitUntilDone();
       

        rTongue.set_value(true);
        intake_hood_roller.move(-127);
        intake_motor.move(-127);
        pros::delay(250);
        intake_motor.move(127);
        intake_hood_roller.move(80);
        pros::delay(1800);

        intake_hood_roller.move(0);
        hoodPiston.set_value(false);
        
        chassis.moveToPoint(chassis.getPose().x-1.3, chassis.getPose().y+36.7, 1300, {.maxSpeed = 60});
        chassis.waitUntilDone();
        left_drive.move_velocity(30);
        right_drive.move_velocity(30);
        pros::delay(1800);
        left_drive.move_velocity(0);
        right_drive.move_velocity(0);
        chassis.turnToHeading(0, 650);
        chassis.waitUntilDone();
        chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y-44.3, 1100, {.forwards = false, .maxSpeed = 80});
        intake_hood_roller.move(100);
        chassis.waitUntilDone();

        hoodPiston.set_value(true);
        chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y-5, 500, {.forwards = false});
        chassis.waitUntilDone();
        chassis.turnToHeading(0,500);
        chassis.waitUntilDone();
        intake_hood_roller.move(-127);
        intake_motor.move(-127);
        pros::delay(150);
        intake_motor.move(127);
        hoodPiston.set_value(true);
        intake_hood_roller.move(80);
        pros::delay(2200);
        rTongue.set_value(false);
        hoodPiston.set_value(false);
        chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y+7, 600);
        chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y-5.5, 600,{.forwards=false, .maxSpeed=60});
        chassis.waitUntilDone();
        chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y+8, 900, {.maxSpeed = 95});
        chassis.waitUntilDone();
        chassis.turnToHeading(91, 500);
        hoodPiston.set_value(false);
        intake_hood_roller.move(0);
        chassis.waitUntilDone();
        moveB(2000, true, false, 95, 0, 900);
        moveF(690, true, true, 68, 0, 1200);
        chassis.waitUntilDone();

        chassis.turnToHeading(0, 650);
        pros::delay(200);
        rTongue.set_value(true);
        chassis.waitUntilDone();

       chassis.moveToPoint(chassis.getPose().x+0.3, chassis.getPose().y+19.6, 800, { .maxSpeed=65});

        chassis.waitUntilDone();
        left_drive.move_velocity(70);
        right_drive.move_velocity(70);
        pros::delay(1900);
        chassis.turnToPoint(chassis.getPose().x+15, chassis.getPose().y-42, 500,{.forwards=false});
        chassis.moveToPoint(chassis.getPose().x+15,  chassis.getPose().y-42, 1000, {.forwards=false, .maxSpeed=85});
        rTongue.set_value(false);
        chassis.waitUntilDone();
        chassis.turnToHeading(0, 500);
        chassis.waitUntilDone();
        moveB(790, false, true, 85, 0, 1200);
        chassis.turnToHeading(-90, 500);
        chassis.waitUntilDone();
        moveB(501, true, false, 85, 0, 600);
        chassis.turnToHeading(180, 500);
        chassis.waitUntilDone();
        intake_hood_roller.move(100);
        moveF(1131, false, false, 75, 0, 700);
        hoodPiston.set_value(true);
        rTongue.set_value(true);
        chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y+6, 400, {.forwards = false, .maxSpeed = 85});
        intake_hood_roller.move(-127);
        intake_motor.move(-127);
        pros::delay(250);
        intake_motor.move(127);
        intake_hood_roller.move(90);
        pros::delay(2000);

        intake_hood_roller.move(0);
        hoodPiston.set_value(false);
        chassis.moveToPoint(chassis.getPose().x-0.5, chassis.getPose().y-46, 1300, {.maxSpeed = 63});
        chassis.waitUntilDone();
        left_drive.move_velocity(60);
        right_drive.move_velocity(60);
        pros::delay(1800);
        chassis.moveToPoint(chassis.getPose().x+0.5, chassis.getPose().y+45.6, 1000, {.forwards = false, .maxSpeed = 75});
        intake_hood_roller.move(100);
        chassis.waitUntilDone();
        hoodPiston.set_value(true);
        chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y+6, 500, {.forwards = false});
        intake_hood_roller.move(-127);
        intake_motor.move(-127);
        pros::delay(250);
        intake_motor.move(127);
        intake_hood_roller.move(80);
        pros::delay(2200);
        rTongue.set_value(false);
        hoodPiston.set_value(false);
        chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y-8, 600);
        chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y+8, 600,{.forwards=false, .maxSpeed=80});
        chassis.waitUntilDone();
        chassis.moveToPoint(chassis.getPose().x-25, chassis.getPose().y-35, 800, {.maxSpeed = 95});
        chassis.waitUntilDone();
        chassis.turnToHeading(240, 500);
        chassis.waitUntilDone();
        chassis.moveToPoint(chassis.getPose().x-40, chassis.getPose().y-4, 3000, {.maxSpeed = 105, .minSpeed = 100});
        intake_hood_roller.move(127);
        pros::delay(470);
        rTongue.set_value(true);
        hoodPiston.set_value(true);
        pros::delay(700);
        rTongue.set_value(false);
        chassis.waitUntilDone();
        chassis.moveToPoint(chassis.getPose().x+7, chassis.getPose().y-5, 1000, {.forwards=false,.maxSpeed = 95});
}