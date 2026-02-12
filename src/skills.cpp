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

        chassis.setPose(12, 26, 90);

        chassis.moveToPoint(48.5, 26, 1500);
        chassis.waitUntilDone();
        pros::delay(500);

        chassis.turnToHeading(180, 500);
        chassis.waitUntilDone();

        intakeSpin=true;
        intake_motor.move(127);
        rTongue.set_value(true);

        chassis.moveToPoint(47.8, 0.5, 500, {.maxSpeed = 70});
        chassis.waitUntilDone();

        left_drive.move_velocity(127);
        right_drive.move_velocity(127);
        pros::delay(1700);


        chassis.turnToPoint(57.5, 45, 500, {.forwards = false});
        chassis.waitUntilDone();
        rTongue.set_value(false);

        chassis.moveToPoint(57.5, 45, 1500, {.forwards = false, .maxSpeed = 65});
        chassis.waitUntilDone();
        chassis.turnToHeading(180, 500);
        
        chassis.moveToPoint(chassis.getPose().x+3.5, chassis.getPose().y+65, 2000, {.forwards = false,.maxSpeed = 75});
        chassis.waitUntilDone();
        chassis.turnToHeading(270, 700);
        chassis.moveToPoint(chassis.getPose().x-15.6, chassis.getPose().y, 900);
        chassis.waitUntilDone();
        chassis.turnToHeading(0, 500);
        chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y -10, 960, {.forwards = false});
        left_drive.move_velocity(-45);
        right_drive.move_velocity(-30);
        chassis.waitUntilDone();
        left_drive.move_velocity(-50);
        intake_hood_roller.move(100);
        hoodPiston.set_value(true);
        // Manual unjam sequence removed
        right_drive.move_velocity(0);
        left_drive.move_velocity(-10);
    
        enc_vertical.reset_position();
        pros::delay(500);
        imu.reset();
        pros::delay(2000);

        chassis.setPose(48,102, 0);

        rTongue.set_value(true);
        hoodPiston.set_value(false);
        intake_hood_roller.move(0);

        chassis.moveToPoint(48.5, 148, 900, {.maxSpeed = 75});
        chassis.waitUntilDone();

        left_drive.move_velocity(127);
        right_drive.move_velocity(127);
        pros::delay(1600);

        chassis.moveToPoint(48, 102, 1000, {.forwards = false});
        chassis.waitUntilDone();
        rTongue.set_value(false);

        intake_hood_roller.move(127);
        hoodPiston.set_value(true);
        chassis.moveToPoint(48, 120, 750);
        chassis.waitUntilDone();
        chassis.turnToHeading(270, 500);
        chassis.moveToPoint(chassis.getPose().x-100, chassis.getPose().y, 3000, {.maxSpeed = 95});
        chassis.waitUntilDone();
        chassis.turnToHeading(0, 500);
        rTongue.set_value(true);
        chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y + 14, 960, {.maxSpeed = 75});
        pros::delay(1700);
        chassis.turnToPoint(chassis.getPose().x-11, chassis.getPose().y - 45, 750, {.forwards = false});
        chassis.moveToPoint(chassis.getPose().x-11, chassis.getPose().y - 45, 750, {.forwards = false});
        chassis.waitUntilDone();
        rTongue.set_value(false);
        chassis.turnToHeading(0, 500);
        chassis.moveToPoint(chassis.getPose().x-2.5, chassis.getPose().y-65, 2000, {.forwards = false, .maxSpeed = 75});
        chassis.waitUntilDone();
        chassis.turnToHeading(90, 500);
        chassis.moveToPoint(chassis.getPose().x+15, chassis.getPose().y , 750);
        chassis.waitUntilDone();
        chassis.turnToHeading(180, 500);
        chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y -10, 960, {.forwards = false});
        left_drive.move_velocity(-30);
        right_drive.move_velocity(-30);
        chassis.waitUntilDone();
        left_drive.move_velocity(-43);

        intake_hood_roller.move(100);
        hoodPiston.set_value(true);
        right_drive.move_velocity(0);
    
        enc_vertical.reset_position();
        pros::delay(2500);
        chassis.setPose( -48,42, 180);
        chassis.moveToPoint(-48.5, 0, 900, {.maxSpeed = 75});
        rTongue.set_value(true);
        chassis.waitUntilDone();
        left_drive.move_velocity(127);
        right_drive.move_velocity(127);
        pros::delay(1600);
        chassis.moveToPoint(-48, 42, 1000, {.forwards = false});
        chassis.waitUntilDone();
        rTongue.set_value(false);
        intake_hood_roller.move(107);
        hoodPiston.set_value(true);
        pros::delay(1500);
        chassis.moveToPoint(-48, 24, 750);
        chassis.waitUntilDone();
        chassis.turnToPoint(-38, 12, 750);
        chassis.moveToPoint(-38, 12, 1000, {.maxSpeed = 95});
        chassis.waitUntilDone();
        chassis.turnToHeading(90, 500);
        chassis.moveToPoint(12, 12, 2000, {.maxSpeed = 85});
        chassis.waitUntilDone();
}