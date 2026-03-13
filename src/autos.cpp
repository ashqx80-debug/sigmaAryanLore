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
#include "autos.hpp"
#include "globals.hpp"

void left7(){
    chassis.setPose(-12,26,0);
    chassis.moveToPoint(-27, 50, 1200, {.maxSpeed=75});
    intake_motor.move(127);
    chassis.waitUntil(20);
    rTongue.set_value(true);
    chassis.waitUntilDone();
    chassis.turnToPoint(-52,20, 800);
    chassis.moveToPoint(-52, 20, 1200);
    chassis.waitUntil(16);
    intake_motor.move(0);
    chassis.turnToHeading(180, 800);
    chassis.moveToPoint(-50,42, 800, {.forwards = false});
    chassis.waitUntilDone();
    intake_motor.move(127);
    intake_hood_roller.move(127);
    hoodPiston.set_value(true); 
    pros::delay(1200);
    chassis.turnToHeading(180, 800);
    chassis.moveToPoint(-49, -2, 1000, {.minSpeed=75});
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
    intake_hood_roller.move(127);
    hoodPiston.set_value(true);
    pros::delay(750);
    chassis.moveToPoint(-48, 30, 200);
    hoodPiston.set_value(false);
    intake_hood_roller.move(0);
    intake_motor.move(0);
    chassis.moveToPoint(-48, 48, 500, {.minSpeed = 80});
}

void right7(){
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
    intake_hood_roller.move(127);
    hoodPiston.set_value(true);
    rTongue.set_value(true);
    pros::delay(700);
    chassis.turnToHeading(180, 800);
    chassis.moveToPoint(49, -4, 1200, {.maxSpeed=75});
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
    intake_hood_roller.move(127);
    hoodPiston.set_value(true);
    pros::delay(700);
    chassis.moveToPoint(48, 30, 300);
    hoodPiston.set_value(false);
    intake_hood_roller.move(0);
    intake_motor.move(0);
    chassis.moveToPoint(48, 48, 1000, {.forwards=false, .minSpeed = 80});
}

void midthreeplusfour(){
    //blue left (mid goal)
    chassis.setPose(-12,26,270);
    chassis.moveToPoint(-48.8, 26, 1000, {.maxSpeed=95});
    chassis.waitUntilDone();
    chassis.turnToHeading(180, 500);
    rTongue.set_value(true);
    chassis.moveToPoint(-48.5, -0.5, 950, {.maxSpeed = 65});
    intake_motor.move(127);
    left_drive.move_velocity(127);
    right_drive.move_velocity(127);
    pros::delay(750);
    chassis.moveToPoint(chassis.getPose().x+0.5, chassis.getPose().y + 38, 950, {.forwards = false, .maxSpeed=80});
    intake_hood_roller.move(127);
    chassis.waitUntil(36.5);
    left_drive.move_velocity(-20);
    right_drive.move_velocity(-20);
    rTongue.set_value(false);

    hoodPiston.set_value(true);
    pros::delay(1500);
    hoodPiston.set_value(false);
    intake_hood_roller.move(0);
    chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y-20, 700);
    chassis.waitUntilDone();
    chassis.turnToPoint(-15, 51, 500);
    chassis.moveToPoint(-15, 51, 1250, {.maxSpeed=85});
    chassis.waitUntilDone();
    rTongue.set_value(true);
    pros::delay(200);
    chassis.turnToHeading(-135, 800);
    chassis.waitUntilDone();
    pros::delay(300);
    chassis.moveToPoint(chassis.getPose().x+10, chassis.getPose().y+13, 900, {.forwards = false});
    chassis.waitUntilDone();
    midgoalPiston.set_value(true);
    intake_hood_roller.move(-97);
    intake_motor.move(100);
    pros::delay(1500);
    chassis.turnToPoint(chassis.getPose().x-26.7, chassis.getPose().y-20, 500);
    chassis.moveToPoint(chassis.getPose().x-26.7, chassis.getPose().y-20, 850);
    midgoalDescore.set_value(false);
    chassis.waitUntilDone();
    midgoalDescore.set_value(true);
    chassis.turnToHeading(180, 500);
    Snacky.set_value(false);
    chassis.moveToPoint(chassis.getPose().x+2, chassis.getPose().y+18, 1050, {.forwards = false});
    chassis.waitUntilDone();
}

void dheerarightfourballrush(){
    //red right (four ball)
    // elims right (4 ball descore) speed i need this
    chassis.setPose(12, 26, 90);
    Snacky.set_value(true);
    chassis.moveToPoint(49, 26,1200);
    chassis.waitUntilDone();
    chassis.turnToHeading(180,450);
    rTongue.set_value(true);
    chassis.moveToPoint(chassis.getPose().x, 0,900, {.maxSpeed=75});
    intake_motor.move(127);
    left_drive.move_velocity(127);
    right_drive.move_velocity(127);
    pros::delay(300);
    left_drive.move_velocity(0);
    right_drive.move_velocity(0);
    //  chassis.turnToPoint(47, 46,950, {.forwards = false});
    chassis.moveToPoint(chassis.getPose().x+1, 45,1600, {.forwards = false, .maxSpeed=85});
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
    chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y + 14, 2000, {.forwards = false});
    Snacky.set_value(false);
}

void dheeraleftfourballrush(){
    chassis.setPose(12, 26, 90);
    Snacky.set_value(true);
    intake_motor.move(127);
    
    chassis.moveToPoint(27, 52, 1250, {.maxSpeed=75});
    chassis.waitUntilDone();
    rTongue.set_value(true);
    chassis.moveToPoint(49, 26,1200);
    chassis.turnToHeading(180,950);
    
    chassis.moveToPoint(48, 0,1400, {.maxSpeed=65});
    intake_motor.move(127);
    left_drive.move_velocity(127);
    right_drive.move_velocity(127);
    pros::delay(300);
    left_drive.move_velocity(0);
    right_drive.move_velocity(0);
    chassis.turnToPoint(chassis.getPose().x, 46,950, {.forwards = false});
    chassis.moveToPoint(45.7, 46,950, {.forwards = false, .maxSpeed=85});
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
    chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y - 10, 500);
    rTongue.set_value(false);
    chassis.turnToHeading(90,500);
    chassis.moveToPoint(64.3,34,1000);
    chassis.waitUntilDone();
    chassis.turnToHeading(-180,500);
    Snacky.set_value(false);  
    chassis.moveToPoint(chassis.getPose().x+1, chassis.getPose().y + 12, 2000, {.forwards = false});
    
}

void sigSawp(){
            chassis.setPose(12, 26, 90);
            chassis.moveToPoint(48.5, 26,950);
            chassis.turnToHeading(180,450);
            rTongue.set_value(true);
            chassis.moveToPoint(chassis.getPose().x, 0,800, {.maxSpeed=68});

            intake_motor.move(127);
            pros::delay(600);
            chassis.moveToPoint(chassis.getPose().x+1.6, chassis.getPose().y + 48.6,850, {.forwards = false, .maxSpeed=74});
            chassis.turnToHeading(180, 400);
            intake_hood_roller.move(127);
            chassis.waitUntilDone();
            chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y+5, 500);
            hoodPiston.set_value(true);
            pros::delay(1000);
            intake_hood_roller.move(0);
            chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y - 10 ,900);
            intake_motor.move(127);
    chassis.moveToPoint(chassis.getPose().x, -0.5, 900, {.maxSpeed = 65});
    chassis.waitUntilDone();
    left_drive.move_velocity(127);
    right_drive.move_velocity(127);
    pros::delay(400);
    chassis.moveToPoint(chassis.getPose().x-0.5, chassis.getPose().y + 44.5, 780, {.forwards = false, .maxSpeed = 85});
    intake_hood_roller.move(127);
    chassis.waitUntilDone();
    hoodPiston.set_value(true);
    chassis.moveToPoint(chassis.getPose().x-2, chassis.getPose().y + 5, 450, {.forwards = false});
    pros::delay(800);
    rTongue.set_value(false);
    moveF(820, true, true, 100, 0, 350);
    rTongue.set_value(false);
            hoodPiston.set_value(false);
            chassis.moveToPoint(chassis.getPose().x-27, chassis.getPose().y +18,900);
            chassis.waitUntil(23);
            rTongue.set_value(true);
            pros::delay(200);
            rTongue.set_value(false);
            chassis.turnToHeading(270,500);
            chassis.moveToPoint(chassis.getPose().x-50, chassis.getPose().y,700, {.maxSpeed=85});
            chassis.waitUntil(53);
            rTongue.set_value(true);
            pros::delay(300);
            chassis.turnToHeading(210,350);
            chassis.moveToPoint(chassis.getPose().x+29, chassis.getPose().y+20, 800, {.forwards = false,.maxSpeed=65});
            pros::delay(400);
            midgoalPiston.set_value(true);
            intake_hood_roller.move(-100);
            left_drive.move_velocity(127);
            right_drive.move_velocity(127);
            pros::delay(1000);
            // midgoalPiston.set_value(false);
            // intake_hood_roller.move(0);
            // chassis.moveToPoint(chassis.getPose().x-33, chassis.getPose().y-33.5, 900, {.maxSpeed=90});
            // chassis.turnToHeading(180,450);
            // rTongue.set_value(true);
            // chassis.moveToPoint(chassis.getPose().x-0.5,chassis.getPose().y-25, 600, {.maxSpeed=70});
            // chassis.waitUntilDone();
            // pros::delay(600);
            // chassis.moveToPoint(chassis.getPose().x +0.75, chassis.getPose().y + 42,850, {.forwards = false, .maxSpeed=70});
            // intake_hood_roller.move(127);
            // chassis.waitUntilDone();
            // chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y+5, 500);
            //  hoodPiston.set_value(true);
; 

}
void nigrSawp(){
    chassis.setPose(12,26,90);
    moveF(700, true, true, 95, 0, 700);
    chassis.turnToHeading(180, 400);
    pros::delay(200);
    rTongue.set_value(true);
    chassis.waitUntilDone();


    intake_motor.move(127);
    chassis.moveToPoint(chassis.getPose().x, -0.5, 900, {.maxSpeed = 65});
    chassis.waitUntilDone();
    left_drive.move_velocity(127);
    right_drive.move_velocity(127);
    pros::delay(400);
    chassis.moveToPoint(chassis.getPose().x-0.5, chassis.getPose().y + 44.5, 780, {.forwards = false, .maxSpeed = 85});
    intake_hood_roller.move(127);
    chassis.waitUntilDone();
    hoodPiston.set_value(true);
    chassis.moveToPoint(chassis.getPose().x-2, chassis.getPose().y + 5, 450, {.forwards = false});
    pros::delay(800);
    rTongue.set_value(false);
    moveF(820, true, true, 100, 0, 350);
    rTongue.set_value(false);
            hoodPiston.set_value(false);
            chassis.moveToPoint(chassis.getPose().x-27, chassis.getPose().y +18,900);
            chassis.waitUntil(23);
            rTongue.set_value(true);
            pros::delay(200);
            rTongue.set_value(false);
            chassis.turnToHeading(270,500);
            chassis.moveToPoint(chassis.getPose().x-50, chassis.getPose().y,700, {.maxSpeed=85});
            chassis.waitUntil(53);
            rTongue.set_value(true);
            pros::delay(300);
            chassis.turnToHeading(210,350);
            chassis.moveToPoint(chassis.getPose().x+29, chassis.getPose().y+20, 800, {.forwards = false,.maxSpeed=65});
            pros::delay(400);
            midgoalPiston.set_value(true);
            intake_hood_roller.move(-100);
            left_drive.move_velocity(127);
            right_drive.move_velocity(127);
            pros::delay(1000);
            midgoalPiston.set_value(false);
            intake_hood_roller.move(0);
            chassis.moveToPoint(chassis.getPose().x-33, chassis.getPose().y-33.5, 900, {.maxSpeed=90});
            chassis.turnToHeading(180,450);
            rTongue.set_value(true);
            chassis.moveToPoint(chassis.getPose().x-0.5,chassis.getPose().y-25, 600, {.maxSpeed=70});
            chassis.waitUntilDone();
            pros::delay(600);
            chassis.moveToPoint(chassis.getPose().x +0.75, chassis.getPose().y + 42,850, {.forwards = false, .maxSpeed=70});
            intake_hood_roller.move(127);
            chassis.waitUntilDone();
            chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y+5, 500);
             hoodPiston.set_value(true);

}
