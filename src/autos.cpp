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
    chassis.moveToPoint(-48.5, 26, 1000, {.maxSpeed=95});
    chassis.waitUntilDone();
    chassis.turnToHeading(180, 500);
    rTongue.set_value(true);
    chassis.moveToPoint(-47.2   , -2, 850, {.maxSpeed = 75});
    intake_motor.move(127);
    left_drive.move_velocity(127);
    right_drive.move_velocity(127);
    pros::delay(750);
    chassis.moveToPoint(chassis.getPose().x+1.5, chassis.getPose().y + 32, 850, {.forwards = false, .maxSpeed=75});
    chassis.waitUntilDone();
    left_drive.move_velocity(-20);
    right_drive.move_velocity(-20);
    rTongue.set_value(false);
    intake_hood_roller.move(90);
    hoodPiston.set_value(true);
    pros::delay(1000);
    hoodPiston.set_value(false);
    intake_hood_roller.move(0);
    chassis.moveToPoint(chassis.getPose().x+3, chassis.getPose().y-15, 700);
    chassis.turnToPoint(-13, 57, 300);
    chassis.moveToPoint(-13, 57, 850, {.maxSpeed=85});
    chassis.waitUntilDone();
    rTongue.set_value(true);
    pros::delay(200);
    chassis.turnToPoint(-52, 28, 700);
    chassis.waitUntilDone();
    pros::delay(1000);
    chassis.moveToPoint(chassis.getPose().x+6.8, chassis.getPose().y+6.5, 900, {.forwards = false});
    chassis.waitUntilDone();
    intake_motor.move(-70);
    midgoalPiston.set_value(true);
    intake_hood_roller.move(-97);
    pros::delay(250);
    intake_motor.move(100);
    pros::delay(1000);
    chassis.turnToPoint(chassis.getPose().x-27.5, chassis.getPose().y-20, 500);
    chassis.moveToPoint(chassis.getPose().x-27.5, chassis.getPose().y-20, 850);
    chassis.waitUntilDone();
    chassis.turnToHeading(180, 500);
    Snacky.set_value(false);
    chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y+18, 1050, {.forwards = false});
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
    //right sig swap
    chassis.setPose(12,26,90);
    chassis.moveToPoint(50, 26, 800);
    chassis.waitUntilDone(); //checkpoint 1

    chassis.turnToHeading(180, 350);
    chassis.waitUntilDone();

    intake_motor.move(127);
    rTongue.set_value(true);

    chassis.moveToPoint(47.5, 0, 900, {.maxSpeed = 75}); //checkpoint 2
    left_drive.move_velocity(90);
    right_drive.move_velocity(90); // matchload
    pros::delay(400);
    chassis.turnToHeading(180, 400);
    chassis.moveToPoint(chassis.getPose().x-2.5, chassis.getPose().y + 40, 870, {.forwards = false, .maxSpeed = 75});
    rTongue.set_value(false); 
    //checkpoint 3
    chassis.waitUntil(38);
    intake_hood_roller.move(110);
    intake_motor.move(110);
    left_drive.move_velocity(-20);
    right_drive.move_velocity(-20);
    hoodPiston.set_value(true);
    pros::delay(400);
    left_drive.move_velocity(0);
    right_drive.move_velocity(0);  
    //scores blocks
    pros::delay(500);
    hoodPiston.set_value(false);
    intake_hood_roller.move(0);

    intake_motor.move(127);
    chassis.moveToPoint(41, 24, 450); //checkpoint 4
    chassis.waitUntilDone();
    intake_motor.move(127);
    chassis.turnToPoint(25, 51, 200);
    chassis.moveToPoint(25, 51, 700, {.maxSpeed=90});
    chassis.waitUntilDone(); //checkpoint 5
    rTongue.set_value(false);
    chassis.moveToPoint(chassis.getPose().x-63.8, chassis.getPose().y+1.35, 1250, {.maxSpeed=85});
    chassis.waitUntilDone();
    rTongue.set_value(true); //checkpoint 6
    intake_motor.move(80);

    chassis.turnToPoint(chassis.getPose().x+8, chassis.getPose().y+7, 350, {.forwards = false});
    chassis.moveToPoint(chassis.getPose().x+6.5, chassis.getPose().y+7, 650, {.forwards = false});
    chassis.waitUntilDone(); //scores midgoal, checkpoint 7
    midgoalPiston.set_value(true);
    left_drive.move_velocity(-40);
    right_drive.move_velocity(-40);
    intake_hood_roller.move(-85);

    intake_motor.move(-100);
    pros::delay(125);
    left_drive.move_velocity(0);
    right_drive.move_velocity(0);
    intake_motor.move(100);

    pros::delay(250);

    intake_motor.move(-80);
    pros::delay(125);
    intake_motor.move(127);

    pros::delay(1200);

    intake_motor.move(0);
    intake_hood_roller.move(0);
    midgoalPiston.set_value(false);
    chassis.turnToPoint(chassis.getPose().x-45, chassis.getPose().y - 52, 100);
    chassis.moveToPoint(chassis.getPose().x-45, chassis.getPose().y - 52, 1000); 
    chassis.waitUntilDone(); //checkpoint 8
    chassis.turnToHeading(180, 200);
    chassis.moveToPoint(chassis.getPose().x-1, chassis.getPose().y -40, 680, {.maxSpeed=75});
    chassis.waitUntilDone(); //checkpoint 9
    intake_motor.move(127);

    left_drive.move_velocity(90);
    right_drive.move_velocity(90);
    pros::delay(500);
    chassis.moveToPoint(chassis.getPose().x-1, chassis.getPose().y +39, 750, {.forwards = false, .maxSpeed=85});
    chassis.waitUntilDone(); //checkpoint 10
    hoodPiston.set_value(true);
    intake_hood_roller.move(127);
    intake_motor.move(127);
    left_drive.move_velocity(-30);
    right_drive.move_velocity(-30);
    Snacky.set_value(false);
    pros::delay(1000);
}

void unjamTest(){
    intakeSpin = true;
    intake_motor.move(127);
    intake_hood_roller.move(127);
    pros::delay(2000);
    
}