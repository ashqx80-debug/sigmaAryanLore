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
    chassis.setPose(-15,25,270);
    chassis.moveToPoint(-49.5, 26, 1200, {.maxSpeed=95});
    chassis.waitUntilDone();
    chassis.turnToHeading(180, 500);
    rTongue.set_value(true);
    chassis.moveToPoint(-50.5, -1.5, 1050, { .maxSpeed = 65, .minSpeed = 50});
    intake_motor.move(127);
    left_drive.move_velocity(127);
    right_drive.move_velocity(127);
    pros::delay(850);
    chassis.moveToPoint(chassis.getPose().x-0.5, chassis.getPose().y + 38, 1050, {.forwards = false, .maxSpeed=85});
    chassis.waitUntil(36.5);
    left_drive.move_velocity(-30);
    right_drive.move_velocity(-30);
    rTongue.set_value(false);
    intake_hood_roller.move(127);
    hoodPiston.set_value(true);
    pros::delay(900);
  
    intake_hood_roller.move(0);
    chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y-24, 700);
    chassis.waitUntilDone();
    hoodPiston.set_value(false);
    chassis.turnToPoint(-15, 51, 500);
    chassis.moveToPoint(-15, 51, 1250, {.maxSpeed=85});
    chassis.waitUntil(24);
    rTongue.set_value(true);
    pros::delay(200);
    chassis.turnToHeading(-135, 800);
    chassis.waitUntilDone();
    pros::delay(1250);
    chassis.moveToPoint(chassis.getPose().x+16.5, chassis.getPose().y+13.5, 900, {.forwards = false});
    chassis.waitUntil(12);
    left_drive.move_velocity(-25);
    right_drive.move_velocity(-25);
    chassis.waitUntilDone();
    chassis.turnToHeading(-135, 300);
    
    midgoalPiston.set_value(true);
    intake_hood_roller.move(-97);
    intake_motor.move(-100);
    pros::delay(250);
    left_drive.move_velocity(0);
    right_drive.move_velocity(0);
    intake_motor.move(127);
    pros::delay(300);
    intake_motor.move(-127);
    pros::delay(250);
    intake_motor.move(127);
    pros::delay(1200);
    chassis.turnToPoint(chassis.getPose().x-27, chassis.getPose().y-22, 500);
    chassis.moveToPoint(chassis.getPose().x-27, chassis.getPose().y-22, 850);
    chassis.waitUntilDone();
    intake_hood_roller.move(0);
    chassis.turnToHeading(180, 500);
    Snacky.set_value(false);
    chassis.moveToPoint(chassis.getPose().x+0.5, chassis.getPose().y+16.3, 1050, {.forwards = false});
    chassis.waitUntilDone();
    chassis.turnToHeading(230, 500);
    chassis.turnToHeading(180, 500);
}

void dheerarightfourballrush(){
    //red right (four ball)
    // elims right (4 ball descore) speed i need this
    chassis.setPose(12, 26, 90);
    Snacky.set_value(true);
    chassis.moveToPoint(49, 26,1200, {.maxSpeed = 85});
    chassis.waitUntilDone();
    chassis.turnToHeading(180,450);
    rTongue.set_value(true);
    intake_motor.move(127);
    chassis.moveToPoint(chassis.getPose().x+0.1, -2,950, {.maxSpeed=65});
    left_drive.move_velocity(127);
    right_drive.move_velocity(127);
    pros::delay(750);
    left_drive.move_velocity(0);
    right_drive.move_velocity(0);
    //  chassis.turnToPoint(47, 46,950, {.forwards = false});
    chassis.moveToPoint(chassis.getPose().x+1, 42,1600, {.forwards = false, .maxSpeed=85});
    chassis.waitUntilDone();
    intake_hood_roller.move(127);
    hoodPiston.set_value(true);
    rTongue.set_value(false);
    pros::delay(1500);

    intake_hood_roller.move(0);
    intake_motor.move(0);
    chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y - 10, 500);
    hoodPiston.set_value(false);
    rTongue.set_value(false);
    chassis.turnToHeading(90,500);
    chassis.moveToPoint(64.2,40,1000);
    chassis.waitUntilDone();
    chassis.turnToHeading(-180,500);
    chassis.moveToPoint(chassis.getPose().x-3.5, chassis.getPose().y + 14, 2000, {.forwards = false});
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
    chassis.moveToPoint(48.5, 26, 800);
    chassis.waitUntilDone(); //checkpoint 1

    chassis.turnToHeading(180, 350);
    chassis.waitUntilDone();

    intake_motor.move(127);
    rTongue.set_value(true);

    chassis.moveToPoint(47.4, -2, 900, {.maxSpeed = 75}); //checkpoint 2
    left_drive.move_velocity(127);
    right_drive.move_velocity(127); // matchload
    pros::delay(500);
    chassis.turnToHeading(180, 400);
    chassis.moveToPoint(chassis.getPose().x-2.5, chassis.getPose().y + 37.5, 870, {.forwards = false, .maxSpeed = 85});

    //checkpoint 3
    chassis.waitUntil(36.5);
    rTongue.set_value(false); 
    left_drive.move_velocity(-10);
    right_drive.move_velocity(-10);
    intake_hood_roller.move(127);
    intake_motor.move(127);

    hoodPiston.set_value(true);
    pros::delay(400);
    left_drive.move_velocity(0);
    right_drive.move_velocity(0);  
    //scores blocks
    pros::delay(600);

    intake_hood_roller.move(0);


    chassis.moveToPoint(41, 24, 450);
    chassis.waitUntil(8);
    hoodPiston.set_value(false); //checkpoint 4
    intake_motor.move(127);
    chassis.turnToPoint(25, 51, 200);
    chassis.moveToPoint(25, 51, 700, {.maxSpeed=90});
    chassis.waitUntilDone(); //checkpoint 5
    rTongue.set_value(false);
    chassis.moveToPoint(chassis.getPose().x-64.7, chassis.getPose().y+1.35, 1200, {.maxSpeed=95});
    pros::delay(200);
    rTongue.set_value(true);
    pros::delay(600);
    rTongue.set_value(false);
    pros::delay(200);
    rTongue.set_value(true);
    chassis.waitUntilDone(); //checkpoint 6
    intake_motor.move(90);

    chassis.turnToPoint(chassis.getPose().x+6.4, chassis.getPose().y+7.5, 500, {.forwards = false});
    chassis.moveToPoint(chassis.getPose().x+6.4, chassis.getPose().y+7.5, 750, {.forwards = false});
    chassis.turnToHeading(-135, 350);
    chassis.waitUntilDone(); //scores midgoal, checkpoint 7
    midgoalPiston.set_value(true);
    left_drive.move_velocity(-20);
    right_drive.move_velocity(-25);
    intake_hood_roller.move(-95);

    left_drive.move_velocity(0);
    right_drive.move_velocity(0);
    intake_motor.move(-127);
    pros::delay(200);
    intake_motor.move(127);

    pros::delay(750);

    intake_motor.move(0);
    intake_hood_roller.move(0);
    midgoalPiston.set_value(false);
    chassis.turnToPoint(chassis.getPose().x-34.9, 23 , 500);
    chassis.moveToPoint(chassis.getPose().x-34.9, 23, 1100); 
    chassis.waitUntilDone(); //checkpoint 8
    chassis.turnToHeading(180, 350);
    chassis.moveToPoint(chassis.getPose().x-1, chassis.getPose().y -32, 680, {.maxSpeed=75});
    chassis.waitUntilDone(); //checkpoint 9
    intake_motor.move(127);

    left_drive.move_velocity(127);
    right_drive.move_velocity(127);
    pros::delay(500);
    chassis.moveToPoint(chassis.getPose().x-1, chassis.getPose().y +38.5, 950, {.forwards = false, .maxSpeed=85});
    chassis.waitUntilDone(); //checkpoint 10
    hoodPiston.set_value(true);
    intake_hood_roller.move(127);
    intake_motor.move(127);
    Snacky.set_value(false);
    pros::delay(2000);
}

void right7push(){
    chassis.setPose(12, 26, 90);
                 Snacky.set_value(true);
                 intake_motor.move(127);
                 
                 chassis.moveToPoint(27, 53, 1250, {.maxSpeed=75});
                 chassis.waitUntil(25);
                 rTongue.set_value(true);
                 chassis.moveToPoint(49, 26,1200);
                 chassis.turnToHeading(180,950);
                 
                 chassis.moveToPoint(48, -2,1400, {.maxSpeed=70});
                 intake_motor.move(127);
                 left_drive.move_velocity(127);
                 right_drive.move_velocity(127);
                 pros::delay(300);
                 left_drive.move_velocity(0);
                 right_drive.move_velocity(0);
                 chassis.turnToPoint(chassis.getPose().x, 46,950, {.forwards = false});
                 chassis.moveToPoint(45.7, 48,950, {.forwards = false, .maxSpeed=5});
                 chassis.waitUntilDone();
                 pros::delay(250);
                 intake_hood_roller.move(100);
                 hoodPiston.set_value(true);
                 intake_motor.move(-127);
                 pros::delay(250);
                 intake_motor.move(127);
                 pros::delay(1500);
                 rTongue.set_value(false);
                 hoodPiston.set_value(false);
                 intake_hood_roller.move(0);
                 intake_motor.move(0);
                 chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y - 10, 500);
                 rTongue.set_value(false);
                 chassis.turnToHeading(90,500);
                 chassis.moveToPoint(64.3,38,1000);
                 chassis.waitUntilDone();
                 chassis.turnToHeading(-180,500);
                 Snacky.set_value(false);  
                 chassis.moveToPoint(chassis.getPose().x-2, chassis.getPose().y + 12, 2000, {.forwards = false});
}


void new4plus3(){
    chassis.setPose(-12,25,-90);
    chassis.moveToPoint(-48, 26, 1100);
    chassis.turnToHeading(-180, 500);
    rTongue.set_value(true);
    intake_motor.move(127);
    chassis.moveToPoint(-47.5, -2, 800, {.maxSpeed=70,.minSpeed=60}); //matchload
    chassis.waitUntilDone();
    chassis.turnToHeading(-180, 200);
    pros::delay(700);
    //pros::delay(50000);

    chassis.moveToPoint(-47, 42, 1200, {.forwards=false, .maxSpeed=70});
    chassis.waitUntilDone();
    hoodPiston.set_value(true);
    intake_hood_roller.move(127);
    rTongue.set_value(false);
    //chassis.waitUntilDone();
    //chassis.setPose(-48,24+17,imu.get_heading());
    pros::delay(600);
    chassis.moveToPoint(-48, 16, 2500, {.forwards=true});
    //pros::delay(5000000);
    chassis.waitUntil(10);
    intake_hood_roller.move(0);
    hoodPiston.set_value(false);
    chassis.waitUntilDone();
    chassis.turnToPoint(-22,48,1000);
    // pros::delay(50000);
    chassis.moveToPoint(-22, 48, 1000);
    chassis.waitUntil(27);
    rTongue.set_value(true);
    chassis.turnToPoint(-4, 66, 500, {.forwards = false});
    chassis.moveToPoint(-4, 66, 1000 , {.forwards = false, .maxSpeed=60});
    chassis.waitUntilDone();
    // left_drive.move(-30);
    // right_drive.move(-30);
    intake_motor.move(-127);
    intake_hood_roller.move(-127);
    pros::delay(250);
    midgoalPiston.set_value(true);
    intake_motor.move(127);
    intake_hood_roller.move(-70);
    pros::delay(500);
    rTongue.set_value(false);
    pros::delay(100000);
    // chassis.moveToPoint(-48+10, 16+10, 500);
    // chassis.turnToHeading(-180, 300);
    // left_drive.move_velocity(-90);
    // right_drive.move_velocity(-90);
    // pros::delay(500);
    // right_drive.move_velocity(0);
    // left_drive.move_velocity(0);
  
}


void lateralTest(){
    chassis.setPose(0,0,0);
    chassis.moveToPoint(0,24,2000);
    pros::delay(2000);
    chassis.moveToPoint(0, 48, 2000);
    pros::delay(2000);
    chassis.moveToPoint(0, 0, 2000, {.forwards = false});
}