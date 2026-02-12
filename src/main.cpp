// #include "pros/llemu.hpp"
// #define MCL_OVERRIDE_CHASSIS 0
// #define ENABLE_MCL 0

#include "main.h"
#include "autos.hpp"
#include "lemlib/api.hpp"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include <numeric>
#include <cmath>
#include <iostream>
#include "distance.h"
#include "globals.hpp"
#include "skills.hpp"
#include "helpers.hpp"




// #include <vector>
// #include <random>
// #include <cmath>
// #include <algorithm>

// Hardware definitions moved to src/hardware.cpp


bool unjamEnabled = false;
bool intakeSpin = false;
int intakeSpeed = 127;

void intakeCtrlTask(void *) {
    while (true) {
        if (unjamEnabled && intakeSpin) {
            // If motor should be spinning but actual velocity is ~0 → jammed
            if (std::abs(intake_motor.get_actual_velocity()) < 25) {
                // Reverse briefly to unjam
                intake_motor.move(-127);
                pros::delay(250);
                // Resume forward
                intake_motor.move(intakeSpeed);
                pros::delay(100);
            }
        }
        if (!unjamEnabled){
            
        }
        pros::delay(5);
    }
}

// mech and helper implementations moved to src/helpers.cpp
void initialize()
{
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    pros::lcd::initialize();
    chassis.calibrate();
    pros::delay(1000);

    // Start intake-unjam monitor AFTER calibration so it never blocks init
    pros::Task intakeTask(intakeCtrlTask, nullptr, "intakeCtrl");

    // if (ENABLE_MCL)
    // {
    //     mclInit(0, 0, 0);
    //     pros::Task(mclTask, nullptr);
    // }

   //pros::Task(mechTask, nullptr);
}

double expo(double normalizedInput) {
    normalizedInput = pow(normalizedInput, 3);
    return (normalizedInput / 15000);
}

void autonomous()
{
    // Enable unjam detection for all autonomous routines
    unjamEnabled = true;
    
    /*
    1 - left safe | working
    2 - right safe | working
    3 - mid goal (left) | in dev
    4 - 4 ball rush (left) | not made
    5 - 4 ball rush (right) | not made
    6 - sig sawp | not made
    7 - skills | not made
    */
    int autonSelector = 8;
    // chassis.setPose(estX, estY, estH * 180 / M_PI);
    Snacky.set_value(true);
    switch (autonSelector){    
        case 1:
        left7();
        break;

        case 2:
        right7();
        break;

        case 3:
        midthreeplusfour();
        break;

        case 4:
        dheerarightfourballrush();
        break;

        case 5:
        dheeraleftfourballrush();
        break;

        case 6:
        sigSawp();
        break;

        case 7:
        skillsProg();
        break;

        case 8:
        unjamTest();
        break;
    }
};


void opcontrol() {
    // Reset unjam state from autonomous
    unjamEnabled = false;
    intakeSpin = false;
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

    while (true) {



        int leftY  = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / 1.1;

        int driveForward = expo(leftY);
        int driveTurn    = expo(rightX);

        left_drive.move(driveForward + driveTurn);
        right_drive.move(driveForward - driveTurn);


        

        // --- Manual intake control (only when unjam not enabled) ---
        if (!unjamEnabled) {
            if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
                intake_motor.move(127);
                intake_hood_roller.move(127);
            }
            else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
                intake_motor.move(127);
            }
            else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
                intake_motor.move(-127);
                intake_hood_roller.move(-127);
            }
            else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
                intake_motor.move(90);
                intake_hood_roller.move(-90);
            }
            else {
                intake_motor.move(0);
                intake_hood_roller.move(0);
            }
        }
        hoodPiston.set_value(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2));
        midgoalPiston.set_value(master.get_digital(pros::E_CONTROLLER_DIGITAL_A));
        // else if (!isSkills && master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
        //     intakePower = 90;
        //     hoodRollerPower = 90;
        //     intakeRunning = false;
        //     midgoalActive = true;
        
        // }
        
        //     intakePower = 0;
        //     hoodRollerPower = 0;
        //     midgoalActive = false;
        // }
       




        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            tong = !tong;
            rTongue.set_value(tong);
        }

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
            snack = !snack;
            Snacky.set_value(snack);
        }

        //pros::lcd::print(0, "X %.1f Y %.1f H %.1f", estX, estY, estH * 180 / M_PI);
        pros::lcd::print(1, "X %.2lf Y %.2lf",chassis.getPose().x, chassis.getPose().y);
        pros::lcd::print(2, "H %.2lf", imu.get_heading());




        pros::delay(20); //niniga 
    }
}