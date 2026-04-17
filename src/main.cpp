#include "main.h"
#include "autos.hpp"
#include "lemlib/api.hpp"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include <numeric>
#include <cmath>
#include <iostream>
#include "distance.h"
#include "globals.hpp"
#include "skills.hpp"
#include "helpers.hpp"

bool sus  = false;
bool tong = false;
bool snack = false;

void intakeCtrlTask(void *) {
    while (true) {
        if (unjamEnabled && intakeSpin) {
            if (std::abs(intake_motor.get_actual_velocity()) < 25) {
                intake_motor.move(-127);
                pros::delay(250);
                intake_motor.move(127);
                pros::delay(50);
            }
        }
        pros::delay(5);
    }
}

void initialize() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    pros::lcd::initialize();
    chassis.calibrate();
    pros::delay(1000);
    drawUI();
}

void autonomous() {
    Snacky.set_value(true);
    runSelectedAuton();
}

void opcontrol() {
    currentMode = DRIVER;
    markDirty();
    bool dih = false;

    while (true) {
        updateMode();
        handleTouch();
        handleDiagnostics();
        handleGame();
        drawUI();

        int leftY  = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) * 2;
        int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / 1.27;

        if (currentMode == DRIVER) {
            chassis.arcade(leftY, rightX, true, 0.7);
        } else {
            chassis.arcade(0, 0);
        }

        if (currentMode == DRIVER) {

            if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
                if(dih){
                    intake_motor.move(-127);
                    intake_hood_roller.move(-127);
                    pros::delay(250);
                    dih = false;
                }
                intake_motor.move(127);
                intake_hood_roller.move(127);
            }
            else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
                intake_motor.move(127);
                intake_hood_roller.move(127);
            }
            else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
                intake_motor.move(-127);
                intake_hood_roller.move(-127);
            }
            else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
                intake_motor.move(127);
                intake_hood_roller.move(127);
            }
            else {
                intake_motor.move(0);
                intake_hood_roller.move(0);
            }

            midgoalPiston.set_value(master.get_digital(pros::E_CONTROLLER_DIGITAL_A));

            if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
                sus = !sus;
                midgoalDescore.set_value(sus);
            }

            if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
                tong = !tong;
                rTongue.set_value(tong);
            }

            if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
                snack = !snack;
                Snacky.set_value(snack);
            }

            if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
                Hoard.set_value(true);
            }
            if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
                Hoard.set_value(false);
                dih = true;
            }
            if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
                Hoard.set_value(true);
            }
            if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
                Hoard.set_value(false);
            }
        }

        pros::delay(20);
    }
}