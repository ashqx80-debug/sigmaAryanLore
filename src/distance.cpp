// Distance-based PID helpers
#include "main.h"
#include "distance.h"
#include <cmath>
#include <iostream>

extern pros::Distance intake_dist;
extern pros::MotorGroup left_drive;
extern pros::MotorGroup right_drive;

void moveF(double distance, bool forwards, bool decreasing, int maxSpeed, int minSpeed, int timeOutMs) {
	// PID constants - tune these for optimal performance
	const double kP = 0.22;  // Proportional gain
	const double kD = 0.525;  // Derivative gain
	const double tolerance = 5.0;  // Distance tolerance in mm
	const int settleTime = 100;  // Time to stay within tolerance before exiting (ms)
    
	// Error tracking
	double prevError = 0;
	double error = 0;
	uint32_t startTime = pros::millis();
	uint32_t settleStartTime = 0;
	bool isSettled = false;
    
	while (true) {
		// Check timeout
		if (pros::millis() - startTime > timeOutMs) {
			break;
		}
        
		// Read current distance from front sensor
		int32_t currentDistance = intake_dist.get();
        
		// If sensor returns error or max reading, stop
		if (currentDistance == PROS_ERR || currentDistance >= 9999) {
			std::cout << "Front distance sensor error or no object detected" << std::endl;
			break;
		}
        
		// Calculate error based on whether we want distance to decrease or increase
		if (decreasing) {
			error = currentDistance - distance;  // Positive error means we need to move closer
		} else {
			error = distance - currentDistance;  // Positive error means we need to move away
		}
        
		// Check if we're within tolerance
		if (std::abs(error) <= tolerance) {
			if (!isSettled) {
				settleStartTime = pros::millis();
				isSettled = true;
			} else if (pros::millis() - settleStartTime >= settleTime) {
				// Successfully settled at target distance
				break;
			}
		} else {
			isSettled = false;
		}
        
		// Calculate derivative
		double derivative = error - prevError;
        
		// Calculate motor power using PD control
		double motorPower = (kP * error) + (kD * derivative);
        
		// Clamp motor power to maxSpeed
		if (motorPower > maxSpeed) {
			motorPower = maxSpeed;
		} else if (motorPower < -maxSpeed) {
			motorPower = -maxSpeed;
		}
        
		// Apply minimum power threshold to overcome static friction
		int effectiveMinSpeed = (minSpeed > 0) ? minSpeed : 15;
		if (std::abs(motorPower) > 0 && std::abs(motorPower) < effectiveMinSpeed) {
			motorPower = (motorPower > 0) ? effectiveMinSpeed : -effectiveMinSpeed;
		}
        
		// Determine motor direction based on forwards parameter
		int finalMotorPower = forwards ? motorPower : -motorPower;
        
		// Move motors
		left_drive.move(finalMotorPower);
		right_drive.move(finalMotorPower);
        
		// Update previous error for next iteration
		prevError = error;
        
		// Debug output
		std::cout << "Front Distance: " << currentDistance << "mm, Target: " << distance
				  << "mm, Error: " << error << "mm, Power: " << finalMotorPower << std::endl;
        
		// Small delay for loop timing
		pros::delay(20);
	}
    
	// Stop motors when done
	left_drive.move(0);
	right_drive.move(0);
    
	std::cout << "moveF complete" << std::endl;
}
