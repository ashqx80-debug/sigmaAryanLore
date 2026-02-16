#include "helpers.hpp"
#include "globals.hpp"
#include <iostream>
#include <cmath>

// If helpers need other heavy headers, they should include them here.

// constexpr double FIELD_MIN = -72;
// constexpr double FIELD_MAX = 72;
// constexpr int NUM_PARTICLES = 300;

// pros::Distance distLeft(7);
// pros::Distance distRight(8);
// pros::Distance distFront(9);
// pros::Distance distBack(6);

// struct Particle
// {
//     double x;
//     double y;
//     double h;
//     double w;
// };

// std::vector<Particle> particles;

// std::default_random_engine rng(pros::millis());
// std::normal_distribution<double> linNoise(0, 0.25);
// std::normal_distribution<double> angNoise(0, 0.75 * M_PI / 180);

// double estX = 0;
// double estY = 0;
// double estH = 0;
// bool mclEnabled = ENABLE_MCL;

// double angleDiff(double a, double b)
// {
//     double d = a - b;
//     while (d > M_PI)
//     {
//         d -= 2 * M_PI;
//     }
//     while (d < -M_PI)
//     {
//         d += 2 * M_PI;
//     }
//     return d;
// }

// double eLeft(const Particle &p)
// {
//     return p.x - FIELD_MIN;
// }

// double eRight(const Particle &p)
// {
//     return FIELD_MAX - p.x;
// }

// double eFront(const Particle &p)
// {
//     return FIELD_MAX - p.y;
// }

// double eBack(const Particle &p)
// {
//     return p.y - FIELD_MIN;
// }

// void mclInit(double x, double y, double h)
// {
//     particles.clear();
//     for (int i = 0; i < NUM_PARTICLES; i++)
//     {
//         Particle p;
//         p.x = x + linNoise(rng) * 2;
//         p.y = y + linNoise(rng) * 2;
//         p.h = h + angNoise(rng);
//         p.w = 1.0 / NUM_PARTICLES;
//         particles.push_back(p);
//     }

    // if (MCL_OVERRIDE_CHASSIS == 1)
    // {
    //     chassis.setPose(x, y, h * 180 / M_PI);
    // }

//     estX = x;
//     estY = y;
//     estH = h;
// }

// void relocalizeFromWall()
// {
//     // double dl = distLeft.get_distance() / 25.4;
//     // double dr = distRight.get_distance() / 25.4;
//     // double df = distFront.get_distance() / 25.4;
//     // double db = distBack.get_distance() / 25.4;

//     // particles.clear();

//     // for (int i = 0; i < NUM_PARTICLES; i++)
//     // {
//     //     Particle p;

//     //     p.h = estH + angNoise(rng);

//     //     if (dl < 12)
//     //     {
//     //         p.x = FIELD_MIN + dl;
//     //         p.y = FIELD_MIN + i * 144.0 / NUM_PARTICLES;
//     //     }
//     //     else if (dr < 12)
//     //     {
//     //         p.x = FIELD_MAX - dr;
//     //         p.y = FIELD_MIN + i * 144.0 / NUM_PARTICLES;
//     //     }
//     //     else if (df < 12)
//     //     {
//     //         p.y = FIELD_MAX - df;
//     //         p.x = FIELD_MIN + i * 144.0 / NUM_PARTICLES;
//     //     }
//     //     else
//     //     {
//     //         p.y = FIELD_MIN + db;
//     //         p.x = FIELD_MIN + i * 144.0 / NUM_PARTICLES;
//     //     }

//     //     p.w = 1.0 / NUM_PARTICLES;
//     //     particles.push_back(p);
//     // }
// }

// void mclTask(void *)
// {
//     double lastHeading = imu.get_heading() * M_PI / 180;

//     while (true)
//     {
//         if (!mclEnabled)
//         {
//             pros::delay(20);
//             continue;
//         }

//         double heading = imu.get_heading() * M_PI / 180;
//         double dtheta = angleDiff(heading, lastHeading);

//         double var = 2.0 * 2.0;
//         double sum = 0;

//         for (int i = 0; i < particles.size(); i++)
//         {
//             particles[i].h += dtheta + angNoise(rng);
//             particles[i].x += linNoise(rng);
//             particles[i].y += linNoise(rng);

//             double dl = distLeft.get_distance() / 25.4;
//             double dr = distRight.get_distance() / 25.4;
//             double df = distFront.get_distance() / 25.4;
//             double db = distBack.get_distance() / 25.4;

//             if (dl < 2 || dr < 2 || df < 2 || db < 2 || dl > 70 || dr > 70 || df > 70 || db > 70)
//             {
//                 continue;
//             }

//             double w = exp(-pow(dl - eLeft(particles[i]), 2) / (2 * var));
//             w *= exp(-pow(dr - eRight(particles[i]), 2) / (2 * var));
//             w *= exp(-pow(df - eFront(particles[i]), 2) / (2 * var));
//             w *= exp(-pow(db - eBack(particles[i]), 2) / (2 * var));

//             particles[i].w = w;
//             sum += w;
//         }

//         if (sum < 1e-6)
//         {
//             relocalizeFromWall();
//         }
//         else
//         {
//             for (int i = 0; i < particles.size(); i++)
//             {
//                 particles[i].w /= sum;
//             }
//         }

//         double sx = 0;
//         double sy = 0;
//         double sc = 0;
//         double ss = 0;

//         for (int i = 0; i < particles.size(); i++)
//         {
//             sx += particles[i].x;
//             sy += particles[i].y;
//             sc += cos(particles[i].h);
//             ss += sin(particles[i].h);
//         }

//         estX = sx / NUM_PARTICLES;
//         estY = sy / NUM_PARTICLES;
//         estH = atan2(ss, sc);

//         if (MCL_OVERRIDE_CHASSIS == 1)
//         {
//             chassis.setPose(estX, estY, estH * 180 / M_PI);
//         }

//         for (int i = 0; i < std::min(50, (int)particles.size()); i++)
//         {
//             pros::lcd::print(i, "P%d: X %.1f Y %.1f", i, particles[i].x, particles[i].y);
//         }

//         lastHeading = heading;
//         pros::delay(20);
//     }
// }

// Definitions for mech queue and mutex
std::vector<MechAction> mechQueue;
pros::Mutex mechMutex;

void enqueueMotor(pros::Motor &m, int v, int t)
{
    mechMutex.take();
    MechAction a;
    a.type = MechAction::MOTOR;
    a.motor = &m;
    a.digital = nullptr;
    a.value = v;
    a.time = t;
    mechQueue.push_back(a);
    mechMutex.give();
}

void enqueueDigital(pros::adi::DigitalOut &d, bool v, int t)
{
    mechMutex.take();
    MechAction a;
    a.type = MechAction::DIGITAL;
    a.motor = nullptr;
    a.digital = &d;
    a.value = v ? 1 : 0;
    a.time = t;
    mechQueue.push_back(a);
    mechMutex.give();
}

void mechTask(void *)
{
    int dt = 20;
    std::vector<MechAction> active;

    while (true)
    {
        mechMutex.take();
        for (int i = 0; i < mechQueue.size(); i++)
        {
            active.push_back(mechQueue[i]);
        }
        mechQueue.clear();
        mechMutex.give();

        for (int i = active.size() - 1; i >= 0; i--)
        {
            if (active[i].type == MechAction::MOTOR)
            {
                active[i].motor->move(active[i].value);
            }
            else
            {
                active[i].digital->set_value(active[i].value);
            }

            if (active[i].time > 0)
            {
                active[i].time -= dt;

                if (active[i].time <= 0)
                {
                    if (active[i].type == MechAction::MOTOR)
                    {
                        active[i].motor->move(0);
                    }
                    else
                    {
                        active[i].digital->set_value(0);
                    }

                    active.erase(active.begin() + i);
                }
            }
        }

        pros::delay(dt);
    }
}

void moveF(double distance, bool forwards, bool decreasing, int maxSpeed, int minSpeed, int timeOutMs)
{
    const double kP = 0.22;
    const double kD = 0.525;
    const double tolerance = 5.0;
    const int settleTime = 100;

    double prevError = 0;
    double error = 0;
    uint32_t startTime = pros::millis();
    uint32_t settleStartTime = 0;
    bool isSettled = false;

    while (true)
    {
        if (pros::millis() - startTime > timeOutMs)
        {
            break;
        }

        int32_t currentDistance = intake_dist.get();

        if (currentDistance == PROS_ERR || currentDistance >= 9999)
        {
            std::cout << "Front distance sensor error or no object detected" << std::endl;
            break;
        }

        if (decreasing)
        {
            error = currentDistance - distance;
        }
        else
        {
            error = distance - currentDistance;
        }

        if (std::abs(error) <= tolerance)
        {
            if (!isSettled)
            {
                settleStartTime = pros::millis();
                isSettled = true;
            }
            else if (pros::millis() - settleStartTime >= settleTime)
            {
                break;
            }
        }
        else
        {
            isSettled = false;
        }

        double derivative = error - prevError;

        double motorPower = (kP * error) + (kD * derivative);

        if (motorPower > maxSpeed)
            motorPower = maxSpeed;
        else if (motorPower < -maxSpeed)
            motorPower = -maxSpeed;

        int effectiveMinSpeed = (minSpeed > 0) ? minSpeed : 15;
        if (std::abs(motorPower) > 0 && std::abs(motorPower) < effectiveMinSpeed)
        {
            motorPower = (motorPower > 0) ? effectiveMinSpeed : -effectiveMinSpeed;
        }

        int finalMotorPower = forwards ? motorPower : -motorPower;

        left_drive.move(finalMotorPower);
        right_drive.move(finalMotorPower);

        prevError = error;

        std::cout << "Front Distance: " << currentDistance << "mm, Target: " << distance
                  << "mm, Error: " << error << "mm, Power: " << finalMotorPower << std::endl;

        pros::delay(20);
    }

    left_drive.move(0);
    right_drive.move(0);

    std::cout << "moveF complete" << std::endl;
}
void moveB(double distance, bool forwards, bool decreasing, int maxSpeed, int minSpeed, int timeOutMs) {
    // PID constants - tune these for optimal performance
    const double kP = 0.3;  // Proportional gain
    const double kD = 0.75;  // Derivative gain
    const double tolerance = 2.5;  // Distance tolerance in mm
    const int settleTime = 55;  // Time to stay within tolerance before exiting (ms)
    
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
        
        // Read current distance from back sensor
        int32_t currentDistance = intake_dist.get();
        
        // If sensor returns error or max reading, stop
        if (currentDistance == PROS_ERR || currentDistance >= 9910) {
            std::cout << "Distance sensor error or no object detected" << std::endl;
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
        // If forwards is true and we have positive error (need to move), motors go forward
        // If forwards is false, invert the motor power
        int finalMotorPower = forwards ? motorPower : -motorPower;
        
        // Move motors
        left_drive.move(finalMotorPower);
        right_drive.move(finalMotorPower);
        
        // Update previous error for next iteration
        prevError = error;
        
        // Debug output
        std::cout << "Distance: " << currentDistance << "mm, Target: " << distance 
                  << "mm, Error: " << error << "mm, Power: " << finalMotorPower << std::endl;
        
        // Small delay for loop timing
        pros::delay(20);
    }
    
    // Stop motors when done
    left_drive.move(0);
    right_drive.move(0);
    
    std::cout << "moveB complete" << std::endl;
}

/**
 * @brief Move robot based on front distance sensor reading
 * 
 * This function uses a PD control loop to move the robot until the front distance
 * sensor reads the target distance. The robot will move forwards or backwards based on
 * the parameters, adjusting speed based on error from the target distance.
 * 
 * @param distance Target distance in millimeters that the sensor should read
 * @param forwards True to move forwards, false to move backwards  
 * @param decreasing True if we want the distance reading to decrease (move closer), false to increase (move away)
 * @param maxSpeed Maximum motor speed (0-127)
 * @param timeOutMs Timeout in milliseconds
 */