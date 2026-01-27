// #include "pros/llemu.hpp"
// #define ENABLE_MCL 0
// #define MCL_OVERRIDE_CHASSIS 0

#include "main.h"
#include "lemlib/api.hpp"
#include "pros/adi.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
// #include <vector>
// #include <random>
// #include <cmath>
// #include <algorithm>

pros::Motor intake_motor(4, pros::MotorGears::blue);
pros::Motor intake_hood_roller(3, pros::MotorGears::blue);

pros::adi::DigitalOut hoodPiston('A');
pros::adi::DigitalOut rTongue('C');
pros::adi::DigitalOut Snacky('B');

bool hood = false;
bool snack = false;
bool tong = false;

pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::MotorGroup left_drive({9, -8, -10}, pros::MotorGears::blue);
pros::MotorGroup right_drive({7, 6, -5}, pros::MotorGears::blue);

lemlib::Drivetrain drivetrain(&left_drive, &right_drive, 11, lemlib::Omniwheel::NEW_325, 450, 2);

pros::Imu imu(8);
pros::Rotation enc_vertical(-2);
pros::Rotation enc_horizontal(-22);

lemlib::TrackingWheel vertTW(&enc_vertical, lemlib::Omniwheel::NEW_275, +0.5);
lemlib::TrackingWheel horzTW(&enc_horizontal, lemlib::Omniwheel::NEW_2, -5.75);

lemlib::OdomSensors odomSensors(&vertTW, nullptr, &horzTW, nullptr, &imu);

lemlib::ControllerSettings lateral(5, 0.0001, 21, 0, 0, 0, 0, 0, 30);
lemlib::ControllerSettings angular(2.45, 0.0001, 22.5, 3, 1, 200, 3, 600, 90);

lemlib::Chassis chassis(drivetrain, lateral, angular, odomSensors);


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

struct MechAction
{
    enum Type
    {
        MOTOR,
        DIGITAL
    } type;
    pros::Motor *motor;
    pros::adi::DigitalOut *digital;
    int value;
    int time;
};

std::vector<MechAction> mechQueue;
pros::Mutex mechMutex;

void enqueueMotor(pros::Motor &m, int v, int t = 0)
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

void enqueueDigital(pros::adi::DigitalOut &d, bool v, int t = 0)
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
void initialize()
{
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    pros::lcd::initialize();
    chassis.calibrate();
    // pros::delay(500);

    // if (ENABLE_MCL)
    // {
    //     mclInit(0, 0, 0);
    //     pros::Task(mclTask, nullptr);
    // }

   pros::Task(mechTask, nullptr);
}

double expo(double normalizedInput) {
    normalizedInput = pow(normalizedInput, 3);
    return (normalizedInput / 18500);
}

void autonomous()
{
    /*
    1 - left safe | working
    2 - right safe | working
    3 - mid goal (left) | in dev
    5 - low goal (right) | not made
    7 - red sig sawp | not made
    8 - blue sig sawp | not made
    9 - skills | not made
    10 - pid angular | working
    11 - pid lateral | working
    12 - enqueue test left| not tested
    13 - enqueue test right| not tested
    14 - mcl test | not made
    */
    int autonSelector = 2;
    // chassis.setPose(estX, estY, estH * 180 / M_PI);
    Snacky.set_value(true);
    switch (autonSelector) {
        case 1:
            chassis.setPose(-12,26,0);
            chassis.moveToPoint(-27, 50, 1500, {.maxSpeed=75});
            intake_motor.move(127);
            chassis.waitUntil(20);
            rTongue.set_value(true);
            chassis.waitUntilDone();
            chassis.turnToPoint(-52,20, 800);
            chassis.moveToPoint(-52, 20, 1500);
            chassis.waitUntil(16);
            intake_motor.move(0);
            chassis.turnToHeading(180, 800);
            chassis.moveToPoint(-50,42, 800, {.forwards = false});
            chassis.waitUntilDone();
            intake_motor.move(127);
            intake_hood_roller.move(-127);
            hoodPiston.set_value(true);
            rTongue.set_value(true);
            pros::delay(1200);
            chassis.turnToHeading(180, 800);
            chassis.moveToPoint(-49, 0, 1200, {.maxSpeed=65});
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
            intake_hood_roller.move(-127);
            hoodPiston.set_value(true);
            pros::delay(750);
            chassis.moveToPoint(-48, 30, 200);
            hoodPiston.set_value(false);
            intake_hood_roller.move(0);
            intake_motor.move(0);
            chassis.moveToPoint(-48, 48, 500, {.minSpeed = 80});
            break;

            case 2:
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
            intake_hood_roller.move(-127);
            hoodPiston.set_value(true);
            rTongue.set_value(true);
            pros::delay(1200);
            chassis.turnToHeading(180, 800);
            chassis.moveToPoint(49, 0, 1200, {.maxSpeed=75});
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
            intake_hood_roller.move(-127);
            hoodPiston.set_value(true);
            pros::delay(1500);
            chassis.moveToPoint(48, 30, 1000);
            hoodPiston.set_value(false);
            intake_hood_roller.move(0);
            intake_motor.move(0);
            chassis.moveToPoint(48, 48, 1000, {.forwards=false, .minSpeed = 80});
            break;

            case 3:
            //blue left (mid goal)
            chassis.setPose(-12,26,0);
            chassis.moveToPoint(-24, 45, 1300, {.maxSpeed=75});
            intake_motor.move(127);
            chassis.waitUntil(20);
            rTongue.set_value(true);
            chassis.waitUntilDone();
            chassis.turnToPoint(-15,54, 500, {.forwards = false});
            chassis.waitUntilDone();
            intake_motor.move(-70);
            //pros::delay(1000000000);
            chassis.moveToPoint(-15, 54, 800, {.forwards = false, .maxSpeed=70});
            // pros::delay(300);
            
            chassis.waitUntilDone();
            intake_hood_roller.move(127);
            intake_motor.move(127);
            hoodPiston.set_value(true);
            pros::delay(1500);

            chassis.moveToPoint(-24, 48, 1200);
            chassis.turnToPoint(-52,20, 700);
            chassis.moveToPoint(-52, 20, 1000);
            chassis.waitUntil(5);
            intake_motor.move(0);
            hoodPiston.set_value(true);
            chassis.turnToHeading(180, 700);
            chassis.waitUntilDone();
            rTongue.set_value(true);
            intake_motor.move(127);
            chassis.moveToPoint(-48, 0, 1000, {.maxSpeed=70});
            chassis.waitUntil(5);
            intake_hood_roller.move(0);
            hoodPiston.set_value(false);
            chassis.waitUntilDone();
            pros::delay(700);
            chassis.moveToPoint(-48,24, 700, {.forwards = false});
            chassis.turnToHeading(180, 700);
            chassis.moveToPoint(-48, 42, 700, {.forwards = false});
            rTongue.set_value(false);
            chassis.waitUntilDone();
            intake_motor.move(127);
            intake_hood_roller.move(-127);
            hoodPiston.set_value(true);
            pros::delay(1200);
            hoodPiston.set_value(false);
            intake_hood_roller.move(0);
            intake_motor.move(0);
            break;

            //red right (low goal)
            case 6:
            chassis.setPose(12,26,0);
            chassis.moveToPoint(27, 50, 1300, {.maxSpeed=75});
            intake_motor.move(127);
            chassis.waitUntil(20);
            rTongue.set_value(true);
            chassis.waitUntilDone();
            chassis.turnToPoint(50,62, 700 );
            chassis.moveToPoint(50, 62, 1000);
            pros::delay(700);
            intake_motor.move(0);
            chassis.waitUntilDone();
            intake_hood_roller.move(127);
            intake_motor.move(-127);
            pros::delay(900);
            chassis.moveToPoint(27, 50, 1200);
            chassis.turnToPoint(52,20, 700);
            chassis.moveToPoint(52, 20, 1000);
            chassis.waitUntil(16);
            intake_motor.move(0);
            chassis.turnToHeading(180, 700);
            chassis.waitUntilDone();
            rTongue.set_value(true);
            intake_motor.move(127);
            chassis.moveToPoint(48, 0, 1000, {.maxSpeed=70});
            chassis.waitUntil(5);
            intake_hood_roller.move(0);
            hoodPiston.set_value(false);
            chassis.waitUntilDone();
            pros::delay(700);
            chassis.moveToPoint(48,24, 700, {.forwards = false});
            chassis.turnToHeading(180, 700);
            chassis.moveToPoint(48, 42, 700, {.forwards = false});
            rTongue.set_value(false);
            chassis.waitUntilDone();
            intake_motor.move(127);
            intake_hood_roller.move(-127);
            hoodPiston.set_value(true);
            pros::delay(1200);
            hoodPiston.set_value(false);
            intake_hood_roller.move(0);
            intake_motor.move(0);
            break;

            case 7:
            //left sig swap
            chassis.setPose(-12,26,0);
            chassis.moveToPoint(-50, 26, 1500, {.maxSpeed=75});
            break;

            case 8:
            //right sig swap
            chassis.setPose(12,26,0);
            chassis.moveToPoint(48, 26, 1500);
            chassis.waitUntilDone();
            chassis.turnToHeading(90, 2000);
            intake_motor.move(127);
            chassis.moveToPoint(48, 0, 1500, {.maxSpeed=75});
            pros::delay(800);
// sup
            break;

            case 9:
            //skills
            break;

            case 10:
            chassis.setPose(0,0,0);
            chassis.turnToHeading(180, 2000);
            chassis.waitUntilDone();
            pros::delay(1000);
            chassis.turnToHeading(-90, 2000);
            chassis.waitUntilDone();
            pros::delay(1000);
            chassis.turnToHeading(90, 2000);
            chassis.waitUntilDone();
            pros::delay(1000);
            chassis.turnToHeading(0, 2000);

            break;

            case 11:
            chassis.setPose(0,0,0);
            chassis.moveToPoint(0,24, 2000);
            chassis.waitUntilDone();
            pros::delay(1000);
            chassis.moveToPoint(0, 48, 2000);
            chassis.waitUntilDone();
            pros::delay(1000);
            chassis.moveToPoint(0, -24, 2000);
            chassis.waitUntilDone();
            pros::delay(1000); 
            chassis.moveToPoint(0, 0, 2000);
            break;
            case 12:
            // Enqueue version of case 1 (left safe)
            chassis.setPose(-12, 26, 0);

            enqueueMotor(intake_motor, 127);
            enqueueMotor(intake_hood_roller, 0);
            enqueueDigital(rTongue, true);

            chassis.moveToPoint(-27, 50, 1500, {.maxSpeed=75});
            chassis.waitUntil(20);

            enqueueMotor(intake_motor, 127);
            enqueueDigital(rTongue, true);

            chassis.turnToPoint(-52, 20, 800);
            chassis.moveToPoint(-52, 20, 1500);
            chassis.waitUntil(16);

            enqueueMotor(intake_motor, 127);
            enqueueMotor(intake_hood_roller, -127);
            enqueueDigital(hoodPiston, true);
            enqueueDigital(rTongue, true);
            pros::delay(1200);

            chassis.turnToHeading(180, 800);
            chassis.moveToPoint(-49, 0, 1200, {.maxSpeed=65});
            chassis.waitUntil(5);

            enqueueMotor(intake_hood_roller, 0);
            enqueueDigital(hoodPiston, false);

            chassis.moveToPoint(-48, 24, 800, {.forwards = false});
            chassis.turnToHeading(180, 800);
            chassis.moveToPoint(-48, 42, 800, {.forwards = false});

            enqueueDigital(rTongue, false);
            enqueueMotor(intake_motor, 127);
            enqueueMotor(intake_hood_roller, -127);
            enqueueDigital(hoodPiston, true);
            pros::delay(750);

            enqueueDigital(hoodPiston, false);
            enqueueMotor(intake_hood_roller, 0);
            enqueueMotor(intake_motor, 0);

            chassis.moveToPoint(-48, 48, 500, {.minSpeed = 80});
            break;

        case 13:
            // Enqueue version of case 2 (right safe)
            chassis.setPose(12, 26, 0);

            enqueueMotor(intake_motor, 127);
            enqueueMotor(intake_hood_roller, 0);
            enqueueDigital(rTongue, true);

            chassis.moveToPoint(27, 50, 1500, {.maxSpeed=75});
            chassis.waitUntil(20);

            enqueueMotor(intake_motor, 127);
            enqueueDigital(rTongue, true);

            chassis.turnToPoint(52, 20, 800);
            chassis.moveToPoint(52, 20, 1500);
            chassis.waitUntil(16);

            enqueueMotor(intake_motor, 127);
            enqueueMotor(intake_hood_roller, -127);
            enqueueDigital(hoodPiston, true);
            enqueueDigital(rTongue, true);
            pros::delay(1200);

            chassis.turnToHeading(180, 800);
            chassis.moveToPoint(49, 0, 1200, {.maxSpeed=75});
            chassis.waitUntil(5);

            enqueueMotor(intake_hood_roller, 0);
            enqueueDigital(hoodPiston, false);

            chassis.moveToPoint(48, 24, 800, {.forwards = false});
            chassis.turnToHeading(180, 800);
            chassis.moveToPoint(48, 42, 800, {.forwards = false});

            enqueueDigital(rTongue, false);
            enqueueMotor(intake_motor, 127);
            enqueueMotor(intake_hood_roller, -127);
            enqueueDigital(hoodPiston, true);
            pros::delay(1500);

            enqueueDigital(hoodPiston, false);
            enqueueMotor(intake_hood_roller, 0);
            enqueueMotor(intake_motor, 0);

            chassis.moveToPoint(48, 48, 1000, {.forwards=false, .minSpeed = 80});
            break;



    }
       
}


void opcontrol() {
    while (true) {

        bool isSkills = false;

        int leftY  = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / 1.1;

        int driveForward = expo(leftY);
        int driveTurn    = expo(rightX);

        left_drive.move(driveForward + driveTurn);
        right_drive.move(driveForward - driveTurn);

        bool intakeRunning = false;
        int intakePower = 0;
        int hoodRollerPower = 0;
        

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            intakePower = 127;
            hoodRollerPower = -127;
            intakeRunning = true;
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            intakePower = 127;
            intakeRunning = false;
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            intakePower = -127;
            hoodRollerPower = 127;
            intakeRunning = false;
            
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
            intakePower = 75;
            hoodRollerPower = 75;
        }
        else if (!isSkills && master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
            intakePower = 80;
            hoodRollerPower = -60;
            intakeRunning = false;
        
        }
        else if (isSkills && master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
            intakePower = 60;
            hoodRollerPower = -45;
            intakeRunning = false;
        
        }
        intake_motor.move(intakePower);
        intake_hood_roller.move(hoodRollerPower);
        hoodPiston.set_value(intakeRunning);


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



