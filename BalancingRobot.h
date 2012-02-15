#ifndef _balancingrobot_h_
#define _balancingrobot_h_

#include "mbed.h"

#define RAD_TO_DEG 57.295779513082320876798154814105 // 180/pi

DigitalOut onboardLED1(LED1);
DigitalOut onboardLED2(LED2);
DigitalOut onboardLED3(LED3);
DigitalOut onboardLED4(LED4);

/* Left motor */
DigitalOut leftA(p21);
DigitalOut leftB(p22);
PwmOut leftPWM(p23);
//AnalogIn leftCurrentSense(p15); // Not used

/* Right motor */
DigitalOut rightA(p24);
DigitalOut rightB(p25);
PwmOut rightPWM(p26);
//AnalogIn rightCurrentSense(p16); // Not used

/* IMU */
AnalogIn gyroY(p17);
AnalogIn accX(p18);
AnalogIn accY(p19);
AnalogIn accZ(p20);

// Zero voltage values for the sensors - [0] = gyroY, [1] = accX, [2] = accY, [3] = accZ
double zeroValues[4];

/* Kalman filter variables and constants */
const float Q_angle = 0.001; // Process noise covariance for the accelerometer - Sw
const float Q_gyro = 0.003; // Process noise covariance for the gyro - Sw
const float R_angle = 0.03; // Measurent noise covariance - Sv

double angle = 180; // It starts at 180 degrees
double bias = 0;
double P_00 = 0, P_01 = 0, P_10 = 0, P_11 = 0;
double dt, y, S;
double K_0, K_1;

// Results
double accYangle;
double gyroYrate;
double pitch;

/* PID values */
// Motors
double Kp = 11;
double Ki = 2;
double Kd = 12;
double targetAngle = 90;
double lastTargetAngle;

double lastError;
double iTerm;

/* Used for timing */
long timer;

const long STD_LOOP_TIME = 10*1000; // Fixed time loop of 10 milliseconds
long lastLoopTime = STD_LOOP_TIME;
long lastLoopUsefulTime = STD_LOOP_TIME;
long loopStartTime;

enum Motor {
    left,
    right,
    both,
};
enum Direction {
    forward,
    backward,
};

bool steerForward;
bool steerForwardFull;
bool steerBackward;
bool steerBackwardFull;
bool steerLeft;
bool steerRotateLeft;
bool steerRight;
bool steerRotateRight;

void calibrateSensors();
void PID(double restAngle);
double kalman(double newAngle, double newRate, double looptime);
double getGyroYrate();
double getAccY();
void move(Motor motor, Direction direction, float speed);
void stop(Motor motor);
void processing();
void receiveSerial();
void stopAndReset();

#endif