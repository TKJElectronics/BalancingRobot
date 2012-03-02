#ifndef _balancingrobot_h_
#define _balancingrobot_h_

#define RAD_TO_DEG 57.295779513082320876798154814105 // 180/pi

BusOut LEDs(LED1, LED2, LED3, LED4);

/* Left motor */
DigitalOut leftA(p21);
DigitalOut leftB(p22);
PwmOut leftPWM(p23);

/* Right motor */
DigitalOut rightA(p24);
DigitalOut rightB(p25);
PwmOut rightPWM(p26);

/* IMU */
AnalogIn gyroY(p17);
AnalogIn accX(p18);
AnalogIn accY(p19);
AnalogIn accZ(p20);

/* Battery voltage */
AnalogIn batteryVoltage(p15); // The analog pin is connected to a 56k-10k voltage divider
DigitalOut buzzer(p5); // Connected to a BC547 transistor - there is a protection diode at the buzzer 

// Zero voltage values for the sensors - [0] = gyroY, [1] = accX, [2] = accY, [3] = accZ
double zeroValues[4];

/* Kalman filter variables and constants */
const float Q_angle = 0.001; // Process noise covariance for the accelerometer - Sw
const float Q_gyro = 0.003; // Process noise covariance for the gyro - Sw
const float R_angle = 0.03; // Measurement noise covariance - Sv

double angle = 180; // It starts at 180 degrees
double bias = 0;
double P_00 = 0, P_01 = 0, P_10 = 0, P_11 = 0;
double dt, y, S;
double K_0, K_1;

// Results
double accYangle;
double gyroYrate;
double pitch;

/* PID variables */
double Kp = 11;
double Ki = 2;
double Kd = 12;
double targetAngle = 90;

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
bool steerBackward;
bool steerStop = true; // Stop by default
bool steerLeft;
bool steerRotateLeft;
bool steerRight;
bool steerRotateRight;

bool stopped;

const double turnSpeed = 0.1;
const double rotateSpeed = 0.2;

double targetOffset = 0;

uint8_t loopCounter = 0; // Used for wheel velocity
long wheelPosition;
long lastWheelPosition;
long wheelVelocity;
long targetPosition;
int zoneA = 2000;
int zoneB = 1000;
double positionScaleA = 250; // one resolution is 464 pulses
double positionScaleB = 500; 
double positionScaleC = 1000;
double velocityScaleMove = 40;
double velocityScaleStop = 40;

void calibrateSensors();
void PID(double restAngle, double offset);
double kalman(double newAngle, double newRate, double looptime);
double getGyroYrate();
double getAccY();
void move(Motor motor, Direction direction, float speed);
void stop(Motor motor);
void processing();
void receivePS3();
void receiveXbee();
void stopAndReset();

#endif