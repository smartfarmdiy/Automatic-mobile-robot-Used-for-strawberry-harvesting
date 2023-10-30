#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <Wire.h>

#include "I2Cdev.h"
#include "MPU6050.h"

MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;

int OdomCount = 0;
int OdomWait = 3;

const float GYRO_SENS = 160.0;

int leftMotor = 1;
int rightMotor = 0;

bool isLeftMotorForward = true;
bool isRightMotorForward = true;

int motorNumbers[2] = {leftMotor, rightMotor};

double motorVelocities[2] = {0.0, 0.0};
int motorSpeeds[2] = {0, 0};

double wheelVelocities[2] = {0.0, 0.0};
long encoderValues[2] = {0, 0};
double distances[2] = {0.0, 0.0};
long times[2] = {0, 0};

const float wheelSeparation = 0.24;
const float wheelDiameter = 0.07;
const int encoderTicksPerRotation = 700;

volatile int counterLeft = 0;
volatile int counterRight = 0;

int accelerationMultiplier = 3;
int direction;

int botVelocity;
int count = 0;

int direction1 = 38, pwm1 = 39, direction2 = 40, pwm2 = 41;

int motorSpeedLeft, motorSpeedRight;

ros::NodeHandle nodeHandle;
geometry_msgs::Twist odomMessage;
ros::Publisher publisher("ard_odom", &odomMessage);

void countLeft()
{
    if (wheelVelocities[0] > 0)
    {
        counterLeft++;
    }
    else
    {
        counterLeft--;
    }
}

void countRight()
{
    if (wheelVelocities[1] > 0)
    {
        counterRight++;
    }
    else
    {
        counterRight--;
    }
}

void setMotorForward(int motor)
{
    digitalWrite(motorNumbers[motor], LOW);
    direction = motor;
}

void setMotorBackward(int motor)
{
    digitalWrite(motorNumbers[motor], HIGH);
    direction = motor + 2;
}

void setMotorLeft()
{
    digitalWrite(direction1, LOW);
    digitalWrite(direction2, LOW);
    direction = 4;
}

void setMotorRight()
{
    digitalWrite(direction1, HIGH);
    digitalWrite(direction2, HIGH);
    direction = 5;
}

void stopMotors()
{
    analogWrite(pwm1, 0);
    analogWrite(pwm2, 0);
}

void velocityCallback(const geometry_msgs::Twist &commandVelocity)
{
    double linearVelocity = commandVelocity.linear.x;
    double angularVelocity = commandVelocity.angular.z;
    double rightWheelVelocity = 0.0;
    double leftWheelVelocity = 0.0;

    if (linearVelocity == 0)
    {
        rightWheelVelocity = angularVelocity * wheelSeparation / 2.0;
        leftWheelVelocity = -rightWheelVelocity;
    }
    else if (angularVelocity == 0)
    {
        leftWheelVelocity = rightWheelVelocity = linearVelocity;
    }
    else
    {
        leftWheelVelocity = linearVelocity - angularVelocity * wheelSeparation / 2.0;
        rightWheelVelocity = linearVelocity + angularVelocity * wheelSeparation / 2.0;
    }

    wheelVelocities[0] = leftWheelVelocity;
    wheelVelocities[1] = rightWheelVelocity;
}

ros::Subscriber<geometry_msgs::Twist> subscriber("cmd_vel", &velocityCallback);

double calculateCorrectedSpeed(int motor, double commandVelocity)
{
    if (times[0] == 0 && times[1] == 0)
    {
        times[0] = millis();
        times[1] = millis();
        return 0.0;
    }

    if (motor == 0)
    {
        encoderValues[0] = counterLeft;
        counterLeft = 0;
    }
    else if (motor == 1)
    {
        encoderValues[1] = counterRight;
        counterRight = 0;
    }

    long currentTime = millis();
    int deltaTime = currentTime - times[motor];
    times[motor] = currentTime;

    distances[motor] = ticksToMeters(encoderValues[motor]);
    double encoderVelocity = (distances[motor] / deltaTime) * 1000;

    wheelVelocities[motor] = abs(encoderVelocity);
    commandVelocity = abs(commandVelocity);

    double velocityDifference = encoderVelocity - commandVelocity;
    motorSpeeds[motor] -= velocityDifference * accelerationMultiplier;

    if (motorSpeeds[motor] < 0)
    {
        motorSpeeds[motor] = 0;
    }

    if (commandVelocity == 0)
    {
        motorSpeeds[motor] = 0;
    }

    return motorSpeeds[motor];
}

double ticksToMeters(int ticks)
{
    return (static_cast<double>(ticks) * 3.14 * wheelDiameter) / encoderTicksPerRotation;
}

void motorWrite()
{
    for (int i = 0; i < 2; i++)
    {
        double motorSpeed = calculateCorrectedSpeed(i, wheelVelocities[i]);

        if (i == 0)
        {
            motorSpeedLeft = motorSpeed;
        }
        else if (i == 1)
        {
            motorSpeedRight = motorSpeed;
        }
    }

    motorControl(motorSpeedLeft, motorSpeedRight);
}

void motorControl(uint8_t leftSpeed, uint8_t rightSpeed)
{
    if (wheelVelocities[0] > 0 && wheelVelocities[1] > 0)
    {
        setMotorForward(direction);
        if (wheelVelocities[0] == wheelVelocities[1])
        {
            analogWrite(pwm1, leftSpeed);
            analogWrite(pwm2, leftSpeed);
        }
        else
        {
            analogWrite(pwm1, leftSpeed);
            analogWrite(pwm2, rightSpeed);
        }
    }
    else if (wheelVelocities[0] < 0 && wheelVelocities[1] < 0)
    {
        setMotorBackward(direction);
        if (wheelVelocities[0] == wheelVelocities[1])
        {
            analogWrite(pwm1, leftSpeed);
            analogWrite(pwm2, leftSpeed);
        }
        else
        {
            analogWrite(pwm1, leftSpeed);
            analogWrite(pwm2, rightSpeed);
        }
    }
    else if (wheelVelocities[0] > 0 && wheelVelocities[1] < 0)
    {
        setMotorLeft();
        analogWrite(pwm1, leftSpeed);
        analogWrite(pwm2, leftSpeed);
    }
    else if (wheelVelocities[0] < 0 && wheelVelocities[1] > 0)
    {
        setMotorRight();
        analogWrite(pwm1, leftSpeed);
        analogWrite(pwm2, leftSpeed);
    }
    else
    {
        analogWrite(pwm1, leftSpeed);
        analogWrite(pwm2, rightSpeed);
    }
}

void setup()
{
    nodeHandle.initNode();
    nodeHandle.advertise(publisher);
    nodeHandle.subscribe(subscriber);

    pinMode(direction1, OUTPUT);
    pinMode(direction2, OUTPUT);
    pinMode(pwm1, OUTPUT);
    pinMode(pwm2, OUTPUT);

    pinMode(2, INPUT_PULLUP);
    pinMode(3, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(2), countLeft, RISING);
    attachInterrupt(digitalPinToInterrupt(3), countRight, RISING);

    Wire.begin();
    accelgyro.initialize();
    accelgyro.setI2CBypassEnabled(true);
}

void loop()
{
    int imu;
    nodeHandle.spinOnce();

    if (OdomCount > OdomWait)
    {
        odomMessage.linear.x = wheelVelocities[0];
        odomMessage.linear.y = wheelVelocities[1];

        accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        if (wheelVelocities[0] == 0 && wheelVelocities[1] == 0)
        {
            odomMessage.linear.z = 0;
        }
        else
        {
            imu = static_cast<int>(gz / GYRO_SENS) + 1;

            if (imu > 0)
            {
                imu += 10;
            }
            else if (imu < 0)
            {
                imu -= 10;
            }

            odomMessage.linear.z = imu;
        }

        publisher.publish(&odomMessage);
    }
    else
    {
        OdomCount++;
    }

    motorWrite();
}
