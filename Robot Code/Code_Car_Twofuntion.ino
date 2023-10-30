#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include <PS2X_lib.h>

MPU6050 accelgyro;
PS2X ps2x; // Create PS2 Controller Object

int16_t ax, ay, az;
int16_t gx, gy, gz;
int OdomCount = 0;
int OdomWait = 3;
static const float GYRO_SENS = 160;
int LMotor = 1;
int RMotor = 0;
bool left_forward;
bool right_forward;
int MotorNum[2] = {LMotor, RMotor};
double Vels[2] = {0, 0};
int CVEL[2] = {0, 0};
int Mspeeds[2] = {0, 0};
double WCS[2] = {0, 0};
long EncoderVal[2] = {0, 0};
double DDis[2] = {0, 0};
long Time[2] = {0, 0};
float WheelSeparation = 0.24;
float WheelDiameter = 0.07;
int TPR = 700;
volatile int counterL = 0;
volatile int counterR = 0;
int AccParam = 3;
int dir;
int bot_vel;
int count = 0;
int dir1 = 38, pwm1 = 39, dir2 = 40, pwm2 = 41;
int motor_speed_l, motor_speed_r;
ros::NodeHandle nh;
geometry_msgs::Twist odom_msg;
ros::Publisher Pub("ard_odom", &odom_msg);

void countL()
{
  if (WCS[0] > 0)
    counterL++;
  else
    counterL--;
}

void countR()
{
  if (WCS[1] > 0)
    counterR++;
  else
    counterR--;
}

void forward()
{
  digitalWrite(dir1, 0);
  digitalWrite(dir2, 1);
  dir = 0;
}

void backward()
{
  digitalWrite(dir1, 1);
  digitalWrite(dir2, 0);
  dir = 1;
}

void left()
{
  digitalWrite(dir1, 0);
  digitalWrite(dir2, 0);
  dir = 2;
}

void right()
{
  digitalWrite(dir1, 1);
  digitalWrite(dir2, 1);
  dir = 3;
}

void stop()
{
  analogWrite(pwm1, 0);
  analogWrite(pwm2, 0);
}

void velCallback(const geometry_msgs::Twist &CVel)
{
  double vel_x = CVel.linear.x;
  double vel_th = CVel.angular.z;
  double right_vel = 0.0;
  double left_vel = 0.0;

  if (vel_x == 0)
  {
    right_vel = vel_th * WheelSeparation / 2.0;
    left_vel = (-1) * right_vel;
  }
  else if (vel_th == 0)
  {
    left_vel = right_vel = vel_x;
  }
  else
  {
    left_vel = vel_x - vel_th * WheelSeparation / 2.0;
    right_vel = vel_x + vel_th * WheelSeparation / 2.0;
  }

  WCS[0] = left_vel;
  WCS[1] = right_vel;
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &velCallback);

double MWS[2] = {0, 0};

double CorrectedSpeed(int M, double CVel)
{
  if (Time[0] == 0 && Time[1] == 0)
  {
    Time[0] = millis();
    Time[1] = millis();
    return 0;
  }

  if (M == 0)
  {
    EncoderVal[0] = counterL;
    counterL = 0;
  }
  if (M == 1)
  {
    EncoderVal[1] = counterR;
    counterR = 0;
  }

  long T = millis();
  int DTime = T - Time[M];
  Time[M] = T;

  DDis[M] = TicksToMeters(EncoderVal[M]);

  double EVel = (DDis[M] / DTime) * 1000;

  Vels[M] = EVel;

  EVel = abs(EVel);
  CVel = abs(CVel);

  double dif = EVel - CVel;

  MWS[M] = MWS[M] - (dif * (AccParam));

  if (MWS[M] < 0)
    MWS[M] = 0;

  if (CVel == 0)
    MWS[M] = 0;

  CVEL[M] = MWS[M];

  return MWS[M];
}

double TicksToMeters(int Ticks)
{
  return (Ticks * 3.14 * WheelDiameter) / TPR;
}

void MotorWrite()
{
  int DIR;
  int min_speed;

  for (int i = 0; i < 2; i++)
  {
    double MSpeed = CorrectedSpeed(i, WCS[i]);

    if (i == 0)
      motor_speed_l = MSpeed;
    else if (i == 1)
      motor_speed_r = MSpeed;
  }
  motorGo(motor_speed_l, motor_speed_r);
}

void motorGo(uint8_t l, uint8_t r)
{

  if (WCS[0] > 0 && WCS[1] > 0)
  {
    forward();
    if (WCS[0] == WCS[1])
    {
      analogWrite(pwm1, l);
      analogWrite(pwm2, l);
    }
    else
    {
      analogWrite(pwm1, l);
      analogWrite(pwm2, r);
    }
  }
  else if (WCS[0] < 0 && WCS[1] < 0)
  {
    backward();
    if (WCS[0] == WCS[1])
    {
      analogWrite(pwm1, l);
      analogWrite(pwm2, l);
    }
    else
    {
      analogWrite(pwm1, l);
      analogWrite(pwm2, r);
    }
  }
  else if (WCS[0] > 0 && WCS[1] < 0)
  {
    left();
    analogWrite(pwm1, l);
    analogWrite(pwm2, l);
  }
  else if (WCS[0] < 0 && WCS[1] > 0)
  {
    right();
    analogWrite(pwm1, l);
    analogWrite(pwm2, l);
  }
  else
  {
    analogWrite(pwm1, l);
    analogWrite(pwm2, r);
  }
}

void setup()
{
  nh.initNode();
  nh.advertise(Pub);
  nh.subscribe(sub);

  pinMode(38, OUTPUT);
  pinMode(39, OUTPUT);
  pinMode(40, OUTPUT);
  pinMode(41, OUTPUT);
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), countL, RISING);
  attachInterrupt(digitalPinToInterrupt(3), countR, RISING);

  Wire.begin();
  accelgyro.initialize();
  ps2x.config_gamepad(13, 11, 10, 12); // PS2 Controller setup
}

void loop()
{
  int imu;
  nh.spinOnce();

  if (OdomCount > OdomWait)
  {
    odom_msg.linear.x = Vels[0];
    odom_msg.linear.y = Vels[1];
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    if (Vels[0] == 0 and Vels[1] == 0)
    {
      odom_msg.linear.z = 0;
    }
    else
    {
      imu = (int)(gz / GYRO_SENS) + 1;
      if (imu > 0)
        imu += 10;
      else if (imu < 0)
        imu -= 10;

      odom_msg.linear.z = imu;
    }
    Pub.publish(&odom_msg);
  }
  else
    OdomCount++;

  MotorWrite();
  
  // PS2 Controller Handling
  ps2x.read_gamepad(); // Corrected: Remove the assignment

  if (ps2x.ButtonPressed(PSB_START))
  {
    // Toggle control mode between joystick and your existing code
    // Implement your switching logic here
    // For example, you can use a global variable to keep track of the control mode
    // and switch between joystick control and other control based on the variable's value.
  }

  // Handle joystick input to control the robot
  int joyX = ps2x.Analog(PSS_LX);
  int joyY = ps2x.Analog(PSS_LY);

  // Implement your joystick control logic here
  // You can use joyX and joyY to determine the joystick position and control the robot accordingly.
  // For example, you can use joyX for left/right motion and joyY for forward/backward motion.

  delay(100); // Delay to control the update rate
}