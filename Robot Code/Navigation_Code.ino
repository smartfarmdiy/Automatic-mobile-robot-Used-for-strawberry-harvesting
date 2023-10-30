
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h> //[บอกระบบพิกัด]
#include <stdlib.h>

// ใช้กับ GY87 Gyro sensor
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;

int OdomCount = 0;

int OdomWait = 3;
static const float GYRO_SENS = 160;

int LMotor = 1;
int RMotor = 0;

bool left_forward; // ความเร็วล้อ
bool right_forward;

int MotorNum[2] = {LMotor, RMotor};

double Vels[2] = {0, 0};
int CVEL[2] = {0, 0};
int Mspeeds[2] = {0, 0};

double WCS[2] = {0, 0};      // WCS ความเร็วที่สั่ง motor ให้หมุน { Left,Right } >> ค่าความเร็วจริง m/s
long EncoderVal[2] = {0, 0}; // ค่าที่อ่านได้จาก Encoder
double DDis[2] = {0, 0};     // ระยะทาง
long Time[2] = {0, 0};       // เวลา

float WheelSeparation = 0.24; // ความกว้างล้อจาก Left >> Right หน่วยเมตร
float WheelDiameter = 0.07;   // เส้นผ่านศูนย์กลางล้อ
int TPR = 700;                // Encoder ticks per rotation ล้อที่หมุนสร้าง Encoder ได้กี่ลูก

volatile int counterL = 0; // ค่าของตัวนับคล็อกของ motor -- Interrupt
volatile int counterR = 0;

int AccParam = 3; // acceleration multiplier. ปรับความเร็ว motor >> ให้้ตามต้องการ

int dir;

int bot_vel;
int count = 0;

int dir1 = 38, pwm1 = 39, dir2 = 40, pwm2 = 41; // กำหนดขาบน Bord >> DC Motor Driver

int motor_speed_l, motor_speed_r; // ความเร็ว motor >> ค่า PWM >> นำไปปรับความเร็ว motor

ros::NodeHandle nh; // คือการทำให้โปรแกรมของเราสามารถสร้าง publishers และ subscribers ได้

geometry_msgs::Twist odom_msg; // message ค่า Odometry >> Raspberry Pi
ros::Publisher Pub("ard_odom", &odom_msg);

void countL() // call interrupt >> ถูกเรียกเมื่อมีการ Interrupt
{             // การ Interrupt ทำให้รับค่าทันเวลา
  if (WCS[0] > 0)
    counterL++; // มากกว่า 0 = เดินหน้า
  else
    counterL--; // น้อยกว่า 0 = ถอยหลัง
}

void countR()
{
  if (WCS[1] > 0)
    counterR++; // มากกว่า 0 = เดินหน้า
  else
    counterR--; // น้อยกว่า 0 = ถอยหลัง
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

void velCallback(const geometry_msgs::Twist &CVel) // ถูกเรียกเมื่อ cmd_vel เข้ามา                     --เมื่อมีคำสั่งให้หุ่นเดินหน้า/ถอยหลัง ** เป็นฟังก์ชันแยกความเร็วล้อ ** for set vel of 2 motor.

{                                 // message ROS >> ให้หุ่นยนต์เคลื่อนที่
  double vel_x = CVel.linear.x;   // ความเร็วในแนวตรง
  double vel_th = CVel.angular.z; // ความเร็วในแนวเชิงมุม
  double right_vel = 0.0;
  double left_vel = 0.0;

  // turning
  if (vel_x == 0)
  {
    right_vel = vel_th * WheelSeparation / 2.0; // ความเร็วล้อขวา = ความเร็วเชิงมุม x WheelSeparation หาร 2
    left_vel = (-1) * right_vel;
  }
  // forward / backward
  else if (vel_th == 0)
  {
    left_vel = right_vel = vel_x;
  }
  // moving doing arcs
  else
  {
    left_vel = vel_x - vel_th * WheelSeparation / 2.0;
    right_vel = vel_x + vel_th * WheelSeparation / 2.0;
  }
  // write new command speeds to global vars
  WCS[0] = left_vel;  // ใส่ความเร็วล้อซ้าย
  WCS[1] = right_vel; // ใส่ความเร็วล้อขวา
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &velCallback);

// motor write speed - in motor units
double MWS[2] = {0, 0};

double CorrectedSpeed(int M, double CVel)
{
  // if fist time in program return 0 and init time vars
  if (Time[0] == 0 && Time[1] == 0)
  {
    Time[0] = millis();
    Time[1] = millis();
    return 0;
  }

  // read encoder ticks
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

  // differencial of time in seconds
  long T = millis();
  int DTime = T - Time[M];
  Time[M] = T;

  // diferential of distance in meters >> คำนวณ ระยะทาง = ความเร็ม x เวลา
  DDis[M] = TicksToMeters(EncoderVal[M]);

  // calculate short term measured velocity >> คำนวณความเร็ว = ระยะทาง / เวลา
  double EVel = (DDis[M] / DTime) * 1000; // แปลงเป็น dS/dT >> เมตรต่อวินาที

  // save to publish to /ard_odom
  Vels[M] = EVel; // ส่งกลับไปที่ >> ROS ผ่าน Topic "ard_odom"

  EVel = abs(EVel);
  CVel = abs(CVel);

  double dif = EVel - CVel; // ปรับความเร็ว motor >> ความเร็วจริง - ความเร็ยวที่ต้องการ >> เป็นการ dif

  MWS[M] = MWS[M] - (dif * (AccParam)); // ความเร็วที่ต้องการ

  if (MWS[M] < 0)
    MWS[M] = 0;

  if (CVel == 0)
    MWS[M] = 0;

  // DEBUG
  CVEL[M] = MWS[M];

  return MWS[M];
}

double TicksToMeters(int Ticks) // แปลงเป็นระยะทาง
{
  return (Ticks * 3.14 * WheelDiameter) / TPR;
}

///////////////////////////////////////// motor codes /////////////////////////////////////////
void MotorWrite()
{
  int DIR;
  int min_speed;

  for (int i = 0; i < 2; i++) // ** loop 2 รอบ **
  {
    // correct speed with encoder data
    double MSpeed = CorrectedSpeed(i, WCS[i]); // พยายามปรับ PWM ให้เป็นไปตามที่ต้องการ >> tune PWM

    if (i == 0)
      motor_speed_l = MSpeed; // Left
    else if (i == 1)
      motor_speed_r = MSpeed; // Right
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

void setup() //--------------------------------------- Setup ---------------------------------------
{

  nh.initNode();
  nh.advertise(Pub);
  nh.subscribe(sub);

  pinMode(38, OUTPUT); // setpinmode DC Motor 
  pinMode(39, OUTPUT);
  pinMode(40, OUTPUT);
  pinMode(41, OUTPUT);

  pinMode(2, INPUT_PULLUP); // Port Interrup -- Encoder >> Left
  pinMode(3, INPUT_PULLUP); // Right

  attachInterrupt(digitalPinToInterrupt(2), countL, RISING); // interrupt pins for encoder Achange
  attachInterrupt(digitalPinToInterrupt(3), countR, RISING);

  Wire.begin();

  // ============================ MPU6050 ============================
  accelgyro.initialize();

  // Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  // Starts up with accel +/- 2 g and gyro +/- 250 deg/s scale
  accelgyro.setI2CBypassEnabled(true); // set bypass mode
}

void loop() //--------------------------------------- Loop ---------------------------------------
{
  int imu;
  nh.spinOnce(); // ให้้คุยกับ ROS ได้ >> Subscriber can do

  if (OdomCount > OdomWait)
  {
    odom_msg.linear.x = Vels[0]; // Left >> หลอกโค้ดเพื่อยืม odom_msg เป็นเส้นทางสู่ RaspberryPi
    odom_msg.linear.y = Vels[1]; // Right
    // หากไม่หลอกโค้ด ต้องสร้างโครงสร้างข้อมูลเอง ยิ่งยาก!!!
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); // ค่า IMU >> Gyro

    if (Vels[0] == 0 and Vels[1] == 0)
    {
      odom_msg.linear.z = 0;
    }
    else
    {
      imu = (int)(gz / GYRO_SENS) + 1; // ส่วนนี้แต่ละคนต้อง Calibration จาก LAB
      if (imu > 0)                     // only my calibration
        imu += 10;
      else if (imu < 0)
        imu -= 10;

      odom_msg.linear.z = imu;
    }
    Pub.publish(&odom_msg); // ส่งค่าไปทาง Topic
  }
  else
    OdomCount++;

  MotorWrite(); // Takes WCS and corrects speed of motors with encoders
  // ทำงานทุกครั้ง  >> ปรับความเร็วหุ่นยนต์ให้ตรงกับที่ต้องการ ** loop 2 รอบ **
}
