#include <PS2X_lib.h>                  // เรียกใช้งานไลบรารีสำหรับจอยสติ๊ก PS2
#include <Servo.h> 
#define PS2_DAT   8                    // กำหนดขา Data    เป็นขา 8
#define PS2_CMD   9                    // กำหนดขา Command เป็นขา 9
#define PS2_SEL   10                   // กำหนดขา Select  เป็นขา 10
#define PS2_CLK   11                   // กำหนดขา Clock   เป็นขา 11
Servo myservo;         // create servo object to control a servo 

Servo myservo2;
int enA = 3;       // PWM A
int in1 = 2;       // motorA
int in2 = 4;       // motorA

int enB = 5;       // PWM B
int in3 = 6;       // motor B
int in4 = 7;       // motor B

int val1 = 255;    // speed 1  
int val2 = 150;    // speed 2  
int old_incom = 0;
byte valservo1 = 120; 
byte valservo2 = 100; 
PS2X ps2x;         // ประกาศตัวแปรสำหรับจอยสติ๊ก PS2 


void setup()
{
  pinMode (enA, OUTPUT); 
  pinMode (in1, OUTPUT);
  pinMode (in2, OUTPUT); 
  pinMode (enB, OUTPUT);
  pinMode (in3, OUTPUT); 
  pinMode (in4, OUTPUT);
  delay(1000);                         // หน่วงเวลา 1 วินาทีเพื่อรอให้บอร์ดพร้อมทำงาน
  Serial.begin(9600);  
  Serial.println("Connecting");        // แสดงข้อความเพื่อแจ้งว่า กำลังเชื่อมต่อกับจอยสติ๊ก 

  while(true)                          // วนรอการเชื่อมต่อกับจอยสติ๊ก
  {
    int error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, false, false);
    if(error == 0)                     // กรณีที่เชื่อมต่อได้ Error = 0
    {
      Serial.println("OK");            // แสดงข้อความว่า เชื่อมต่อกับจอยสติ๊กเรียบร้อยแล้ว
      delay(1000);                     // หน่วงเวลา 1 วินาที
      break;                           // ออกจาก while(true)
    } 
    delay(500);          // หน่วงเวลา 500 มิลลิวินาทีเพื่อรอการเชื่อมต่อครั้งต่อไปในกรณีที่เชื่อมต่อไม่สำเร็จ
  }

    myservo.attach(13);               // กำหนด myservo ตัวยก-วาง  ใช้ขา 13 
    myservo2.attach(12);              // กำหนด myservo2 ตัวหนีบ-คลาย  ใช้ขา 12 
    delay(300);
    myservo.write(valservo1);               // กำหนดตำแหน่งเริ่มต้น servo ตัวยก-วาง
    myservo2.write(valservo2);              // กำหนดตำแหน่งเริ่มต้น servo ตัวหนีบ-คลาย
    delay(1000);
}

void foward1 ()            // หุ่นยนต์เดินหน้า
{
   digitalWrite (in1, LOW); 
   digitalWrite (in2, HIGH); 
   digitalWrite (in3, HIGH); 
   digitalWrite (in4, LOW); 
   analogWrite (enA, val2);     // ตั้งความเร็ว PWM ไว้ตามค่า val2      
   analogWrite (enB, val2);     // ตั้งความเร็ว PWM ไว้ตามค่า val2      
 }
 void turnLeft ()
{
  digitalWrite (in1, HIGH);
  digitalWrite (in2, LOW); 
  digitalWrite (in3, HIGH); 
  digitalWrite (in4, LOW); 
  analogWrite (enA, val2);     // ตั้งความเร็ว PWM ไว้ตามค่า val2        
  analogWrite (enB, val2);     // ตั้งความเร็ว PWM ไว้ตามค่า val2          
 }
 void turnRight ()
{
  digitalWrite (in1, LOW);
  digitalWrite (in2, HIGH); 
  digitalWrite (in3, LOW); 
  digitalWrite (in4, HIGH); 
  analogWrite (enA, val2);     // ตั้งความเร็ว PWM ไว้ตามค่า val2        
  analogWrite (enB, val2);     // ตั้งความเร็ว PWM ไว้ตามค่า val2            
 }
void back1 ()
{
  digitalWrite (in1, HIGH);
  digitalWrite (in2, LOW); 
  digitalWrite (in3, LOW); 
  digitalWrite (in4, HIGH); 
  analogWrite (enA, val2);     // ตั้งความเร็ว PWM ไว้ตามค่า val2        
  analogWrite (enB, val2);     // ตั้งความเร็ว PWM ไว้ตามค่า val2 
     } 
 void stopBot ()
{
  digitalWrite (in1, LOW); 
  digitalWrite (in2, LOW); 
  digitalWrite (in3, LOW); 
  digitalWrite (in4, LOW); 
  analogWrite (enA, 0);       // ตั้งความเร็ว PWM ไว้ตามค่า val2       
  analogWrite (enB, 0);       // ตั้งความเร็ว PWM ไว้ตามค่า val2       
     } 
//****************************************************************************************************************************
void servo1_up ()  //ควบคุมการยกแขนขึ้น
{
 if(valservo1 > 80)
  {
   valservo1 = valservo1-5;  //ลดค่าครั้งละ 5
   myservo.write(valservo1);             
   delay(10); 
  }  
   Serial.println(valservo1);
} 
void servo1_down ()  //ควบคุมการวางแขนลง
{
  if(valservo1 < 180)
  {
   valservo1 = valservo1+5;    //เพิ่มค่าครั้งละ 5
   myservo.write(valservo1);             
   delay(10); 
  }  
   Serial.println(valservo1); 
} 
//*****************************************************************
void servo2_out ()  //ควบคุมการลายออก
{
  if(valservo2 < 145)
  {
   valservo2 = valservo2+5;
   myservo2.write(valservo2);             
   delay(200); 
  } 
   Serial.println(valservo2);  
} 
void servo2_in ()  //ควบคุมการหนีบเข้า
{
 if(valservo2 > 50)
  {
   valservo2 = valservo2-5;
   myservo2.write(valservo2);           
   delay(200); 
  }  
   Serial.println(valservo2);
} 
//*********************************************************************************************************************************************
void loop()
{
   ps2x.read_gamepad(false, false);     // อ่านข้อมูลจาก PS2 Controller  
  if(ps2x.Button(PSB_L1))               // ถ้าปุ่ม L1 ถูกกด
    {
      Serial.println("L1");             // แสดงข้อความว่า L1 
      servo1_up ();
    }
  else if(ps2x.Button(PSB_L2))          // ถ้าปุ่ม L2 ถูกกด
    {
      Serial.println("L2");             // แสดงข้อความว่า L2
      servo1_down ();
    } 
  else if(ps2x.Button(PSB_R1))          // ถ้าปุ่ม R1 ถูกกด
    {  
      Serial.println("R1");
      servo2_in();
    }
  else if(ps2x.Button(PSB_R2))         // ถ้าปุ่ม R2 ถูกกด
    {
      Serial.println("R2");
      servo2_out();
    }  
  else if(ps2x.Button(PSB_START))      // ถ้าปุ่ม Start ถูกกด
   {
      Serial.println("Start");           
      val2 = 255;                      // เปลี่ยนค่า val2 ให้เท่ากับ 255 
   }
  else if(ps2x.Button(PSB_SELECT))     // ถ้าปุ่ม Select ถูกกด
  {
      Serial.println("Select");          
      val2 = 60;                       // เปลี่ยนค่า val2 ให้เท่ากับ 150
  }
  else if(ps2x.Button(PSB_PAD_UP))     // ถ้าปุ่ม Up ถูกกด
  {
    Serial.println("Up");              // แสดงข้อความว่า Up 
    if(old_incom !=1)
    {
    foward1 ();                        // กระโดดไปทำงานฟังก์ชั่น forward1()
    }
    old_incom = 1;
  }
  else if(ps2x.Button(PSB_PAD_DOWN))   // ถ้าปุ่ม Down ถูกกด
  {
    Serial.println("Down");            // แสดงข้อความว่า Up 
    if(old_incom !=2)
    {
    back1 ();                          // กระโดดไปทำงานฟังก์ชั่น back1()
    }
    old_incom = 2;
  }
  else if(ps2x.Button(PSB_PAD_LEFT))   // ถ้าปุ่ม Left ถูกกด
    {
    Serial.println("Left");            // แสดงข้อความว่า Up 
    if(old_incom !=3)
    {
    turnLeft ();                       // กระโดดไปทำงานฟังก์ชั่น turnLeft ()
    }
    old_incom = 3;
  }
  else if(ps2x.Button(PSB_PAD_RIGHT))  // ถ้าปุ่ม Right ถูกกด
    {
    Serial.println("Right");           // แสดงข้อความว่า Up 
    if(old_incom !=4)
    {
    turnRight ();                      // กระโดดไปทำงานฟังก์ชั่น turnRight ()
    }
    old_incom = 4;
  }
  else if(ps2x.ButtonPressed(PSB_TRIANGLE))  // ถ้าปุ่ม Triangle ถูกกด
    {
    Serial.println("Triangle");              // แสดงข้อความว่า Triangle
    valservo1 = 100;                         // ให้ valservo1=100
    myservo.write(valservo1); 
    delay(200);    
    }
  else  //ถ้าไม่ใช่ทุกกรณี
  {
    stopBot ();      // กระโดดไปทำงานฟังก์ชั่น stopBot ()
    old_incom = 0;
    delay(10);       // หน่วงเวลา 50 มิลลิวินาที
  }
  delay(10);
}