
//ขาสัญญาณ PWM สำหรับควบคุมมอเตอร์
#define RPWM 5
#define LPWM 6

void setup() {
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);

  // สั่งให้มอเตอร์หยุด
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 0);
}

void loop() {

  // เร่งความเร็ว หมุนไปด้านหน้า
  digitalWrite(RPWM, LOW);
  for (int i = 0; i < 255; i+=2) {
    analogWrite(LPWM, i);
    delay(10);
  }

  delay(500);

  // ลดความเร็ว หมุนไปด้านหน้า
  for (int i = 255; i >= 0; i-=2) {
    analogWrite(LPWM, i);
    delay(10);
  }

  delay(500);

  // เร่งความเร็ว หมุนไปด้านหลัง
  digitalWrite(LPWM, LOW);
  for (int i = 0; i < 255; i++) {
    analogWrite(RPWM, i);
    delay(20);
  }

  delay(500);

  // ลดความเร็ว หมุนไปด้านหลัง
  for (int i = 255; i >= 0; i--) {
    analogWrite(RPWM, i);
    delay(20);
  }

  delay(500);
}