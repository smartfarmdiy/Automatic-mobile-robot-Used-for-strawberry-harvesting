#include <PS2X_lib.h> // PS2 Controller Library
#include <AFMotor.h>  // Adafruit Motor Shield Library

// Define motor pins
AF_DCMotor motor1(1);
AF_DCMotor motor2(2);

// Create a PS2X object
PS2X ps2x;

void setup() {
  // Initialize the PS2 joystick
  ps2x.config_gamepad(9, 10, 11, 12, true, true);

  // Set motor speed
  motor1.setSpeed(255);
  motor2.setSpeed(255);
}

void loop() {
  // Read joystick data
  ps2x.read_gamepad();

  // Control motors based on joystick input
  int xValue = ps2x.Analog(PSS_LX);
  int yValue = ps2x.Analog(PSS_LY);

  // Map joystick values to motor speed
  int leftSpeed = yValue + xValue;
  int rightSpeed = yValue - xValue;

  // Control the motors
  motor1.setSpeed(abs(leftSpeed));
  motor2.setSpeed(abs(rightSpeed));

  // Set motor directions
  if (leftSpeed > 0) {
    motor1.run(FORWARD);
  } else {
    motor1.run(BACKWARD);
  }
  if (rightSpeed > 0) {
    motor2.run(FORWARD);
  } else {
    motor2.run(BACKWARD);
  }
}
//Make sure to install the necessary libraries for the PS2 joystick and Adafruit Motor Shield using the Arduino Library Manager.

//This code initializes the PS2 joystick, reads its values, and maps the joystick input to motor speed and direction. It controls two DC motors using the Adafruit Motor Shield library. You may need to adjust the pin numbers and motor driver library if you're using different hardware.

//Remember that this is a basic example, and you can enhance it by adding features like speed control, turning, and safety mechanisms (e.g., obstacle avoidance). Additionally, ensure proper power management and consider adding a microcontroller for wireless control if needed.





