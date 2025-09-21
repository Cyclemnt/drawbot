#include "AccelStepper.h" // 28BYJ-48
#include <Wire.h> // BNO055
#include <Adafruit_Sensor.h> // BNO055
#include <Adafruit_BNO055.h> // BNO055
#include <utility/imumaths.h> // BNO055
#include <Servo.h> // Servo

#include "constants.h"

/*
   Connections BNO055
   ===========
   Connect SCL to A5
   Connect SDA to A4
   Connect VDD to 5V DC
   Connect GND to GND
*/

bool running = true;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);

// Initialize with pin sequence IN1-IN3-IN2-IN4 for using the AccelStepper library with 28BYJ-48 stepper motor:
AccelStepper leftStepper = AccelStepper(MOTOR_INTERFACE_TYPE, LEFT_STEPPER_IN1, LEFT_STEPPER_IN3, LEFT_STEPPER_IN2, LEFT_STEPPER_IN4);
AccelStepper rightStepper = AccelStepper(MOTOR_INTERFACE_TYPE, RIGHT_STEPPER_IN1, RIGHT_STEPPER_IN3, RIGHT_STEPPER_IN2, RIGHT_STEPPER_IN4);

Servo servo;

#include "functions.h"

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);  // wait for serial port to open!

  /* Initialise the BNO055 */
  if(!bno.begin()) {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  delay(3000);
  bno.setExtCrystalUse(true);

  /* Initialise the 28BYJ_48 */
  leftStepper.setMaxSpeed(MAX_SPEED);  // Set the maximum steps per second:
  rightStepper.setMaxSpeed(MAX_SPEED); // 1024 : 15 rpm @ 4096 steps/rotation

  /* Initialise the FS90MG */
  servo.attach(SERVO_PIN);
  servo.write(0);
}

void loop() {
  if (running) {
    pen(false);
    delay(2000);

    pen(true);
    drawVitruvian2(8);
    pen(false);

    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    float actualAngle = 360 - euler.x();
    turn(-actualAngle);
    move(20);
    turn(90);
    move(12);

    pen(true);
    drawParametricCurve(xHeart, yHeart, 0, 2*M_PI, 0.2, 0); // Coeur
    pen(false);

    turn(-52);
    move(16);
    turn(180);

    pen(true);
    drawParametricCurve(xStar, yStar, 0, 6*M_PI+0.48, 0.5, -0.5); // Etoile
    pen(false);

    endChime();
    running = false;
  }
  delay(10000);
}
