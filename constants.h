#ifndef CONSTANTS_H
#define CONSTANTS_H

#define WHEEL_RADIUS 0.042  // Rayon des roues du robot
#define ROBOT_TRACK 0.171  // Espace entre les roues du robot

#define LEFT_WHEEL_ERROR 1//0.999264

#define MAX_SPEED 512      // Vitesse maximale du moteur en rad/s

#define KP 64.0    // constante PID proportionnelle
#define KI 0.04    // constante PID intégrale
#define KD 0.074   // constante PID dérivée

#define TURNING_ANGLE_TRESHOLD 0.05 // Seuil d'arrêt de rotation (°)

// Motor pin definitions:
#define LEFT_STEPPER_IN1  4   // IN1 on the ULN2003 driver
#define LEFT_STEPPER_IN2  5   // IN2 on the ULN2003 driver
#define LEFT_STEPPER_IN3  6  // IN3 on the ULN2003 driver
#define LEFT_STEPPER_IN4  7   // IN4 on the ULN2003 driver

#define RIGHT_STEPPER_IN1  10  // IN1 on the ULN2003 driver
#define RIGHT_STEPPER_IN2  11  // IN2 on the ULN2003 driver
#define RIGHT_STEPPER_IN3  12 // IN3 on the ULN2003 driver
#define RIGHT_STEPPER_IN4  13 // IN4 on the ULN2003 driver

// Define the AccelStepper interface type; 4 wire motor in half step mode:
#define MOTOR_INTERFACE_TYPE 8 // 4096 steps / rotation

#define SERVO_PIN 9   // servo pin

#endif // CONSTANTS_H