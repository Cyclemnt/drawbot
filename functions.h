#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include "AccelStepper.h" // 28BYJ-48
#include <Wire.h> // BNO055
#include <Adafruit_Sensor.h> // BNO055
#include <Adafruit_BNO055.h> // BNO055
#include <utility/imumaths.h> // BNO055
#include <Servo.h> // Servo

#include "constants.h"
#include "math.h"

// Calcul du signal PID
float pidCompute(const float error, float &prevError, float &integral, unsigned long &lastTime) {
  // Temps actuel
  unsigned long now = millis();  // ou micros() si vous voulez une meilleure précision
  float deltaTime = (now - lastTime) / 1000.0;  // Convertir en secondes
  // Protéger contre des deltas nuls ou trop petits
  if (deltaTime <= 0.0) deltaTime = 1.0 / 1000.0;  // Assume un intervalle minimum de 1 ms

  // Calcul des termes PID
  integral += error * deltaTime;  // Somme des erreurs pour le terme intégral
  float derivative = (error - prevError) / deltaTime;  // Taux de variation de l'erreur

  prevError = error;  // Sauvegarde de l'erreur précédente
  lastTime = now;

  // Calcul du signal PID
  return KP * error + KI * integral + KD * derivative;

  // Antiwindup
  //if(!(abs(u) >= 2*32767 && (((error >= 0) && (integral >= 0)) || ((error < 0) && (integral < 0))))) {
	//	integral += error * deltaTime;  // rectangular integration
	//}

  //return u;
}

// Normalisation de la différence d'angle
float normalizeAngleError(float target, float current) {
  float error = fmod(fmod(target - current + 180, 360.0) + 360, 360) - 180;
  return error;
}

// Contrôle du crayon (true : baissé ; false : monté)
inline void pen(bool down) {
  servo.write(!down * 30); // Servo +|- 30°
  delay(200);
}

// Translation du robot
void move(float distance) {
  // Set the current position to 0
  leftStepper.setCurrentPosition(0);
  rightStepper.setCurrentPosition(0);

  int direction = (distance >= 0) ? 1 : -1; // Récupération de la direction
  distance = fabs(distance * 0.010113); // Conversion en mètres

  // Run the motor forward at max steps/second until the motor reaches "steps" steps (4096 * distance / wheel_perimeter)
  long const steps = 4096 * distance / (2.0 * M_PI * WHEEL_RADIUS);
  
  leftStepper.setSpeed(-direction * MAX_SPEED * LEFT_WHEEL_ERROR); // Moteur gauche inversé par rapport au droit (-)
  rightStepper.setSpeed(direction * MAX_SPEED);
  while (abs(leftStepper.currentPosition()) != steps) {
    leftStepper.runSpeed();
    rightStepper.runSpeed();
  }
}

// Rotation du robot
void turn(float angle) {
  //if (angle >= 0) angle -= 0.3273;
  //else angle += 0.3273;
  // Récupérer l'angle initial
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  float actualAngle = 360 - euler.x();

  // Calculer et normaliser l'angle cible
  float targetAngle = angle + actualAngle;
  float error = normalizeAngleError(targetAngle, actualAngle);

  // PID variables
  double correction = 0.0;
  float prevError = 0.0, integral = 0.0;
  unsigned long lastTime = millis();

  while (fabs(error) >= TURNING_ANGLE_TRESHOLD) {
    // Calcul PID
    correction = pidCompute(error, prevError, integral, lastTime);

    // Constrain vitesse
    correction = constrain(correction, -MAX_SPEED, MAX_SPEED);

    // Appliquer la correction
    leftStepper.setSpeed(correction); // Moteur inversé
    rightStepper.setSpeed(correction);
    leftStepper.runSpeed();
    rightStepper.runSpeed();

    // Mettre à jour l'angle actuel
    euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    actualAngle = 360 - euler.x();
    // Mettre à jour l'erreur
    error = normalizeAngleError(targetAngle, actualAngle);
  }
}

// Tracer un cercle
void drawCircle(double radius) {
  float const k = 44.24 * radius; // Constante erreur proportionnelle au rayon (détermination empirique)
  // Set the current position to 0
  leftStepper.setCurrentPosition(0);
  rightStepper.setCurrentPosition(0);

  // Calcul des rayons pour les roues intérieure et extérieure
  radius = abs(0.01 * radius); // Conversion en mètres
  // Calcul des rayons décrits par les roues
  double const innerRadius = radius - (ROBOT_TRACK / 2.0);
  double const outerRadius = radius + (ROBOT_TRACK / 2.0);
  double ratio = innerRadius / outerRadius; // Ratio de distance et de vitesse à respecter
  // Calcul du nombre de pas que les roues doivent réaliser
  long innerSteps = 4096 * (innerRadius / WHEEL_RADIUS);// + k * ratio;
  long outerSteps = 4096 * (outerRadius / WHEEL_RADIUS) + k;

  // Calculer les vitesses ajustées des roues
  double innerSpeed = -MAX_SPEED * ratio;
  double const outerSpeed = MAX_SPEED;

  // Appliquer les vitesses différentielles
  leftStepper.setSpeed(innerSpeed);
  rightStepper.setSpeed(outerSpeed);

  while (abs(rightStepper.currentPosition()) != abs(outerSteps)) {
    double currentRatio = -leftStepper.currentPosition() / (double)rightStepper.currentPosition();

    // Ajustez les vitesses dynamiquement pour respecter le ratio
    if (currentRatio > ratio) innerSpeed += 1;
    else if (currentRatio < ratio) innerSpeed -= 1;

    // Appliquez la nouvelle vitesse à la roue intérieure
    leftStepper.setSpeed(innerSpeed);

    leftStepper.runSpeed();
    rightStepper.runSpeed();
  }
}


// Tracer un carré
inline void drawSquare(const float sideLength) {
  for (int i = 0; i < 4; i++) {
    pen(true);
    move(sideLength);
    pen(false);
    turn(90);
  }
}

// Tracer un polygone régulier
inline void drawPolygon(const float sidesNumber, const float sideLength, const float sidesAmount) {
  for (int i = 0; i < sidesAmount; i++) {
    pen(true);
    move(sideLength);
    pen(false);
    turn(360 / sidesNumber);
  }
}







inline void drawVitruvian2(const float scale) {

  // CERCLE
  pen(true);
  drawCircle(1.2 * scale);
  pen(false);

  // Récupérer l'angle avant carré
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  float initialAngle = 360 - euler.x();

  // CARRé
  pen(true);
  move(scale);
  turn(90);
  move(2*scale);
  turn(90);
  move(2*scale);
  turn(90);
  move((2 -0.025) * scale); // CORRECTION
  turn(90);
  move(scale);

  pen(false);
  
  // Récupérer l'angle après carré
  euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  float actualAngle = 360 - euler.x();
  turn(90 + (initialAngle - actualAngle)); // CORRECTION
  move(scale);
  
  // Buste
  pen(true);
  move((0.75 -0.0625) * scale); // CORRECTION
  pen(false);
  
  turn(-90);
  
  // Récupérer l'angle avant tête
  euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  initialAngle = 360 - euler.x();
  // TÊTE
  pen(true);
  drawCircle(0.125 * scale);
  pen(false);
  // Récupérer l'angle après tête
  euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  actualAngle = 360 - euler.x();

  turn(-90 + (initialAngle - actualAngle)); // CORRECTION
  // COU
  move(0.125 * scale);

  // Récupérer l'angle avant bras
  euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  initialAngle = 360 - euler.x();
  turn(90);
  
  // BRAS
  for (int i = -1; i <= 1; i+=2) {
    pen(true);
    move(scale * (1 + i*0.0125)); // CORRECTION
    pen(false);
    turn(i * (79.72192)); // 1.39141 rad
    move(-0.35685 * scale);
    turn(i * (79.72192 - 3*(i==-1))); // 1.39141 rad CORRECTION
    pen(true);
    move(scale);
    pen(false);
    if (i == -1) turn(-20.55601 +1); // -0.35877 rad CORRECTION
  }

  // Récupérer l'angle après bras
  euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  actualAngle = 360 - euler.x();

  turn(initialAngle - actualAngle); // -69.44420-8.5 ; -1.21203 rad // CORRECTION
  move((0.625 -0.075) * scale); // CORRECTION
  turn(-33.68992); // -0.588 rad
  
  // Jambes
  for (int i = -1; i <= 1; i+=2) {
    pen(true);
    move(-i * scale);
    pen(false);
    turn(i * 78.82009); // 1.37567 rad
    move(i * 0.38778 * scale);
    turn(i * 78.82009); // 1.37567 rad
    pen(true);
    move(-i * scale);
    if (i == -1) turn(44.71935); // 0.7805 rad
  }

  pen(false);
}












// Dessiner l'homme vitruve en bonhomme bâton (liste d'instructions)
inline void drawVitruvian(const float scale) {
  // Cadre
  pen(true);
  //drawSquare(2 * scale);
  for (int i = 0; i < 4; i++) {
    pen(true);
    move((2 -(i==4)*0.025) * scale); // -CORRECTION
    pen(false);
    turn(90);
  }
  pen(false);
  move(scale);
  pen(true);
  drawCircle(1.2 * scale);
  pen(false);
  
  turn(90 -0); // CORRECTION
  move(scale);
  
  // Buste
  pen(true);
  move((0.75 -0.0625) * scale); // -CORRECTION
  pen(false);
  
  turn(-90);
  
  // Tête
  pen(true);
  drawCircle(0.125 * scale);
  pen(false);
  
  turn(-90 +9); // CORRECTION
  move(0.125 * scale);
  turn(90);
  
  // BRAS
  for (int i = -1; i <= 1; i+=2) {
    pen(true);
    move(scale * (1 - (i==-1)*0.0125));
    pen(false);
    turn(i * (79.72192)); // 1.39141 rad
    move(-0.35685 * scale);
    turn(i * (79.72192 - 3*(i==-1))); // 1.39141 rad -CORRECTION
    pen(true);
    move(scale);
    pen(false);
    if (i == -1) turn(-20.55601 +2); // -0.35877 rad +CORRECTION
  }

  turn(-69.44420 -8.5); // -1.21203 rad // +CORRECTION
  move((0.625 -0.075) * scale); // -CORRECTION
  turn(-33.68992 +1); // -0.588 rad // CORRECTION
  
  // JAMBES
  for (int i = -1; i <= 1; i+=2) {
    pen(true);
    move(-i * scale);
    pen(false);
    turn(i * 78.82009); // 1.37567 rad
    move(i * 0.38778 * scale);
    turn(i * 78.82009); // 1.37567 rad
    pen(true);
    move(-i * scale);
    pen(false);
    if (i == -1) turn(44.71935); // 0.7805 rad
  }

  move(2.0 * scale);
}




// ========================== //
// ====== EXPéRIMENTAL ====== //
// ========================== //

// Turn but without PID
void fastTurn(float angle, float empiricErr) {
  // Convertir l'angle en radians
  float angleRad = (angle + empiricErr) * (M_PI / 180.0);

  // Calcul de la distance que chaque roue doit parcourir
  // Circonférence décrite par chaque roue en fonction de l'angle
  float turnCircumference = ROBOT_TRACK / 2.0 * angleRad; // Longueur de l'arc pour la rotation

  // Nombre de pas nécessaires pour chaque roue
  long wheelSteps = (4096 * turnCircumference) / (2.0 * M_PI * WHEEL_RADIUS);

  // Déterminer les directions de rotation
  int direction = (angle >= 0) ? 1 : -1;

  // Configurer les positions initiales des moteurs
  leftStepper.setCurrentPosition(0);
  rightStepper.setCurrentPosition(0);

  // Configurer les vitesses des roues (opposées pour tourner)
  leftStepper.setSpeed(direction * MAX_SPEED); // Moteur gauche inversé
  rightStepper.setSpeed(direction * MAX_SPEED);

  // Faire tourner les roues simultanément
  while (abs(leftStepper.currentPosition()) < abs(wheelSteps)) {
    leftStepper.runSpeed();
    rightStepper.runSpeed();
  }
}

void drawParametricCurve(float (*xFunc)(float), float (*yFunc)(float), float tStart, float tEnd, float tStep, float empiricErr) {
    float t = tStart;
    float prevX = xFunc(t), prevY = yFunc(t);
    t += tStep;

    // Initial next point to calculate the first angle
    float nextX = xFunc(t), nextY = yFunc(t);
    float dx = nextX - prevX;
    float dy = nextY - prevY;
    float prevAngle = atan2(dy, dx) * 180.0 / M_PI;

    // Output the first point
    move(sqrt(dx * dx + dy * dy));

    prevX = nextX;
    prevY = nextY;
    t += tStep;

    while (t <= tEnd) {
        nextX = xFunc(t);
        nextY = yFunc(t);

        // Calculate delta
        dx = nextX - prevX;
        dy = nextY - prevY;

        // Calculate distance and angle
        float distance = sqrt(dx * dx + dy * dy);
        float currentAngle = atan2(dy, dx) * 180.0 / M_PI;
        float turnAngle = currentAngle - prevAngle;

        // Normalize the turn angle to [-180, 180]
        while (turnAngle > 180) turnAngle -= 360;
        while (turnAngle < -180) turnAngle += 360;

        // Output the instructions
        fastTurn(turnAngle, empiricErr);
        pen(true);
        move(distance);
        pen(false);

        // Update for the next segment
        prevX = nextX;
        prevY = nextY;
        prevAngle = currentAngle;
        t += tStep;
    }
}

// Heart 0-2pi 11*10cm
float xHeart(float t) {return (16*sin(t)*sin(t)*sin(t)) / 3.0;}
float yHeart(float t) {return (13*cos(t)-5*cos(2*t)-2*cos(3*t)-cos(4*t)) / 3.0;}
// drawParametricCurve(xHeart, yHeart, 0, 2*M_PI, 0.2, 0);

// Star 0-6pi 10*10cm
float xStar(float t) {return (2*cos(t) + 5*cos(2*t/3.0))*0.8;}
float yStar(float t) {return (2*sin(t) - 5*sin(2*t/3.0))*0.8;}
// drawParametricCurve(xStar, yStar, 0, 6*M_PI+0.48, 0.5, -0.5);


endChime() {
  int melody[] = {
    659, 587, 370, 415, 
    554, 494, 294, 330, 
    494, 440, 277, 330,
    440
  };
  int durations[] = {
    8, 8, 4, 4,
    8, 8, 4, 4,
    8, 8, 4, 4,
    2
  };
  pinMode(8, OUTPUT);

  int size = sizeof(durations) / sizeof(int);

  for (int note = 0; note < size; note++) {
    //to calculate the note duration, take one second divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int duration = 1000 / durations[note];
    tone(8, melody[note], duration);

    //to distinguish the notes, set a minimum time between them.
    //the note's duration + 30% seems to work well:
    int pauseBetweenNotes = duration * 1.30;
    delay(pauseBetweenNotes);
    
    //stop the tone playing:
    noTone(8);
  }
}


#endif // FUNCTIONS_H
