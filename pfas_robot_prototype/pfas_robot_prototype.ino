#include <Zumo32U4.h>
#include <Wire.h>

Zumo32U4Encoders encoders;
Zumo32U4Motors motors;
Zumo32U4Buzzer buzzer;
Zumo32U4IMU imu;
Zumo32U4LCD display;
Zumo32U4LineSensors lineSensors;
Zumo32U4ButtonA buttonA;

int countsLeft;
int countsRight;
int stage;
int speed = 150;
int maxSpeed = 150;
int tour;

int sample = 0;
unsigned long startMillis;
unsigned long currentMillis;

//VARIABLER FOR GYRO:
uint32_t turnAngle = 0;
int16_t turnRate;
int16_t gyroOffset;
uint16_t gyroLastUpdate = 0;
uint32_t CurrentAngle;
int WantedAngle = 0;
int turn = 0;

const int32_t turnAngle45 = 0x20000000;
const int32_t turnAngle90 = turnAngle45 * 2;
const int32_t turnAngle1 = (turnAngle45 + 22) / 45;

//VARIABLER FOR LINEFOLLOWER
int16_t lastError = 0;
float Kmax = 0.4;
float kp = 0.6 * Kmax;
float kd = 0.3;  //if overshoot decrease
float Ki = 0.1;

#define NUM_SENSORS 5
unsigned int lineSensorValues[NUM_SENSORS];

//VARIABLER FOR AFSTAND UD TIL VANDKANTEN
int a = 0;
//int list[4] = { 20, 15, 40, 17 };
//double coastLineDistance[] = { 84, 83.4, 81.65, 78, 67.3, 63.1, 61.95, 62.45, 64.75, 70.25, 76, 77.93, 78.13, 77.82, 77.2, 76.05, 74, 71, 66.48, 65.67, 69 };
double coastLineDistance[] = { 65, 64.4, 62.65, 59, 48.3, 44.1, 42.95, 43.45, 45.75, 51.25, 57, 58.93, 59.13, 58.82, 58.2, 57.05, 55, 52, 47.48, 46.67, 50 };


//Drive straight and specific distance
int lineDistanceDriven = 0;
double ForwardVal[] = { 200, 200 };
double MotorChange = 0.1;
double EncoderMultipliers[] = { 1, 0.9951183256 };
double EncoderArray[2];
float wheelCirc = 12.6;

void setup() {
  Serial.begin(9600);
  turnSensorSetup();
  delay(10);
  turnSensorReset();
  startMillis = millis();
  delay(100);
  lineSensors.initFiveSensors();
  calibrateSensors();
  buzzer.playNote(NOTE_E(4), 350, 15);
}

void loop() {
  if (buttonA.isPressed()) {
    stage = 2;
  }
  switch (stage) {
    case 0:
      if (tour == 1) {
        encoderReset();
        lineDistanceDriven += 20;
        lineFollow(20);
        delay(10);
        stage = 1;
      } else {
        motors.setSpeeds(0, 0);
      }
      break;
    case 1:
      turnByDegree(90);  //vinkelret venstre
      encoderReset();
      display.print(coastLineDistance[(lineDistanceDriven / 5)]);
      forwardByEncoder(coastLineDistance[((lineDistanceDriven / 5) + 2)]);
      encoderReset();
      sampleCollect();
      turnByDegree(180);
      display.print(coastLineDistance[(lineDistanceDriven / 5)]);
      forwardByEncoder(coastLineDistance[((lineDistanceDriven / 5) + 2)]);
      encoderReset();
      turnByDegree(90);
      a++;
      stage = 2;
      break;
    case 2:
      if (sample != 2) {
        encoderReset();
        lineDistanceDriven += 20;
        lineFollow(20);
        delay(10);
        stage = 1;
      } else {
        returnToBase();
      }
      break;
    case 3:
      buzzer.playNote(NOTE_E(4), 350, 15);
      delay(1000);
      buzzer.playNote(NOTE_E(4), 350, 15);
      delay(1000);
      tour += 1;
      stage = 0;
      break;
  }
}


int CalcDistance(double Left, double Right) {
  int DrivenDistance = (int)(Left / (12 * 75) * wheelCirc + Right / (12 * 75) * wheelCirc) / 2;
  return DrivenDistance;
}



void sampleCollect() {
  buzzer.playNote(NOTE_E(4), 350, 15);
  delay(400);
  sample += 1;
}

void returnToBase() {
  turnByDegree(180);
  encoderReset();
  if (tour == 1) {
    lineFollow(80);
  } else {
    lineFollow(40);
  }
  delay(100);
  turnByDegree(180);
  delay(100);
  sample = 0;
  stage = 3;
}







