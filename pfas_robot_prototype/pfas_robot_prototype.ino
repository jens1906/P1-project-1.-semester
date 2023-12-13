#include <Zumo32U4.h>
#include <Wire.h>

Zumo32U4Encoders encoders;
Zumo32U4Motors motors;
Zumo32U4Buzzer buzzer;
Zumo32U4IMU imu;
//Zumo32U4LCD display;
Zumo32U4OLED display;
Zumo32U4LineSensors lineSensors;
Zumo32U4ButtonA buttonA;

int countsLeft;
int countsRight;
int stage = -1;
int speed = 150;
int turnSpeed = 125;
int maxSpeed = 150;
int tour = 1;

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
double coastLineDistance[] = { 65, 64.4, 62.65, 59, 48.3, 44.1, 42.95, 43.45, 45.75, 51.25, 57, 58.93, 59.13, 58.82, 58.2, 57.05, 55, 52, 47.48, 46.67, 50 };


//Drive straight and specific distance
int lineDistanceDriven = 0;
double ForwardVal[] = { 200, 200 };
double MotorChange = 0.1;
double EncoderMultipliers[] = { 0.995, 1 };
double EncoderArray[2];
float wheelCirc = 12.45;

const int resolutionBetweenTest = 20;
const float offset = 1.5;
const int startPos = 1;
const float dif = 1.5;
const int turningDif = 15;

void setup() {
  Serial.begin(9600);
  startMillis = millis();
  delay(100);
  lineSensors.initFiveSensors();
  calibrateSensors();
  delay(100);
  turnSensorSetup();
  delay(10);
  turnSensorReset();
  encoderReset();
  buzzer.playNote(NOTE_E(4), 350, 15);
}

void loop() {
  if (buttonA.isPressed()) {
    stage = 0;
    delay(1000);
  }

  switch (stage) {
    case 0:
      nextPoint();
      break;
    case 1:
      driveAndCollect(coastLineDistance[(((lineDistanceDriven) / 5) + startPos)]);
      delay(10);
      break;
    case 2:
      driveAndContinue();
      break;
    case 3:
      sampleDropOff();
      break;
  }
}

void driveAndContinue() {
  encoderReset();
  lineFollow(lineDistanceDriven);
  delay(10);
  stage = 0;
}

void printing(int ting1, int ting2) {
  display.clear();
  display.print(ting1);
  display.gotoXY(0, 1);
  display.print(ting2);
}

void nextPoint() {
  encoderReset();
  lineDistanceDriven += (resolutionBetweenTest + offset);
  lineFollow(resolutionBetweenTest + offset);
  delay(10);
  stage = 1;
}


void sampleDropOff() {
  for (int i = 0; i < sample; i++) {
    buzzer.playNote(NOTE_E(4), 350, 15);
    delay(1000);
  }
  display.print("JAAA");
  delay(100);
  tour += 1;
  sample = 0;
  stage = 2;
}

void sampleCollect() {
  buzzer.playNote(NOTE_E(4), 350, 15);
  delay(400);
  sample += 1;
}

void driveAndCollect(double Distance) {
  turnByDegree(90);  //vinkelret venstre
  delay(1000);
  encoderReset();
  printing((((lineDistanceDriven) / 5) + startPos), coastLineDistance[(((lineDistanceDriven) / 5) + startPos)]);
  forwardByEncoder(Distance);
  sampleCollect();
  turnByDegree(175);
  encoderReset();
  printing((((lineDistanceDriven) / 5) + startPos), coastLineDistance[(((lineDistanceDriven) / 5) + startPos)]);
  forwardByEncoder(Distance);
  encoderReset();
  stage = 0;
  sample != 2 ? turnByDegree(90) : returnToBase();
}

void returnToBase() {
  print();
  turnByDegree(270);
  encoderReset();
  lineFollow(lineDistanceDriven);
  delay(100);
  turnByDegree(180);
  delay(100);
  stage = 3;
  print();
}

void print() {
  display.clear();
  display.print(stage);
  display.gotoXY(0, 1);
  display.print(sample);
}
