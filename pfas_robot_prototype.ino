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
int list[4] = { 2000, 1500, 4000, 1700 };
double coastLineDistance[] = {84, 83.4, 81.65, 78, 67.3, 63.1, 61.95, 62.45, 64.75, 70.25, 76, 77.93, 78.13, 77.82, 77.2, 76.05, 74, 71, 66.48, 65.67, 69.15, 69};


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
        lineFollow(6000);
        delay(10);
        stage = 1;
      } else {
        motors.setSpeeds(0, 0);
      }
      break;
    case 1:
      turnByDegree(90);  //vinkelret venstre
      encoderReset();
      forwardByEncoder(list[a]);
      encoderReset();
      sampleCollect();
      turnByDegree(180);
      forwardByEncoder(list[a]);
      encoderReset();
      turnByDegree(90);
      a++;
      stage = 2;
      break;
    case 2:
      if (sample != 2) {
        encoderReset();
        lineFollow(2000);
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

void buzzerAndDelay() {
  currentMillis = millis();
  if (currentMillis - startMillis >= 1000) {
    buzzer.playNote(NOTE_E(4), 350, 15);
    startMillis = currentMillis;
  }
}

void encoderReset() {
  countsLeft = encoders.getCountsAndResetLeft();
  countsRight = encoders.getCountsAndResetRight();
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
    lineFollow(8000);
  } else {
    lineFollow(4000);
  }
  delay(100);
  turnByDegree(180);
  delay(100);
  sample = 0;
  stage = 3;
}

void forwardByEncoder(int EncoderWanted) {  //kører fremad indtil gennemsnittet af højre og venstre bælte bliver størrer end EncoderWanted
  delay(100);
  motors.setSpeeds(speed, speed);
  do {
    countsLeft = encoders.getCountsLeft();
    countsRight = encoders.getCountsRight();
  } while (((countsLeft + countsRight) / 2) < EncoderWanted);
  motors.setSpeeds(0, 0);
  delay(100);
}

void turnByDegree(int DegreesWanted) {
  WantedAngle = DegreesWanted;

  //Kør så længe gyro siger man IKKE er ved den rette vinkel
  while (turnSensorUpdate() != DegreesWanted) {
    display.clear();
    display.print(turnSensorUpdate());

    //For at finde nuværende position
    int Way = turnSensorUpdate();
    ((DegreesWanted - Way) > 0) ? motors.setSpeeds(-100, 100) : motors.setSpeeds(-100, 100);  //Hvis ? Så gør : Ellers;  For at udregne korteste mulige sving (ternary operator)
  }
  motors.setSpeeds(0, 0);  //gør hold
  delay(10);
  turnSensorReset();
}

void lineFollow(int lineDistance) {
  while (((countsLeft + countsRight) / 2) < lineDistance) {
    countsLeft = encoders.getCountsLeft();
    countsRight = encoders.getCountsRight();
    int16_t position = lineSensors.readLine(lineSensorValues);
    int16_t integral = 0;
    // Our "error" is how far we are away from the center of the line, which corresponds to position 2000.
    int16_t error = position - 2000;
    integral += error;

    //int16_t speedDifference = (kp) * error + (kd) * (error - lastError);
    int16_t speedDifference = (kp)*error + (Ki)*integral + (kd) * (error - lastError);

    lastError = error;

    // Get individual motor speeds.  The sign of speedDifference
    // determines if the robot turns left or right.
    int16_t leftSpeed = (int16_t)maxSpeed + speedDifference;
    int16_t rightSpeed = (int16_t)maxSpeed - speedDifference;

    leftSpeed = constrain(leftSpeed, -200, (int16_t)maxSpeed);
    rightSpeed = constrain(rightSpeed, -200, (int16_t)maxSpeed);

    motors.setSpeeds(leftSpeed, rightSpeed);
  }
  motors.setSpeeds(0, 0);
  delay(100);
}

//Calibrate linesensors
void calibrateSensors() {
  delay(1000);
  for (uint16_t i = 0; i < 120; i++) {
    if (i > 30 && i <= 90) {
      motors.setSpeeds(-speed, speed);
    } else {
      motors.setSpeeds(speed, -speed);
    }
    lineSensors.calibrate();
  }
  motors.setSpeeds(0, 0);
}

//ALT FOR GYRO HERUNDER:
//Nulstil gyro sensor
void turnSensorReset() {
  gyroLastUpdate = micros();
  turnAngle = 0;
}

//Update gyro sensor kun med værdier fra 0-360
uint32_t turnSensorUpdate() {
  imu.readGyro();
  turnRate = imu.g.z - gyroOffset;
  uint16_t m = micros();
  uint16_t dt = m - gyroLastUpdate;
  gyroLastUpdate = m;
  int32_t d = (int32_t)turnRate * dt;
  turnAngle += (int64_t)d * 14680064 / 17578125;
  return ((((uint32_t)turnAngle >> 16) * 360) >> 16);
}

//setup gyro sensor
void turnSensorSetup() {
  Wire.begin();
  imu.init();
  imu.enableDefault();
  imu.configureForTurnSensing();
  delay(500);
  int32_t total = 0;
  for (uint16_t i = 0; i < 1024; i++) {
    while (!imu.gyroDataReady()) {}
    imu.readGyro();
    total += imu.g.z;
  }
  gyroOffset = total / 1024;
  turnSensorReset();
}
