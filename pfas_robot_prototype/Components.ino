void buzzerAndDelay() {
  currentMillis = millis();
  if (currentMillis - startMillis >= 1000) {
    buzzer.playNote(NOTE_E(4), 350, 15);
    startMillis = currentMillis;
  }
}

double EncoderL() {
  double countsLeft = abs(encoders.getCountsLeft());
  countsLeft, EncoderArray[0] = (countsLeft * EncoderMultipliers[0]);
  return countsLeft;
}
double EncoderR() {
  double countsRight = abs(encoders.getCountsRight());
  countsRight, EncoderArray[1] = (countsRight * EncoderMultipliers[1]);
  return countsRight;
}
void Encoders() {
  EncoderL();
  EncoderR();
}

void encoderReset() {
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
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