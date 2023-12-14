//TURNING

void turnByDegree(int DegreesWanted) {
  //Kør så længe gyro siger man IKKE er ved den rette vinkel
  while (turnSensorUpdate() != DegreesWanted) {
    DegreesWanted < 180 ? motors.setSpeeds(-turnSpeed, (turnSpeed + turningDif)) : motors.setSpeeds(turnSpeed, (-turnSpeed - turningDif));
  }
  //For at finde nuværende position
  motors.setSpeeds(0, 0);  //gør hold
  delay(10);
  turnSensorReset();
}

//LINE DRIVING
void lineFollow(double lineDistance) {
  lineDistance += dif;
  double DrivenDistance = CalcDistance(EncoderL(), EncoderR());
  while (DrivenDistance < lineDistance) {
    DrivenDistance = CalcDistance(EncoderL(), EncoderR());

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
  //lineUp();
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


//DRIVING

void forwardByEncoder(double Distance) {
  encoderReset();
  double DrivenDistance = CalcDistance(EncoderL(), EncoderR());
  while (Distance >= DrivenDistance) {
    motors.setSpeeds(ForwardVal[0], ForwardVal[1]);
    DrivenDistance = CalcDistance(EncoderL(), EncoderR());
    Encoders();
    if (EncoderArray[0] > EncoderArray[1]) {
      ForwardVal[0] = ForwardVal[0] - MotorChange;
      ForwardVal[1] = ForwardVal[1] + MotorChange;
    } else if (EncoderArray[0] < EncoderArray[1]) {
      ForwardVal[0] = ForwardVal[0] + MotorChange;
      ForwardVal[1] = ForwardVal[1] - MotorChange;
    }
    DrivenDistance = CalcDistance(EncoderL(), EncoderR());
    delay(5);
  }
  motors.setSpeeds(0, 0);
  ForwardVal[0], ForwardVal[1] = 200;
}

double CalcDistance(double Left, double Right) {
  double DrivenDistance = (Left / (12 * 75) * wheelCirc + Right / (12 * 75) * wheelCirc) / 2;
  return DrivenDistance;
}