#include <Zumo32U4.h>

Zumo32U4Encoders encoders;
Zumo32U4Motors motors;

int countsLeft;
int countsRight;

void setup() {
}

void loop() {
  forwardByEncoder();
}

void forwardByEncoder(){ //kører fremad indtil både højre og venstre bælte bliver == 2000 "hjulklik"
  countsLeft = encoders.getCountsAndResetLeft();
  countsRight = encoders.getCountsAndResetRight();
 
  delay(1000);
  motors.setSpeeds(200,200);
  do {
    countsLeft = encoders.getCountsLeft();
    countsRight = encoders.getCountsRight();
  }  while(countsLeft<2000&&countsRight<2000);   
  motors.setSpeeds(0,0);   
  delay(1000);     

  countsLeft = encoders.getCountsAndResetLeft();   
  countsRight = encoders.getCountsAndResetRight();   
  motors.setSpeeds(-200,-200);   
  do {     
    countsLeft = encoders.getCountsLeft();     
    countsRight = encoders.getCountsRight();   
  }  while(countsLeft>-2000&&countsRight>-2000);
  motors.setSpeeds(0,0);
} 

}