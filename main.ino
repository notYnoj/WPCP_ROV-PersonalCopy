#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include <Servo.h>


int minPulse = 1100;
int zeroPulse = 1500;
int maxPulseT100 = 1794;
int maxPulseT200 = 1836;


byte motorPin[] = {8, 9, 10, 11, 12, 13};
Servo fr;
Servo fl;
Servo br;
Servo bl;
Servo vr;
Servo vl;
Servo motors[] = {fr, fl, br, bl, vr, vl};

//Define input pins and corresponding value storage
int leftX = A0, leftY = A1, rightX = A2, rightY = A3, lButton = 2, rButton = 3;

int lx, ly, rx, ry, lButtonState, rButtonState;

int rcenx, rceny, lcenx, lceny;

//initialize arrays for storing motor thrust info
double thrust[6];
int pulse[6];

//components used to calculate the thrust for each motor
double dir;
double horComp[4];
double speedMod;
double yawComp[4];

double vertComp[2];
double rollComp[2];

void setup() {
  Serial.begin(9600);
  for(int m = 0; m < 6; m++){
    motors[m].attach(motorPin[m]);
    motors[m].writeMicroseconds(1500);
  }
  setCen();
  pinMode(leftX, INPUT);
  pinMode(leftY, INPUT);
  pinMode(rightX, INPUT);
  pinMode(rightY, INPUT);
  pinMode(lButton, INPUT_PULLUP);
  pinMode(rButton, INPUT_PULLUP);
  Serial.println("lcenx: " + (String)lcenx);
  Serial.println("lceny: " + (String)lceny);
  Serial.println("rcenx: " + (String)rcenx);
  Serial.println("rceny: " + (String)rceny);
  Serial.println();
  
}

void loop() {
  getIn();
  Serial.println("lx: " + (String)lx);
  Serial.println("ly: " + (String)ly);
  Serial.println("rx: " + (String)rx);
  Serial.println("ry: " + (String)ry);
  Serial.println();
  moveROV();
  Serial.println("Direction: " + (String)(dir*180/PI));
  Serial.println("Speed: " + (String)speedMod);
  for(int i = 0; i < 6; i++){
    Serial.println("Thrust: "+ (String)thrust[i] + " PW: " + (String)pulse[i]);
  }
  Serial.println();
  delay(500);
}

void moveROV(){
  getIn();
  calcDir();
  calcHorComp();
  calcSpeedMod();
  calcYawComp();
  calcVertComp();
  calcRollComp();
  updateThrust();
  updatePulse();
  motorControl();
}

double mapWithC(double value, double startLow, double startMid, double startHigh, double finalLow, double finalMid, double finalHigh){
  if(startLow <= value && startMid > value){
    return ((value-startLow)/(startMid-startLow))*(finalMid-finalLow)+finalLow;
  }else if(startMid <= value && startHigh >= value){
    return ((value-startMid)/(startHigh-startMid))*(finalHigh-finalMid)+finalMid;
  }else{
    return 0.0;
  }
}

double mod(double dividend, double divisor){
  int numTimes = (int)(dividend/divisor);
  return dividend - (numTimes*divisor);
}

void getIn(){
  lx = analogRead(leftX);
  ly = 1023 - analogRead(leftY);
  rx = analogRead(rightX);
  ry = 1023 - analogRead(rightY);
  lButtonState = digitalRead(lButton);
  rButtonState = digitalRead(rButton);
}

void setCen(){
  rcenx = analogRead(rightX);
  rceny = 1023 - analogRead(rightY);
  lcenx = analogRead(leftX);
  lceny = 1023 - analogRead(leftY);
}

void calcDir(){
  double y = (double)ry - rceny;
  double x = (double)rx - rcenx;
  double ang = atan(y/x);
  if(x < 0){
    ang += PI;
  }else if(y < 0){
    ang += 2*PI;
  }
  ang -= PI/4;
  if(ang < 0){
    ang += 2*PI;
  }
  dir = ang;
}

void calcHorComp(){
  horComp[0] = sin(dir);
  horComp[1] = sin(dir);
  horComp[2] = cos(dir);
  horComp[3] = cos(dir);
}

void calcSpeedMod(){
  double yrat = mapWithC((double)ry, 0.0, (double)rceny, 1023, -1.0, 0.0, 1.0);
  double xrat = mapWithC((double)rx, 0.0, (double)rcenx, 1023, -1.0, 0.0, 1.0);
  speedMod = (double)sqrt((yrat*yrat) + (xrat*xrat));
  double dirC = mod(dir+(PI/4), PI/2);
  if(dirC <= (PI/4)){
    speedMod /= 1/(cos(dirC));
  }else{
    speedMod /= 1/(sin(dirC));
  }
}

void calcYawComp(){
  double rotVal = mapWithC((double)lx, 0.0, (double)lcenx, 1023.0, -1.0, 0.0, 1.0);
  yawComp[0] = -1*rotVal;
  yawComp[1] = rotVal;
  yawComp[2] = rotVal;
  yawComp[3] = -1*rotVal;
}

void calcVertComp(){
  vertComp[0] = mapWithC((double)ly, 0.0, (double)lceny, 1023.0, -1.0, 0.0, 1.0);
  if(-0.25 < vertComp[0] && vertComp[0] < 0.25){
    vertComp[0] = 0;
  }
  vertComp[1] = vertComp[0];
}

void calcRollComp(){
//  rollComp[0] = mapWithC((double)mx, 0.0, (double)mcenx, 1023.0, -1.0, 0.0, 1.0);
  if(lButtonState == rButtonState){
    rollComp[0] = 0;
  }else if(lButtonState == 0){
    rollComp[0] = -0.5;
  }else if(rButtonState == 0){
    rollComp[0] = 0.5;
  }
  rollComp[1] = -1*rollComp[0];
}

void updateThrust(){
  for(int i = 0; i < 4; i++){
    thrust[i] = horComp[i]*speedMod + yawComp[i];
  }
  thrust[4] = vertComp[0] + rollComp[0];
  thrust[5] = vertComp[1] + rollComp[1];
  
  for(int i = 0; i < 6; i++){
    if(thrust[i] > 1){
      thrust[i] = 1;
    }else if(thrust[i] < -1){
      thrust[i] = -1;
    }
  }
}

void updatePulse(){
  for(int i = 0; i < 6; i++){
    if(i < 4){
      pulse[i] = (int)mapWithC(thrust[i], -1, 0, 1, minPulse, zeroPulse, maxPulseT100);
    }else{
      pulse[i] = (int)mapWithC(thrust[i], -1, 0, 1, minPulse, zeroPulse, maxPulseT200);
    }
  }
}

void motorControl(){
  for(int i = 0; i < 6; i++){
    motors[i].writeMicroseconds(pulse[i]);
  }
}
