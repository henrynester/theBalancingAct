/*
 * Balancing Robot Code
 * Henry Nester
 * Rev 2.1
 * 3/9/16
 */

#include <Wire.h>

#include "kalvinman.h"
#include "i2c.h"

#define CAMERA_MOUNTED 0

#define ENCODER_L_B 10
#define ENCODER_R_B 11

#define RED 12
#define GREEN 13
#define SONAR 12

#define CTRL1 4
#define CTRL2 5
#define ENA 6
#define CTRL3 7
#define CTRL4 8
#define ENB 9

#define VOLTAGE_DIVIDER A0
#define VOLTAGE_COMPUTE_CONSTANT 0.0137

Kalman kalmanX;
Kalman kalmanY;

double accX, accY, accZ, gyroX, gyroY;
double gyroXrate, gyroYrate;
double kalAngleX, kalAngleY;
double gyroOffset;

double finalAngle;
double targetAngle;

uint32_t timer;
uint8_t i2cData[14];

float batteryLevel;

int speedLOld, speedROld;
volatile int speedL;
volatile int speedR;
double robotCenterSpeed;
double robotCenterPosition;
int robotTotalTurn;
int motorPower, motorPowerL, motorPowerR;
int dist;

double desiredSpeed = 0;
double inputSpeed = 0;
double inputTurn = 0;
double inputPosition = 0;

#if CAMERA_MOUNTED
double balanceAngle = -13;
#else CAMERA_MOUNTED
double balanceAngle = -0.5;
#endif
double kPAngle = 16;
double kIAngle = 0.46;
double errSum = 0;
double kDAngle = 0.7;
double kPPosition = -2;
double kIPosition = -0.06;
//double kDSpeed = 0;

byte bluetoothReceived[7];
bool packageGood;
byte data[18];

unsigned int count = 0;
unsigned int count2 = 0;
unsigned int obstacle_counter = 0;
byte mode = 1;

#define SHUTDOWN 0
#define STAND_STILL 1
#define AUTONOMOUS 2
#define RC_CONTROL 3
#define STOP_A 4
#define REVERSE 5
#define STOP_B 6
#define TURN 7
#define STOP_C 8

void setup() {
  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  digitalWrite(RED, HIGH);
  digitalWrite(GREEN, LOW);
  Serial.begin(115200);
  Wire.begin(); TWBR = ((F_CPU / 400000) - 16) / 2;

  delay(500);
  
  i2cData[0] = 7;
  i2cData[1] = 0x00;
  i2cData[2] = 0x00;
  i2cData[3] = 0x00;

  while(i2cWrite(0x19, i2cData, 4, false));
  while(i2cWrite(0x6B, 0x01, true));

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) {
    Serial.println(F("Error 7"));
    while(true) {
      digitalWrite(RED, HIGH);
      delay(500);
      digitalWrite(RED, LOW);
      delay(500);
    }
  }
  
  delay(500);

  while(i2cRead(0x3B, i2cData, 14));
  accX = (i2cData[0] << 8) | i2cData[1];
  accY = (i2cData[2] << 8) | i2cData[3];
  accZ = (i2cData[4] << 8) | i2cData[5];

  for(int i = 0; i < 50; i++) {
    while(i2cRead(0x3B, i2cData, 14));
    gyroOffset += (i2cData[8] << 8) | i2cData[9];
  }

  gyroOffset = gyroOffset / 131.0;

  double roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;

  kalmanX.setAngle(roll);
  kalmanY.setAngle(pitch);

  attachInterrupt(0, encoderLUpdate, RISING);
  attachInterrupt(1, encoderRUpdate, RISING);

  pinMode(ENCODER_L_B, INPUT);
  pinMode(ENCODER_R_B, INPUT);

  pinMode(ENA, OUTPUT);
  pinMode(CTRL1, OUTPUT);
  pinMode(CTRL2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(CTRL3, OUTPUT);
  pinMode(CTRL4, OUTPUT);
  
  timer = micros();

  digitalWrite(RED, LOW);
  digitalWrite(GREEN, HIGH);
  delay(500);
  digitalWrite(GREEN, LOW);
  delay(500);
  digitalWrite(GREEN, HIGH);
}

void loop() {
  double angleAverage[3] = {0, 0, 0};
  if(count > 10) {
    count = 0;
    updateBatteryLevel();
    updateSonar();
    switch(mode) {
      case SHUTDOWN:
        motorPowerL = 0; motorPowerR = 0;
        robotCenterPosition = 0;
        inputPosition = 0;
        inputSpeed = 0;
        inputTurn = 0;
        break;
      case STAND_STILL: 
        inputSpeed = 0;
        inputTurn = 0;
        break;
      case AUTONOMOUS:
        updateSonar();
        //Serial.print(dist); Serial.print("\t"); Serial.println(obstacle_counter);
        inputSpeed = 1.25;
        inputTurn = 0;
        /*if(robotCenterSpeed < 0.04) {
          obstacle_counter+=2;
        }
        else if (robotCenterSpeed < 0.2) {
          obstacle_counter+=1;
        }
        else if (robotCenterSpeed > 0) {
          obstacle_counter = 0;
        }*/
        if(dist < 40 || (obstacle_counter > 75 && inputSpeed != 0)) {
          if(delayNoSleep(5)) {
            mode = STOP_A;
            obstacle_counter = 0;
          }
          //Serial.println("AUTONOMOUS done");
        }
        //Serial.println(robotCenterSpeed);
        break;
      case STOP_A:
        inputSpeed = 0;
        inputTurn = 0;
        if(delayNoSleep(5)) { 
          mode = REVERSE;
          //Serial.println("STOP_A done");
        }
        break;
      case REVERSE:
        inputSpeed = -2;
        inputTurn = 0;
        if(delayNoSleep(20)) { 
          mode = STOP_B;
          //Serial.println("STOP_A done");
        }
        break;
      case STOP_B:
        inputSpeed = 0;
        inputTurn = 0;
        if(delayNoSleep(5)) { 
          mode = TURN;
          //Serial.println("STOP_A done");
        }
        break;
      case TURN:
        inputSpeed = 0;
        inputTurn = 2;
        if(delayNoSleep(9)) {
          mode = STOP_C;
          //Serial.println("TURN done");
        }
        break;
      case STOP_C:
        inputSpeed = 0;
        inputTurn = 0;
        if(delayNoSleep(5)) {
          mode = AUTONOMOUS;
          //Serial.println("STOP_C done");
        }
        break;
    }
    count2++;
    //Serial.print(inputSpeed); Serial.print("\t"); Serial.print(dist); Serial.print("\t"); Serial.println(inputTurn);
  }
  if(updateAttitude()) {
    angleAverage[2] = angleAverage[1];
    angleAverage[1] = angleAverage[0];
    angleAverage[0] = kalAngleX;
    finalAngle = (angleAverage[0] + angleAverage[0] + angleAverage[0]) / 3;
    updatePid();
    if(abs(finalAngle) > 60 || mode == SHUTDOWN) {
      motorPowerL = 0; motorPowerR = 0;
      robotCenterPosition = 0;
      inputPosition = 0;
      inputSpeed = 0;
      inputTurn = 0;
    }
    driveMotors();
    count++;
  }
  bluetoothCommunication();
}

bool delayNoSleep(int t) {
  if(count2 > t) {
    count2 = 0;
    return true;
  }
  else {
    return false;
  }
}

bool updateAttitude() {
  if((micros() - timer) > 10000) {
    while(i2cRead(0x3B, i2cData, 14));
    accX = (i2cData[0] << 8) | i2cData[1];
    accY = (i2cData[2] << 8) | i2cData[3];
    accZ = (i2cData[4] << 8) | i2cData[5];
    gyroX = (i2cData[8] << 8) | i2cData[9];
    gyroY = (i2cData[10] << 8) | i2cData[11];

    double dt = (double)(micros() - timer) / 1000000;
    timer = micros();

    double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    double pitch = atan2(-accX, accZ) * RAD_TO_DEG;

    gyroXrate = (gyroX - gyroOffset) / 131.0;
    gyroYrate = gyroY / 131.0;

    if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
      kalmanY.setAngle(pitch);
      kalAngleY = pitch;
    } 
    else {
      kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
      if(abs(kalAngleY) > 90) {
        gyroXrate *= -1;
      }
      kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt);
    }

#if 0
    Serial.print(roll); Serial.print("\t");
    Serial.print(pitch); Serial.print("\t");
    Serial.print(gyroXrate); Serial.print("\t");
    Serial.print(gyroYrate); Serial.println("\t");
#endif

#if 0
    Serial.print(kalAngleX); Serial.print("\t"); Serial.println(gyroXrate);
#endif
    return true;
  }
  return false;
}

void updateBatteryLevel() {
  batteryLevel = analogRead(VOLTAGE_DIVIDER) * VOLTAGE_COMPUTE_CONSTANT;
  
  #if 0
  Serial.println(batteryLevel);
  #endif
}

void updateSonar() {
  digitalWrite(SONAR, LOW);
  delayMicroseconds(2);
  digitalWrite(SONAR, HIGH);
  delayMicroseconds(5);
  digitalWrite(SONAR, LOW);
  pinMode(SONAR, INPUT);
  dist = pulseIn(SONAR, HIGH, 3000) / 58;
  if(dist == 0) { dist = 50; }
  dist = constrain(dist, 0, 50);
  pinMode(SONAR, OUTPUT);
  #if 0
  Serial.println(dist);
  #endif
}

void updatePid() {
  robotCenterSpeed = (robotCenterSpeed * 0.75) + ((speedL + speedR) * 0.125);

  if(speedL == 0 || speedR == 0) {
    obstacle_counter++;
  }
  if(speedL > 0 || speedR > 0) {
    obstacle_counter = 0;
  }

  double posError = robotCenterPosition - inputPosition; //add constrain again for no camera

  desiredSpeed *= 0.99;
  desiredSpeed += 0.01 * ((posError * 0.02) + inputSpeed);

  robotCenterPosition += (speedL + speedR) * 0.5;
  robotTotalTurn += speedL - speedR;

  targetAngle = constrain((balanceAngle - (kPPosition * (robotCenterSpeed-desiredSpeed)) - (kIPosition * posError)), -40.0, 40.0); //change to 20

  errSum += finalAngle - targetAngle;
  errSum = constrain(errSum, -800.0, 800.0);

  motorPower = ((finalAngle - targetAngle) * kPAngle) + (errSum * kIAngle) + (gyroXrate * kDAngle);

  if(inputSpeed != 0) {
    inputPosition = robotCenterPosition + inputSpeed;
  }

  robotTotalTurn -= inputTurn;

  motorPowerL = motorPower + robotTotalTurn + inputTurn;
  motorPowerR = motorPower - robotTotalTurn - inputTurn;
  
  speedL = 0;
  speedR = 0;

#if 0
  Serial.print(robotCenterPosition); Serial.print("\t"); Serial.print(robotCenterSpeed); Serial.print("\t"); Serial.println(robotTotalTurn); 
#endif

#if 0
  Serial.print(targetAngle); Serial.print("\t"); Serial.println(motorPower);
#endif
}

void driveMotors() {
  if (motorPowerL > 0) {
    digitalWrite(CTRL1, LOW);
    digitalWrite(CTRL2, HIGH);
  }
  else {
    digitalWrite(CTRL1, HIGH);
    digitalWrite(CTRL2, LOW);
    motorPowerL *= -1;
  }
  
  if (motorPowerR > 0) {
    digitalWrite(CTRL3, HIGH);
    digitalWrite(CTRL4, LOW);
  }
  else {
    digitalWrite(CTRL3, LOW);
    digitalWrite(CTRL4, HIGH);
    motorPowerR *= -1;
  }

  analogWrite(ENA, motorPowerL);
  analogWrite(ENB, motorPowerR);
}

void bluetoothCommunication() {
  if(Serial.available()) {
    for(int i = 5; i > 0; i--) {
      bluetoothReceived[i] = bluetoothReceived[i-1];
    }
    bluetoothReceived[0] = Serial.read();
  }

  if(bluetoothReceived[5] == 5 && bluetoothReceived[0] == 10) {
    switch(bluetoothReceived[4]) {
      case 1:
        sendTelemetry();
        break;
      case 2:
        updatePIDParameters();
        break;
      case 3:
        savePIDParameters();
        break;
      case 4:
        if(mode == RC_CONTROL) {
          rcControl();
        }
        break;
      case 5:
        setMode();
        break;
    }
    bluetoothReceived[0] = 0; bluetoothReceived[1] = 0; bluetoothReceived[2] = 0; bluetoothReceived[3] = 0; bluetoothReceived[4] = 0; bluetoothReceived[5] = 0; 
  }
}

void sendTelemetry() {
  data[0] = int(batteryLevel*100)/256;
  data[1] = int(batteryLevel*100)%256;
  data[2] = int(robotCenterPosition*100)/256;
  data[3] = int(robotCenterPosition*100)%256;
  data[4] = int((finalAngle+90)*100)/256;
  data[5] = int((finalAngle+90)*100)%256;
  data[6] = int((motorPower)*100)/256;
  data[7] = int((motorPower)*100)%256;
  data[8] = int((balanceAngle+5)*100)/256;
  data[9] = int((balanceAngle+5)*100)%256;
  data[10] = int((kPAngle+50)*100)/256;
  data[11] = int((kPAngle+50)*100)%256;
  data[12] = int((kIAngle+1)*100)/256;
  data[13] = int((kIAngle+1)*100)%256;
  data[14] = int((kDAngle+5)*100)/256;
  data[15] = int((kDAngle+5)*100)%256;
  data[16] = int((kPPosition+5)*100)/256;
  data[17] = int((kPPosition+5)*100)%256;
  data[18] = int((kIPosition+1)*100)/256;
  data[19] = int((kIPosition+1)*100)%256;
  for(int i = 0; i < 20; i++) {
    Serial.write(data[i]);
  }
}

/*KA_P = 25.0;
  KA_D = 3.5;
  KP_P = 30;
  KP_I = 0.34;
  K_Base = 6.7;*/

void updatePIDParameters() {
  switch(bluetoothReceived[3]) {
    case 1:
      balanceAngle = constrain(((float)(bluetoothReceived[1] << 8 | bluetoothReceived[2])/100.0 - 5.0), -5.0, 5.0);
      break;
    case 2:
      kPAngle = constrain(((float)(bluetoothReceived[1] << 8 | bluetoothReceived[2])/100.0 - 50.0), -100.0, 100.0);
      break;
    case 3:
      kIAngle = constrain(((float)(bluetoothReceived[1] << 8 | bluetoothReceived[2])/100.0 - 1.0), -1.0, 1.0);
      break;
    case 4:
      kDAngle = constrain(((float)(bluetoothReceived[1] << 8 | bluetoothReceived[2])/100.0 - 5.0), -5.0, 5.0);
      break;
    case 5:
      kPPosition = constrain(((float)(bluetoothReceived[1] << 8 | bluetoothReceived[2])/100.0 - 5.0), -5.0, 5.0);
      break;
    case 6:
      kIPosition = constrain(((float)(bluetoothReceived[1] << 8 | bluetoothReceived[2])/100.0 - 1.0), -1.0, 1.0);
      break;
  }
}

void savePIDParameters() {
}

void rcControl() {
  switch(bluetoothReceived[3]) {
    case 1:
      inputSpeed = 1.5;
      break;
    case 2:
      inputSpeed = -1.5;
      break;
    case 3:
      inputTurn = -2;
      break;
    case 4:
      inputTurn = 2;
      break;
    case 5:
      inputSpeed = 0;
      inputTurn = 0;
      break;
    case 6:
      inputSpeed = constrain(((float)(bluetoothReceived[1] << 8 | bluetoothReceived[2])/100.0 - 2.0), -2.0, 2.0);
      break;
    case 7:
      inputTurn = constrain(((float)(bluetoothReceived[1] << 8 | bluetoothReceived[2])/100.0 - 3.0), -3.0, 3.0);
      break;
  }
}

void setMode() {
  mode = bluetoothReceived[3];
}

void encoderLUpdate() {
  if(digitalRead(ENCODER_L_B)) {
    speedL++;
  }
  else {
    speedL--;
  }
}

void encoderRUpdate() {
  if(digitalRead(ENCODER_R_B)) {
    speedR++;
  }
  else {
    speedR--;
  }
}
