//usedforpathfinder_no acute turning 
#include <QTRSensors.h>
#include <SparkFun_TB6612.h>
#include <SoftwareSerial.h>

SoftwareSerial HC05(2, 13); // RX, TX
char data;

// QTR Sensor
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

// PID
float Kp = 0.09;
float Ki = 0.0001;
float Kd = 0.33;
int P, I, D, lastError = 0;

// Motor settings
int TURN_SPEED = 200;
const int BLACK_THRESHOLD = 700;
bool turning = false;
String turnDirection = "";

// Speed parameters
uint8_t maxspeeda = 225;
uint8_t maxspeedb = 225;
uint8_t basespeeda = 175;
uint8_t basespeedb = 175;

#define AIN1 3
#define BIN1 7
#define AIN2 4
#define BIN2 8
#define PWMA 5
#define PWMB 6
#define STBY 9

int buttoncalibrate = 10;
int buttonstart = 11;

const int offsetA = 1;
const int offsetB = 1;
Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

// Ramp control
uint8_t currentSpeedA = 0;
uint8_t currentSpeedB = 0;
const uint8_t rampStep = 5;
const uint16_t rampDelay = 15;
unsigned long lastRampTime = 0;

boolean onoff = false;

void setup() {
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);
  qtr.setEmitterPin(12);

  HC05.begin(9600);
  Serial.begin(9600);
  
  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(buttoncalibrate, INPUT);
  pinMode(buttonstart, INPUT);

  boolean Ok = false;
  while (!Ok) {
    if (digitalRead(buttoncalibrate) == HIGH) {
      delay(2000);
      calibration();
      Ok = true;
    }
  }
  brake(motor1, motor2);
}

void calibration() {
  digitalWrite(LED_BUILTIN, HIGH);
  for (uint16_t i = 0; i < 50; i++) {
    motor1.drive(25);
    motor2.drive(-25);
    qtr.calibrate();
    delay(20);
  }
  for (uint16_t i = 0; i < 50; i++) {
    motor1.drive(-25);
    motor2.drive(25);
    qtr.calibrate();
    delay(20);
  }
  brake(motor1, motor2);
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  // Bluetooth input
  if (HC05.available()) {
    data = HC05.read();
    if (data == 'i') Kp += 0.005;
    else if (data == 'd') Kp -= 0.005;
    else if (data == 'j') Ki += 0.00005;
    else if (data == 'e') Ki -= 0.00005;
    else if (data == 'k') Kd += 0.01;
    else if (data == 'f') Kd -= 0.01;
    else if (data == 'l') {
      maxspeeda += 25; maxspeedb += 25;
      basespeeda += 25; basespeedb += 25;
    }
    else if (data == 'g') {
      maxspeeda -= 25; maxspeedb -= 25;
      basespeeda -= 25; basespeedb -= 25;
    }
    else if (data == 's') {
      onoff = false;
      P = I = D = 0;
    }
    else if (data == 'S') {
      onoff = true;
      currentSpeedA = 0;
      currentSpeedB = 0;
      lastRampTime = millis();
    }
    else if (data == 't') TURN_SPEED += 10;
    else if (data == 'T') TURN_SPEED -= 10;

    HC05.print(Kp); HC05.print(" ");
    HC05.print(Ki); HC05.print(" ");
    HC05.println(Kd);
    HC05.print(basespeeda); HC05.print(" ");
    HC05.print(maxspeeda);
  }

  // Button toggle
  if (digitalRead(buttonstart) == HIGH) {
    onoff = !onoff;
    delay(500);
    if (onoff) {
      currentSpeedA = 0;
      currentSpeedB = 0;
      lastRampTime = millis();
    }
  }

  if (onoff) {
    // Gradually increase speed
    if (millis() - lastRampTime >= rampDelay) {
      lastRampTime = millis();
      if (currentSpeedA < basespeeda) currentSpeedA += rampStep;
      if (currentSpeedB < basespeedb) currentSpeedB += rampStep;
    }
    PID_control(currentSpeedA, currentSpeedB);
  } else {
    brake(motor1, motor2);
  }
}

void forward_brake(int posa, int posb) {
  motor1.drive(posa);
  motor2.drive(posb);
}

void PID_control(uint8_t baseA, uint8_t baseB) {
  uint16_t position = qtr.readLineBlack(sensorValues);
  int error = 3500 - position;

  P = error;
  I += error;
  D = error - lastError;
  lastError = error;

  int motorspeed = P * Kp + I * Ki + D * Kd;
  int motorspeeda = baseA + motorspeed;
  int motorspeedb = baseB - motorspeed;

  motorspeeda = constrain(motorspeeda, 0, maxspeeda);
  motorspeedb = constrain(motorspeedb, 0, maxspeedb);

  qtr.read(sensorValues);

  bool lineCentered = (sensorValues[3] > BLACK_THRESHOLD || sensorValues[4] > BLACK_THRESHOLD);
  bool leftTurn = (sensorValues[0] > BLACK_THRESHOLD || sensorValues[1] > BLACK_THRESHOLD);
  bool rightTurn = (sensorValues[6] > BLACK_THRESHOLD || sensorValues[7] > BLACK_THRESHOLD);

  if (turning) {
    if (lineCentered) {
      turning = false;
      turnDirection = "";
    } else {
      if (turnDirection == "left") {
        motor1.drive(TURN_SPEED);
        motor2.drive(-(TURN_SPEED*0.30));
      } else if (turnDirection == "right") {
        motor1.drive(-(TURN_SPEED*0.30));
        motor2.drive(TURN_SPEED);
      }
      return;
    }
  }

  if (leftTurn && !rightTurn) {
    motor1.drive(-30);
    motor2.drive(-30);
    delay(50);
    turning = true;
    turnDirection = "left";
  } else if (rightTurn && !leftTurn) {
    motor1.drive(-30);
    motor2.drive(-30);
    delay(50);
    turning = true;
    turnDirection = "right";
  } else {
    forward_brake(motorspeeda, motorspeedb);
  }
}
