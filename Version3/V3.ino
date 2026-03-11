// alll proitiy_with_2_dip switch
#include <QTRSensors.h>
#include <SparkFun_TB6612.h>

char data;

// QTR Sensor
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

// --- PID CONTROL ---
float Kp = 0.09;
float Ki = 0.0001;
float Kd = 0.33;
int P, I, D, lastError = 0;

// Motor settings
int TURN_SPEED = 150;
int TURN_SPEEDr = -150;

#define DIP1 11
#define DIP2 12

const int BLACK_THRESHOLD = 700;
bool turning = false;
String turnDirection = "";
String lastSeenTurn = "";  // <-- remembers last turn direction

// Speed parameters
uint8_t maxspeeda = 150;
uint8_t maxspeedb = 150;
uint8_t basespeeda = 100;
uint8_t basespeedb = 100;

#define AIN1 3
#define BIN1 7
#define AIN2 4
#define BIN2 8
#define PWMA 5
#define PWMB 6
#define STBY 9

int buttoncalibrate = 10;

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
  qtr.setSensorPins((const uint8_t[]){A0,A1,A2,A3,A4,A5,A6,A7}, SensorCount);
  
  // Update this to wherever you moved the emitter pin! (Using 2 as a placeholder)
  qtr.setEmitterPin(2); 

  Serial.begin(9600);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(buttoncalibrate, INPUT);
  
  // ADDED: Tell the Arduino to read the DIP switches
  pinMode(DIP1, INPUT);
  pinMode(DIP2, INPUT);

  boolean Ok = false;
  while (!Ok) {
    if (digitalRead(buttoncalibrate) == HIGH) {
      delay(1000);
      calibration();
      Ok = true;
    }
  }
  brake(motor1, motor2);
}

void calibration() {
  digitalWrite(LED_BUILTIN, HIGH);
  for (uint16_t i = 0; i < 25; i++) {
    motor1.drive(50);
    motor2.drive(-50); 
    qtr.calibrate();
    delay(20);
  }
  for (uint16_t i = 0; i < 25; i++) {
    motor1.drive(-50);
    motor2.drive(50);
    qtr.calibrate();
    delay(20);
  }
  for (uint16_t i = 0; i < 25; i++) {
    motor1.drive(50);
    motor2.drive(-50);
    qtr.calibrate();
    delay(20);
  }
  for (uint16_t i = 0; i < 25; i++) {
    motor1.drive(-50);
    motor2.drive(50);
    qtr.calibrate();
    delay(20);
  }
  brake(motor1, motor2);
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  if (Serial.available()) {
    data = Serial.read();
    if (data == 'i') Kp += 0.005;
    else if (data == 'd') Kp -= 0.005;
    else if (data == 'j') Ki += 0.00005;
    else if (data == 'e') Ki -= 0.00005;
    else if (data == 'k') Kd += 0.025;
    else if (data == 'f') Kd -= 0.025;
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

    Serial.print(Kp); Serial.print(" ");
    Serial.print(Ki); Serial.print(" ");
    Serial.println(Kd);
    Serial.print(basespeeda); Serial.print(" ");
    Serial.print(maxspeeda);
  }

  // Button toggle
  if (digitalRead(buttoncalibrate) == HIGH) {
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

  // 1. DETECTION
  bool extremeLeft = (sensorValues[0] > BLACK_THRESHOLD);
  bool extremeRight = (sensorValues[7] > BLACK_THRESHOLD);
  bool centerSensors = (sensorValues[3] > BLACK_THRESHOLD || sensorValues[4] > BLACK_THRESHOLD);
  
  bool allWhite = true;
  for (uint8_t i = 0; i < SensorCount; i++) {
    if (sensorValues[i] > BLACK_THRESHOLD) {
      allWhite = false;
      break;
    }
  }

  // 2. INTERSECTION / T-JUNCTION HANDLING
  if (extremeLeft && extremeRight) {
    bool switch1 = digitalRead(DIP1);
    bool switch2 = digitalRead(DIP2);

    // PRIORITY LOGIC
    if (switch1 == HIGH && switch2 == LOW) { // Priority LEFT
      executeHardTurn(-TURN_SPEED, TURN_SPEED); 
      return;
    } 
    else if (switch1 == LOW && switch2 == HIGH) { // Priority RIGHT
      executeHardTurn(TURN_SPEED, -TURN_SPEED);
      return;
    }
    else { 
      // Straight: Just drive forward slightly to clear the intersection
      motor1.drive(basespeeda);
      motor2.drive(basespeedb);
      delay(50); 
    }
  }

  // 3. DEAD END HANDLING (180 Turn)
  if (allWhite && lastSeenTurn == "") {
    // If we lost the line and weren't in a turn, do a 180
    executeHardTurn(TURN_SPEED, -TURN_SPEED); 
    return;
  }

  // 4. NORMAL PID CALCULATIONS
  int error = position - 3500;
  P = error;
  I += error;
  I = constrain(I, -1000, 1000); // Prevent Integral Windup
  D = error - lastError;
  lastError = error;

  int motorspeed = (P * Kp) + (I * Ki) + (D * Kd);
  int motorspeeda = baseA + motorspeed;
  int motorspeedb = baseB - motorspeed;

  motorspeeda = constrain(motorspeeda, 0, maxspeeda);
  motorspeedb = constrain(motorspeedb, 0, maxspeedb);

  // 5. 90-DEGREE TURN HANDLING
  if (extremeLeft && !extremeRight) {
    lastSeenTurn = "left";
    turning = true;
    turnDirection = "left";
  } else if (extremeRight && !extremeLeft) {
    lastSeenTurn = "right";
    turning = true;
    turnDirection = "right";
  }

  if (turning) {
    if (centerSensors) {
      turning = false;
      lastSeenTurn = "";
    } else {
      if (turnDirection == "left") {
        motor1.drive(-TURN_SPEED/2);
        motor2.drive(TURN_SPEED);
      } else {
        motor1.drive(TURN_SPEED);
        motor2.drive(-TURN_SPEED/2);
      }
      return;
    }
  }

  forward_brake(motorspeeda, motorspeedb);
}

// Helper function to handle the "while" loops safely
void executeHardTurn(int speed1, int speed2) {
  motor1.drive(speed1);
  motor2.drive(speed2);
  delay(275); // Short blind turn to clear the current line (TO BE TUNED!!!)

  unsigned long startTurnTime = millis();
  while (millis() - startTurnTime < 1000) { // 1 second timeout safety
    qtr.readLineBlack(sensorValues);
    if (sensorValues[3] > BLACK_THRESHOLD || sensorValues[4] > BLACK_THRESHOLD) {
      break; 
    }
  }
  
  // Brake briefly to stabilize
  motor1.drive(-speed1/2); 
  motor2.drive(-speed2/2);
  delay(30);
  brake(motor1, motor2);
  
  // Reset PID
  P = I = D = lastError = 0;
}
