//allworking_more oscillatiion
#include <QTRSensors.h>
#include <SparkFun_TB6612.h>

char data;

// QTR Sensor
QTRSensors qtr;
const uint8_t SensorCount = 8; //normally 8
uint16_t sensorValues[SensorCount];

// PID
float Kp = 0.09;
float Ki = 0.0001;
float Kd = 0.33;
int P, I, D, lastError = 0;

// Motor settings
int TURN_SPEED = 240; // MORE SPEED MORE OSCILALTION
int TURN_SPEEDr = -150;

const int BLACK_THRESHOLD = 700;
bool turning = false;
String turnDirection = "";
String lastSeenTurn = "";

// Speed parameters
uint8_t maxspeeda = 200;
uint8_t maxspeedb = 200;
uint8_t basespeeda = 150;
uint8_t basespeedb = 150;

#define AIN1 7
#define BIN1 5
#define AIN2 8
#define BIN2 4
#define PWMA 9
#define PWMB 3
#define STBY 6

int buttoncalibrate = 10;
// int buttonstart = 11;

const int offsetA = 1;
const int offsetB = 1;
Motor motor2 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor1 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

// Ramp control
uint8_t currentSpeedA = 0;
uint8_t currentSpeedB = 0;
const uint8_t rampStep = 5;
const uint16_t rampDelay = 15;
unsigned long lastRampTime = 0;

boolean onoff = false;

// Dashed line protection
unsigned long lastWhiteTime = 0;
const unsigned long WHITE_TIMEOUT = 50;

void setup() 
{
    qtr.setTypeAnalog();    
    qtr.setSensorPins((const uint8_t[]){A7, A6, A3, A2, A1, A0}, SensorCount);

    // qtr.setSensorPins((const uint8_t[]){A7, A6, A5, A4, A3, A2, A1, A0}, SensorCount);
    qtr.setEmitterPin(2);

    Serial.begin(9600);

    delay(500);

    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(buttoncalibrate, INPUT_PULLUP);
    // pinMode(buttonstart, INPUT);


    boolean Ok = false;

    while (!Ok) 
    {
        if (digitalRead(buttoncalibrate) == LOW) 
        {
            delay(1000);
            calibration();
            Ok = true;
        }
    }

    brake(motor1, motor2);
}

void calibration() 
{
    digitalWrite(LED_BUILTIN, HIGH);

    for (uint16_t i = 0; i < 25; i++) 
    {
        motor1.drive(50);
        motor2.drive(-50);
        qtr.calibrate();
        delay(20);
    }

    for (uint16_t i = 0; i < 25; i++) 
    {
        motor1.drive(-50);
        motor2.drive(50);
        qtr.calibrate();
        delay(20);
    }

    for (uint16_t i = 0; i < 25; i++) 
    {
        motor1.drive(50);
        motor2.drive(-50);
        qtr.calibrate();
        delay(20);
    }

    for (uint16_t i = 0; i < 25; i++) 
    {
        motor1.drive(-50);
        motor2.drive(50);
        qtr.calibrate();
        delay(20);
    }

    brake(motor1, motor2);
    digitalWrite(LED_BUILTIN, LOW);
}

void loop() 
{
    if (Serial.available()) 
    {
        data = Serial.read();

        if (data == 'i') 
        {
            Kp += 0.005;
        } 
        else if (data == 'd') 
        {
            Kp -= 0.005;
        } 
        else if (data == 'j') 
        {
            Ki += 0.00005;
        } 
        else if (data == 'e') 
        {
            Ki -= 0.00005;
        } 
        else if (data == 'k') 
        {
            Kd += 0.01;
        } 
        else if (data == 'f') 
        {
            Kd -= 0.01;
        } 
        else if (data == 'l') 
        {
            maxspeeda += 25;
            maxspeedb += 25;
            basespeeda += 25;
            basespeedb += 25;
        } 
        else if (data == 'g') 
        {
            maxspeeda -= 25;
            maxspeedb -= 25;
            basespeeda -= 25;
            basespeedb -= 25;
        } 
        else if (data == 's') 
        {
            onoff = false;
            P = I = D = 0;
        } 
        else if (data == 'S') 
        {
            onoff = true;
            currentSpeedA = 0;
            currentSpeedB = 0;
            lastRampTime = millis();
        } 
        else if (data == 't') 
        {
            TURN_SPEED += 10;
        } 
        else if (data == 'T') 
        {
            TURN_SPEED -= 10;
        }

        Serial.print(Kp);
        Serial.print(" ");
        Serial.print(Ki);
        Serial.print(" ");
        Serial.println(Kd);
        Serial.print(basespeeda);
        Serial.print(" ");
        Serial.print(maxspeeda);
    }

    if (digitalRead(buttoncalibrate) ==LOW ) 
    {
        onoff = !onoff;
        delay(500);

        if (onoff) 
        {
            currentSpeedA = 0;
            currentSpeedB = 0;
            lastRampTime = millis();
        }
    }

    if (onoff) 
    {
        if (millis() - lastRampTime >= rampDelay) 
        {
            lastRampTime = millis();

            if (currentSpeedA < basespeeda) 
            {
                currentSpeedA += rampStep;
            }

            if (currentSpeedB < basespeedb) 
            {
                currentSpeedB += rampStep;
            }
        }

        PID_control(currentSpeedA, currentSpeedB);
    } 
    else 
    {
        brake(motor1, motor2);
    }
}

void forward_brake(int posa, int posb) 
{
    motor1.drive(posa);
    motor2.drive(posb);
}

void PID_control(uint8_t baseA, uint8_t baseB) 
{
    uint16_t position = qtr.readLineBlack(sensorValues);

    int error = position - 3500;
    P = error;
    I += error;
    D = error - lastError;
    lastError = error;

    int motorspeed = P * Kp + I * Ki + D * Kd;
    int motorspeeda = baseA + motorspeed;
    int motorspeedb = baseB - motorspeed;

    motorspeeda = constrain(motorspeeda, 0, maxspeeda);
    motorspeedb = constrain(motorspeedb, 0, maxspeedb);

    bool lineCentered = (sensorValues[3] > BLACK_THRESHOLD || sensorValues[4] > BLACK_THRESHOLD);
    bool leftExtreme  = (sensorValues[0] > BLACK_THRESHOLD || sensorValues[1] > BLACK_THRESHOLD);
    bool rightExtreme = (sensorValues[6] > BLACK_THRESHOLD || sensorValues[7] > BLACK_THRESHOLD);

    bool allWhite = true;
    for (uint8_t i = 0; i < SensorCount; i++) 
    {
        if (sensorValues[i] > BLACK_THRESHOLD) 
        {
            allWhite = false;
        }
    }

    if (turning) 
    {
        if (lineCentered) 
        {
            turning = false;
            turnDirection = "";
        } 
        else 
        {
            if (turnDirection == "left") 
            {
                motor1.drive(-(TURN_SPEED / 2));
                motor2.drive(TURN_SPEED);
            } 
            else if (turnDirection == "right") 
            {
                motor1.drive(TURN_SPEED);
                motor2.drive(-(TURN_SPEED / 2));
            }
            return;
        }
    }

    if (leftExtreme && !rightExtreme) 
    {
        lastSeenTurn = "left";
        turning = true;
        turnDirection = "left";
    } 
    else if (rightExtreme && !leftExtreme) 
    {
        lastSeenTurn = "right";
        turning = true;
        turnDirection = "right";
    } 
    else if (allWhite && !lineCentered && lastSeenTurn != "") 
    {
        if (millis() - lastWhiteTime > WHITE_TIMEOUT) 
        {
            turning = true;
            turnDirection = lastSeenTurn;
        }
    } 
    else 
    {
        lastWhiteTime = millis();
        forward_brake(motorspeeda, motorspeedb);
    }
  }
