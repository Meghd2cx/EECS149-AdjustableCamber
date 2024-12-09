#include <Arduino.h>
#include <ESP32Encoder.h>

#define M1_IN_A 12
#define M1_IN_B 27
#define M1_PWM 33
#define M1_SENSOR_A 25
#define M1_SENSOR_B 34
#define STR_POS 26      
#define BTN 4

ESP32Encoder encoder;

// State Machine States
enum State{
  OFF,
  STRAIGHT,
  ENTRY_EXIT,
  APEX
} currentState = OFF;
bool ACC_Enable = false;

// Steering Constants & Variables
const int straightOffset = 2047;
const int maxStraightError = 500;
const double maxSteadyError = 10;
int strPos = 0;
const int windowSize = 5;
int strValues[windowSize];
double strSpeed = 0;

// Desired Camber Angles
double straightCamber = -0.5;
double eLeft = 1;
double eRight = -1;
double apexLeft = 2;
double apexRight = -2;
double camberAngle = straightCamber;

// Motor Variables
const int thetaMax = 8400; // encoder counts per 1 revolution of output shaft
const int a = 200; // encoder ticks per angle // TODO
const int b = 2100; // encoder ticks for zero  // TODO
int theta = 0;
int thetaDes = 0;
int error = 0;
int sumError = 0;
int lastError = 0;

double Kp = 0.5; // TODO
double Ki = 0;
double KiMax = 10;
double Kd = 0;
double X;

// Interrupt Variables
volatile bool buttonPressed = false;
volatile int count = 0;
volatile bool deltaT = false;     // check timer interrupt
hw_timer_t * timer0 = NULL;
portMUX_TYPE timerMux0 = portMUX_INITIALIZER_UNLOCKED;
const int timerFreq = 1000000; // 1 us / 1MHz
const int alarmFreq = 100000; // 10,000 * 1 us = 10 ms

// PWM Properties
const int freq = 5000;
const int resolution = 8;
const int MAX_PWM_VOLTAGE = 255;
const int NOM_PWM_VOLTAGE = 150;

// Interrupt Services
void IRAM_ATTR isr_btn() {  // the function to be called when interrupt is triggered
  buttonPressed = true;
}

void IRAM_ATTR onTime0() {
  portENTER_CRITICAL_ISR(&timerMux0);
  count = encoder.getCount();
  encoder.clearCount();
  deltaT = true; // the function to be called when timer interrupt is triggered
  portEXIT_CRITICAL_ISR(&timerMux0);
}

void setup() {
  // Put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(M1_IN_A, OUTPUT);
  pinMode(M1_IN_B, OUTPUT);
  digitalWrite(M1_IN_A, LOW);   // Set initial state of motor to off
  digitalWrite(M1_IN_B, LOW);
  pinMode(M1_PWM, OUTPUT);
  pinMode(M1_SENSOR_A, INPUT);
  pinMode(M1_SENSOR_B, INPUT);
  pinMode(STR_POS, INPUT);

  ESP32Encoder::useInternalWeakPullResistors = puType::up; // Enable the weak pull up resistors
  encoder.attachHalfQuad(M1_SENSOR_A, M1_SENSOR_B); // Attach pins for use as encoder pins
  encoder.setCount(0);  // Set starting count value after attaching

  // Attach channel to GPIO pin
  ledcAttach(M1_IN_A, freq, resolution);
  ledcAttach(M1_IN_B, freq, resolution);
  ledcAttach(M1_PWM, freq, resolution);

  // Set up button
  attachInterrupt(BTN, isr_btn, RISING);

  // Initilize timers
  timer0 = timerBegin(timerFreq);  // timer 0
  timerAttachInterrupt(timer0, &onTime0); // edge (not level) triggered
  timerAlarm(timer0, alarmFreq, true, 0); // autoreload enabled, infinite reloads

  // Initialize steering values array
  for (int i = 0; i < windowSize; i++) {
    strValues[i] = 0;
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  if (deltaT) {
    portENTER_CRITICAL(&timerMux0);
    deltaT = false;
    portEXIT_CRITICAL(&timerMux0);

    // Get steering angle and update array of last 10 steering values
    strPos = analogRead(STR_POS);
    for (int i = 0; i < (windowSize - 1); i++) {
      strValues[i] = strValues[i+1];
    }
    strValues[windowSize-1] = strPos;
    updateSteeringSpeed();

    // State Machine Logic
    switch (currentState) {
      case OFF:
        if (CheckForButtonPress()) {
          camberAngle = -0.5;
          currentState = STRAIGHT;
        }
        break;
      case STRAIGHT:
        if (!CheckIfStraight()) {
          camberAngle = (TurningLeft()) ? eLeft : eRight;
          currentState = ENTRY_EXIT;
        }
        if (CheckForButtonPress()) {
          currentState = OFF;
        }
        break;
      case ENTRY_EXIT:
        if (SteeringSteady()) {
          camberAngle = (TurningLeft()) ? apexLeft : apexRight;
          currentState = APEX;
        }
        if (CheckIfStraight()) {
          camberAngle = -0.5;
          currentState = STRAIGHT;
        }
        if (CheckForButtonPress()) {
          currentState = OFF;
        }
        break;
      case APEX:
        if (ReturningToStraight()) {
          camberAngle = (TurningLeft()) ? eLeft : eRight;
          currentState = ENTRY_EXIT;
        }
        if (CheckForButtonPress()) {
          currentState = OFF;
        }
        break;
    }

    if (currentState != OFF) {
      controlMotor();
    }
    plotControlData();
  }
}

// Calculate Steering Speed
void updateSteeringSpeed() {
  double sum = 0;
  for (int i = 0; i < (windowSize-1); i++) {
    sum += double(strValues[i+1] - strValues[i]) * (timerFreq/alarmFreq);
  }
  strSpeed = sum / (windowSize-1);
}

// State Machine Checks
bool CheckForButtonPress() {
  if (buttonPressed == true) {
    buttonPressed = false;
    return true;
  } else {
    return false;
  }
}

bool CheckIfStraight() {
  return (abs(strPos-straightOffset) < maxStraightError);
}

bool TurningLeft() {
  return (strPos > straightOffset);
}

bool SteeringSteady() {
  updateSteeringSpeed();
  return (strSpeed < maxSteadyError);
}

bool ReturningToStraight() {
  updateSteeringSpeed();
  if (TurningLeft()) {
    return (strSpeed < 0);
  } else {
    return (strSpeed > 0);
  }
}

void controlMotor() {
  // Update theta from encoder count delta
  theta += count;
  // Convert desired camber angle to desired theta
  thetaDes = camberAngle * a + b;

  // Control
  error = thetaDes - theta;
  sumError += error;

  double P = Kp * error;
  double I = Ki * sumError;
  double D = Kd * (error - lastError);
  X = P + I + D;

  // Clip output & anti-windup
  if (X > MAX_PWM_VOLTAGE) {
    X = MAX_PWM_VOLTAGE;
    sumError -= error;
  } else if (X < -MAX_PWM_VOLTAGE) {
    X = -MAX_PWM_VOLTAGE;
    sumError -= error;
  }

  // Map X to motor direction
  if (X > 0) {
    ledcWrite(M1_IN_A, LOW);
    ledcWrite(M1_IN_B, MAX_PWM_VOLTAGE);
  } else if (X < 0) {
    ledcWrite(M1_IN_A, MAX_PWM_VOLTAGE);
    ledcWrite(M1_IN_B, LOW);
  } else {
    ledcWrite(M1_IN_A, LOW);
    ledcWrite(M1_IN_B, LOW);
  }
  ledcWrite(M1_PWM, X);
  lastError = error;
}

void plotControlData() {
  Serial.println("MOTOR 1 - State, StrPos, StrSpeed, Desired Angle, Motor Pos, Desired Motor Pos, PWM_Duty");
  Serial.print(stateToString(currentState));
  Serial.print(" ");
  Serial.print(strPos-straightOffset);
  Serial.print(" ");
  Serial.print(strSpeed);
  Serial.print(" ");
  Serial.print(camberAngle);
  Serial.print(" ");
  Serial.print(theta);
  Serial.print(" ");
  Serial.print(thetaDes);
  Serial.print(" ");
  Serial.println(X);
}

const char* stateToString(State state) {
  switch (state) {
      case OFF: return "OFF";
      case STRAIGHT: return "STRAIGHT";
      case ENTRY_EXIT: return "ENTRY_EXIT";
      case APEX: return "APEX";
    }
}