// Handles interfacing with LinPot and IMU
// Filters applied here 
// Populates global variables 

#include <ESP32Encoder.h>
#include <ESP32-TWAI-CAN.hpp>
#include <globals.h>
#include <math.h>

#define STR_POS 26
#define BTN 39
#define LED_RIGHT_GREEN 15
#define LED_RIGHT_YELLOW 32
#define LED_STRAIGHT_RED 14
#define LED_LEFT_YELLOW 20
#define LED_LEFT_GREEN 22
#define NUM_SAMPLES_ROLL 10


// Steering Constants & Variables
const int straightOffset = 2047;
const int maxStraightError = 500;
const double maxSteadyError = 70;
const double minSteadyError = 0;
const int windowSize = 5;
int strValues[windowSize];

// Global Variables
int strPos = 0;
int strSpeed = 0;
float IMU_accel[3] = {0, 0, 0}; //Stores in g's
float x_g = 0;
float y_g = 0;
float z_g = 0;
float IMU_ang_accel[3] = {0, 0, 0}; //Stores in degrees per second
float rollAngleBuffer[NUM_SAMPLES_ROLL] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
unsigned long receivedRollTime = 0;
int bufferIndex = 0;
int sampleCount = 0;


// IMU data variables
float angularVelocityx = 0;
float rollAngle = 0;
float rollAngleGyro = 0;
float rollAngleAccel = 0;
float prevRollTime = 0;

// Motor Control constants
const int freq = 5000;
const int resolution = 8;

void initialize_pinouts() {
  
  Serial.begin(115200);

  pinMode(M1_IN_A, OUTPUT);
  pinMode(M1_IN_B, OUTPUT);
  pinMode(M1_PWM, OUTPUT);

  digitalWrite(M1_IN_A, LOW);   // Set initial state of motor to off
  digitalWrite(M1_IN_B, LOW);

  pinMode(M1_SENSOR_A, INPUT);
  pinMode(M1_SENSOR_B, INPUT);
  pinMode(STR_POS, INPUT);

  digitalWrite(M1_IN_A, LOW);
  digitalWrite(M1_IN_B, LOW);


  ESP32Encoder::useInternalWeakPullResistors = puType::up; // Enable the weak pull up resistors
  encoder.attachHalfQuad(M1_SENSOR_A, M1_SENSOR_B); // Attach pins for use as encoder pins
  encoder.setCount(2048);  // Set starting count value after attaching

  // Attach channel to GPIO pin
  ledcAttach(M1_IN_A, freq, resolution);
  ledcAttach(M1_IN_B, freq, resolution);
  ledcAttach(M1_PWM, freq, resolution);

  ledcWrite(M1_IN_A, LOW);
  ledcWrite(M1_IN_B, LOW);
  ledcWrite(M1_PWM, 0);

  // Set up button
  attachInterrupt(BTN, isr_btn, RISING);

  // Initialize steering angle array with 0's
    for (int i = 0; i < windowSize; i++) {
        strValues[i] = 0;
    }

  // Initialize LED pins
  pinMode(LED_RIGHT_GREEN, OUTPUT);
  pinMode(LED_RIGHT_YELLOW, OUTPUT);
  pinMode(LED_STRAIGHT_RED, OUTPUT);
  pinMode(LED_LEFT_YELLOW, OUTPUT);
  pinMode(LED_LEFT_GREEN, OUTPUT);

  digitalWrite(LED_RIGHT_GREEN, HIGH);
  digitalWrite(LED_RIGHT_YELLOW, HIGH);
  digitalWrite(LED_STRAIGHT_RED, HIGH);
  digitalWrite(LED_LEFT_YELLOW, HIGH);
  digitalWrite(LED_LEFT_GREEN, HIGH);
  delay(2000);

  digitalWrite(LED_RIGHT_GREEN, LOW);
  digitalWrite(LED_RIGHT_YELLOW, LOW);
  digitalWrite(LED_STRAIGHT_RED, LOW);
  digitalWrite(LED_LEFT_YELLOW, LOW);
  digitalWrite(LED_LEFT_GREEN, LOW);
}

/*
* Retrives IMU Angular acceleration data and stores in array. Returns array address if succcessful and NULL otherwise.
*/
float * updateIMUangAccel ()
{
  if (ESP32Can.readFrame(rxFrame, 1000)) {
    if (rxFrame.identifier == 0x4EC) {  
      int16_t r_x_dps = rxFrame.data[1] | (rxFrame.data[0] << 8);
      int16_t r_y_dps = rxFrame.data[3] | (rxFrame.data[2] << 8);
      int16_t r_z_dps = rxFrame.data[5] | (rxFrame.data[4] << 8);

      //Convert to degrees per second
      float x_dps = r_x_dps * 0.1;
      float y_dps = r_y_dps * 0.1;
      float z_dps = r_z_dps * 0.1;

      IMU_ang_accel[0] = x_dps;
      IMU_ang_accel[1] = y_dps;
      IMU_ang_accel[2] = z_dps;

      // Addressing floating
      if (x_dps < 2) {x_dps = 0;}

      float currentTime = millis();
      float delta_t = (currentTime - prevRollTime) / 1000;
      prevRollTime = currentTime;
      
      rollAngleGyro += (y_dps + 0.3) * delta_t;

      // Serial.println(rollAngle);
      // Serial.println(y_dps);

      return IMU_ang_accel;
    }
  }
}

float * updateIMUAccel ()
{
  if (ESP32Can.readFrame(rxFrame, 1000)) {
    if (rxFrame.identifier == 0x4ED) {
      //Read raw "g"s value
      int16_t r_x_g = rxFrame.data[1] | (rxFrame.data[0] << 8);
      int16_t r_y_g = rxFrame.data[3] | (rxFrame.data[2] << 8);
      int16_t r_z_g = rxFrame.data[5] | (rxFrame.data[4] << 8);

      //Convert to "g"s
      float x_g = r_x_g * 0.01;
      float y_g = r_y_g * 0.01;
      float z_g = r_z_g * 0.01;

      IMU_accel[0] = x_g;
      IMU_accel[1] = y_g;
      IMU_accel[2] = z_g;
      // Serial.println(x_g);

      //TODO: If implementing averaging filter, run function call here
      rollAngleAccel = -atan2(y_g, sqrt(x_g * x_g + z_g * z_g)) * (180 / M_PI);

      return IMU_accel;
    }
  }
  else
  {
    return NULL; //No data available
  }
}

// Populate variables associated with steering
void populate_steering_data() {
    /**
     * UPDATES 
     *  SteeringLinPotVoltage: voltage read
     *  SteeringSpeed: rate of change of steering angle
     *  LinPotVoltages: window used to average steering angle
     */

    // Update strValues with current LinPot steering data
    //TODO: If implementing averaging filter, run function call here
    strPos = analogRead(STR_POS);

    for (int i = 0; i < (windowSize - 1); i++) {
      strValues[i] = strValues[i + 1];
    }
    strValues[windowSize - 1] = strPos;

    // Compute and update strSpeed 
    double sum = 0;
    for (int i = 0; i < (windowSize - 1); i++) {
      sum += double(strValues[i + 1] - strValues[i]) * (timerFreq / alarmFreq);
    }
    strSpeed = sum / (windowSize - 1);
}

// Calculate Steering Speed
void updateSteeringSpeed() {
  double sum = 0;
  for (int i = 0; i < (windowSize - 1); i++) {
    sum += double(strValues[i + 1] - strValues[i]) * (timerFreq / alarmFreq);
  }
  strSpeed = sum / (windowSize - 1);
}

bool CheckIfStraight() {
  return (abs(strPos - straightOffset) < maxStraightError);
}

bool TurningLeft() {
  return (strPos > straightOffset);
}

bool SteeringSteady() {
  updateSteeringSpeed();
  return (abs(strSpeed) < maxSteadyError); // && (strSpeed > minSteadyError));
}

bool ReturningToStraight() {
  updateSteeringSpeed();
  if (TurningLeft()) {
    return (strSpeed < 0);
  } else {
    return (strSpeed > 0);
  }
}

bool CheckForButtonPress() {
  if (buttonPressed == true) {
    buttonPressed = false;
    return true;
  } else {
    return false;
  }
}

void setLEDPIN(int led) {
   switch (led) {
      case 0:
        digitalWrite(LED_RIGHT_GREEN, LOW);
        digitalWrite(LED_RIGHT_YELLOW, LOW);
        digitalWrite(LED_STRAIGHT_RED, LOW);
        digitalWrite(LED_LEFT_YELLOW, LOW);
        digitalWrite(LED_LEFT_GREEN, LOW);
        break;
      case 1:
        digitalWrite(LED_RIGHT_GREEN, HIGH);
        digitalWrite(LED_RIGHT_YELLOW, LOW);
        digitalWrite(LED_STRAIGHT_RED, LOW);
        digitalWrite(LED_LEFT_YELLOW, LOW);
        digitalWrite(LED_LEFT_GREEN, LOW);
        break;
      case 2:
        digitalWrite(LED_RIGHT_GREEN, LOW);
        digitalWrite(LED_RIGHT_YELLOW, HIGH);
        digitalWrite(LED_STRAIGHT_RED, LOW);
        digitalWrite(LED_LEFT_YELLOW, LOW);
        digitalWrite(LED_LEFT_GREEN, LOW);
        break;
      case 3:
        digitalWrite(LED_RIGHT_GREEN, LOW);
        digitalWrite(LED_RIGHT_YELLOW, LOW);
        digitalWrite(LED_STRAIGHT_RED, HIGH);
        digitalWrite(LED_LEFT_YELLOW, LOW);
        digitalWrite(LED_LEFT_GREEN, LOW);
        break;
      case 4:
        digitalWrite(LED_RIGHT_GREEN, LOW);
        digitalWrite(LED_RIGHT_YELLOW, LOW);
        digitalWrite(LED_STRAIGHT_RED, LOW);
        digitalWrite(LED_LEFT_YELLOW, HIGH);
        digitalWrite(LED_LEFT_GREEN, LOW);
        break;
      case 5:
        digitalWrite(LED_RIGHT_GREEN, LOW);
        digitalWrite(LED_RIGHT_YELLOW, LOW);
        digitalWrite(LED_STRAIGHT_RED, LOW);
        digitalWrite(LED_LEFT_YELLOW, LOW);
        digitalWrite(LED_LEFT_GREEN, HIGH);
        break;
   }
}

void plotControlData() {
  Serial.println("MOTOR 1 - State, IMUState, rollAngle, receivedRollTime, StrPos, StrSpeed, Desired Angle, Motor Pos, Desired Motor Pos, PWM_Duty");
  Serial.print(stateToString(currentState));
  Serial.print(" ");
  Serial.print(IMUstateToString(currentIMUState));
  Serial.print(" ");
  Serial.print(rollAngle);
  Serial.print(" ");
  Serial.print(receivedRollTime);
  Serial.print(" ");
  Serial.print(strPos - straightOffset);
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

void plotDebugData() {
  Serial.println();
}

const char* stateToString(State state) {
  switch (state) {
    case OFF: return "OFF";
    case STRAIGHT: return "STRAIGHT";
    case ENTRY_EXIT: return "ENTRY_EXIT";
    case APEX: return "APEX";
    case ERROR: return "ERROR";
  }
}

const char* IMUstateToString(IMUState state) {
  switch (state) {
    case FALL_BACK: return "FALL_BACK";
    case FUNCTIONAL: return "FUNCTIONAL";
  }
}
