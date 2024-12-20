#include <ESP32Encoder.h>
#include <globals.h>
#include <math.h>

// #define MAX_CAMBER 6           // Maximum allowable camber angle in degrees
// #define MIN_CAMBER -6           // Minimum allowable camber angle in degrees

// Motor Variables
const int thetaMax = 8400;  // encoder counts per 1 revolution of output shaft
int a = -120;          // encoder ticks per angle // TODO
int b = 2048;         // encoder ticks for zero  // TODO
int thetaDes = 0;
int error = 0;
int sumError = 0;
int lastError = 0;

// Motor Control Variables
double Kp = 0.4;  // TODO
double Ki = 0.05;
double Kd = -0.05;
double X;

// // Desired Camber Angles
// double straightCamber = 0;
// double eLeft = 3;
// double eRight = -3;
// double apexLeft = 6;
// double apexRight = -6;
// double desiredCamberAngle = straightCamber;


// PWM Properties
const int MAX_PWM_VOLTAGE = 255;
const int NOM_PWM_VOLTAGE = 150;

// IMU adjustment variables: banked turn --> negative compensation
// Assume 36 degree max banking angle, max compensation of 2 degrees
double compPerDegreeBank = -0.2;

// Global Variables
int theta = 0;

// Function to compute the desired camber actuation
double computeDesiredActuationAngleWithIMU() {
    /** Compute desired actuation angle with IMU data 
     * Banked turns require LESS active camber control
     * In the flat surface case, actuation is 3 degrees on turn entry and 6 degrees on APEX
     * Split this into 3 cases:
     * 1. rollAngle < 15 : no negative compensation
     * 2. 15 <= rollAngle < 30 : negative compensation required, such that entry actuation ranges from [0, 3] and 
     *                           apex actuation ranges from [3, 6]
     * 3. 30 <= rollAngle: extreme negative compensation required, 0 degrees. 
    */
    double desiredCamberAngle = 0;

    if (currentState == STRAIGHT) {
        desiredCamberAngle = 0;
    }

    else if (currentState == ENTRY_EXIT) 
    {
        if (abs(rollAngle) < 15) { // no negative compensation 
            desiredCamberAngle = (TurningLeft()) ? eLeft : eRight;
        }

        else if (15 <= abs(rollAngle) < 30) { 
            // Compensation, results in 0 < desiredCamberAngle < 3. 
            // e.g. rollAngle = 18, compensation = 0.6 degrees, desiredCamberAngle = eLeft - 0.6
            double compensation = compPerDegreeBank * (((double) abs(rollAngle)) - 15);
            Serial.println(compensation);
            desiredCamberAngle = (TurningLeft()) ? eLeft + compensation : eRight - compensation;
        }
        else if (abs(rollAngle) > 30) { // extreme compensation, results in 0 
            desiredCamberAngle = 0;
        }
    }
    else if (currentState == APEX) 
    {
        if (abs(rollAngle) < 15) { // no negative compensation 
            desiredCamberAngle = (TurningLeft()) ? apexLeft : apexRight;
        }
        else if (15 <= abs(rollAngle) <= 30) { 
            // Compensation, results in 3 < desiredCamberAngle < 6. 
            // e.g. rollAngle = 18, compensation = 0.6 degrees, desiredCamberAngle = apexLeft - 0.6
            double compensation = compPerDegreeBank * (((double) abs(rollAngle)) - 15);
            Serial.println(compensation);
            desiredCamberAngle = (TurningLeft()) ? apexLeft + compensation : apexRight - compensation;
        }
        else if (30 <= abs(rollAngle)) { // extreme compensation, results in 0 
            desiredCamberAngle = 0;
        }
    }

    // Clamp the camber angle to the allowable range
    if (desiredCamberAngle > MAX_CAMBER) {
        desiredCamberAngle = MAX_CAMBER;
    } else if (desiredCamberAngle < MIN_CAMBER) {
        desiredCamberAngle = MIN_CAMBER;
    }

    return desiredCamberAngle;
}

void controlMotor(double desiredActuationAngle) {
  // Update theta from encoder count delta
  theta += count;
  // Convert desired camber angle to desired theta
  thetaDes = desiredActuationAngle * a + b;

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

  if (currentState == OFF) {
    ledcWrite(M1_IN_A, LOW);
    ledcWrite(M1_IN_B, LOW);
    ledcWrite(M1_PWM, 0);
    return;
  }

  // Map X to motor direction
  if (X > 0) {
    ledcWrite(M1_IN_A, LOW);
    ledcWrite(M1_IN_B, MAX_PWM_VOLTAGE);
    ledcWrite(M1_PWM, X);
  } else if (X < 0) {
    ledcWrite(M1_IN_A, MAX_PWM_VOLTAGE);
    ledcWrite(M1_IN_B, LOW);
    ledcWrite(M1_PWM, -X);
  } else {
    ledcWrite(M1_IN_A, LOW);
    ledcWrite(M1_IN_B, LOW);
    ledcWrite(M1_PWM, 0);
  }
  lastError = error;
}