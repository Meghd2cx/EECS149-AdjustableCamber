#ifndef GLOBALS_H  // Check if GLOBALS_H is not already defined
#define GLOBALS_H  // Define GLOBALS_H to prevent multiple inclusions

#define M1_IN_A 12
#define M1_IN_B 27
#define M1_PWM 33
#define M1_SENSOR_A 25
#define M1_SENSOR_B 34
#define BTN 4

/** STATE MACHINE VARIABLES  */

// State Machine States
enum State {
  OFF,
  STRAIGHT,
  ENTRY_EXIT,
  APEX,
  ERROR,
};

enum IMUState {
    FALL_BACK, // Rely only on steering position
    FUNCTIONAL
};

/** MOTOR VARIABLES */
extern int theta;

/** PERIPHERALs GLOBALS  */
extern int strPos;
extern int strSpeed;
extern float rollAngle;
extern float rollAngleGyro;
extern float rollAngleAccel;

// Desired Camber Angles
double straightCamber = 0;
double eLeft = 3;
double eRight = -3;
double apexLeft = 6;
double apexRight = -6;
double camberAngle = 0;
double desiredCamberAngle = straightCamber;

#endif  // End of include guard
