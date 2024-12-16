#include <math.h>
#include <globals.h>

#define ERROR_MAX_STEERING_POSITION 4097
#define ERROR_MAX_MOTOR_ANGLE 7
#define ERROR_MAX_ROLL_ANGLE 36 // Max roll angle on any F1 track
#define TIMEOUT 5

/* Check physical bounds for the following values:
returns: true if error
*/
bool linPotChecks() {
    // Null checks for Lin Pot
    if (!strPos) {return true;}
    if (!strSpeed) {return true;}

    // Bound check for lin pot 
    if (abs(strPos) > ERROR_MAX_STEERING_POSITION) {return true;}

    return false; // No error encountered
} 

bool IMUChecks() {
    // Null checks for IMU 
    //TODO: Validate that this isn't necessarily true when just pulling data
    if (ESP32Can.inRxQueue() == 0) {return true;}
  

    // Roll Angle null checks
    if (!rollAngle) {return true;}

    // Roll angle bound checks
   if (abs(rollAngle) > ERROR_MAX_ROLL_ANGLE) {return true;}

   return false; // No error encountered
}

bool motorChecks() {
    // TODO:
    return false;
}

/* Execute all safety checks defined above*/
void run_all_safety_checks() {
    bool linPotError = (linPotChecks());
    bool IMUError = (IMUChecks());
    // bool MotorError = (motorChecks());
    // Change currentState to error if linpot error encountered
    currentState = (linPotError) ? ERROR : currentState;

    // Change to fallback SM if IMU error encountered
    currentIMUState = (IMUError) ? FALL_BACK : FUNCTIONAL;
}
