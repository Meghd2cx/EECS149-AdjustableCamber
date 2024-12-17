#include <math.h>
#include <globals.h>

#define ERROR_MAX_STEERING_POSITION 2098
#define ERROR_MAX_MOTOR_ANGLE 7
#define ERROR_MAX_ROLL_ANGLE 36 // Max roll angle on any F1 track
#define ERROR_NO_CAN_FRAMES_RECEIVED_MS 500
#define TIMEOUT 5

/* Check physical bounds for the following values:
returns: true if error
*/
bool linPotChecks() {
    // Null checks for Lin Pot
    // if (strPos == NULL) {return true;}
    // if (strSpeed == NULL) {return true;}

    // Bound check for lin pot 
    // if (abs(strPos) > ERROR_MAX_STEERING_POSITION) {return true;}

    return false; // No error encountered
} 

bool IMUChecks() {
    // Null checks for IMU 
    //TODO: Validate that this isn't necessarily true when just pulling data
    if (ESP32Can.inRxQueue() == 0) {return true;}
  
    // Roll Angle null checks
    if (rollAngle == NULL) {
      Serial.print("Falling back due to NULL IMU data!");
      return true;
    }

    // ROLL Angle hasn't been updated for >0.5s
    unsigned long currentTime = millis();   
    if (currentTime - receivedRollTime > 500) {
      Serial.print("Falling back due to IMU failure!");
      return true;
    }

    // Roll angle bound checks
    if (abs(rollAngle) > ERROR_MAX_ROLL_ANGLE) {
      Serial.print("Falling back due to IMU failure!");
      return true;
    }

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
    if (linPotError) {currentState = ERROR;}

    // Change to fallback SM if IMU error encountered
    // if (IMUError) {currentIMUState = FALL_BACK;}
    currentIMUState = FALL_BACK;
}
