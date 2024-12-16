#include <Arduino.h>
#include <ESP32Encoder.h>
#include <ESP32-TWAI-CAN.hpp>
#include "globals.h"
// #include "peripheral_interface.ino"
// #include "motor_control.ino"
// #include "safety_critical.ino"


// State Machine States
State currentState = OFF;
IMUState currentIMUState = FUNCTIONAL;
bool ACC_Enable = false;

// Function prototypes
void initPeripheralInterface();
void safetyCheck();
void controlMotorSpeed(int speed);

// Interrupt Variables
volatile bool buttonPressed = false;
int debounceDelay = 500;                // ms
unsigned long lastButtonPressTime = 0;  // To track button press time

ESP32Encoder encoder;

volatile int count = 0;
volatile bool deltaT = false;  // check timer interrupt
hw_timer_t* timer0 = NULL;
portMUX_TYPE timerMux0 = portMUX_INITIALIZER_UNLOCKED;
const int timerFreq = 1000000;  // 1 us / 1MHz
const int alarmFreq = 100000;   // 10,000 * 1 us = 10 ms

// Interrupt Services
void IRAM_ATTR isr_btn() {  // the function to be called when interrupt is triggered
  unsigned long currentMillis = millis();
  if (currentMillis - lastButtonPressTime >= debounceDelay) {
    buttonPressed = true;
    lastButtonPressTime = currentMillis;
  }
}

void IRAM_ATTR onTime0() {
    portENTER_CRITICAL_ISR(&timerMux0);
    count = encoder.getCount();
    encoder.clearCount();
    deltaT = true;  // the function to be called when timer interrupt is triggered
    portEXIT_CRITICAL_ISR(&timerMux0);
}

void setup() {
    Serial.begin(115200);

    // Initialize peripherals
    initialize_pinouts();

    // Set up button
    attachInterrupt(BTN, isr_btn, RISING);

    // Initilize timers
    timer0 = timerBegin(timerFreq);          // timer 0
    timerAttachInterrupt(timer0, &onTime0);  // edge (not level) triggered
    timerAlarm(timer0, alarmFreq, true, 0);  // autoreload enabled, infinite reloads
}

void loop() {
    if (deltaT) {
        portENTER_CRITICAL(&timerMux0);
        deltaT = false;
        portEXIT_CRITICAL(&timerMux0);

        // Popualte steering and IMU data
        populate_steering_data();
        updateIMUangAccel();

        // Run all safety checks, updates IMU status
        run_all_safety_checks();

        switch(currentIMUState) {
            case(FUNCTIONAL):
                functionalIMUStateMachine();
            case(FALL_BACK):
                fallBackStateMachine();
        }
    }
}

void functionalIMUStateMachine() {
    switch(currentState) {
        case OFF:
            if (CheckForButtonPress()) {
                camberAngle = -0.5;
                currentState = STRAIGHT;
            }        
            setLEDPIN(0);
            break;
        case STRAIGHT:
            camberAngle = -0.5;
            if (!CheckIfStraight()) {
                currentState = ENTRY_EXIT;
            }
            if (CheckForButtonPress()) {
                currentState = OFF;
            }
            break;
        case ENTRY_EXIT:
            camberAngle = computeDesiredActuationAngleWithIMU();
            if (SteeringSteady()) {
                currentState = APEX;
            }
            if (CheckForButtonPress()) {
                currentState = OFF;
            }
            if (CheckIfStraight()) {
                camberAngle = -0.5;
                currentState = STRAIGHT;
            }
            if (TurningLeft()) {
                setLEDPIN(4);
            } else {
                setLEDPIN(2);
            }
            break;
        case APEX:
            camberAngle = computeDesiredActuationAngleWithIMU();
            if (ReturningToStraight()) {
                currentState = ENTRY_EXIT;
            }
            if (CheckForButtonPress()) {
                currentState = OFF;
            }
            break;
        case ERROR:
            camberAngle = -0.5;
            controlMotor(camberAngle);
    }

    controlMotor(camberAngle);
    plotControlData();
}

// SM without IMU functionality
void fallBackStateMachine() {
    switch (currentState) {
        case OFF:
            if (CheckForButtonPress()) {
                camberAngle = -0.5;
                currentState = STRAIGHT;
            }
            setLEDPIN(0);
            break;
        case STRAIGHT:
            if (!CheckIfStraight()) {
                camberAngle = (TurningLeft()) ? eLeft : eRight;
                currentState = ENTRY_EXIT;
            }
            if (CheckForButtonPress()) {
                currentState = OFF;
            }
            setLEDPIN(3);
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
            if (TurningLeft()) {
                setLEDPIN(4);
            } else {
                setLEDPIN(2);
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
            if (TurningLeft()) {
                setLEDPIN(5);
            } else {
                setLEDPIN(1);
            }
            break;
        case ERROR: //TERMINAL STATE, NO BREAK
            camberAngle = -0.5;
            controlMotor(camberAngle);

    }

    controlMotor(camberAngle);
    plotControlData();

}
