/**********************************************************************************************
 * An ESP8266 port of:
 * Arduino PID Library - Version 1.0.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under a GPLv3 License
 **********************************************************************************************/

#ifndef PID_INCLUDE_PID_PID_H_
#define PID_INCLUDE_PID_PID_H_

#include "c_types.h"

typedef enum {
  AUTOMATIC = 1, MANUAL = 0
} OperationMode;

typedef enum {
  DIRECT = 3, REVERSE = 4
} Direction;


typedef struct {

  // private 'member' data

  float dispKp;        // * we'll hold on to the tuning parameters in user-entered
  float dispKi;        //   format for display purposes
  float dispKd;        //

  float kp;            // * (P)roportional Tuning Parameter
  float ki;            // * (I)ntegral Tuning Parameter
  float kd;            // * (D)erivative Tuning Parameter

  int controllerDirection;

  float *myInput;      // * Pointers to the Input, Output, and Setpoint variables
  float *myOutput;     //   This creates a hard link between the variables and the
  float *mySetpoint;   //   PID, freeing the user from having to constantly tell us
                       //   what these values are.  with pointers we'll just know.

  uint32_t lastTime;
  float iTerm, lastInput;

  uint32_t sampleTime;
  float outMin, outMax;
  OperationMode inAuto;

} Pid;

// * constructor.  links the PID to the Input, Output, and
//   Setpoint.  Initial tuning parameters are also set here
void pid_init(Pid* self, float*, float*, float*, float, float, float, int);

void pid_setMode(Pid* self, OperationMode newMode); // * sets PID to either Manual (0) or Auto (non-0)

// performs the PID calculation.  it should be
// called every time loop() cycles. ON/OFF and
// calculation frequency can be set using SetMode
// SetSampleTime respectively
bool pid_compute(Pid* self);

//clamps the output to a specific range. 0-255 by default, but
//it's likely the user will want to change this depending on
//the application
void pid_setOutputLimits(Pid* self, float, float);

//available but not commonly used functions ********************************************************

// * While most users will set the tunings once in the
//   constructor, this function gives the user the option
//   of changing tunings during runtime for Adaptive control
void pid_setTunings(Pid* self, float, float, float);

// * Sets the Direction, or "Action" of the controller. DIRECT
//   means the output will increase when error is positive. REVERSE
//   means the opposite.  it's very unlikely that this will be needed
//   once it is set in the constructor.
void pid_setControllerDirection(Pid* self, int);

// * sets the frequency, in Milliseconds, with which
//   the PID calculation is performed.  default is 100
void pid_setSampleTime(Pid* self, int);

//Display functions ****************************************************************
float pid_getKp(Pid* self);   // These functions query the pid for interal values.
float pid_getKi(Pid* self);    //  they were created mainly for the pid front-end,
float pid_getKd(Pid* self);       // where it's important to know what is actually
int pid_getMode(Pid* self);              //  inside the PID.
int pid_getDirection(Pid* self);           //

#endif /* PID_INCLUDE_PID_PID_H_ */
