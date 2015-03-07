/**********************************************************************************************
 * An ESP8266 port of:
 * Arduino PID Library - Version 1.0.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under a GPLv3 License
 **********************************************************************************************/

#include "pid/pid.h"
#include "osapi.h"

#define pid_micros() system_get_time()
#define pid_millis() (system_get_time()/1000)

void ICACHE_FLASH_ATTR pid_reInitialize(Pid* self);

/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
void ICACHE_FLASH_ATTR
pid_init(Pid* self, float* input, float* output, float* setpoint, float kp,
         float ki, float kd, Pid_Direction controllerDirection) {

  self->myOutput = output;
  self->myInput = input;
  self->mySetpoint = setpoint;
  self->inAuto = false;

  pid_setOutputLimits(self, 0, 255);  //default output limit corresponds to
  //the arduino pwm limits

  self->sampleTime = 100;        //default Controller Sample Time is 0.1 seconds

  pid_setControllerDirection(self, controllerDirection);
  pid_setTunings(self, kp, ki, kd);

  // fake last sample time
  self->lastTime = pid_millis() - self->sampleTime;
}

/* Compute() **********************************************************************
 *  This, as they say, is where the magic happens.  this function should be called
 *  every time "void loop()" executes.  the function will decide for itself whether a new
 *  pid Output needs to be computed.  returns true when the output is computed,
 *  false when nothing has been done.
 **********************************************************************************/
bool ICACHE_FLASH_ATTR
pid_compute(Pid* self) {

  static uint32_t iterations = 0;
  iterations++;

  //os_printf("PID: inAuto=%s\n", self->inAuto?"true":"false");

  if (!self->inAuto)
    return false;

  uint32_t now = pid_millis();
  uint32_t timeChange = (now - self->lastTime);
  if (iterations%5==0)
    os_printf("PID: input=%d mySetpoint=%d timeChange=%d sampleTime=%d\n", (int)*(self->myInput), (int)*(self->mySetpoint), timeChange, self->sampleTime );

  if (timeChange >= self->sampleTime) {

    /*Compute all the working error variables*/
    float input = *(self->myInput);
    float error = *(self->mySetpoint) - input;
    self->iTerm += (self->ki * error);
    if (self->iTerm > self->outMax)
      self->iTerm = self->outMax;
    else if (self->iTerm < self->outMin)
      self->iTerm = self->outMin;
    float dInput = (input - self->lastInput);

    /*Compute PID Output*/
    float output = self->kp * error + self->iTerm - self->kd * dInput;

    if (output > self->outMax)
      output = self->outMax;
    else if (output < self->outMin)
      output = self->outMin;

    if (iterations%5==0) os_printf("PID: output=%d\n", (int)output);

    *(self->myOutput) = output;

    /*Remember some variables for next time*/
    self->lastInput = input;
    self->lastTime = now;
    return true;
  } else
    return false;
}

/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted.
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/
void ICACHE_FLASH_ATTR
pid_setTunings(Pid* self, float kp, float ki, float kd) {

  if (kp < 0 || ki < 0 || kd < 0)
    return;

  self->dispKp = kp;
  self->dispKi = ki;
  self->dispKd = kd;

  float SampleTimeInSec = ((float) self->sampleTime) / 1000;
  self->kp = kp;
  self->ki = ki * SampleTimeInSec;
  self->kd = kd / SampleTimeInSec;

  if (self->controllerDirection == PID_REVERSE) {
    self->kp = (0 - self->kp);
    self->ki = (0 - self->ki);
    self->kd = (0 - self->kd);
  }
}

/* SetSampleTime(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed
 ******************************************************************************/
void ICACHE_FLASH_ATTR
pid_setSampleTime(Pid* self, int newSampleTime) {

  if (newSampleTime > 0) {
    float ratio = (float) newSampleTime / (float) self->sampleTime;
    self->ki *= ratio;
    self->kd /= ratio;
    self->sampleTime = (uint32_t) newSampleTime;
  }
}

/* setOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void ICACHE_FLASH_ATTR
pid_setOutputLimits(Pid* self, float newMin, float newMax) {

  if (newMin >= newMax)
    return;
  self->outMin = newMin;
  self->outMax = newMax;

  if (self->inAuto) {
    if (*(self->myOutput) > self->outMax)
      *(self->myOutput) = self->outMax;
    else if (*(self->myOutput) < self->outMin)
      *(self->myOutput) = self->outMin;

    if (self->iTerm > self->outMax)
      self->iTerm = self->outMax;
    else if (self->iTerm < self->outMin)
      self->iTerm = self->outMin;
  }
}

/* setMode(...)****************************************************************
 * Allows the controller Mode to be set to PID_MANUAL or PID_AUTOMATIC
 * when the transition from PID_MANUAL to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/
void ICACHE_FLASH_ATTR
pid_setMode(Pid* self, Pid_OperationMode newMode) {

  if (newMode == !(self->inAuto)) { /*we just went from PID_MANUAL to auto*/
    pid_reInitialize(self);
  }
  self->inAuto = newMode;
}

/* pid_reInitialize()**********************************************************
 *  does all the things that need to happen to ensure a bumpless transfer
 *  from PID_MANUAL to PID_AUTOMATIC mode.
 ******************************************************************************/
void ICACHE_FLASH_ATTR
pid_reInitialize(Pid* self) {

  self->iTerm = *(self->myOutput);
  self->lastInput = *(self->myInput);
  if (self->iTerm > self->outMax)
    self->iTerm = self->outMax;
  else if (self->iTerm < self->outMin)
    self->iTerm = self->outMin;
}

/* SetControllerDirection(...)*************************************************
 * The PID will either be connected to a DIRECT acting process (+Output leads
 * to +Input) or a reverse acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 ******************************************************************************/
void ICACHE_FLASH_ATTR
pid_setControllerDirection(Pid* self, Pid_Direction direction) {

  if (self->inAuto && direction != self->controllerDirection) {
    self->kp = (0 - self->kp);
    self->ki = (0 - self->ki);
    self->kd = (0 - self->kd);
  }
  self->controllerDirection = direction;
}

/* Status Functions*************************************************************
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID.  they're here for display
 * purposes.  this are the functions the PID Front-end uses for example
 ******************************************************************************/
float ICACHE_FLASH_ATTR
pid_getKp(Pid* self) {
  return self->dispKp;
}

float ICACHE_FLASH_ATTR
pid_getKi(Pid* self) {
  return self->dispKi;
}

float ICACHE_FLASH_ATTR
pid_getKd(Pid* self) {
  return self->dispKd;
}

int ICACHE_FLASH_ATTR
pid_getMode(Pid* self) {
  return self->inAuto ? PID_AUTOMATIC : PID_MANUAL;
}

Pid_Direction ICACHE_FLASH_ATTR
pid_getDirection(Pid* self) {
  return self->controllerDirection;
}

