/**********************************************************************************************
   PID Library - Version 1.1
   v1.1 added D part hold for further tests
   based on Arduino PID Library by Brett Beauregard
 **********************************************************************************************/

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "PID.h"

/*Constructor (...)*********************************************************
      The parameters specified here are those for for which we can't set up
      reliable defaults, so we need to have the user set them.
 ***************************************************************************/
PID::PID(double* Input, double* Output, double* Setpoint,
         double Kp, double Ki, double Kd, int ControllerDirection)
{
  myOutput = Output;
  myInput = Input;
  mySetpoint = Setpoint;
  inAuto = false;
  errSum = 0;

  PID::SetOutputLimits(0, 255);	//default output limit corresponds to
  //the arduino pwm limits

  PID::SetControllerDirection(ControllerDirection);
  PID::SetTunings(Kp, Ki, Kd);
}


/* Compute() **********************************************************************
       This, as they say, is where the magic happens.  this function should be called
     every time "void loop()" executes.  the function will decide for itself whether a new
     pid Output needs to be computed.  returns true when the output is computed,
     false when nothing has been done.
 **********************************************************************************/
bool PID::Compute()
{
  if (!inAuto) return false;

  /*Compute all the working error variables*/
  double input = *myInput;   // Get input data
  double error = *mySetpoint - input;    // Compute error = Current value - Needed value

  errDiff = error - lastError;   // Compute error difference rof D part of PID
  errSum += error;              // Add error for integral part
  lastError = error;            // Save error value for Differential part of the PID


  // Variables for Inertion of the D part of the regulator
  const int DTimeHold = 1000;  //us=DTimeHold*loopTime How much D part is working after being triggered
  static double errDiffSaved;
  static int tick = 0;

  //trying to account motor slow react to D part
  //  if (abs(errDiff) > 0) {
  //    tick = DTimeHold;
  //    errDiffSaved = errDiff;
  //  }
  //  else if (tick != 0) {
  //    errDiff = errDiffSaved;
  //    tick--;
  //  }
  if (errDiff != 0)
    errDiffSaved = errDiff;
  else
    errDiff = errDiffSaved;


  if (errSum > outMax) errSum = outMax;
  else if (errSum < outMin) errSum = outMin;

  /*Compute PID Output*/
  double output = kp * error + ki * errSum + kd * errDiff;

  if (output > outMax) output = outMax;
  else if (output < outMin) output = outMin;

  *myOutput = output;

  return true;
}

/* SetTunings(...)*************************************************************
   This function allows the controller's dynamic performance to be adjusted.
   it's called automatically from the constructor, but tunings can also
   be adjusted on the fly during normal operation
 ******************************************************************************/
void PID::SetTunings(double Kp, double Ki, double Kd)
{
  if (Kp < 0 || Ki < 0 || Kd < 0) return;

  dispKp = Kp; dispKi = Ki; dispKd = Kd;

  if (controllerDirection == REVERSE)
  {
    kp = -Kp;
    ki = -Ki;
    kd = -Kd;
  }
  else
  {
    kp = Kp;
    ki = Ki;
    kd = Kd;
  }
}


/* SetOutputLimits(...)****************************************************
       This function will be used far more often than SetInputLimits.  while
    the input to the controller will generally be in the 0-1023 range (which is
    the default already,)  the output will be a little different.  maybe they'll
    be doing a time window and will need 0-8000 or something.  or maybe they'll
    want to clamp it from 0-125.  who knows.  at any rate, that can all be done
    here.
 **************************************************************************/
void PID::SetOutputLimits(double Min, double Max)
{
  if (Min >= Max) return;
  outMin = Min;
  outMax = Max;
}

/* SetMode(...)****************************************************************
   Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
   when the transition from manual to auto occurs, the controller is
   automatically initialized
 ******************************************************************************/
void PID::SetMode(int Mode)
{
  bool newAuto = (Mode == AUTOMATIC);
  if (newAuto && !inAuto)
  { /*we just went from manual to auto*/
    PID::Initialize();
  }
  inAuto = newAuto;
}

/* Initialize()****************************************************************
 	does all the things that need to happen to ensure a bumpless transfer
    from manual to automatic mode.
 ******************************************************************************/
void PID::Initialize()
{
  errSum = 0;
  lastError = *myInput;
}

/* SetControllerDirection(...)*************************************************
   The PID will either be connected to a DIRECT acting process (+Output leads
   to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
   know which one, because otherwise we may increase the output when we should
   be decreasing.  This is called from the constructor.
 ******************************************************************************/
void PID::SetControllerDirection(int Direction)
{
  if (inAuto && Direction != controllerDirection)
  {
    kp = -kp;
    ki = -ki;
    kd = -kd;
  }
  controllerDirection = Direction;
}

/* Status Funcions*************************************************************
   Just because you set the Kp=-1 doesn't mean it actually happened.  these
   functions query the internal state of the PID.  they're here for display
   purposes.  this are the functions the PID Front-end uses for example
 ******************************************************************************/
double PID::GetKp() {
  return  dispKp;
}
double PID::GetKi() {
  return  dispKi;
}
double PID::GetKd() {
  return  dispKd;
}
int PID::GetMode() {
  return  inAuto ? AUTOMATIC : MANUAL;
}
int PID::GetDirection() {
  return controllerDirection;
}
double PID::GetErrDiff() {
  return  errDiff;
}
