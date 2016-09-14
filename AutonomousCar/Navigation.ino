/* Autonomous Car Code

    This code was written for the Sparkfun AVC competition
  Chaitanya.Sanghadia@gmail.com "Chai"
  matt.chandlerAZ@gmail.com "Matt"

*/


/////////////////////////////////////////////////////////////
// Read Direction : Read Magnetometer and calculate direction
//
////////////////////////////////////////////////////////////
float readDirection() {

  sensors_event_t event;
  bno.getEvent(&event);

  return event.orientation.x;
}

////////////////////////////////////////////////////
// getNewAngle: For every turn calculate a new Angle
//
////////////////////////////////////////////////////

// Calculate New Angle: Do the match when crossing 0 degree
int getNewAngle(int currAngle, int turnAngle, int dir)
{
  int returnAngle = 0;

  if (dir == TURN_RIGHT) {                  //  Right turn

    returnAngle = currAngle + turnAngle;    // Turning clockwise, add angle
    if (returnAngle > 360)                  //
      returnAngle -= 360;                   // Fix for crossing 360 degree
  } 
  
  if (dir == TURN_LEFT) {                  // Left turn

    returnAngle = currAngle - turnAngle;  // Turning anti-clockwise, subtract angle
    if (returnAngle < 0)
      returnAngle += 360;                  // Fix for crossing 360 degrees
  }

  return returnAngle;

}

///////////////////////////////////////////////////////////////////////////////////
// setAngle: 
// Use PID to reduce the error to zero
// A Proportional–Integral–Derivative controller (PID controller) is a control loop 
// feedback mechanism (controller) commonly used in industrial control systems. 
// A PID controller continuously calculates an error value {\displaystyle e(t)} e(t) 
// as the difference between a desired setpoint and a measured process variable. 
// The controller attempts to minimize the error over time by adjustment of a control 
// variable {\displaystyle u(t)} u(t),
//////////////////////////////////////////////////////////////////////////////////

int setAngle(int currentAngle, int goal)
{
  int errorAngle = goal - currentAngle;

  if (errorAngle >= 180)
    errorAngle -= 360;         // Drifted too much on RIGHT, Action: Turn LEFT
  if (errorAngle <= -180)
    errorAngle += 360;         // Drifted too much on LEFT, Action: Turn RIGHT
  // Update servo and keep with range of +/- 60
  if (errorAngle > 60)
    errorAngle = 60;           // RIGHT Turn
  if (errorAngle < -60)
    errorAngle = -60;          // LEFT Turn

  Serial.print("CurrentANgle : "); 
  Serial.print(currentAngle); Serial.print(" ");
  Serial.print("TargetAngle : ");
  Serial.print(goal); Serial.print(" ");
  Serial.print("ErrorAngel : ");
  Serial.print(errorAngle); Serial.print(" ");
  Serial.print("DiatanceTraveled = ");
  Serial.println(g_distance_travelled);

  servoSteering.setAngle(STEER_STRAIGHT + errorAngle);
  myDelay(10);           //

  return abs(errorAngle);
}
