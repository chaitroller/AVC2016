/* Autonomous Car Code
 *
 *  This code was written for the Sparkfun AVC competition
  Chaitanya.Sanghadia@gmail.com "Chai"
  matt.chandlerAZ@gmail.com "Matt"

 */

// Read Direction
float readDirection( HMC5883L compass) {

  Vector norm = compass.readNormalize();

  // Calculate heading
  float heading = atan2(norm.YAxis, norm.XAxis);

  // Set declination angle on your location and fix heading
  // You can find your declination on: http://magnetic-declination.com/
  // (+) Positive or (-) for negative
  // For Bytom / Poland declination angle is 4'26E (positive)
  // Formula: (deg + (min / 60.0)) / (180 / M_PI);
  float declinationAngle = (13.0 + (26.0 / 60.0)) / (180 / M_PI);
  heading += declinationAngle;

  // Correct for heading < 0deg and heading > 360deg
  if (heading < 0)
  {
    heading += 2 * PI;
  }

  if (heading > 2 * PI)
  {
    heading -= 2 * PI;
  }

  // Convert to degrees
  float headingDegrees = heading * 180 / M_PI;

  return headingDegrees;
}

void setupCompass()
{
  // Set measurement range
  compass.setRange(HMC5883L_RANGE_1_3GA);
  // Set measurement mode
  compass.setMeasurementMode(HMC5883L_CONTINOUS);
  // Set data rate
  compass.setDataRate(HMC5883L_DATARATE_30HZ);
  // Set number of samples averaged
  compass.setSamples(HMC5883L_SAMPLES_8);
  // Set calibration offset. See HMC5883L_calibration.ino
  compass.setOffset(0, 0);
}

// Take turn based on dir, TURN_RIGHT = 0 and TURN_LEFT = 1,
// Adding gradual change of 10 degree; which can be #defined
void slowTurn(int dir)
{
  int l_steerAngle = servoSteering.readAngle();    // Get Current Angle of steering servo
  int l_rightAngle = l_steerAngle + 10;
  int l_leftAngle = l_steerAngle - 10;

  if ( (dir == TURN_RIGHT) && (l_rightAngle < RIGHT_MAX_ANGLE) ) {

    servoSteering.setAngle(l_rightAngle);        // Turn RIGHT

  } else if ( (dir == TURN_LEFT) &&  (l_leftAngle > LEFT_MIN_ANGLE) ) {

    servoSteering.setAngle(l_leftAngle);        // Turn RIGHT

  }

}


// Course Correction:
// Read the Angle using Compass to Change the Angle of Steering Servo
void courseCorrection(int goal) {

  int error = goal - g_currentAngle;

  //Serial.print("DistanceTravelled = "); Serial.println(g_distance_travelled);

  if (error >= 180)
    error -= 360;         // Drifted too much on RIGHT, Action: Turn LEFT
  if (error <= -180)
    error += 360;         // Drifted too much on LEFT, Action: Turn RIGHT
  // Update servo and keep with range of +/- 60
  if (error > 60)
    error = 60;           // RIGHT Turn
  if (error < -60)
    error = -60;          // LEFT Turn

  servoSteering.setAngle(STEER_STRAIGHT + error);
  myDelay(10); //delay(10);

}

// Calculate New Angle: Do the match when crossing 0 degree
int getNewAngle(int currAngle, int turnAngle, int dir)
{
  int returnAngle = 0;

  if (dir == TURN_RIGHT) {                  //  Right turn

    returnAngle = currAngle + turnAngle;    // Turning clockwise, add angle
    if (returnAngle > 360)                  //
      returnAngle -= 360;                   // Fix for crossing 360 degree

  } else if (dir == TURN_LEFT) {            // Left turn

    returnAngle = currAngle - turnAngle;  // Turning anti-clockwise, subtract angle
    if (returnAngle < 0)
      returnAngle += 360;                  // Fix for crossing 360 degrees
  }

  return returnAngle;

}

// Read compass to know the current position of the vehicle wrt the NORTH direction
float readCompass() {
  Vector norm = compass.readNormalize();

  // Calculate heading
  float heading = atan2(norm.YAxis, norm.XAxis);

  // Set declination angle on your location and fix heading
  // You can find your declination on: http://magnetic-declination.com/
  // (+) Positive or (-) for negative
  // For Bytom / Poland declination angle is 4'26E (positive)
  // Formula: (deg + (min / 60.0)) / (180 / M_PI);
  float declinationAngle = (13.0 + (26.0 / 60.0)) / (180 / M_PI);
  heading += declinationAngle;

  // Correct for heading < 0deg and heading > 360deg
  if (heading < 0)
  {
    heading += 2 * PI;
  }

  if (heading > 2 * PI)
  {
    heading -= 2 * PI;
  }

  // Convert to degrees
  float headingDegrees = heading * 180 / M_PI;

}

// Use PID to reduce the error to zero
void takeTurn(int goal)
{
  // g_currentAngle = readDirection( compass );      //Read magnetometer
  int error = goal - g_currentAngle;

  Serial.print("ReadCompass: CurrentANgle Error ServoAngle = ");
  Serial.print(g_currentAngle); Serial.print(" ");
  Serial.print(error); Serial.print(" ");
  Serial.println(servoSteering.readAngle());

  //Serial.print("DistanceTravelled = "); Serial.println(g_distance_travelled);

  if (error >= 180)
    error -= 360;         // Drifted too much on RIGHT, Action: Turn LEFT
  if (error <= -180)
    error += 360;         // Drifted too much on LEFT, Action: Turn RIGHT
  // Update servo and keep with range of +/- 60
  if (error > 60)
    error = 60;           // RIGHT Turn
  if (error < -60)
    error = -60;          // LEFT Turn
  servoSteering.setAngle(95 + error); //(STEER_STRAIGHT); myservo.write(CENTER + error);
  myDelay(10);           //delay 10 mSec

  if (error < 5)
    LEG_NO++;

}
