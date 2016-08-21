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

// Take turn based on dir, 1 = RIGHT and 0 = LEFT, these 0/1 can be #defined
// Adding gradual change of 10 degree; which can be #defined
void slowTurn(int destAngle, int nowAngle, int dir)
{

  int l_steerAngle = servoSteering.readAngle();    // Get Current Angle of steering servo
  int l_rightAngle = l_steerAngle + 10;
  int l_leftAngle = l_steerAngle - 10;

  if ( (dir == 1) && (l_rightAngle < RIGHT_MAX_ANGLE) ) {

    servoSteering.setAngle(l_rightAngle);        // Turn RIGHT

  } else if ( (dir == 0) &&  (l_leftAngle > LEFT_MIN_ANGLE) ) {

    servoSteering.setAngle(l_leftAngle);        // Turn RIGHT

  }

}



// Course Correction:
// Read the Angle using Compass to Change the Angle of Steering Servo
void courseCorrection() {

  int steerAngle = servoSteering.readAngle();    // Get Current Angle of steering servo

  // Compass Angle:
  // Calculate minAngle
  g_minAngle = g_startAngle - g_driftAngle;
  if (g_minAngle < 0)
    g_minAngle += 360;

  // Caclulate maxAngle
  g_maxAngle = g_startAngle + g_driftAngle;
  if (g_maxAngle > 360)
    g_maxAngle -= 360;

  //  Serial.print("SteerAngle = "); Serial.print(steerAngle); Serial.print(" ");
  //  Serial.print("CurrentAngle = "); Serial.print(currentAngle); Serial.print(" ");
  //  Serial.print("StartAngle: "); Serial.print(startAngle); Serial.print("  ");
  //  Serial.print("minAngle: "); Serial.print(minAngle); Serial.print("  ");
  //  Serial.print("maxAngle: "); Serial.print(maxAngle); Serial.println("  ");

  if ( (g_currentAngle < g_minAngle) && !(steerAngle < LEFT_MIN_ANGLE) && !(steerAngle > RIGHT_MAX_ANGLE) ) {          // Check if the vehicle is drifting on left
    //    Serial.println("------------ Turn Right");
    servoSteering.setAngle(steerAngle + 5);    // Steering Servo: turn right

  } else if ( (g_currentAngle > g_maxAngle) && !(steerAngle < LEFT_MIN_ANGLE) && !(steerAngle > RIGHT_MAX_ANGLE) ) {    // Check if the vehicle is drifting on right
    //    Serial.println("Turn Left ------------ ");
    servoSteering.setAngle(steerAngle - 5);    // Steering Servo: turn left

  } else if ( (steerAngle < LEFT_MIN_ANGLE) || (steerAngle > RIGHT_MAX_ANGLE) ) {        // Something is wrong, go straight

    servoSteering.setAngle(STEER_STRAIGHT);     // Steering Servo : Set the servo angle to go straight

  } else {

    servoSteering.setAngle(STEER_STRAIGHT);    // Steering Servo : Set the servo angle to go straight

  }

}

// Calculate New Angle: Do the match when crossing 0 degree
int getNewAngle(int currAngle, int turnAngle, int dir)
{
  int returnAngle = 0;

  if (dir == RIGHT) {        //  Right turn

    returnAngle = g_currentAngle + turnAngle;  // Turning clockwise, add angle
    if (currAngle + turnAngle > 360)         //
      returnAngle -= 360;                   // Fix for crossing 360 degree

  } else if (dir == LEFT) { // Left turn

    returnAngle = g_currentAngle - turnAngle;  // Turning anti-clockwise, subtract angle
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


