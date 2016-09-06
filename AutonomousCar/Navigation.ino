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

  lsm.read();
  // Calculate heading
  float heading = atan2(lsm.magData.y, lsm.magData.x);

  // You can find your declination on: http://magnetic-declination.com/
  // Formula: (deg + (min / 60.0)) / (180 / M_PI);
  float declinationAngle = (10.0 + (26.0 / 60.0)) / (180 / M_PI);   // Declination for Phoenix = 10.x Degrees
  heading += declinationAngle;

  // Correct for heading < 0deg and heading > 360deg
  if (heading < 0) heading += 2 * PI;

  if (heading > 2 * PI) heading -= 2 * PI;

  // Convert to degrees
  float headingDegrees = heading * 180 / M_PI;
  return headingDegrees;
}

///////////////////////////////////////////////
// SetupCompass: 
//
///////////////////////////////////////////////
void setupCompass()

{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
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

// Use PID to reduce the error to zero
int setAngle(int goal)
{
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
  servoSteering.setAngle(STEER_STRAIGHT + error); //(STEER_STRAIGHT); myservo.write(CENTER + error);
  myDelay(10);           //delay 10 mSec

  return error;
}
