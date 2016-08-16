/* Autonomous Car Code
 *
 *  This code was written for the Sparkfun AVC competition
  Chaitanya.Sanghadia@gmail.com "Chai"
  matt.chandlerAZ@gmail.com "Matt"

 */

void GoStraight(int feet, int velocity)
{
  // Throttle for = 25 Count is ~ 2500 mSec
  steering.write(STRAIGHT);
  delay(500);
}

//============================================================================================
void TurnLeft()
{
  // Turn Left and delay
  steering.write(STEER_LEFT);
  delay(DELAYAFTERLEFT);
}
//============================================================================================
void SetSteeringGoStraight()
{
  // Set the steering to go straight
  steering.write(STRAIGHT);
  delay(STEERING_DELAY_MS);
}
void TurnEngineOff()
{
  //TODO
}
//============================================================================================
void TurnRight()
{
  // Turn Right and delay
  steering.write(STEER_RIGHT);
  delay(DELAYAFTERRIGHT);
}
//============================================================================================
void Backup(int feet)
{}
//============================================================================================
void StartupThrottleEngine(int iSpeed, int delayTime)
{
  // Write to the drive servo and add delay to get the THROTTLE going and to avoid timing issues
  // related to XL5 Motor Controller
  //============================================================================================
  drive.writeMicroseconds(iSpeed);
  delay(delayTime);
}
void NavigationRecheck(int leg)
{
}

float GetDirection(Adafruit_LSM9DS0 lsm ) {

  lsm.read();
  float heading = atan2((int)lsm.magData.y, (int)lsm.magData.x);

  float declinationAngle = 0.22;
  heading += declinationAngle;

  // Correct for when signs are reversed.
  if (heading < 0)
    heading += 2 * PI;

  // Check for wrap due to addition of declination.
  if (heading > 2 * PI)
    heading -= 2 * PI;

  // Convert radians to degrees for readability.
  return heading * 180 / M_PI;

}

void CourseCorrection( int minAngle, int maxAngle, int currentAngle) {

  minAngle = currentAngle - driftAngle;
  maxAngle = currentAngle + driftAngle;

  Serial.print("currentAngle: "); Serial.print(currentAngle); Serial.print("  ");
  Serial.print("minAngle: "); Serial.print(minAngle); Serial.print("  ");
  Serial.print("maxAngle: "); Serial.print(maxAngle); Serial.println("  ");

  if (currentAngle < minAngle) {
    //turn right
    TurnRight();
  } else if (currentAngle > maxAngle) {
    //turn left
    TurnLeft();
  }

}


