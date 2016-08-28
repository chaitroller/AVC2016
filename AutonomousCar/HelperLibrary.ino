


void myDelay(int interval)
{
  unsigned long previousMillis = millis();
  while (millis() - previousMillis <= interval) {}
}


void debugPrint_1_ToSerial()
{
  Serial.print("Case 1: Current Angle = "); Serial.print(g_currentAngle); Serial.print(" ");
  //Serial.print("Upper Threshold = "); Serial.print(g_Angle_UT); Serial.print(" ");
  //Serial.print("Lower Threshold = "); Serial.print(g_Angle_LT); Serial.print(" ");
  Serial.print("Start Angle = "); Serial.print(g_startAngle); Serial.print(" ");                    // Current Angle
  Serial.println("Steering Servo Angle = "); Serial.println(servoSteering.readAngle());

}

void debugPrint_0_ToSerial(int nAngle)
{
  Serial.println();
  Serial.print("case-0-inside-if() NewStartAngle distanceTravelled =  ");
  Serial.print(nAngle); Serial.print(" ");
  //Serial.print(g_startAngle); Serial.print(" ");
  //Serial.print(g_Angle_UT); Serial.print(" ");
  //Serial.print(g_Angle_LT); Serial.print(" ");
  Serial.println(g_distance_travelled);
}

void debugPrint_0_ToFile()
{
  //dataFile = SD.open(String(fileName), FILE_WRITE);

  //dataFile.close();
}

void debugPrint_ToFile()
{
  //    if (dataFile) {
  //      Serial.println("Start Angle UT LT = ");
  //      Serial.print(g_startAngle); Serial.print(" ");
  //      Serial.print(g_Angle_UT); Serial.print(" ");
  //      Serial.println(g_Angle_LT);
  //    }
  //    dataFile.close();
}

