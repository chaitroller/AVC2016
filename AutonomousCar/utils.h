////////////////////////////////////////////////////////////////
//
//
//
///////////////////////////////////////////////////////////////
#include <Arduino.h>
#include <Servo.h>

int g_rotation = 0;
float g_distance_travelled = 0;
int LEG_NO = 0;
int MODE = 0; // MODE = 0 = Travel Certain Distance, MODE = 1 = Take Turn 
int newAngle = 0;
//int diffAngle = 0;
int g_currentAngle = 0;
int g_startAngle = 0;
bool g_updateStartAngle = true;

class ESC_Throttle
{
    Servo servo;              // the servo
    int pos;              // current servo position
    int increment;        // increment to move for each interval
    int  updateInterval;      // interval between updates
    unsigned long lastUpdate; // last update of position
    int startSpeed;
    //int Distance;

  public:
    ESC_Throttle()
    {
      updateInterval = 15;
      increment = 1;
      startSpeed = 1700;
      //Distance = 60;
    }

    void Attach(int pin)
    {
      servo.attach(pin);
    }

//    void updateSpeed(int newSpeed)
//    {
//      startSpeed = newSpeed;
//    }

    void Detach()
    {
      servo.detach();
    }

    void driveMotor(int newSpeed)
    {
      servo.writeMicroseconds(newSpeed);
    }

    void setAngle(int servoAngle)
    {
      servo.write(servoAngle);
    }
    
    int readAngle()
    {     
      return servo.read();    
    }

    ///////////////////////////////////////////////////////////////////
    //
    //
    ////////////////////////////////////////////////////////////////////
    void GoForward(int Distance)
    {
      if ((millis() - lastUpdate) > updateInterval) // time to update
      {
        lastUpdate = millis();
        servo.writeMicroseconds(startSpeed);
        if (g_distance_travelled > Distance)
        {
          Serial.println("DONE");
          if (LEG_NO++ > 3)
            startSpeed = 1500;        // STOP
          g_distance_travelled = 0;
        }
      }
    }

    //////////////////////////////////////////////////////////////////////
    //
    //
    //////////////////////////////////////////////////////////////////////
    void takeTurn(int servoAngle)
    {
      if ((millis() - lastUpdate) > updateInterval) // time to update
      {
        lastUpdate = millis();
        //pos += increment;
        servo.write(servoAngle);
      } else {
        //servo.write(95);

      }
    }


    void timeDelay(int t1)
    {

      if ((millis() - lastUpdate) > t1) // time to update
      {
        lastUpdate = millis();
        LEG_NO++;
        newAngle = g_currentAngle + 20;
        Serial.print("New Angle = ******************: "); Serial.println(newAngle);
      } 
    }


};

