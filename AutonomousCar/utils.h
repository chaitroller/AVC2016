////////////////////////////////////////////////////////////////
//
//
//
///////////////////////////////////////////////////////////////
#include <Arduino.h>
#include <Servo.h>



int rotation = 0;
int distance_travelled = 0;
int LEG_NO = 0;
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

    void updateSpeed(int newSpeed)
    {
      startSpeed = newSpeed;
    }

    void Detach()
    {
      servo.detach();
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
        if (distance_travelled > Distance)
        {
          Serial.println("DONE");
          if (LEG_NO++ > 3)
            startSpeed = 1500;        // STOP
          distance_travelled = 0;
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


};

