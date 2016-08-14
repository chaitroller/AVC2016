////////////////////////////////////////////////////////////////
//
//
//
///////////////////////////////////////////////////////////////
#include <Arduino.h>
#include <Servo.h>

int rotation = 0;
int distance_travelled = 0;

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
    ESC_Throttle(int interval)
    {
      updateInterval = interval;
      increment = 1;
      startSpeed = 1700;
      //Distance = 60;
    }

    void Attach(int pin)
    {
      servo.attach(pin);
    }

    void Detach()
    {
      servo.detach();
    }

    void GoForward(int Distance)
    {
      if ((millis() - lastUpdate) > updateInterval) // time to update
      {
        lastUpdate = millis();
        servo.writeMicroseconds(startSpeed);
        Serial.print(distance_travelled); Serial.print(" ");
        Serial.println(Distance);
        if(distance_travelled > Distance)
        {
          Serial.println("DONE");
          startSpeed = 1500;        // STOP         
        }
      }
    }
};

