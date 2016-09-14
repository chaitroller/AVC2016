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
int newAngle = 0;
int g_currentAngle = 0;
int g_targetAngle = 0;
bool g_updateStartAngle = true;

class ESC_Throttle
{
    Servo servo;              // the servo
    int pos;                  // current servo position
    int increment;            // increment to move for each interval
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

    void Detach()
    {
      servo.detach();
    }

    void applyThrottle(int newSpeed)
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

};

