#include <Servo.h>

Servo servo_pan;
Servo servo_tilt; 

uint32_t angles[2];
uint32_t goal[2];
uint32_t rotation_speed = 1;
uint32_t update_delay = 10;
  
void setup() {
  Serial.begin(9600);
  Serial.println("Guten Tag!");
  servo_pan.attach(10);
  servo_tilt.attach(11);
  angles[0] = 90;
  angles[1] = 90;
  goal[0] = 90;
  goal[1] = 90;
  servo_pan.write(angles[0]);
  servo_tilt.write(angles[1]);
}


void loop() {
  if (Serial.available() > 0)
  {    
    /*if (Serial.readBytes((char*)&angles, sizeof(angles[0]) * 2) != sizeof(angles[0]) * 2)
    {
      Serial.println("arduino received trash!");
    }
    else
    {
      Serial.print("arduino received: [");
      for (int i = 0; i < 2; ++i)
      {
        Serial.print(angles[i], DEC);
        Serial.print(",");
      }
      Serial.println("]");
      servo_pan.write(angles[0]);
      servo_tilt.write(angles[1]);
    }*/
    if (Serial.readBytes((char*)&angles, sizeof(angles[0]) * 2) == sizeof(angles[0]) * 2)
    {
      servo_pan.write(angles[0]);
      servo_tilt.write(angles[1]);
    }
  }
  
  if (goal[0] != angles[0])
  {
    angles[0] += (goal[0] > angles[0] ? rotation_speed : -rotation_speed);
    servo_pan.write(angles[0]);
  }
  
  if (goal[1] != angles[1])
  {
    angles[1] += (goal[1] > angles[1] ? rotation_speed : -rotation_speed);
    servo_tilt.write(angles[1]);
  }
  
  delay(update_delay);
}

