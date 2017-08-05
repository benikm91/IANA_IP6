#include <Servo.h>

Servo servo_pan;
Servo servo_tilt; 

uint32_t angles[2];
  
void setup() {
  Serial.begin(9600);
  Serial.println("Guten Tag!");
  servo_pan.attach(10);
  servo_tilt.attach(11);
  angles[0] = 40;
  angles[1] = 40;
  servo_pan.write(angles[0]);
  servo_tilt.write(angles[1]);
}


void loop() {
  if (Serial.available() > 0)
  {
    //float in = Serial.parseFloat();
    //int in = Serial.read();
    
    if (Serial.readBytes((char*)&angles, sizeof(angles[0]) * 2) != sizeof(angles[0]) * 2)
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
    }
  }
  servo_pan.write(angles[0]);
  servo_tilt.write(angles[1]);
}

