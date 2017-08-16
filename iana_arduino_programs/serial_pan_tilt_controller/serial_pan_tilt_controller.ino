#include <Servo.h>

Servo servo_pan;
Servo servo_tilt; 

uint32_t state[2];
uint32_t goal[2];
uint32_t received[2];
uint32_t rotation_speed = 1;
uint32_t update_delay = 10;

void resetInputBuffer();

void setup() {
  Serial.begin(9600);
  servo_pan.attach(10);
  servo_tilt.attach(11);
  state[0] = 90;
  state[1] = 90;
  goal[0] = state[0];
  goal[1] = state[1];
  servo_pan.write(state[0]);
  servo_tilt.write(state[1]);
}


void loop() {
  
  if (Serial.available() == (sizeof(received[0]) * 2))
  {    
    if (
      Serial.readBytes((char*)&received, sizeof(received[0]) * 2) == sizeof(received[0]) * 2 &&
      received[0] > 0 && 
      received[0] <= 180 &&
      received[1] > 0 && 
      received[1] <= 180
      )
    {
      goal[0] = received[0];
      goal[1] = received[1];
      Serial.write(0);
    }
    else 
    {
      Serial.write(1);
    }
  }
  else if (Serial.available() > (sizeof(received[0]) * 2))
  {
     int asdfsadf = Serial.available();
     resetInputBuffer();
     Serial.write(asdfsadf);
  }
  
  if (goal[0] != state[0])
  {
    state[0] += (goal[0] > state[0] ? rotation_speed : -rotation_speed);
    servo_pan.write(state[0]);
  }
  
  if (goal[1] != state[1])
  {
    state[1] += (goal[1] > state[1] ? rotation_speed : -rotation_speed);
    servo_tilt.write(state[1]);
  }
  
  delay(update_delay);
}

void resetInputBuffer()
{
  while(Serial.available() > 0) 
  {
    char t = Serial.read();
  }
} 

