#include <Servo.h>
#include "Arduino.h"
int i = 0;
unsigned long time = 0;
bool flag = HIGH;
int tmp1;
int tmp2;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(10, OUTPUT); //direction control PIN 10 with direction wire 
  pinMode(6, OUTPUT); //PWM PIN 11  with PWM wire
}

void loop() {
  // put your main code here, to run repeatedly:
  if (millis() - time > 5000)  {
    flag = !flag;
    digitalWrite(10, flag);
    time = millis();
  }
  tmp1 = Serial.parseInt();
  if (tmp1 != 0)  {
    tmp2 = tmp1;
    //analogWrite(6, Serial.parseInt());  //input speed (must be int)
    analogWrite(6, tmp2);
    //delay(200);
  }
  for(int j = 0;j<8;j++)  {
    i += pulseIn(9, HIGH, 500000); //SIGNAL OUTPUT PIN 9 with  white line,cycle = 2*i,1s = 1000000us，Signal cycle pulse number：27*2
  }
  i = i >> 3;
  Serial.print(111111 / i); //speed   r/min  (60*1000000/(45*6*2*i))
  Serial.println("  r/min");
  i = 0;
}
