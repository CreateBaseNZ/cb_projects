#include <Arduino.h>
#include <CB_LineFollower.h>
#include <CB_ObstacleAvoidance.h>
#include <CB_RadioControl.h>

int echos[] = {3,4,5,6,7,8};
CB_ObstacleAvoidance oa;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  oa.initObstacleAvoidance();
}

void loop() {
  // put your main code here, to run repeatedly:
  oa.printAllEchoPins();
  delay(2000);
}