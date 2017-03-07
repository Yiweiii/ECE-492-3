#include <masonbot.h>
#include <stdio.h>
#include <stdlib.h>
#include <TimerOne.h>


void setup() {
  Serial.begin(9600);
  MasonBot();


}

void loop() {
  


  MasonBot().moveRotateCW();

  delay(5000);

  MasonBot().moveRotateCCW();

  delay(5000);

  MasonBot().moveForward();

  delay(5000);

  MasonBot().moveStop();

  delay(5000);

}
