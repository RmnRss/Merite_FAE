/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Sweep
*/

#include <Servo.h>
Servo servo;  
const byte BOUTON = 8;

void setup() {
  servo.attach(9);
  //pinMode(BOUTON);
}

void loop() {
 
if(digitalRead(BOUTON)==(0)){
        servo.write(30); // write to servo
    }else{
        servo.write(150); // write to servo
    }
}
