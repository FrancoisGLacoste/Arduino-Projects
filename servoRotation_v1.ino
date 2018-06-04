//  servoRotation_v1.ino
 

// servoRotation class, which inherits from Servo class
// without any potentiometer control. 

#include "servoRotation_v1.h" 


// Instantiate an object of servoRotation class in Arduino 
servoRotation servo;  // values members are undefined after construction.
// (We dont want the pin values to be global as if we declare them here)

void setup(){
  
   // set the system: 
   int potPin = A0;
   uint8_t servoPin = 7; 

   servo.setServo(servoPin);

  /*
  // open the serial port at 9600 baud: 
  Serial.begin(9600);  //
   */
}

void loop() {

  servo.scanLeftRight(1);  // OK


  servo.actualizeServo(90);  //OK

 
  servo.actualizeServo(30,180);

 
}
