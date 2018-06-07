//  servoRotation.ino
 

// servoRotation class, which inherits from Servo class
// without any potentiometer control. 

#include "servoRotation.h" 

#include <math.h> // not used in servoRotation.cpp


// Instantiate an object of servoRotation class in Arduino 
servoRotation servo;  // values members are undefined after construction.
// (We dont want the pin values to be global as if we declare them here)


void setup(){
  
   uint8_t servoPin = 7; 

   servo.setServo(servoPin);

  /*
  // open the serial port at 9600 baud: 
  Serial.begin(9600);  //
   */
}

void loop() {
  
  int nbr =3; //scan number: OK although nbr is uint8_t in the function definition 
  // servo.scanLeftRight(nbr);     // OK 



   
  /*  along a specific trajectory       */

  // timeStep corresponds to the time for travelling one degree at constant angular velocity 60 degree/sec. 
  const uint8_t timeStep = 16; //= 1000/60; 
  uint16_t timeLength =4096;
  // so that time = 0,1,.... , (2**12 -1 )  (before 4096) [ms]
  
  uint8_t* thetaVect; // Sinus function just for testing
  thetaVect = fctSinus( thetaVect, timeLength, timeStep);
  
  servo.actualizeServoNonCst(thetaVect,  timeLength , timeStep) ;
  //  uint8_t* thetaVect,  uint16_t timeLength ,   uint8_t timeStep

 
  //servo.actualizeServo(90);  //

 
  //servo.actualizeServo(30,180);

 
 
}


/* ========================================================================================*/

// instead:  we could use std::vector
uint8_t*   fctSinus(uint8_t* vect, uint16_t timeLength, uint8_t timeStep){
  double a = timeLength/(TWO_PI ); // TWO_PI is a constant defined in arduino.h

  uint16_t vectLength = timeLength/timeStep;
  double theta;
  uint16_t t=0;
  for (uint16_t i=0; i < vectLength; i++ ){
      t += timeStep ;
      theta= 90* sin(a*t ) + 90;  
      vect[i]= (uint8_t) round(theta);  // casting in Arduino C ?? 
  }
  return vect; 
}

