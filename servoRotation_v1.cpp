//  servoRotation_v1.cpp

#include <Servo.h>
#include <StandardCplusplus.h> // standard libraries for Arduino
#include <vector>

// output: digital pin for servo
void servoRotation::setServo(uint8_t pin){

    servoPin =pin ; // servoPin is a member of servoRotation class
    attach(servoPin);  // attach() :  is a fct member of Servo class: attach the servo on servoPin.

    angleVect[0]=read();   // read() is a function member of Servo class
    deltatVect[0] = 0; // member of servoRotation class
}


// input: analog pin for potentiometer (the wire in middle in the potentiometer)
void servoRotation::setPot(uint8_t pin1, uint8_t pin2){
   potPin = pin1;  
   potControlPin=pin2;
   
}

/*   
* omega : angular velocity [omega degree per sgn sec] between angle and angle+sgn
* generally we take sgn =  +1 or -1 so that omega is in degree/sec
*/ 
uint8_t servoRotation::turnServo1deg(int sgn,uint8_t omega=60 ){
  uint8_t tt = 1000/omega  ;    // [ms]
  angle  +=sgn;   
  servo.write(angle); 
  delay(tt);   
  return angle;

} 
 
/*  Delta : angular variation  [degrees]
 *  when Delta > 0: turn right; 
 *  when Delta <0:  turn left;                               
 *  omega:  constant angular velocity [degree/s]
 */
void servoRotation::turnServo(int Delta =1, uint8_t omega =60){
  uint8_t angle, tt;
  int sgn ;
  tt = 1000/omega  ; //[ms]  
  sgn = sign(Delta);  // =1 ,  -1, or 0
  angle = servo.read();
  while ( (angle != (angle+Delta)) && (angle>0)&& (angle<180) ){
    servo.write(angle +sgn); 
    delay(tt);   
  }
}

/* Actualize the servo from current position to angular position theta1.  
 *  omega :  constant angular velocity   [degree/s ]
 *  
 * (very similar to turnServo() )
 *                                                           */ 
void servoRotation::actualizeServo(uint8_t theta1=90, uint8_t omega=60) {
  uint8_t theta0 , tt , angle; 
  int Delta, sgn;
   
  theta0 = servo.read();     // current angular position
  Delta =  theta1 - theta0  ; //  90 - 93 = 253 if delta is uint8_t
  sgn = sign(Delta) ;
  tt = 1000/omega  ;  // [ms per degree]; inverse of angular velocity
  angle =theta0; 
  while (  abs(theta1-angle)>0){ 
      angle+=sgn;
      servo.write(angle); 
      delay(tt);   
  }
}   


/* n scanning left-right   */
void servoRotation::scanLeftRight(uint8_t n=1,uint8_t theta0 = 0,uint8_t theta1= 180, uint8_t omega=60, bool sonar=false){
    uint8_t angle;
    int sgn =1;

    // update member value:
    omega0=omega1;
    omega1 =omega; 
    angle = servo.read();  
    
   
      // n scans :

      // first reaches the range of angles to scan. 
      if ( angle <= theta0){
          actualizeServo(theta0+1, omega );
          angle = theta0+1; //max(theta0+1, 0);
      }
      else if ( angle >= theta1) {
          actualizeServo(theta1 - 1, omega ); 
          angle = theta1-1; // min(theta1-1, 179);
          sgn=-1;
      }//if
  /*    Serial.print(   "scanLeftRight:   ");   //DEBUG
      Serial.println(angle);   //DEBUG
   */   
      for (uint8_t i=0; i<n; i++){
         while ( min(abs(angle - theta0),abs(angle - theta1))>0  ){
              angle = turnServo1deg(angle, sgn , omega);
              //angle +=sgn ;
   //           Serial.println(angle);   //DEBUG
         
         } 
         // at this point:  (angle ==theta0)||(angle==theta1) 
         sgn *=(-1) ;// hence sgn alternates 1, -1, 1, -1....     
                      
      }//for
         
}



/*  with variable angular velocity  
 *  thetaArray:  angular position at each time step 
 *  omegaArray:  angular velocity at each time step 
 *  In order to have omegaArray == thetaArray, we assign a zero velocity to the last index..  
 */

/*
uint8_t servoRotation::actualizeServoNonCst(uint8_t thetaArray[]  , uint8_t ttArray[] ) ;

*/

