//  servoRotation.cpp

#include "servoRotation.h"





// Should be included in another file instead. 
int sign(int x){
  if (x>0)       return  1;
  else if (x<0)  return -1;
  else if (x==0) return  0;
}


// output: digital pin for servo
void servoRotation::setServo(uint8_t pin){

    servoPin =pin ; // servoPin is a member of servoRotation class
    attach(servoPin);  // attach() :  is a fct member of Servo class: attach the servo on servoPin.

}


/*   
* omega : angular velocity [omega degree per sgn sec] between angle and angle+sgn
* generally we take sgn =  +1 or -1 so that omega is in degree/sec
*/ 
void servoRotation::turnServo1deg(int sgn,uint8_t omega=60 ){
  uint8_t tt = 1000/omega  ;    // [ms]
  omega0 =omega1;
  omega1 = omega; // update omega
  angle = read();  // should not be needed but just in case... 

  angle  +=sgn;   // update angle
  write(angle); 
  delay(tt);   
  

} 
 
/*  Delta : angular variation  [degrees]
 *  when Delta > 0: turn right; 
 *  when Delta <0:  turn left;                               
 *  omega:  constant angular velocity [degree/s]
 */
void servoRotation::turnServo(int Delta =1, uint8_t omega =60){

  uint8_t tt;
  int sgn ;

  omega0 =omega1;
  omega1 = omega; // update omega
  
  tt = 1000/omega1  ; //[ms]  
  sgn = sign(Delta);  // =1 ,  -1, or 0
  angle = read();
  while ( (angle != (angle+Delta)) && (angle>0)&& (angle<180) ){
    write(angle +sgn); 
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

  omega0 =omega1;
  omega1 = omega; // update omega1
   
  angle= read();     // update current angular position
  Delta =  theta1 - angle  ; //  90 - 93 = 253 if delta is uint8_t
  sgn = sign(Delta) ;
  tt = 1000/omega1  ;  // [ms per degree]; inverse of angular velocity

  while (  abs(theta1-angle)>0){ 
      angle+=sgn;
      write(angle); 
      delay(tt);   
  }
}   


/* n scanning left-right   */
void servoRotation::scanLeftRight(uint8_t n=1 ,uint8_t theta0 = 0    ,uint8_t theta1= 180, uint8_t omega=60, bool sonar=false){
    int sgn =1;

    // update member value:
    omega0=omega1;
    omega1 =omega; 
    angle = read();  
  
      // first reaches the range of angles to scan. 
      if ( angle <= theta0){
          actualizeServo(theta0+1, omega1 );
          angle = theta0+1; //max(theta0+1, 0);
      }
      else if ( angle >= theta1) {
          actualizeServo(theta1 - 1, omega1 ); 
          angle = theta1-1; // min(theta1-1, 179);
          sgn=-1;
      }//if
  /*    Serial.print(   "scanLeftRight:   ");   //DEBUG
      Serial.println(angle);   //DEBUG
   */   
       // n scans :
      for (uint8_t i=0; i<n; i++){
         while ( min(abs(angle - theta0),abs(angle - theta1))>0  ){
              turnServo1deg(sgn , omega1);  // increase or decrease angle by one step of 1 degree
              //angle +=sgn ;
   //           Serial.println(angle);   //DEBUG
         
         } 
         // at this point:  (angle ==theta0)||(angle==theta1) 
         sgn *=(-1) ;// hence sgn alternates 1, -1, 1, -1....     
                      
      }//for
         
}


/*  with variable angular velocity  
 *  thetaArray:  angular position at each time step 
 *  tArray:  angular velocity at each time step 
 *  In order to have omegaArray == thetaArray, we assign a zero velocity to the last index..  
 */

void servoRotation::actualizeServoNonCst(uint8_t* thetaVect, uint8_t* omegaVect, uint16_t lenVect  ) {
    uint8_t tt; // time step
    angle=read();
    if (angle != thetaVect[0]){
        actualizeServo(thetaVect[0], omegaVect[0]);   // Then, omegaVect needs to not begin with 0
    }
    for (uint16_t i=1; i < lenVect; i++){
        omega0 =omega1;
        omega1 = omegaVect[i];
        
        tt = 1000/omega1  ; //[ms]  
        angle = thetaVect[i];
        write(angle); 
        delay(tt);            
     //   Serial.println(i);
    }
}


  //overloaded : thetaVect with constant time step tt [ms]
  void servoRotation::actualizeServoNonCst(uint8_t* thetaVect,  uint16_t lenVect ,   uint8_t tt) {
    // indices:  0... (lenVect-1)
    omega1 = 1000/tt ;   
    angle=read();
    
    if (angle != thetaVect[0]){
        actualizeServo(thetaVect[0], omega1);   
    }
    for (uint16_t i=1; i < lenVect; i++){
        angle = thetaVect[i];
        write(angle); 
        delay(tt);           
    }

}


/*
//  Overloaed function for vector containers rather than pointers.  
void servoRotation::actualizeServoNonCst(uint8vect thetaVect, uint8vect omegaVect ) {
 
    for (uint8_t i; i < thetaVect.size(); i++){
        //  size()  is a member of the vector template class.
        
    
    }
  

}


*/
