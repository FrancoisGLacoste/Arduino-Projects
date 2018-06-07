//  servoRotation.h
 
#include "Arduino.h"
#include <Servo.h> 

// #include "SomeUsefulFunctions.h"


//    Include Guard
#ifndef __servoRot_H_INCLUDED__  
#define __servoRot_H_INCLUDED__  

/*
 * // Used in  actualizeServoNonCst(uint8Vect  ,uint8Vect) 
#include <StandardCplusplus.h> // standard libraries for Arduino
#include <vector>
typedef std::vector<uint8_t> uint8vect;
*/

int sign(int );


/* ============================================================ 
 *   Class used to manage the rotation of servos
 *   Keep track of the angle and angular velocity
 *   
 * ============================================================ */
class servoRotation: protected Servo {
  // public members of Servo object are only accessible to servoRotation class, not to loop()
  // in Servo :  read() ; write();  attach() 
  
  private: 
    // pins definition
    uint8_t servoPin ;        // digital output: pin for servo position

    // private methods: 
    void turnServo1deg(int, uint8_t omega=60  );  

  protected:
    uint8_t  angle ; 
    uint8_t  omega0; //  angular velocity from step -1 to step 0
    uint8_t  omega1; //  angular velocity from step 0 to step 1 
  
    //eventually for data acquisition of servo position ....  
    // when serial communication is possible, (angle, deltat) should be directly send instead of accumulating the data in these vectors:
//    uint8vect angleVect  ;
//    uint8vect deltatVect ;// deltatVect[0]=0
    // time difference betweem each angle measures. storing time itself would take more memory (more than 8 bit per step )

  public: 
    // The constructor is not explicitly defined. 
    
    void setServo(uint8_t );
    
    void turnServo(int Delta =1, uint8_t omega =60);
    void actualizeServo(uint8_t theta1=90, uint8_t omega=60) ;
    void scanLeftRight(uint8_t n=1 ,uint8_t theta0 = 0    ,uint8_t theta1= 180, uint8_t omega=60, bool sonar=false   );         
    void actualizeServoNonCst(uint8_t*  ,uint8_t*, uint16_t) ;    // with a vector of angular velocity
    void actualizeServoNonCst(uint8_t*  , uint16_t ,uint8_t) ; // with constant time step 
        
  /*  void actualizeServoNonCst(uint8Vect  ,uint8Vect) ;  // for vector containers.... but problem: adds some conflicts :   
       I get : " error  'int8_t Servo::min' is private   "   (it seems there is a min fct defined in Servo class ....  and we derived from Servo... )
  */
    
};


#endif 
