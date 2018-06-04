//  servoRotation_v1.h
 

#include <Servo.h> 

//    Include Guard
#ifndef __servoRot_H_INCLUDED__  
#define __servoRot_H_INCLUDED__  

#include <StandardCplusplus.h> // standard libraries for Arduino
#include <vector>

// data type for servo position acquisition vs time
typedef vector<uint8_t> uint8vect;
  
class servoRotation: protected Servo {
  // public members of Servo object are only accessible to servoRotation class, not to loop()
  // in Servo :  read() ; write();  attach() 
  
  private: 
    // pins definition
    const uint8_t servoPin ;        // digital output: pin for servo position

    // private methods: 
    uint8_t turnServo1deg(int );  

  protected:
    uint8_t  angle ; 
    uint8_t  omega0; //  angular velocity from step -1 to step 0
    uint8_t  omega1; //  angular velocity from step 0 to step 1 
  
    //eventually for data acquisition of servo position ....  
    // when serial communication is possible, (angle, deltat) should be directly send instead of accumulating the data in these vectors:
    uint8vect angleVect  ;
    uint8vect deltatVect ;// deltatVect[0]=0
    // time difference betweem each angle measures. storing time itself would take more memory (more than 8 bit per step )

  public: 
    // The constructor is not explicitly defined. 
    
    void setServo(uint8_t )
    
    void turnServo(int , uint8_t )
    void actualizeServo(uint8_t , uint8_t ) 
    void scanLeftRight(uint8_t ,uint8_t ,uint8_t , bool )         
    void actualizeServoNonCst(uint8_t []  , uint8_t [] ) 
  
};


#endif 
