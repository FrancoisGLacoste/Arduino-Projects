
/* ==============================================================================
 * Includes header files and Defines Constants 
 * ============================================================================== */
 #include <Servo.h>

 #include "servoRotation.h" // for motion of the servo where the sonar is mounted

//'average' ultrasound velocity in air: 330 m/s , ( Actually it is varying with temperature and humidity )
#define soundVelocity 33  //  cm/ms   

/* pin definitions  */
//  Ultrasounds
#define trigPin 13       
#define echoPin 12
#define Led1 4           //  far detection
#define Led2 3           // middle distance detection
#define Led3 2           // close detection 

// IR sensor1
#define Led1 11 
#define IRpin1 6    

// IR sensor2
#define Led2  10 
#define IRpin2 7

/* ======================================================================== 
 * Global Definition of the SERVO MOTOR associated with the sonar:  
 *  Has to be defined BEFORE the setup() function. 
 * ========================================================================*/

// non-constant variables defined before main loop
uint8_t angle =90;
uint8_t sgn =1;
// Servo object: But would be better to define a special sonar class if special functions are to be added)
Servo servo;  

/* ======================================================================== 
 * Setup : preparing the Arduino pins    
 * ========================================================================*/
void setup() {
  // ultrasons

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(Led1, OUTPUT);
  pinMode(Led2, OUTPUT);
  pinMode(Led3, OUTPUT);

  // IR sensors:
  pinMode (Led1, OUTPUT) ;
  pinMode (IRpin1, INPUT) ;

  pinMode (Led2, OUTPUT) ;
  pinMode (IRpin2, INPUT) ;

  // servo for sonar: 
  servo.attach(9);  // servo input attached on pin 9 
 
   //Serial.begin (9600);   // if we had a LCD screen for Aduino 
}
/*======================================================================
 *  ultrasonic detection of distance
 *  
 *  Functions:   
 *      sonarDistance()                                   : returns a distance in cm
 *      isObstacleAtDist(float distance, float distDetect):  returns HIGH or LOW
 *  
 *  Assumes ultrasound velocity in air: 330 m/s              
 *  http://www.vaultrasound.com/wp-content/uploads/2012/03/Propagation-Velocity-Chart.png
 ======================================================================= */
 
long sonarDistance() {
    // returns distance from obstacle in  cm  (rounded)
    long duration, distance;

    // a 2 MICROsec pulse  
    digitalWrite(trigPin, LOW);  
    delayMicroseconds(2); 
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // collect the echo and convert it into a distance 
    duration = pulseIn(echoPin, HIGH); // [ms]

    distance = (duration/2)*soundVelocity; // [ms]*[cm/ms]
    // typically, when distance = 1m = 100 cm, duration = 100 cm/( 66 cm/ms) = 1.5 ms
    return distance;
}


uint8_t isObstacleAtDist(long distance, long distDetect){  
    // returns HIGH whether there is an obstacle at less than the distance determined by distDetect 
    // and LOW otherwise
    
    //  HIGH and LOW are predefined constants ( HIGH =1 and LOW=0 )
    if (distance < distDetect)
        return HIGH  ;  
    else  
        return LOW ;
}


void SonarLedDisplay(long distance, uint8_t Led[3]){
   
   valLed1 = isObstacleAtDist(distance, 100) ;
   valLed2 = isObstacleAtDist(distance, 50) ;
   valLed3 = isObstacleAtDist(distance, 20) ;
   digitalWrite(Led[0],valLed1); 
   digitalWrite(Led[1],valLed2); 
   digitalWrite(Led[2],valLed3);
}


/* ================================================================
 *   Infrared Sensor:  obstacle detection
 * 
 * ================================================================ */

bool IRsensor(uint8_t IRpin, uint8_t Led){
    bool isObstacle = false ;
    if (digitalRead(IRpin) == HIGH){
        digitalWrite (Led, LOW);
        isObstacle= true;
    }
    else
       digitalWrite (Led, HIGH);
       
    return isObstacle   ; 
}

/* ===============================================================
 *  dataAcquisition   TO DO
 * ===============================================================*/
void dataAcquisition(long distanceSonar, uint8_t tooCloseIR[] ){
    return void;  
}
/* ==============================================================
*   Main loop 
*   
*  ==============================================================*/
void loop() {
   long distanceSonar;   // 32 bits
   bool tooCloseIR1, tooCloseIR2 ; //8 bits 
   bool tooCloseIR[2];    
   uint8_t valLed1,valLed2,valLed3 ;  // 8 bits 


   
   // Servo alternately, turns left and turns right from theta0 to theta1 
   // while sonar and IR sensors are active 
   servoLeftRight( ) ; // In servoRotation class, in the files : servoRotation.h and servoRotation.cpp
   
   //  Sonar : 
   distance = sonarDistance() ; 
   SonarLedDisplay(distance) ;

   // IR sensors :
   tooCloseIR1  = IRsensor(IRpin1, Led1);   // IRpin1, Led1 are globally defined
   tooCloseIR2  = IRsensor(IRpin1, Led1);   // IRpin1, Led1 are globally defined
   tooCloseIR ={tooCloseIR1, tooCloseIR2};
    
   //  data acquisition in real time: to be done.. 
   dataAcquisition(distanceSonar, tooCloseIR)  ; 
   
} 
