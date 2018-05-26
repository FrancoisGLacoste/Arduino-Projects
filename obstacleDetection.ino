
/* ==============================================================================
 * Includes header files and Defines Constants 
 * ============================================================================== */
 #include <Servo.h>
 
/* pin definitions  */
//  Ultrasounds
#define trigPin 13
#define echoPin 12
#define Led1 4  //  far detection
#define Led2 3
#define Led3 2 // close detection 

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
int angle =90;
int sgn =1;
// Servo object: But would be better to define a special sonar class if special functions are to be added)
Servo servo;  

/* ======================================================================== 
 * Setup : preparing the Arduino pins    
 * ========================================================================*/
void setup() {
  // ultasons
  //Serial.begin (9600);   // if we had a LCD screen for Aduino 
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
 
float sonarDistance() {
    // returns distance from obstacle in cm
    long duration, distance;

    // a 2 MICROsec pulse  
    digitalWrite(trigPin, LOW);  
    delayMicroseconds(2); 
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // collect the echo and convert it into a distance   ???? POURTANT CA MARCHAIT COMME DANS LE CODE D"ORIGINE.... 
    duration = pulseIn(echoPin, HIGH);
    distance = (duration/2) * 33;// 330 m/s = 33 cm/ms  is the 'average' ultrasound velocity in air. Actually it is varying.
    return distance;
    // typically, when distance = 1m = 100 cm, duration = 100 cm/( 66 cm/ms) = 1.5 ms
}


int isObstacleAtDist(float distance, float distDetect){  
    // returns HIGH whether there is an obstacle at less than the distance determined by distDetect 
    //  and LOW otherwise
    
    //  HIGH and LOW are predefined int constants
    if (distance < distDetect)
        return HIGH  ;  
    else  
        return LOW ;
}

/* ================================================================
 *   Infrared Sensor:  obstacle detection
 * 
 * ================================================================ */

bool IRsensor(int IRpin, int Led){
    bool isObstacle = false ;
    if (digitalRead(IRpin) == HIGH){
        digitalWrite (Led, LOW);
        isObstacle= true;
    }
    else
       digitalWrite (Led, HIGH);
    return isObstacle   ; 
}


/* ================================================================
 * Servo-motor: alternately turns Left and Right along a half rotation
 * Half rotation :  from 0 to 180 degrees       
 *  (Would be better to develop specific fcts for a special class of servo dedicated to sonar)
 * ================================================================ */
void servoLeftRight(int theta0=0, int theta1=180, int tt=2 )
// tt inverse of the rotation velocity: in  " number of second  for of a half-rotation"
// default  2s , approx:  1800 ms/180 = 10 ms /degree  >  1.5 ms/degree   
// typically, ultrasounds travel 1m during 100 cm/( 66 cm/ms) = 1.5 ms
// It means 300 ms/(half-rotation) would already be fast enough to scan every degrees 
{
  if (angle ==theta0 || angle ==theta1 )
             sgn *=-1;
  angle  +=sgn;  // +1 or -1 
  servo.write(angle); 
  // save angle in an array                
  delay(tt);   
   
/********************  on pourrait utiliser un pointeur sur un array de donnees  pour enregistrer ****************/
   
} 

/* ==============================================================
*   Main loop 
*   
*  ==============================================================*/
void loop() {
   long distance;
   int valLed1,valLed2,valLed3 ; 

   
   // Servo alternately, turns left and turns right from theta0 to theta1 
   // while sonar and IR sensors are active 
   servoLeftRight( ) ; //  0 to 180 deg with 2 sec/deg
   
   //  Sonar : 
   distance = sonarDistance() ; 

   valLed1 = isObstacleAtDist(distance, 100) ;
   valLed2 = isObstacleAtDist(distance, 50) ;
   valLed3 = isObstacleAtDist(distance, 20) ;
   digitalWrite(Led1,valLed1); 
   digitalWrite(Led2,valLed2); 
   digitalWrite(Led3,valLed3); 

   // IR sensors 
   IRsensor(IRpin1, Led1);
   IRsensor(IRpin2, Led2);

   
} 
