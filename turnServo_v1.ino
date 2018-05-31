//  turnServo_v1.h
//  version SANS classe specifique aux mouvements du servo 

// A TESTER 




#include <Servo.h>

Servo servo;  

const int potPin = A0;        // input: analog pin for potentiometer (the wire in middle in the potentiometer)
const uint8_t servoPin =7 ;   // output: digital pin for servo


void setup() {
  servo.attach(servoPin);  // attaches the servo on servoPin .

   // open the serial port at 9600 bps:
  Serial.begin(9600);
  Serial.println("Beginning") ;
}


int sign(int x){

 if (x>0 )
    return 1;
 if (x<0 )   
    return -1;

 if (x==0)   
    return 0;
}

/*
 *  C arrays are not objects or structures. They do not have any length parameter stored anywhere by default.
 *  WE SHOULD / WOULD CREATE A SPECIFIC CLASS FOR DATA arrays
 */
uint8_t Length(int thetaArray[] ){ 
 return sizeof(thetaArray)/sizeof( * thetaArray );  
 //sizeof(thetaArray)/sizeof(uint8_t);  
}

/* increment the angular position by Delta degree; with velocity omega [ degrees/sec] 
If Delta <0 :  cannot turn
*/
void turnServoRight(int Delta=1, uint8_t omega =60){
  uint8_t angle, tt ;
  tt = 1000/omega  ;  
  angle = servo.read();
  while ( (angle < (angle+Delta)) && (angle<180) ){
    servo.write(angle++); 
    delay(tt);   
  }
}


/* decrement the angular position by Delta degree , with velocity omega [ degrees/sec] 
Delta must be <0 ;   If Delta >0 :  cannot turn*/
void turnServoLeft(int Delta =-1, uint8_t omega =60){
  uint8_t angle, tt ;
  tt = 1000/omega  ;  
  angle = servo.read();
  while ( (angle > (angle+Delta)) && (angle>0) ){
    servo.write(angle--); 
    delay(tt);   
  }
}

/*  Delta : angular variation in degree we want the servo to turn
 *  when Delta > 0: turn right; 
 *  when Delta <0:  turn left;                               
 *  omega:  constant angular velocity ;
 */
void turnServo(int Delta =1, uint8_t omega =60){
  uint8_t angle, tt;
  int sgn ;
  tt = 1000/omega  ;  
  sgn = sign(Delta);  // =1 ,  -1, or 0
  angle = servo.read();
  while ( (angle != (angle+Delta)) && (angle>0)&& (angle<180) ){
    servo.write(angle +sgn); 
    delay(tt);   
  }
}

/*  only one step left or right. 
 *  
 *  theta0, theta1 :uint8_t  :  degrees 
 *  omega :  angular velocity  [degree/s] i.e. [degree / step]
 */ 
uint8_t turnServo1deg(uint8_t angle, int sgn,uint8_t omega=60 ){
  // omega : angular velocity [omega degree per sgn sec] between angle and angle+sgn
  //   generally we take sgn =  +1 or -1 so that omega is in degree/sec
  uint8_t tt = 1000/omega  ;    // [ms]
  angle  +=sgn;   
  servo.write(angle); 
  delay(tt);   
  return angle;

} 

/* Actualize the servo from current position to angular position theta1.  
 *  omega :  constant angular velocity   [degree/s ]
 *  
 * (very similar to turnServo() )
 *                                                           */ 
void actualizeServo(uint8_t theta1=90, uint8_t omega=60) {
  uint8_t theta0 , tt , angle; 
  int Delta, sgn;
   
  theta0 = servo.read();     // current angular position
  Delta =  theta1 - theta0  ; //  90 - 93 = 253 if delta is uint8_t
  sgn = sign(Delta) ;
  tt = 1000/omega  ;  // [ms per degree]; inverse of angular velocity
  angle =theta0; 
  while (  abs(theta1-angle)>0){ 
      angle+=sgn;
      Serial.print("servo.read() =");   //DEBUG
      Serial.println(angle);   //DEBUG
      Serial.println(abs(theta1-angle)) ; //  DEBUG


     servo.write(angle); 
     delay(tt);   
  }
}   

/*  with variable angular velocity  
 *  thetaArray:  angular position at each time step 
 *  omegaArray:  angular velocity at each time step 
 *  In order to have omegaArray == thetaArray, we assign a zero velocity to the last index..
 *  
 *  When inversing the velocity to obtain the delay; we    ??? 
 */

/*
uint8_t actualizeServoNonCst(uint8_t thetaArray[]  , uint8_t omegaArray[] ) {
  
  uint8_t theta0 , tt, sgn, len ; 
  int Delta;
  len = Length(thetaArray); // defined above for static arrays
  uint8_t ttArray[len] ={0};

  if (Length(omegaArray) == len){
    //ERROR cannot convert 'uint8_t* {aka unsigned char*}' to 'int*' for argument '1' to 'uint8_t Length(int*)'


    omegaArray[len-1] = 0;

    // ttArray[i] that does not correspond to a omegaArray[i] are 0 by default. 
    for (uint8_t i=0; i<(len-3); i++ ) {
       ttArray[i] = 1/omegaArray[i];// the last index is len-2
    }
    
    theta0 = servo.read();     // current angular position  

    if (thetaArray[0]!=theta0){
       actualizeServo(theta0) ;
    }
    for (uint8_t i=0; i<(len-2); i++ ) {
      // the last index is len-1
       servo.write(thetaArray[i]); 
       delay(ttArray[i]);   
    } 
    return 0 ; // 
    
  } //if 
  else 
     return -1;    //  error message... 
}     */

/* n scanning left-right   */
void scanLeftRight(uint8_t n=1,uint8_t theta0 = 0,uint8_t theta1= 180, uint8_t omega=60, bool sonar=false){
    uint8_t angle;
    int sgn =1;
    
    angle = servo.read();
    
    if (not sonar) {
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
        }
        Serial.print(   "scanLeftRight:   ");   //DEBUG
        Serial.println(angle);   //DEBUG
        
        for (uint8_t i=0; i<n; i++){
           while ( min(abs(angle - theta0),abs(angle - theta1))>0  ){
                angle = turnServo1deg(angle, sgn , omega);
                //angle +=sgn ;
                Serial.println(angle);   //DEBUG
           
           } 
           // at this point:  (angle ==theta0)||(angle==theta1) 
           sgn *=(-1) ;// hence sgn alternates 1, -1, 1, -1....     
                      
        }
   /* else if(sonar =true){        
    *  // as above but with sonar... 
        }  */ 
    }      
}


uint8_t servoPot(){
    int angleOutput;// 16 bits
    angleOutput = analogRead(potPin);                    // reads the output potentiometer value on 10 bits: between 0 and (2^10 -1 )= 1023.
    angleOutput = map(angleOutput, 0, 1023, 0, 179);     // maps the output value to an angle for the servo: between 0 and 180-1. 
  
}

void loop() {


  //  on pourrait considerer angle comme une variable global
  // en ce moment on passe la variable par valeur a toutes les fcts. 
   
  // ENCORE MIEUX:  creer une classe pour les rotations de servo, avec attribut angle. 

  // un scans from 0 to  180,with omega=60 degrees/sec 
 scanLeftRight(1);


// actualizeServo(90);  //OK

 
// actualizeServo(uint8_t theta1=90, uint8_t omega=60)
 /*actualizeServo(30,180);

 
 delay(500);
 turnServo(-25, 180);
 turnServo(25, 180);
 
 delay(500);
 turnServoLeft(-25, 180);
 //turnServoLeft(25, 180);
 
// turnServoRigth(-25, 180);
 turnServoRight(25, 180);
*/

/*
 // with the potentiomter:
 servoPot();
*/ 
 
}
