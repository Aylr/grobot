// This program is for test run of your motors with the motor shields.
//Please make your connections to motor and motor shield as given in handout.

//After making all connections and uploading the program to arduino,
//open serial monitor and input your option 1, 2, 3 or 4, the motors will run according to that.

#include <math.h>

//The pins below can be any digital pin

#define FL1 22
#define FL2 24
#define FR1 26
#define FR2 28
#define BL1 30
#define BL2 32
#define BR1 34
#define BR2 36


//These pins below, should be PWM pins on arduino

#define pwm1 6
#define pwm2 8
#define pwm3 9
#define pwm4 7

#define DEBUG TRUE        // uncomment to show serial debug messages
//#define LEARNABOUTMATHS // uncomment to get a bunch of serial output to learn about sin() maths


void setup() {
  Serial.begin(9600);
  Serial.println("Hello!");

  pinMode(FL1, OUTPUT);
  pinMode(FL2, OUTPUT);
  pinMode(FR1, OUTPUT);
  pinMode(FR2, OUTPUT);
  pinMode(BL1, OUTPUT);
  pinMode(BL2, OUTPUT);
  pinMode(BR1, OUTPUT);
  pinMode(BR2, OUTPUT);

  pinMode(pwm1, OUTPUT);
  pinMode(pwm2, OUTPUT);
  pinMode(pwm3, OUTPUT);
  pinMode(pwm4, OUTPUT);

  pinMode(13, OUTPUT);

#ifdef LEARNABOUTMATHS
  Serial.println("Testing Maths");
  float direction = 0;

  Serial.print("sin(((45-");
  Serial.print(direction);
  Serial.print("direction)/57.2957795) = ");
  Serial.println(sin(((45-direction)/57.2957795)));

  Serial.print("sin(((135-");
  Serial.print(direction);
  Serial.print("direction)/57.2957795) = ");
  Serial.println(sin(((135-direction)/57.2957795)));

  Serial.print("sin(((225-");
  Serial.print(direction);
  Serial.print("direction)/57.2957795) = ");
  Serial.println(sin(((225-direction)/57.2957795)));

  Serial.print("sin(((315-");
  Serial.print(direction);
  Serial.print("direction)/57.2957795) = ");
  Serial.println(sin(((315-direction)/57.2957795)));


  direction = 45;
  Serial.print("sin(((45-");
  Serial.print(direction);
  Serial.print("direction)/57.2957795) = ");
  Serial.println(sin(((45-direction)/57.2957795)));

  Serial.print("sin(((135-");
  Serial.print(direction);
  Serial.print("direction)/57.2957795) = ");
  Serial.println(sin(((135-direction)/57.2957795)));

  Serial.print("sin(((225-");
  Serial.print(direction);
  Serial.print("direction)/57.2957795) = ");
  Serial.println(sin(((225-direction)/57.2957795)));

  Serial.print("sin(((315-");
  Serial.print(direction);
  Serial.print("direction)/57.2957795) = ");
  Serial.println(sin(((315-direction)/57.2957795)));
#endif
 

  //runWheelsIndividually(500);   // test each wheel CCW & CW for 500ms

  goTheta(4000, 0, 255);        // go straight for 4 seconds
  square(1500, 200);            // draw a square of size 2000 and speed 255
  rotate(1000, 255, 0);


}


void loop(){
  //put serial listener here
}


void goTheta(unsigned int time, float heading, unsigned char speed){
  // This function takes a time, heading (positive 0-360 degrees CCW from front of robot), and speed (0-255)
  // It then does some maths and runs each wheel appropriately to get the robot to run
  // the given headings

  float ratios[4];    // used for maths
  float max = 0;      // used to find the largest ratio vector
  int pwm[4];         // used for final 0-255 pwm
  bool tempDir = 0;   // used to hold the final heading

  #ifdef DEBUG
    Serial.print("\ngoTheta called with time = ");
    Serial.print(time);
    Serial.print(" and heading = ");
    Serial.println(heading);
  #endif

  // ghetto sanity checking
  // This bit may definitely cause trouble in the future if values outside 0-360 are expected to work
  if(heading>360){      //if given heading is more than 360, just use 360
    heading=360;
  }else if(heading<0){  // if given heading is negative, just go 0
    heading = 0;
  }

  // determine relative ratios of motor power
  ratios[0] = sin(((45-heading)/57.2957795)); //Front Left ***convert to radians!!!
  ratios[1] = sin(((135-heading)/57.2957795)); //Back Left
  ratios[2] = sin(((225-heading)/57.2957795)); //Back Right
  ratios[3] = sin(((315-heading)/57.2957795)); //Front Right


  #ifdef DEBUG                    //print some useful stuff to check the maths
    Serial.print("ratios: ");
    for(int i=0;i<4;i++){
      Serial.print("ratios[");
      Serial.print(i);
      Serial.print("]= ");
      Serial.print((float)ratios[i],3);
      Serial.print(", ");
    }
    Serial.print("\n");
  #endif

  //find the maximum ratio so we can scale the values up later
  max = abs(ratios[0]);             // start with max = first element

  for(int i=1;i<4;i++) {            // for each element in the array
    if(abs(ratios[i]) > max){       // if |val| > max, then set new max
      max = abs(ratios[i]);         //...set the new maximum as |ratio|
    }
  }

  #ifdef DEBUG
    Serial.print("max ratio = ");
    Serial.println(max);
  #endif

  for(int i=0;i<4;i++){             // for each element in the array
    pwm[i] = round(speed*(ratios[i]/max));    //do the scaling math (and truncate float to int)
  }

  #ifdef DEBUG                    //print some useful stuff to check the maths
    Serial.print("pwm: ");
    for(int i=0;i<4;i++){
      Serial.print("pwm[");
      Serial.print(i);
      Serial.print("]= ");
      Serial.print(pwm[i]);
      Serial.print(", ");
    }
    Serial.print("\n");
  #endif


  for(int i=0;i<4;i++){     // for each of 4 wheels
    if(pwm[i]>=0){            // if pwm speed is positive
      tempDir = 1;            // ... set wheel to CCW
    }else if(pwm[i]<0){       // if pwm speed is negative
      tempDir = 0;            // ... set wheel to CW
    }
    runWheel((i+1),tempDir,abs(pwm[i]));    // run the wheel
    // (adding 1 to get the wheel # from the array index) with the speed and dir computed
  }

  digitalWrite(13, HIGH); //LED indicator ON
  delay(time);     //how long to actually go
  stop();

}


void stop(){
  // This simple function stops all motors without any kind of intelligent braking.
  analogWrite(pwm1, 0);
  analogWrite(pwm2, 0);
  analogWrite(pwm3, 0);
  analogWrite(pwm4, 0);

  digitalWrite(FL1, LOW);//wheel 1 FL
  digitalWrite(FL2, LOW);
  digitalWrite(FR1, LOW);//wheel 2 FR
  digitalWrite(FR2, LOW);
  digitalWrite(BL1, LOW);//wheel 3 BL
  digitalWrite(BL2, LOW);
  digitalWrite(BR1, LOW); //wheel 4 BR
  digitalWrite(BR2, LOW);

  digitalWrite(13, LOW);  //LED inidicator OFF
}


//Rotate

void rotate(unsigned int time, unsigned char speed, bool direction){

  runWheel(1,direction,speed);
  runWheel(2,direction,speed);
  runWheel(3,direction,speed);
  runWheel(4,direction,speed);

  digitalWrite(13, HIGH); //LED indicator ON

  delay(time); //how long to actually go
  stop();
}

void runWheel(unsigned char wheelNum, bool direction, unsigned char speed){
  // This function abstracts out all the mucking about with setting the appropriate
  // pin HIGH/LOW for each wheel. The idea is to simply make a call to runWheel()
  // so you don't have to constantly remember which HIGH/LOW combination is which
  // direction. For example, to run wheel 3 CW at speed 150, run runWheel(3,1,150)

  unsigned char tempWheelPin1;
  unsigned char tempWheelPin2;
  unsigned char tempPWMPin;

  if(wheelNum == 1){            //wheel 1 FL
    tempPWMPin = pwm1;
    tempWheelPin1 = FL1;
    tempWheelPin2 = FL2;
  }else if(wheelNum == 2){      //wheel 2 BL
    tempPWMPin = pwm2;
    tempWheelPin1 = BL1;
    tempWheelPin2 = BL2;
  }else if(wheelNum == 3){      //wheel 3 BR
    tempPWMPin = pwm3;
    tempWheelPin1 = BR1;
    tempWheelPin2 = BR2;    
  }else if(wheelNum == 4){      //wheel 4 FR
    tempPWMPin = pwm4;
    tempWheelPin1 = FR1;
    tempWheelPin2 = FR2;    
  }

  analogWrite(tempPWMPin, speed);   //set the speed on the given wheel

  if(direction == 1){            //decide whether CW or CCW is 1 or 0
    digitalWrite(tempWheelPin1, HIGH);
    digitalWrite(tempWheelPin2, LOW);
  }else if(direction == 0){
    digitalWrite(tempWheelPin1, LOW);
    digitalWrite(tempWheelPin2, HIGH);
  }
}


void square(unsigned int time, unsigned char speed){     // make the robot perform a square of given "time" size & given speed
  goTheta(time, 0, speed);
  goTheta(time, 90, speed);
  goTheta(time, 180, speed);
  goTheta(time, 270, speed);
}


void simpleRunWheelTest(unsigned int time, unsigned int speed){
  // This funciton runs each wheel CW & CCW for given time/speed using the new 
  // runWheel() function to make life simpler.

  runWheel(1,1,speed);
  delay(time);
  runWheel(1,0,speed);
  delay(time);
  
  runWheel(2,1,speed);
  delay(time);
  runWheel(2,0,speed);
  delay(time);

  runWheel(3,1,speed);
  delay(time);
  runWheel(3,0,speed);
  delay(time);

  runWheel(4,1,speed);
  delay(time);
  runWheel(4,0,speed);
  delay(time);
}


void runWheelsIndividually(unsigned int time){
  //FL CCW
  analogWrite(pwm1, 255);
  digitalWrite(FL1, HIGH);//wheel 1 FL
  digitalWrite(FL2, LOW);
  delay(time);
  //FL CW
  analogWrite(pwm1, 255);
  digitalWrite(FL1, LOW);//wheel 1 FL
  digitalWrite(FL2, HIGH);
  delay(time);
  stop();

  //BL CCW
  analogWrite(pwm2, 255);
  digitalWrite(BL1, HIGH);
  digitalWrite(BL2, LOW);
  delay(time);
  //BL CW
  analogWrite(pwm2, 255);
  digitalWrite(BL1, LOW);
  digitalWrite(BL2, HIGH);
  delay(time);
  stop();

  //BR CCW
  analogWrite(pwm3, 255);
  digitalWrite(BR1, HIGH);
  digitalWrite(BR2, LOW);
  delay(time);
  //BR CW
  analogWrite(pwm3, 255);
  digitalWrite(BR1, LOW);
  digitalWrite(BR2, HIGH);
  delay(time);
  stop();

  //FR CCW
  analogWrite(pwm4, 255);
  digitalWrite(FR1, HIGH);
  digitalWrite(FR2, LOW);
  delay(time);
  //FR CW
  analogWrite(pwm4, 255);
  digitalWrite(FR1, LOW);
  digitalWrite(FR2, HIGH);
  delay(time);
  stop();
}
