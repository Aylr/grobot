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

// Important global variables

byte tempSerial;                            // used as a place to store an incoming byte of serial data
byte tempPWM = 0;                           // stores the latest theta PWM speed
int tempTheta = 0;                          // stores the latest theta
bool tempRotateDirection = 0;               // stores the latest rotation direction
bool tempJoystickToggleButton = 0;          // stores the latest toggle button
byte tempABCDButtons = 0;                   // stores the latest buttons
byte tempRotatePWM = 255;                   // stores the latest rotation PWM speed
bool newCommandToRotate = false;            // a global flag used to set if there is a new rotate command available

// #define DEBUG TRUE        // uncomment to show serial debug messages


void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
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
 

  //runWheelsIndividually(500);   // test each wheel CCW & CW for 500ms

//  goTheta(0, 200);        // go straight for 4 seconds
//  delay(2000);
//  stop();

//  square(1500, 200);            // draw a square of size 2000 and speed 255
//  rotate(1000, 255, 0);


}


void loop(){
    if (Serial1.available()>0){                     // are there incoming serial bytes?
        if (getValidCommandFromSerial() == 1){      // are those a valid command?
            runCommands();                          // yes to both? Great, execute that command
        }
    }

    delay(2);                                       // wait for a bit.

}


void runCommands(){                                 // executes commands from the global command variables
    #ifdef DEBUG
        Serial.println("Running new command");
    #endif

    // The main question here is which has priority? Or more importantly, can BOTH go at once? How?

    if (newCommandToRotate){                        // if there is a new rotate command
        rotate(tempRotatePWM, tempRotateDirection); // rotate
    }else{                                          // if not, do gotheta
        goTheta(float(tempTheta), tempPWM);
    }
}


int getValidCommandFromSerial(){           // listens to the serial and returns 0 if invalid command and 1 if valid commands received
    tempSerial = byte(Serial1.read());

    if(tempSerial == byte(255)){                    // is the first byte the start character?
        tempPWM = readByte();                       // next byte is PWM
        tempTheta = readByte();                     // next byte is first part of theta
        tempTheta = tempTheta + readByte();         // next byte is second part of theta (addition of two bytes (0-360))
        tempJoystickToggleButton = readByte();      // next byte is joystick toggle
        tempABCDButtons = readByte();               // next byte is button detail

        // rotation button logic
        if (tempABCDButtons == char(66)) {          // if left button (CCW) has been pressed
            tempRotateDirection = 0;
            newCommandToRotate = true;
        }else if (tempABCDButtons == char(67)){     // if right button (CW) has been pressed
            tempRotateDirection = 1;
            newCommandToRotate = true;
        }else{                                      // if niether left or right has been pressed
            newCommandToRotate = false;
        }

        // logic for override button and ball drop button should probably go here
        // and be similar to the section above.
        
        #ifdef DEBUG
            Serial.print("Start command!");
            Serial.print(" PWM= ");
            Serial.print(tempPWM);
            Serial.print("  Theta= " );
            Serial.print(tempTheta);
            Serial.print(" Override= ");
            Serial.print(tempJoystickToggleButton);
            Serial.print(" newCommandToRotate= ");
            Serial.print(newCommandToRotate);
            Serial.print(" tempRotateDirection= ");
            Serial.print(tempRotateDirection);
            Serial.print(" ABCD = ");
            Serial.println(tempABCDButtons);
        #endif

        return 1;           // valid commands received and stored
    }else{
        return 0;           // no valid commands received
    }
}


int readByte(){               // This blocks anything else until reading next byte from serial
  while(true){
    if(Serial1.available()>0){
      return Serial1.read();
  }
 }
}


void goTheta(float heading, unsigned char speed){
  // This function takes a heading (positive 0-360 degrees CCW from front of robot), and speed (0-255)
  // It then does some maths and runs each wheel appropriately to get the robot to run
  // the given headings

  float ratios[4];    // used for maths
  float max = 0;      // used to find the largest ratio vector
  int pwm[4];         // used for final 0-255 pwm
  bool tempDir = 0;   // used to hold the final heading

  #ifdef DEBUG
    Serial.print("\ngoTheta called with speed = ");
    Serial.print(speed);
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

  // find the maximum ratio so we can scale the values up later
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
    pwm[i] = round(speed * (ratios[i]/max));    //do the scaling math (and truncate float to int)
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

void rotate(unsigned char speed, bool direction){
  for(int i=0;i<4;i++){
    runWheel((i+1),direction,speed);
  }
  
  digitalWrite(13, HIGH); //LED indicator ON
}


void runWheel(unsigned char wheelNum, bool direction, unsigned char speed){
  // This function abstracts out all the mucking about with setting the appropriate
  // pin HIGH/LOW for each wheel. The idea is to simply make a call to runWheel()
  // so you don't have to constantly remember which HIGH/LOW combination is which
  // direction. For example, to run wheel 3, CW at speed 150, run runWheel(3,1,150)

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

  if(direction == 1){                   // 1 = CCW
    digitalWrite(tempWheelPin1, HIGH);
    digitalWrite(tempWheelPin2, LOW);
  }else if(direction == 0){             // 0 = CW
    digitalWrite(tempWheelPin1, LOW);
    digitalWrite(tempWheelPin2, HIGH);
  }
}


void square(unsigned int time, unsigned char speed){
    // make the robot perform a square of given "time" size & given speed
    goTheta(0, speed);
    delay(time);
    goTheta(90, speed);
    delay(time);
    goTheta(180, speed);
    delay(time);
    goTheta(270, speed);
    delay(time);
}


void simpleRunWheelTest(unsigned int time, unsigned int speed){
  // This funciton runs each wheel CW & CCW for given time/speed using the new 
  // runWheel() function to make life simpler.

  for(int i=1;i<5;i++){       // for each of 4 wheels
    runWheel(i,1,speed);      // run CCW
    delay(time);
    runWheel(i,0,speed);      // run CW
    delay(time);
  }

  stop();
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
