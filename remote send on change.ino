// Every loop check for a change in the inputs.
// If there is a change, do something (send serial data to robot).
// If there isn't don't do anything!

#include <SoftwareSerial.h>
SoftwareSerial mySerial(2, 3); // RX, TX

// #define DEBUG_SERIAL_TO_COMPUTER true               // comment out to hide verbose serial debug details

// Pin definitions

#define Override 4
#define Rotate_CCW 6
#define Rotate_CW 13
#define Ball_drop 5
#define toggle 12
#define analogJoystickPinX 0
#define analogJoystickPinY 1
#define loopDelay 10                           // milliseconds between loops (fastest response time)

// Global variables of note

float x;
float y;
int r;
int theta;
byte theta_to_send_byte_1;
byte theta_to_send_byte_2;

int currentAnalogPin0;                      // this chunk of variables stores the current and last state of each input
int currentAnalogPin1;                      // which is used to compare if there was a change each loop.
int lastAnalogPin0;
int lastAnalogPin1;
bool currentToggle;
bool lastToggle;
bool currentRotateCCW;
bool lastRotateCCW;
bool currentRotateCW;
bool lastRotateCW;
bool currentOverride;
bool lastOverride;
bool currentBallDrop;
bool lastBallDrop;


void setup(){  
  pinMode(toggle, INPUT);                               //Set the Joystick 'toggle'button as an input
  digitalWrite(toggle, HIGH);                           //Enable toggle button
  pinMode(Override, INPUT);                             //Sets button 0 as an input
  digitalWrite(Override, HIGH);                         //Enable Override 
  pinMode(Rotate_CCW, INPUT);                           //Sets button 1 as an input
  digitalWrite(Rotate_CCW, HIGH);                       //Enable Rotate_CCW
  pinMode(Rotate_CW, INPUT);                            //Sets button 2 as an input
  digitalWrite(Rotate_CW, HIGH);                        //Enable Rotate_CW
  pinMode(Ball_drop, INPUT);                            //Sets the button 3 as an input
  digitalWrite(Ball_drop, HIGH);                        //Enable Ball_drop  
  
  mySerial.begin(9600);
  Serial.begin(9600); 
}


void loop() {
    readJoystickAndButtons();

    // if any of the input pin has changed DO STUFF. NOTE: may require addition of other inputs as well.
    if ( (currentAnalogPin0 != lastAnalogPin0) | (currentAnalogPin1 != lastAnalogPin1) | (currentToggle != lastToggle) | (currentRotateCW != lastRotateCW) | (currentRotateCCW != lastRotateCCW) | (currentOverride != lastOverride) | (currentBallDrop != lastBallDrop) ){
        computeAndSendCommands();                           // do the actual stuff
    }

    setLastCommandsForComparison();
    
    delay(loopDelay);
}


void readJoystickAndButtons (){
    // get new reading for each input and store in the "current" version of each global variable
    currentAnalogPin0 = analogRead(analogJoystickPinX);
    currentAnalogPin1 = analogRead(analogJoystickPinY);
    currentToggle = digitalRead(toggle);
    currentRotateCCW = digitalRead(Rotate_CCW);
    currentRotateCW = digitalRead(Rotate_CW);
    currentOverride = digitalRead(Override);
    currentBallDrop = digitalRead(Ball_drop);
}

void setLastCommandsForComparison(){
    lastAnalogPin0 = currentAnalogPin0;                     // set the last reading as current for the next loop
    lastAnalogPin1 = currentAnalogPin1;                     // to look for change next time.
    lastToggle = currentToggle;
    lastRotateCCW = currentRotateCCW;
    lastRotateCW = currentRotateCW;
    lastOverride = currentOverride;
    lastBallDrop = currentBallDrop;
}


void computeAndSendCommands(){
    #ifdef DEBUG_SERIAL_TO_COMPUTER
        Serial.print("Toggle: ");
        Serial.print(currentToggle);
        Serial.print(" X: ");
        Serial.print(currentAnalogPin0);
        Serial.print(" Y: ");
        Serial.print(currentAnalogPin1);
        Serial.print(" CCW: ");
        Serial.print(currentRotateCCW);
        Serial.print(" CW: ");
        Serial.print(currentRotateCW);
        Serial.print("\n");
    #endif

    Serial.print(char (255));                 // start character
    mySerial.print(char (255));

    //
    // Maths
    //

    if(currentAnalogPin0 < 506 || currentAnalogPin0 > 510){      // Assigns a threshold for joystick X
        x = (currentAnalogPin0-508.0)*0.345;
    }else{
        x= 0;
    }

    if(currentAnalogPin1 < 512 || currentAnalogPin1 > 516){      // Assigns a threshold for joystick Y
        y = (currentAnalogPin1-514.0)*0.345;
    }else{
        y=0;
    }

    //
    // PWM aka radius aka speed
    //

    r = round(sqrt(pow(x,2) + pow(y,2)));   // ***THINK about adding a round() here

    #ifdef DEBUG_SERIAL_TO_COMPUTER
        Serial.print(" r: ");
        Serial.print(r);
        Serial.print(" r byte: ");
    #endif

    Serial.write(r);
    mySerial.write(r);

    #ifdef DEBUG_SERIAL_TO_COMPUTER
        Serial.print(" ");
    #endif

    //
    // Theta
    //

    theta = round(atan2(y,x)*180/3.1415-90);

    if ( y == 0 && x == 0){                             // NOTE you have to send both bytes, so send two "zeros"
        Serial.write((byte)0);                          // for some reason the compiler wants to know that this is
        mySerial.write((byte)0);                        // the (byte)0, not the "null" character
        Serial.write((byte)0);                          // for some reason the compiler wants to know that this is
        mySerial.write((byte)0);                        // the (byte)0, not the "null" character
        theta_to_send_byte_1 = 0;
        theta_to_send_byte_2 = 0;
        theta = 0;

        #ifdef DEBUG_SERIAL_TO_COMPUTER
            Serial.print(" theta: ");
            Serial.print(theta);
            Serial.print(" ");
        #endif
    }else{
        if (theta < 0){                                 // if theta is negative, add 360
            theta = theta+360;
        }

        if (theta <= 254){                              // Because theta can be up to 360, we need to split theta
            theta_to_send_byte_1 = theta;               // into two bytes of information, since 1 byte maxes out at 255.
            theta_to_send_byte_2 = 0;                   // Being careful to not use the 255 "start" character
        }else if (theta > 254){
            theta_to_send_byte_1 = 254;
            theta_to_send_byte_2 = theta - 254;
        }

        //note look at byte 2 being null/0

        #ifdef DEBUG_SERIAL_TO_COMPUTER
            Serial.print(" theta: ");
            Serial.print(theta);
            Serial.print(" tb1: ");
            Serial.print(theta_to_send_byte_1);
            Serial.print(" tb2: ");
            Serial.print(theta_to_send_byte_2);
            Serial.print(" ");
        #endif

        Serial.write(theta_to_send_byte_1);             // write both bytes
        Serial.write(theta_to_send_byte_2);
        mySerial.write(theta_to_send_byte_1);           // write both bytes
        mySerial.write(theta_to_send_byte_2);
    }

    theta_to_send_byte_1 = 0;                           // reset both bytes to 0 for the next loop
    theta_to_send_byte_2 = 0;


    //
    // Buttons
    //

    #ifdef DEBUG_SERIAL_TO_COMPUTER
        Serial.print(" toggle: ");
    #endif

    if(currentToggle == LOW){                       // Checks if the toggle button has been pressed
        Serial.print(1);
        mySerial.print(1);
    }else{
        Serial.print(0);
        mySerial.print(0);
    }

    #ifdef DEBUG_SERIAL_TO_COMPUTER
        Serial.print(" buttons: ");
    #endif

    if (currentOverride == LOW){           // Checks if the Override button has been pressed
        Serial.println(char(65));
        mySerial.println(char(65));              // If the Override command has been given display "Override" on the serial monitor   
    }else if(currentRotateCCW == LOW){    // Checks if the Rotate_CCW button has been pressed
        Serial.println(char(66));
        mySerial.println(char(66));              // If the Rotate_CCW command has been given display "Rotate Conterclockwise" on the serial monitor
    }else if (currentRotateCW == LOW){    // Checks if the Rotate_CW button has been pressed
        Serial.println(char(67));
        mySerial.println(char(67));              // If the Rotate_CW command has been given display "Rotate Clockwise" on the serial monitor
    }else if(currentBallDrop == LOW){     // Checks if the Ball_drop button has been pressed
        Serial.println(char(68));
        mySerial.println(char(68));              // If the Ball_drop command has been given display "Ball Drop" on the serial monitor
    }else{
        Serial.println(0);
        mySerial.println(0);
    }
}