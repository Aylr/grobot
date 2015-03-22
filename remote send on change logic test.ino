// This is a simple sketch for the joystick to prove out a logic idea:
// Every loop check for a change in the inputs.
// If there is a change, do something (send serial data to robot).
// If there isn't don't do anything!

#define Override 4
#define Rotate_CCW 6
#define Rotate_CW 13
#define Ball_drop 5
#define toggle 12
#define analogJoystickPinX 0
#define analogJoystickPinY 1

int currentAnalogPin0;
int currentAnalogPin1;
int lastAnalogPin0;
int lastAnalogPin1;
bool currentToggle;
bool lastToggle;
bool currentRotateCCW;
bool lastRotateCCW;
bool currentRotateCW;
bool lastRotateCW;


void setup(){
    pinMode(toggle, INPUT);                               //Set the Joystick 'toggle'button as an input
    digitalWrite(toggle, HIGH);                           //Enable toggle button
    pinMode(Rotate_CCW, INPUT);                           //Sets button 1 as an input
    digitalWrite(Rotate_CCW, HIGH);                       //Enable Rotate_CCW
    pinMode(Rotate_CW, INPUT);                            //Sets button 2 as an input
    digitalWrite(Rotate_CW, HIGH);                        //Enable Rotate_CW
  
    Serial.begin(9600);
}

void loop() {
    currentAnalogPin0 = analogRead(analogJoystickPinX);
    currentAnalogPin1 = analogRead(analogJoystickPinY);
    currentToggle = digitalRead(toggle);
    currentRotateCCW = digitalRead(Rotate_CCW);
    currentRotateCW = digitalRead(Rotate_CW);

    // if either pin has changed: NOTE: may require addition of all other input buttons as well.
    if ((currentAnalogPin0 != lastAnalogPin0) | (currentAnalogPin1 != lastAnalogPin1) | (currentToggle != lastToggle) | (currentRotateCW != lastRotateCW) | (currentRotateCCW != lastRotateCCW)){
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
    }

    
    lastAnalogPin0 = currentAnalogPin0;            // set the last reading as current for the next loop
    lastAnalogPin1 = currentAnalogPin1;            // to look for change next time.
    lastToggle = currentToggle;
    lastRotateCCW = currentRotateCCW;
    lastRotateCW = currentRotateCW;
    
    delay(10);
}