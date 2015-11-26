/*
  State change detection (edge detection)
 	
 Often, you don't need to know the state of a digital input all the time,
 but you just need to know when the input changes from one state to another.
 For example, you want to know when a button goes from OFF to ON.  This is called
 state change detection, or edge detection.
 
 This example shows how to detect when a button or button changes from off to on
 and on to off.
 	
 The circuit:
 * pushbutton attached to pin 2 from +5V
 * 10K resistor attached to pin 2 from ground
 * LED attached from pin 13 to ground (or use the built-in LED on
   most Arduino boards)
 
 created  27 Sep 2005
 modified 30 Aug 2011
 by Tom Igoe

This example code is in the public domain.
 	
 http://arduino.cc/en/Tutorial/ButtonStateChange
 
 */

// this constant won't change:
const int ledPin = 13;       // the pin that the LED is attached to

const int right_buttonPin = 8;    // the pin that the pushbutton is attached to
const int right_highPin = 9;
const int right_lowPin = 10;

const int left_buttonPin = 1;    // the pin that the pushbutton is attached to
const int left_highPin = 2;
const int left_lowPin = 3;

// Variables will change:
int right_buttonPushCounter = 0;   // counter for the number of button presses
int right_buttonState = 0;         // current state of the button
int right_lastButtonState = 0;     // previous state of the button

int left_buttonPushCounter = 0;   // counter for the number of button presses
int left_buttonState = 0;         // current state of the button
int left_lastButtonState = 0;     // previous state of the button

void setup() {
  // initialize the button pin as a input:
  pinMode(right_buttonPin, INPUT);
  pinMode(left_buttonPin, INPUT);
  // initialize the LED as an output:
  pinMode(ledPin, OUTPUT);

  pinMode(right_highPin, OUTPUT);
  digitalWrite(right_highPin, HIGH);

  pinMode(right_lowPin, OUTPUT);
  digitalWrite(right_lowPin, LOW);
  
  pinMode(left_highPin, OUTPUT);
  digitalWrite(left_highPin, HIGH);

  pinMode(left_lowPin, OUTPUT);
  digitalWrite(left_lowPin, LOW);
  
  // initialize serial communication:
  Serial.begin(9600);
}


void loop() {
  // read the pushbutton input pin:
  right_buttonState = digitalRead(right_buttonPin);
  left_buttonState = digitalRead(right_buttonPin);

  // compare the buttonState to its previous state
  if (right_buttonState != right_lastButtonState) {
    // if the state has changed, increment the counter
    if (right_buttonState == HIGH) {
      // if the current state is HIGH then the button
      // wend from off to on:
      right_buttonPushCounter++;
      Serial.println("on right");
      Serial.print("number of right wheel:  ");
      Serial.println(right_buttonPushCounter);
    } 
    else 
    {
      // if the current state is LOW then the button
      // went from on to off:
      Serial.println("off right"); 
    }
  }
  // save the current state as the last state, 
  //for next time through the loop
  right_lastButtonState = right_buttonState;

  // compare the buttonState to its previous state
  if (left_buttonState != left_lastButtonState) {
    // if the state has changed, increment the counter
    if (left_buttonState == HIGH) {
      // if the current state is HIGH then the button
      // wend from off to on:
      left_buttonPushCounter++;
      Serial.println("on left");
      Serial.print("number of left wheel:  ");
      Serial.println(left_buttonPushCounter);
    } 
    else 
    {
      // if the current state is LOW then the button
      // went from on to off:
      Serial.println("off left"); 
    }
  }
  // save the current state as the last state, 
  //for next time through the loop
  left_lastButtonState = left_buttonState;
  
  // turns on the LED every four button pushes by 
  // checking the modulo of the button push counter.
  // the modulo function gives you the remainder of 
  // the division of two numbers:
  if (right_buttonPushCounter % 4 == 0 || left_buttonPushCounter % 4 == 0)
  {
    digitalWrite(ledPin, HIGH);
  } 
  else 
  {
   digitalWrite(ledPin, LOW);
  }
  
}









