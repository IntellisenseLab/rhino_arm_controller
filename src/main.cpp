#include <Arduino.h>

typedef enum {  NONE, M1 } states;

states state = NONE;

// encoder connections
int encoderPinA = 2;
int encoderPinB = 3;
int limiterPin  = 4;

// Motor connections
int pwmPin = 5;
int enableForwardPin = 6;
int enableBackwardPin = 7;

// led for limiter
int led = 13;

volatile int count          = 0;
volatile bool statePhaseA    = false;
volatile bool statePhaseB    = false;

int ProtectedCount = 0;
int TargetCount = 0;

int inputValue = 0;

void isrA() {
  statePhaseA = digitalRead(encoderPinA) == LOW;
  count += (statePhaseA != statePhaseB) ? -1 : +1;
}

void isrB() {
  statePhaseB = digitalRead(encoderPinB) == LOW;
  count += (statePhaseA == statePhaseB) ? -1 : +1;
}

void setup() {
  Serial.begin(115200);

  // Set all the motor control pins to outputs
  pinMode(pwmPin, OUTPUT);
  pinMode(enableForwardPin, OUTPUT);
  pinMode(enableBackwardPin, OUTPUT);

  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  pinMode(limiterPin, INPUT_PULLUP);
  
  statePhaseA = (bool)digitalRead(encoderPinA);
  statePhaseB = (bool)digitalRead(encoderPinB);

  attachInterrupt(digitalPinToInterrupt(encoderPinA), isrA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), isrB, CHANGE);
  
  // Turn off motors - Initial state
  digitalWrite(enableForwardPin, LOW);
  digitalWrite(enableBackwardPin, LOW);
}

void setM1MotorTarget (const unsigned int value)
{
  TargetCount = 100;
}

void handleState ()
{
  switch (state)
  {
    case M1:
      setM1MotorTarget (inputValue);
      break;
    case NONE:
      break;
  } 
}

void processIncomingByte (const byte c)
{
  if (isdigit (c))
  {
    inputValue *= 10;
    inputValue += c - '0';
  }
  else 
  {
    // The end of the number signals a state change
    handleState ();

    // set the new state, if we recognize it
    switch (c)
    {
    case 'A':
      state = M1;
      break;
    default:
      state = NONE;
      break;
    } 
  }
  
}

void loop() {

  while (Serial.available ()) processIncomingByte(Serial.read ());

  noInterrupts();
  ProtectedCount = count;
  interrupts();

  analogWrite(pwmPin, 30);
  
  if(ProtectedCount == 100) 
  {
    digitalWrite(enableForwardPin, LOW);
    digitalWrite(enableBackwardPin, LOW);
  } 
  else if (ProtectedCount > 100)
  {
    digitalWrite(enableForwardPin, LOW);
    digitalWrite(enableBackwardPin, HIGH);
  }
  else if (ProtectedCount < 100)
  {
    digitalWrite(enableForwardPin, HIGH);
    digitalWrite(enableBackwardPin, LOW);
  }

  // if (readI == LOW)
  // {
  //   digitalWrite(enableForwardPin, LOW);
  //   digitalWrite(enableBackwardPin, LOW);
  //   digitalWrite(led, HIGH);
  // }

  Serial.println(ProtectedCount);
}
