#include <Arduino.h>

typedef enum {  NONE, M1 } states;

states state = NONE;

#define readA bitRead(PIND,22) //faster than digitalRead() and attached to Pin A
#define readB bitRead(PIND,24) //faster than digitalRead() and attached to Pin B
#define readO bitRead(PIND,26) //faster than digitalRead() and attached to Pin B

// Motor connections
int enA = 2;
int in1 = 28;
int in2 = 30;

// led for limiter
int led = 13;

// encoder connections
int encoderPinA = 22;
int encoderPinB = 24;
int limiterPin  = 26;

volatile int count = 0;
volatile bool hitLimiter = false;

int ProtectedCount = 0;
int TargetCount = 0;

int inputValue = 0;

void isrA() {
  if(readB != readA) 
  {
    count ++;
  } 
  else 
  {
    count --;
  }
}

void isrB() {
  if (readA == readB) 
  {
    count ++;
  } 
  else 
  {
    count --;
  }
}

void isrStop() {
  if (readO == LOW) 
  {
    hitLimiter = true;
  }
}

void setup() {
  Serial.begin(115200);

	// Set all the motor control pins to outputs
	pinMode(enA, OUTPUT);
	pinMode(in1, OUTPUT);
	pinMode(in2, OUTPUT);

  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  pinMode(limiterPin, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(encoderPinA), isrA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), isrB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), isrStop, FALLING);
	
	// Turn off motors - Initial state
	digitalWrite(in1, LOW);
	digitalWrite(in2, LOW);
}

void setM1MotorTarget (const unsigned int value)
{
  TargetCount = value;
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

  analogWrite(enA, 255);
  
  if(ProtectedCount == TargetCount) 
  {
    digitalWrite(in1, LOW);
	  digitalWrite(in2, LOW);
  } 
  else if (ProtectedCount > TargetCount)
  {
    digitalWrite(in1, LOW);
	  digitalWrite(in2, HIGH);
  }
  else if (ProtectedCount < TargetCount)
  {
    digitalWrite(in1, HIGH);
	  digitalWrite(in2, LOW);
  }

  if (hitLimiter)
  {
    digitalWrite(in1, LOW);
	  digitalWrite(in2, LOW);
    digitalWrite(led, HIGH);
    Serial.println(ProtectedCount);
  }
}