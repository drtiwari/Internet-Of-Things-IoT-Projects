#include <elapsedMillis.h>
elapsedMillis timeElapsed;

int RPWM = 10;   
int LPWM = 11;
int sensorPin = A0;
int potPin = A1;
int potVal;

int sensorVal;
int Speed = 255;
int Buffer = 4;

int maxAnalogReading;
int minAnalogReading;

void setup() {
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(sensorPin, INPUT);
  pinMode(potPin, INPUT);
  Serial.begin(9600);
  maxAnalogReading = moveToLimit(1);
  minAnalogReading = moveToLimit(-1);
}

void loop(){
  potVal = map(analogRead(potPin), 0, 1023, minAnalogReading, maxAnalogReading);
  sensorVal = analogRead(sensorPin);
  if(potVal > (sensorVal+Buffer)){               //addition gives buffer to prevent actuator from rapidly vibrating due to noisy data inputs
    driveActuator(1, Speed);
  }
  else if(potVal < (sensorVal-Buffer)){             
    driveActuator(-1, Speed);
  }
  else{
    driveActuator(0, Speed);
  }
  Serial.print("Potentiometer Reading: ");
  Serial.print(potVal);
  Serial.print("\tActuator reading: ");
  Serial.println(sensorVal);
  delay(10);
}

int moveToLimit(int Direction){
  int prevReading=0;
  int currReading=0;
  do{
    prevReading = currReading;
    driveActuator(Direction, Speed);
    timeElapsed = 0;
    while(timeElapsed < 200){ delay(1);}           //keep moving until analog reading remains the same for 200ms
    currReading = analogRead(sensorPin);
  }while(prevReading != currReading);
  return currReading;
}

void driveActuator(int Direction, int Speed){
  switch(Direction){
    case 1:       //extension
      analogWrite(RPWM, Speed);
      analogWrite(LPWM, 0);
      break;
   
    case 0:       //stopping
      analogWrite(RPWM, 0);
      analogWrite(LPWM, 0);
      break;

    case -1:      //retraction
      analogWrite(RPWM, 0);
      analogWrite(LPWM, Speed);
      break;
  }
}
