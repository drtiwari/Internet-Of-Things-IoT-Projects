#include <Arduino.h>
#include <TM1637Display.h>

#define CLK 5 
#define DIO 3

#define TEST_DELAY 1000

TM1637Display display(CLK, DIO);

const int trigPin = 2, echoPin = 4;

int i;
float value = 0;
const int numReadings = 10;
float duration, distance;

void setup() {
   
pinMode(trigPin, OUTPUT);
pinMode(echoPin, INPUT);
Serial.begin(115200);

} 

void loop() {

  delay(TEST_DELAY);
  dismeasure();
  smoothing();
  printresults();
  displayresults();
  
}

void dismeasure(){

  digitalWrite(trigPin, LOW); 
  delayMicroseconds(2); 
  digitalWrite(trigPin, HIGH); 
  delayMicroseconds(10); 
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);

}

void smoothing(){

  for (i = 0; i < numReadings; i++){
    value = value + duration;
    delay(1);
  }
  
  value = value/numReadings;
  distance = value*340/20000;
  
}

void printresults(){
  
  Serial.print(distance);
  Serial.print("cm"); 
  Serial.println(); 
  
}

void displayresults(){

  display.setBrightness(0x0f);
  uint8_t data[] = { 0x0, 0x0, 0x0, 0x0 };
  display.setSegments(data);
  display.showNumberDec(distance, false, 4,0);
  
}
