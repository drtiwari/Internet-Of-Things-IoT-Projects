//TIPPING BUCKET DEFINITIONS
#define       RAIN_PIN                          3               //Tipping bucket input
#define       CALC_INTERVAL                     1000            //Increment of measurements in milliseconds
#define       DEBOUNCE_TIME                     80              //Time in milliseconds to get through bounce noise

const double  bucketAmount        =             0.4089;         //mm equivalent of ml to trip the bucket
//const double  bucketAmount        =             0.3537; 
//const double  bucketAmount        =             0.3030; 
//const double  bucketAmount        =             0.2794; 

double        totalRain           =             0.0;
double        rainRatemm_hr       =             0.0;

unsigned long nextCalc;                                         //"Average Switch closure time is 135 ms"
unsigned long timer;                                            //"Bounce Settling Time: 0.75 ms"

unsigned int  rainTrigger         =             0;
unsigned long last_micros_rg;                                   //will store last time bucket tipped
unsigned long tipTime;

void countingRain() {
 if ((long)(micros() - last_micros_rg) >= DEBOUNCE_TIME) {
    tipTime = (micros() - last_micros_rg);						//Time taken to fill 1 bucket
    rainTrigger += 1;
    last_micros_rg = micros();
  }
}

void setup() {
 
 Serial.begin(115200);

 attachInterrupt(digitalPinToInterrupt(RAIN_PIN), countingRain, RISING);
 pinMode(RAIN_PIN, INPUT);
 nextCalc = millis() + CALC_INTERVAL;

}

void loop() {
 timer = millis();
 if (timer > nextCalc) {
  
   nextCalc = timer + CALC_INTERVAL;
   totalRain = rainTrigger * bucketAmount;
  
   //rainRate = bucketAmount(mm)/tipTime(hr)								    // 1 us = 2.777 x 10^-10 hr = 2.777e-10 hr
   //rainRate = (bucketAmount / tipTime)*36*pow(10,8);                            // Multiplied by 10 later due to rain machine
   rainRatemm_hr = 14688000.0 / tipTime;                            // Multiplied by 10 later due to rain machine
   if (rainRatemm_hr >= 0) {
    rainRatemm_hr = rainRatemm_hr;
   }
   else {
    rainRatemm_hr = 0;
   };

  //Serial.print("Total Tips: ");
  //Serial.print((float) rainTrigger);
  //Serial.print("; ");
  //Serial.print("Total Rain: ");
  Serial.print((float) totalRain);
  //Serial.print(" mm; ");
  Serial.print(", ");
  //Serial.print("Rain Rate: ");
  Serial.println((float) rainRatemm_hr);  
  //Serial.println(" mm/hr.");
 }
}
