#include <Chrono.h>
//Chrono sharp;
Chrono wait1(Chrono::MICROS);                                                    // timer for 10 second intervals for publishing level sensor to Serial
Chrono wait2(Chrono::MICROS);

//ThingsBoard and Wifi Setup
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
//#include <ArduinoJson.h>
#include <ThingsBoard.h>

#define WIFI_AP "WebPocket-1DAF"
#define WIFI_PASSWORD "557CHTEX"

#define TOKEN "QdqQZ2cQe1OftSn1o2I1"
char thingsboardServer[] = "demo.thingsboard.io";

WiFiClient wifiClient;
ThingsBoard tb(wifiClient);

int status = WL_IDLE_STATUS;
unsigned long lastSend;
/////////////////////////////////////////////////////
//BME680 Setup
#include <EEPROM.h>
//#include "uptime.h"
#include "bsec.h"
#include "bsec_serialized_configurations_iaq.h"

#define STATE_SAVE_PERIOD	UINT32_C(360 * 60 * 1000) // 360 minutes - 4 times a day

// Helper functions declarations
void checkIaqSensorStatus(void);
void errLeds(void);
void loadState(void);
void updateState(void);

// set address
#define     I2C_SDA_PIN                D2
#define     I2C_SCL_PIN                D1
#define     BME680_ADDRESS             0x77

// Create an object of the class Bsec
Bsec iaqSensor;
uint8_t bsecState[BSEC_MAX_STATE_BLOB_SIZE] = {0};
uint16_t stateUpdateCounter = 0;

String output;

float       Temp                   =     0;
float       Hum                    =     0;
float       IAQ                    =     0;
float       CO2Eq                  =     0;
float       bVOC                   =     0;
//////////////////////////////////////////////////////////
//Dust sensor Specifications
const int   sharpLEDPin         =      D5;   // Digital pin 5 connect to sensor LED.
const int   sharpVoPin          =      A0;   // Analog pin 0 connect to sensor Vo.


#define     USE_AVG                          // For averaging last N raw voltage readings.
#ifdef      USE_AVG                               
#define     N                          100
static unsigned long VoRawTotal =      0;
static int  VoRawCount          =      0;
#endif // USE_AVG


static float Voc                 =     0.6;   // Typical output voltage in Volts when there is zero dust.
const float K                    =     0.5;   // Typical sensitivity in units of V per 100ug/m3.

float       dustDensity          =     0;
float       dV                   =     0;
/////////////////////////////////////////////////////////////////////////////////////
void InitWiFi()
{
  Serial.println("Connecting to AP ...");
  // attempt to connect to WiFi network

  WiFi.begin(WIFI_AP, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to AP");
}

// Entry point for the example
void setup() {

  EEPROM.begin(BSEC_MAX_STATE_BLOB_SIZE + 1); // 1st address for the length
  Serial.begin(115200);
  
  //Start BME680 Sensor
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  iaqSensor.begin(BME680_ADDRESS, Wire);
  
  //output = "\nBSEC library version " + String(iaqSensor.version.major) + "." + String(iaqSensor.version.minor) + "." + String(iaqSensor.version.major_bugfix) + "." + String(iaqSensor.version.minor_bugfix);
  //Serial.println(output);
  checkIaqSensorStatus();

  iaqSensor.setConfig(bsec_config_iaq);
  checkIaqSensorStatus();

  loadState();

  bsec_virtual_sensor_t sensorList[10] = {
    BSEC_OUTPUT_RAW_TEMPERATURE,
    BSEC_OUTPUT_RAW_PRESSURE,
    BSEC_OUTPUT_RAW_HUMIDITY,
    BSEC_OUTPUT_RAW_GAS,
    BSEC_OUTPUT_IAQ,
    BSEC_OUTPUT_STATIC_IAQ,
    BSEC_OUTPUT_CO2_EQUIVALENT,
    BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
  };

  iaqSensor.updateSubscription(sensorList, 10, BSEC_SAMPLE_RATE_LP);
  checkIaqSensorStatus();

  // Print the header
  output = "Timestamp [ms], RWTemperature [°C], Pressure [hPa], RWHumidity [%], Gas [Ohm], IAQ, sIAQ, CO2Eq [ppm], bVOC [ppm], HCTemperature [°C], HCHumidity [%]";
  Serial.println(output);
 
  InitWiFi();
  lastSend = 0;
}

// Function that is looped forever
void loop() {
  
  GetBMEData();
  GetSHARPData();
  
  if ( !tb.connected() ) {
    reconnect();
  }
  
  if ( millis() - lastSend > 3500 ) { // Update and send only after 3.5 seconds
    SendBMEData();
    SendSHARPData();
    lastSend = millis();
  }

  tb.loop();
}

void GetSHARPData() {  
    
  digitalWrite(sharpLEDPin, LOW);               // Turn on the dust sensor LED by setting digital pin LOW.
  wait1.restart();                              // Add a 280us delay between sensor readings
  while (wait1.hasPassed(280)) {}
  //delayMicroseconds(280);                     // Wait 0.28ms before taking a reading of the output voltage as per spec.
  int VoRaw = analogRead(sharpVoPin);           // Record the output voltage. This operation takes around 100 microseconds.
  digitalWrite(sharpLEDPin, HIGH);              // Turn the dust sensor LED off by setting digital pin HIGH.
  wait2.restart();                              // Add a 9620us delay between sensor readings
  while (wait2.hasPassed(9620)) {}
  //delayMicroseconds(9620);                    // Wait for remainder of the 10ms cycle = 10000 - 280 - 100 microseconds.

  //float Vo = VoRaw;
  
  float Vo = VoRaw;                             // Smoothing data by averaging
  #ifdef USE_AVG
  VoRawTotal += VoRaw;
  VoRawCount++;
  if ( VoRawCount >= N ) {
    Vo = 1.0 * VoRawTotal / N;
    VoRawCount = 0;
    VoRawTotal = 0;
  } else {
    return;
  }
  #endif // USE_AVG
  
  
  Vo = Vo / 1024.0 * 3.3;                        // Compute the output voltage in Volts.
  dV = Vo - Voc;                                 // Convert to Dust Density in units of ug/m3.
  if ( dV < 0 ) {
    dV = 0;
    Voc = Vo;
  }
  dustDensity = dV / K * 100.0;
  //Serial.println("Vo" + String(Vo*1000.0) + "mV");
  //Serial.println("Dust Density" + String(dustDensity) + "ug/m3");
  
}

void SendSHARPData() {  
  //if (sharp.hasPassed(3000)) {
    //sharp.restart();
    tb.sendTelemetryFloat("Dust Density", dustDensity);
  //}
}

void GetBMEData()
{  

  unsigned long time_trigger = millis();
  if (iaqSensor.run()) { // If new data is available
    output = String(time_trigger);
    output += ", " + String(iaqSensor.rawTemperature);
    output += ", " + String(iaqSensor.pressure / 1e2);
    output += ", " + String(iaqSensor.rawHumidity);
    output += ", " + String(iaqSensor.gasResistance / 1e3);
    output += ", " + String(iaqSensor.iaq);
    output += ", " + String(iaqSensor.staticIaq);
    output += ", " + String(iaqSensor.co2Equivalent);
    output += ", " + String(iaqSensor.breathVocEquivalent);
    output += ", " + String(iaqSensor.temperature);
    output += ", " + String(iaqSensor.humidity);
    
    Serial.println(output);

    Temp = iaqSensor.temperature;
    Hum = iaqSensor.humidity;
    IAQ = iaqSensor.staticIaq;
    CO2Eq = iaqSensor.co2Equivalent;
    bVOC = iaqSensor.breathVocEquivalent;
    
    updateState();
  } else {
    checkIaqSensorStatus();
  }
  
}

void SendBMEData() 
{

  /*
  Serial.println("Sending data to ThingsBoard:");
  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.print(" %\t");
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" *C ");
  */
  
  tb.sendTelemetryFloat("Temperature", Temp);
  tb.sendTelemetryFloat("Humidity", Hum);
  tb.sendTelemetryFloat("IAQ", IAQ);
  tb.sendTelemetryFloat("CO2Eq", CO2Eq);
  tb.sendTelemetryFloat("bVOC", bVOC);
}

// Helper function definitions
void checkIaqSensorStatus(void)
{
  if (iaqSensor.status != BSEC_OK) {
    if (iaqSensor.status < BSEC_OK) {
      output = "BSEC error code : " + String(iaqSensor.status);
      Serial.println(output);
      for (;;)
        errLeds(); // Halt in case of failure 
    } else {
      output = "BSEC warning code : " + String(iaqSensor.status);
      Serial.println(output);
    }
  }

  if (iaqSensor.bme680Status != BME680_OK) {
    if (iaqSensor.bme680Status < BME680_OK) {
      output = "BME680 error code : " + String(iaqSensor.bme680Status);
      Serial.println(output);
      for (;;)
        errLeds(); // Halt in case of failure 
    } else {
      output = "BME680 warning code : " + String(iaqSensor.bme680Status);
      Serial.println(output);
    }
  }
  iaqSensor.status = BSEC_OK;
}

void errLeds(void)
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
}

void loadState(void)
{
  if (EEPROM.read(0) == BSEC_MAX_STATE_BLOB_SIZE) {
    // Existing state in EEPROM
    Serial.println("Reading state from EEPROM");

    for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE; i++) {
      bsecState[i] = EEPROM.read(i + 1);
      Serial.println(bsecState[i], HEX);
    }

    iaqSensor.setState(bsecState);
    checkIaqSensorStatus();
  } else {
    // Erase the EEPROM with zeroes
    Serial.println("Erasing EEPROM");

    for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE + 1; i++)
      EEPROM.write(i, 0);

    EEPROM.commit();
  }
}

void updateState(void)
{
  bool update = false;
  // Set a trigger to save the state. Here, the state is saved every STATE_SAVE_PERIOD with the first state being saved once the algorithm achieves full calibration, i.e. iaqAccuracy = 3
  if (stateUpdateCounter == 0) {
    if (iaqSensor.iaqAccuracy >= 3) {
      update = true;
      stateUpdateCounter++;
    }
  } else {
    // Update every STATE_SAVE_PERIOD milliseconds
    if ((stateUpdateCounter * STATE_SAVE_PERIOD) < millis()) {
      update = true;
      stateUpdateCounter++;
    }
  }

  if (update) {
    iaqSensor.getState(bsecState);
    checkIaqSensorStatus();

    Serial.println("Writing state to EEPROM");

    for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE ; i++) {
      EEPROM.write(i + 1, bsecState[i]);
      Serial.println(bsecState[i], HEX);
    }

    EEPROM.write(0, BSEC_MAX_STATE_BLOB_SIZE);
    EEPROM.commit();
  }
}


void reconnect() 
{
  // Loop until we're reconnected
  while (!tb.connected()) {
    status = WiFi.status();
    if ( status != WL_CONNECTED) {
      WiFi.begin(WIFI_AP, WIFI_PASSWORD);
      while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
      }
      Serial.println("Connected to AP");
    }
    Serial.print("Connecting to ThingsBoard node ...");
    if ( tb.connect(thingsboardServer, TOKEN) ) {
      Serial.println( "[DONE]" );
    } else {
      Serial.print( "[FAILED]" );
      Serial.println( " : retrying in 5 seconds]" );
      // Wait 5 seconds before retrying
      delay( 5000 );
    }
  }
}
