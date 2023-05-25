#include "ESPHelper.h"
#include <Metro.h>

#define AVG_COUNT 50

const char* voltTopic = "/HOME/volt";
const char* ampTopic = "/HOME/amp";
const char* wattTopic = "/HOME/watt";

const char* hostnameStr = "PWR-Node";
const char* otaPass = "Your OTA Password";

netInfo homeNet = {  .mqttHost = "tailor.cloudmqtt.com",     //can be blank if not using MQTT
          .mqttUser = "fnuhrpik",   //can be blank
          .mqttPass = "0wNgF8gcdK1F",   //can be blank
          .mqttPort = 12708,         //default port for MQTT is 1883 - only chance if needed.
          .ssid = "WebPocket-1DAF", 
          .pass = "557CHTEX"};

ESPHelper myESP(&homeNet);

Metro powerMetro = Metro(30000);

void setup() {
  Serial.begin(115200);

  myESP.OTA_enable();
  myESP.OTA_setPassword(otaPass);
  myESP.OTA_setHostnameWithVersion(hostnameStr);
  myESP.setHopping(false);
  myESP.begin();
}

void loop(){
  int count = 0;

  //where to store the data to be averaged
  double watts[AVG_COUNT];
  double volts[AVG_COUNT];
  double amps[AVG_COUNT];

  //vars to maintain averages for all data points
  double wattAvg = 0;
  double voltAvg = 0;
  double ampAvg = 0;

  //the serial buffer of 64 bytes
  char serialBuf[64];

  while(1){

    //reset the count when we hit the max. The average acts and a rolling average
    if(count >= AVG_COUNT){
      count = 0;
    }

    //get data from serial line
    while(Serial.available()){
      // '*' marks the beginning of a transmission
      bool start = Serial.find('*');

      //parse out the floats
      if(start){
        volts[count] = Serial.parseFloat();
        amps[count] = Serial.parseFloat();
        watts[count++] = Serial.parseFloat(); 
        break;
      }
      delay(1);
    }

    //calculate averages
    wattAvg = 0;
    ampAvg = 0;
    voltAvg = 0;
    for(int i = 0; i < AVG_COUNT; i++){
      wattAvg += watts[i];
      voltAvg += volts[i];
      ampAvg += amps[i];
    }
    wattAvg /= AVG_COUNT;
    ampAvg /= AVG_COUNT;
    voltAvg /= AVG_COUNT;




    //only send the data every so often (set by the metro timer) and only when connected to WiFi and MQTT
    if(myESP.loop() == FULL_CONNECTION && powerMetro.check()){

      //post just watts
      char wattStr[10];
      dtostrf(wattAvg,4,1,wattStr);
      myESP.publish(wattTopic,wattStr, true);
      delay(5);

      //post just volts
      char voltStr[10];
      dtostrf(voltAvg,4,1,voltStr);
      myESP.publish(voltTopic,voltStr, true);
      delay(5);

      //post just amps
      char ampStr[10];
      dtostrf(ampAvg,4,1,ampStr);
      myESP.publish(ampTopic,ampStr, true); 
      delay(5);
    }

    yield();
  }
  
}
void callback(char* topic, uint8_t* payload, unsigned int length) {

}
