#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
  
#define DEBUG
#define L1 13   //output pin to control Lamp state
  
//WIFI configuration
const char* ssid = "WebPocket-1DAF";           // SSID 
const char* password =  "XXXXXX";              // password
  
//MQTT broker information - Verify the information generated by CloudMQTT
const char* mqttServer = "farmer.cloudmqtt.com";   //server
const char* mqttUser = "ubvsasrg";              // user
const char* mqttPassword = "_TJQSMJcJptJ";      // password
const int mqttPort = 14124;                     // port
const char* mqttTopicSub ="home/L1";            // topic
  

WiFiClient espClient;
  
PubSubClient client(espClient);
  
void setup() {
 Serial.begin(115200);
 pinMode(L1, OUTPUT);
 WiFi.begin(ssid, password);
 while (WiFi.status() != WL_CONNECTED) {
   delay(500);
   #ifdef DEBUG
   Serial.println("Connected to WiFi network..");
   #endif
 }
 #ifdef DEBUG
 Serial.println("Connecting to a WiFi network");
 #endif
 client.setServer(mqttServer, mqttPort);
 client.setCallback(callback);
 while (!client.connected()) {
   #ifdef DEBUG
   Serial.println("Connecting to MQTT Broker...");
   #endif
   if (client.connect("ESP8266Client", mqttUser, mqttPassword )) {
     #ifdef DEBUG
     Serial.println("Connected");  
     #endif
   } else {
     #ifdef DEBUG 
     Serial.print("state failure  ");
     Serial.print(client.state());
     #endif
     delay(2000);
   }
 }
 //Topic
 client.subscribe(mqttTopicSub);
}
  
void callback(char* topic, byte* payload, unsigned int length) {
 //stores the received message in a string
 payload[length] = '\0';
 String strMSG = String((char*)payload);
 #ifdef DEBUG
 Serial.print("The message arrived from the topic: ");
 Serial.println(topic);
 Serial.print("message:");
 Serial.print(strMSG);
 Serial.println();
 Serial.println("-----------------------");
 #endif
 //set output pin
 if (strMSG == "1"){        //if msg equal "1"
    digitalWrite(L1, LOW);  //output LOW to turn on the Lamp -> the RELAY module used has inverted logic . If necessary adjust for your module
 }  else if (strMSG == "0"){   //if msg equal "0"
    digitalWrite(L1, HIGH);   //output LOW to turn off the Lamp -> the RELAY module used has inverted logic. If necessary adjust for your module
 }
}
  
//Function to reconnect MQTT broker
void reconect() {
 //while disconnected
 while (!client.connected()) {
   #ifdef DEBUG
   Serial.print("Trying connect to MQTT broker");
   #endif
   bool connected = strlen(mqttUser) > 0 ?
                    client.connect("ESP8266Client", mqttUser, mqttPassword) :
                    client.connect("ESP8266Client");
   if (connected) {
     #ifdef DEBUG
     Serial.println("Connected!");
     #endif
     //subscribe to the topic
     client.subscribe(mqttTopicSub, 1); 
   } else {
     #ifdef DEBUG
     Serial.println("Failed during connection. Code: ");
     Serial.println( String(client.state()).c_str());
     Serial.println("Retrying in 10 sec");
     #endif
     //Waiting 10 seconds
     delay(10000);
   }
 }
}
  
void loop() {
 if (!client.connected()) {
   reconect();
 }
 client.loop();
}
