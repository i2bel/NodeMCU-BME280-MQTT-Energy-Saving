/*
 This sketch publishes temperature humidity and barometric pressure data from a bme280
 device to a MQTT topic.
 It also publish the battery voltage but you have to install a voltage divider. (see 
 https://en.wikipedia.org/wiki/Voltage_divider if needed).
 

        Vin <---o
              __|_
             |    |
             | R1 |
             |____|
                |                                       R2
                o-------> Vout --> A0         Vout = --------- x Vin
              __|_                                   (R1 + R2)
             |    |
             | R2 |                           Vout MUST BE < 1 V !!!
             |____|
                |                       (I choose R1 = 470 k and R2 = 100 k
               _|_                    but feel free to choose your own values!)
               /// Gnd
                
 
 This sketch goes in deep sleep mode once the data has been sent to the MQTT
 topic and wakes up periodically (configure SLEEP_DELAY_IN_SECONDS accordingly).
 

 
*/
//#include <ESP8266WiFi.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
//#include <Adafruit_Sensor.h>
//#include <GyverBME280.h>
#include <AHT20.h>
#define SLEEP_DELAY_IN_SECONDS  1800

AHT20 aht20;

float tempC ;    // Temperature in Celsius degrees
const char* MQTTtempC ;
float humidity ; // Relative Humidity in %
const char* MQTThumidity;
const char* MQTTtempOnBoard;
//float pressHpa ; // Barometric Pressure in hectopascals
//const char* MQTTpressure;
float level ;    // Battery voltage in volts
char temperatureString[6];
char humidityString[6];
//char pressHpaString[6];
char voltageString[6];
char temperatureonboardString[6];
String responseJson;

const char* ssid = "network"; // WiFi network SSID
const char* password = ".kgljgllkh;lh;lhlhlhk'lj'j;j"; // WiFi network password
const char* mqtt_server = "192.168.1.99"; // MQTT broker URL
const char* mqtt_username = "tyruytrytruy";      // MQTT client user name, if needed
const char* mqtt_password = "gkglj";   // MQTT client password, if needed

//We will use static ip
IPAddress ip(192, 168, 1, 38 );// pick your own suitable static IP address
IPAddress gateway(192, 168, 1, 1); // may be different for your network
IPAddress subnet(255, 255, 255, 0); // may be different for your network (but this one is pretty standard)
IPAddress dns(1, 1, 1, 1);

WiFiClient Esp1Client;

PubSubClient client(Esp1Client);

void setup() {

  // setup serial port
  Serial.begin(115200);
  Wire.begin(); //Join I2C bus
  esp_sleep_enable_timer_wakeup(uint64_t (SLEEP_DELAY_IN_SECONDS * 1000000));
  //Check if the AHT20 will acknowledge
  if (aht20.begin() == false)
  {
    Serial.println("AHT20 not detected. Please check wiring. Program stopped!");
    while (1);
  }
  Serial.println("AHT20 found!");

}


void setup_wifi()   // Connecting to the WiFi network
{
  while (WiFi.status() != WL_CONNECTED)
  {
    
    WiFi.config(ip, dns, gateway, subnet); 
    
    Serial.print("Connecting to WiFi network ");
    Serial.print(ssid);
   
      WiFi.begin(ssid, password);
      uint8_t trial_counter=10;

    while (WiFi.status() != WL_CONNECTED)
      {
        delay(500);
        Serial.print(".");
        trial_counter--;
        if (trial_counter==0)
        {
         Serial.println();
         Serial.println("WiFi not availale!!!");
         Serial.println("I'm going to Deep sleep!");
         esp_deep_sleep_start();
         //esp_sleep_enable_timer_wakeup(uint64_t (SLEEP_DELAY_IN_SECONDS * 1000000));
         //ESP.deepSleep(SLEEP_DELAY_IN_SECONDS * 1000000, WAKE_RF_DEFAULT);        
        }
      }

    Serial.println(" connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  }
}

void callback(char* topic, byte* payload, unsigned int length) // Reading received topic
{
  Serial.print("Reveived topic [");
  Serial.print(topic);
  Serial.print("] ");
  
  for (int i = 0; i < length; i++)
    {
      Serial.print((char)payload[i]);
    }
  Serial.println();
}

void connect_mqtt() //Connecting to the MQTT broker
{
  
  uint8_t mqtt_trial_counter=3;
  
  while (!client.connected())
   
    {
    Serial.print("Connecting to MQTT broker at ");
    Serial.print(mqtt_server);
    Serial.print("...");
            
    // Attempt to connect
    if (client.connect("MobSensor1", mqtt_username, mqtt_password))
      {
        Serial.println(" connected!");
        // Once connected, publish an announcement...
        Serial.println("Sending topic(s)...");
        client.publish("sensor1/external", responseJson.c_str(), true);
        client.publish("sensor1/test/voltage", dtostrf(level/100, 2, 2, voltageString), false);
        client.publish("sensor1/test/temperature", MQTTtempC, false);
        client.publish("sensor1/test/humidity", MQTThumidity, false);
        //client.publish("sensor1/test/pressure", MQTTpressure, false);
        client.publish("sensor1/test/chip_temperature", MQTTtempOnBoard, false);
      }
     else
      {
        mqtt_trial_counter--;
        Serial.print("Connection failed, MQTT_client_state=");
        Serial.println(client.state());
        Serial.println("will retry in 5 seconds");
        delay(500);
        Serial.print("Attempts left ");
        Serial.println(mqtt_trial_counter);
        
        if (mqtt_trial_counter==0)
        {
         Serial.println();
         Serial.println("MQTT Server not availale!!!");
         Serial.println("I'm going to Deep sleep!");
         esp_deep_sleep_start();
         //esp_sleep_enable_timer_wakeup(uint64_t (SLEEP_DELAY_IN_SECONDS * 1000000));
         //ESP.deepSleep(SLEEP_DELAY_IN_SECONDS * 1000000, WAKE_RF_DEFAULT);        
        }
        
        // Wait 5 seconds before retrying
        delay(5000);
      }
    }
}

void loop()
{
  

  if (aht20.available() == true)
  {
    //Reading temperature and humidity values
    float tempC = aht20.getTemperature();
    float humidity = aht20.getHumidity();
 
    // Reading voltage
    float rawLevel = analogRead(A0);
    level = (float)rawLevel / 10 / (10000. / (33400 + 10000)); // You need to adjust these values according to the voltage divider you install

    Serial.println("");
    Serial.println("Temperature:");
    printValueAndUnits(tempC, " °C");

    //Serial.println("Barometric Pressure:");
    //printValueAndUnits(pressHpa, " hPa");

    Serial.println("Relative Humidity:");
    printValueAndUnits(humidity, " %");

    Serial.println("Voltage: ");
    printValueAndUnits(level/100, " V");

    //Internal esp32c3 temperature sensor start
  float temp_core_celsius = temperatureRead();
  
  Serial.print("Temperature onBoard:");
  printValueAndUnits(temp_core_celsius, " °C");
  
    //


    responseJson = "";
    responseJson += "{";
    responseJson +=     "\"temperature\":" + String(tempC) + ",";
    responseJson +=     "\"humidity\":" + String(humidity) + ",";
    //responseJson +=     "\"pressure\":" + String(pressHpa) + ",";
    responseJson +=     "\"voltage\":" + String(level/100);
    responseJson += "}";
    
    MQTTtempC =  dtostrf(tempC, 2, 2, temperatureString);
    MQTThumidity = dtostrf(humidity, 2, 2, humidityString);
    //MQTTpressure = dtostrf(pressure, 2, 2, pressHpaString);
    MQTTtempOnBoard = dtostrf(temp_core_celsius, 2, 2, temperatureonboardString);
    
    setup_wifi();
    
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);
    connect_mqtt();

    delay(100);
    
    Serial.println("Closing connection to MQTT broker...");
    client.disconnect();

    delay(100);

    Serial.print("Closing connection to WiFi network ");
    Serial.print(ssid);
    Serial.println("...");
    WiFi.disconnect();

    Serial.print("Going to sleep for ");
    Serial.print(SLEEP_DELAY_IN_SECONDS);
    Serial.println(" seconds...");
    delay(100);
   
    esp_deep_sleep_start();
    //ESP.deepSleep(SLEEP_DELAY_IN_SECONDS * 1000000, WAKE_RF_DEFAULT);
    //esp_sleep_enable_timer_wakeup(uint64_t (SLEEP_DELAY_IN_SECONDS * 1000000));
    delay(100);
}
}  
  
void printValueAndUnits(float value, String units)
{
    Serial.print("   ");
    Serial.print(value);
    Serial.print(" ");
    Serial.println(units);
    Serial.println("");
}
