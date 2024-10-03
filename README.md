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
 
 Hookup guide:
 
 - connect D0 pin to RST pin in order to enable the ESP8266 to wake up periodically
   (only after uploading the sketch!)
 - connect the Vout of your voltage divider to NodeMCU A0
 - connect the bme280:
      BME280 GND --> NodeMCU GND
      BME280 3.3V --> NodeMCU 3V3
      BME280 SDA --> NodeMCU D2
      BME280 SCL --> NodeMCU D1

      On ESP12E module pullup MISO 10k to Vcc if DeepSleep does not work!!!
     
 This sketch is the result of the compilation of some diyers and arduinoers. Thanks to
 them!
 And of course it is in public domain.


P.S. Added esp32c3 version with aht20 sensor. It work no goood with low Vcc. You need add dc-dc converter like here https://mysku.club/blog/diy/95537.html/



 Enjoy!
 
