#include <Wire.h>
#include <DHT.h>

#define DHT11_pin A0
#define DHT_Type DHT11

DHT DHT11_Sensor(DHT11_pin, DHT_Type);

void setup() {
  Wire.begin(1); //address to first slave
  Serial.begin(9600);
  delay(500);
  DHT11_Sensor.begin();
  Wire.onRequest(requestEvent);
}

float temp = 0 ;
float hum = 0 ;

unsigned long start_time =  0 ;

void loop() {
  unsigned long now = millis();
  if (now - start_time > 500)
  {
    start_time = now ;
    hum = DHT11_Sensor.readHumidity();
    temp    = DHT11_Sensor.readTemperature() ;
      Serial.println( "reading -- success");
  }
}


void requestEvent() {
  String data = String(hum, 1) + "," + String(temp, 1);
  Wire.write((uint8_t*)data.c_str(), data.length());
  Serial.print(">> sending over I2C: ");
  Serial.println(data);


}
