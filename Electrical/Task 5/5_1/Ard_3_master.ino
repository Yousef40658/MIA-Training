#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x3F, 16, 2);

void setup() {
  Wire.begin(); // Master does not need address
  Serial.begin(9600);

  //LCD
  lcd.init();
  lcd.backlight() ;
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Reading");
  lcd.setCursor(0,1) ;
  lcd.print("Sensors...");
  delay(2000);            //for DHT22 , DHT11 6 secs response according to
}

void loop() {
  float hum_1, temp_1; // Sensor 1
  float hum_2, temp_2; // Sensor 2

  get_data(1, &hum_1, &temp_1); // From Slave 1
  get_data(2, &hum_2, &temp_2); // From Slave 2

  float avg_hum = (hum_1 + hum_2) / 2;
  float avg_temp = (temp_1 + temp_2) / 2;

  //Serial
  Serial.print("Avg Humidity: ");
  Serial.print(avg_hum);
  Serial.print(" % , Avg Temperature: ");
  Serial.print(avg_temp);
  Serial.println(" C");

  Serial.println("-----------------------------------");

  //LCD
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Hum : ");
  lcd.print(avg_hum, 1);
  lcd.print("%") ;

  lcd.setCursor(0,1) ;
  lcd.print("Temp : ");
  lcd.print(avg_temp,1);
  lcd.print("C") ;


  delay(2000);
}
//////////////////
//Helper Functions 
//////////////////
void get_data(int slave_address, float* hum, float* temp) {
  Wire.requestFrom(slave_address, 40);
  delay(100); // Give slave time to respond

  String data = "";
  while (Wire.available()) {
    int val = Wire.read();
    if (val >= 32 && val <= 126) {
      data += (char)val;
  }
  }
  data.trim();
  parse_data(data, hum, temp);
}


void parse_data(String data, float* hum, float* temp) {
  int comma_index = data.indexOf(',');                    //returns index or , 
  if (comma_index > 0) {
    *hum = data.substring(0, comma_index).toFloat();      // From start to comma
    *temp = data.substring(comma_index + 1).toFloat();    // From after comma to end
  } else {
    //Serial.println("Error: No comma found in data!");
    *hum = 0;
    *temp = 0;
  }
}
