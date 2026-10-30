#include <Wire.h>

// You must change these pins to match yours. They do change from device to device. 
const int I2C_SDA = 5;  // GPIO5
const int I2C_SCL = 4;  // GPIO4

#define DS3231M_ADDRESS 0x68
#define AGING_OFFSET_REGISTER 0x10

void setDS3231MAgingOffset(int8_t value)
{
  Wire.beginTransmission(DS3231M_ADDRESS);
  Wire.write(AGING_OFFSET_REGISTER);
  Wire.write((uint8_t)value);

  uint8_t error = Wire.endTransmission();

  Serial.println();
  Serial.println(F("DS3231M AGING OFFSET WRITE"));

  if (error == 0) {
    Serial.printf("Value : %d\n", value);
    Serial.printf("Raw   : 0x%02X\n", (uint8_t)value);
    Serial.println(F("Write OK"));
  } else {
    Serial.printf("I2C ERROR : %u\n", error);
  }
}

void setup()
{
  Serial.begin(115200);

  delay (2000); 

  Wire.begin(I2C_SDA, I2C_SCL);

  delay(500);

  Serial.println("I2C initialized");
  
  setDS3231MAgingOffset(0); // Default is 0, change this to your desired Aging Register value.
}

void loop()
{
  Serial.println(F("----------------------------------------"));
  Serial.println(F("You are done. Aging register programmed."));
  Serial.println(F("Reboot and load your shiny new firmware."));
  Serial.println(F("Don't leave this firmware loaded; otherwise, it will re-write the AG every time you reboot."));
  Serial.println(F("Constantly re-writing the AG is not exactly recommended."));
  Serial.println(F("----------------------------------------"));

  delay(10000);
}
