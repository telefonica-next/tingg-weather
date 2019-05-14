#include <Arduino.h>
#include <Wire.h>

#include <Adafruit_Sensor.h>
#include <VEML7700.h>

#define I2C_SDA_PIN          23
#define I2C_SCL_PIN          22

VEML7700 veml7700;

void setupVEML7700()
{
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  veml7700.begin();
}

float readVEML7700()
{
    float lux;
    veml7700.getALSLux(lux);
    return lux;
}

void setup() {
    Serial.begin(115200);
    Serial.println(F("tingg lux sensor"));

    setupVEML7700();
    Serial.println(); // gap
}

void loop() {
  float lux = readVEML7700();
  Serial.print("Lux = ");
  Serial.println(lux);
  delay(1000);
}
