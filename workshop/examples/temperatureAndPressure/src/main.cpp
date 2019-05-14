#include <Arduino.h>
#include <Wire.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

#define SEALEVELPRESSURE_HPA (1033.90f)

float initialPressure = 0.0f;


#define I2C_SDA_PIN          22
#define I2C_SCL_PIN          23

// BMP280
#define BMP280_I2C_ADDR 0x76 // 0x76 and 0x77

Adafruit_BMP280 bmp; // I2C

void setupBMP280()
{
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  if (!bmp.begin(BMP280_I2C_ADDR)) {
      Serial.println("Could not find a valid BMP280 sensor, check wiring!");
      return;
  }
  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}

void setup() {
    Serial.begin(115200);
    Serial.println(F("tingg temperature and air pressure sensor"));

    setupBMP280();

    Serial.println(); // gap
}

void loop() {

  Serial.print("Temperature = ");
  float temperature = bmp.readTemperature();
  Serial.print(temperature);
  Serial.println(" *C");

  Serial.print("Pressure = ");
  float pressure = bmp.readPressure() / 100.0F;
  Serial.print(pressure);
  Serial.println(" hPa");

  if(pressure > initialPressure)
  {
    initialPressure = pressure;
  }

  Serial.print("Relative Altitude = ");
  Serial.print(bmp.readAltitude(initialPressure));
  Serial.println(" m");

  delay(1000);
}
