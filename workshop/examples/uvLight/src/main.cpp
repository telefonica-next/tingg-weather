#include <Arduino.h>
#include <Wire.h>

#define I2C_SDA_PIN          23
#define I2C_SCL_PIN          22

//  VEML6070 with Rset=270k on breakout => UVA sensitivity: 5.625 uW/cm²/step
#define VEML6070_I2C_ADDR 0x38 //0x38 and 0x39
// Integration Time
#define IT_1_2 0x0 //1/2T
#define IT_1   0x1 //1T
#define IT_2   0x2 //2T
#define IT_4   0x3 //4T

void setupVEML6070()
{
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.beginTransmission(VEML6070_I2C_ADDR);
  Wire.write((IT_1<<2) | 0x02);
  Wire.endTransmission();
  delay(500);
}

uint16_t readVEML6070()
{
  byte msb=0, lsb=0;
  uint16_t uv;

  Wire.requestFrom(VEML6070_I2C_ADDR+1, 1); //MSB
  delay(1);
  if(Wire.available())
  {
    msb = Wire.read();
  }
  else
  {
    return 0;
  }

  Wire.requestFrom(VEML6070_I2C_ADDR+0, 1); //LSB
  delay(1);
  if(Wire.available())
  {
    lsb = Wire.read();
  }
  else
  {
    return 0;
  }

  uv = (msb<<8) | lsb;

  return uv;
}


void setup() {
    Serial.begin(115200);
    Serial.println(F("tingg uv sensor"));

    setupVEML6070();

    Serial.println(); // gap
}

void loop() {
  uint16_t uv = readVEML6070();
  Serial.print("UV A = ");
  Serial.print(uv);
  Serial.println(" steps");

  float uv_translated = 5.625f * uv;
  Serial.print("UV A = ");
  Serial.print(uv_translated);
  Serial.println(" uW/cm^2");
  delay(1000);
}
