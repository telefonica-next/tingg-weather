#include <Arduino.h>
#include <Wire.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <VEML7700.h>

#include <WiFi.h>
#include <PubSubClient.h>




const char* ssid = "<YOUR_SSID>";
const char* password = "<YOUR_WIFI_PASSWORD>";

const char* thing_id = "<YOUR-TINGG-THINGID>";
const char*  key = "<YOUR-TINGG-THINGKEY>";
const char* username = "thing";

const char* mqtt_server = "mqtt.tingg.io";



#define LEDPIN 13
#define BUTTONPIN 12

#define SEALEVELPRESSURE_HPA (1031.80f)

#define I2C_SDA_PIN          23
#define I2C_SCL_PIN          22

#define BMP_SDA_PIN          22
#define BMP_SCL_PIN          23

//  VEML6070 with Rset=270k on breakout => UVA sensitivity: 5.625 uW/cm²/step
#define VEML6070_I2C_ADDR 0x38 //0x38 and 0x39
// Integration Time
#define IT_1_2 0x0 //1/2T
#define IT_1   0x1 //1T
#define IT_2   0x2 //2T
#define IT_4   0x3 //4T

VEML7700 veml7700;

#define BMP280_I2C_ADDR 0x76 // 0x76 and 0x77
Adafruit_BMP280 bmp; // I2C


WiFiClient espClient;
PubSubClient client(espClient);

// Vars
int val;
char buf[12];
long lastMsg = 0;
char msg[50];
int value = 0;

void setup_wifi() {

  delay(10);

    // We start by connecting to a WiFi network
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);

    WiFi.mode(WIFI_STA);

    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
      Serial.println("");
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      randomSeed(micros());
}


void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect(thing_id, username, key)) {
      Serial.println("connected");
      client.subscribe("led");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}


String message(byte* payload, unsigned int length) {
  payload[length] = '\0';
  String s = String((char*)payload);
  return s;
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.println(topic);
  String msg = message(payload, length);
  Serial.print("received: ");
  Serial.println(msg);

  if (msg.equals("1") ) {
    digitalWrite(LEDPIN,HIGH);
  }
  else {
    digitalWrite(LEDPIN,LOW);
  }
}



void setupVEML6070()
{
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.beginTransmission(VEML6070_I2C_ADDR);
  Wire.write((IT_1<<2) | 0x02);
  Wire.endTransmission();
  delay(500);
}

void setupVEML7700()
{
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  veml7700.begin();
}

void setupBMP280()
{
  Wire.begin(BMP_SDA_PIN, BMP_SCL_PIN);
  if (!bmp.begin(BMP280_I2C_ADDR)) {
      Serial.println("Could not find a valid BMP280 sensor, check wiring!");
  }
}

uint16_t readVEML6070()
{
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
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

float readVEML7700()
{
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    float lux;
    veml7700.getALSLux(lux);
    return lux;
}

void setup() {
    Serial.begin(115200);
    Serial.println(F("tingg weather sensor"));

    setup_wifi();
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);

    pinMode(BUTTONPIN, INPUT);
    pinMode(LEDPIN, OUTPUT);
    setupVEML6070();
    setupVEML7700();
    setupBMP280();

    Serial.println(); // gap
}

void printValues() {
      setupVEML6070();
      uint16_t uv = readVEML6070();
      Serial.print("UV A = ");
      Serial.print(uv);
      Serial.println(" steps");

      float uv_translated = 5.625f * uv;
      Serial.print("UV A = ");
      Serial.print(uv_translated);
      Serial.println(" uW/cm^2");

      float lux = readVEML7700();
      //publish to mqtt
      String str_lux = String(lux);
      str_lux.toCharArray(msg, 50);
      Serial.print("Publish lux message: ");
      Serial.println(msg);
      client.publish("lux", msg);

      setupBMP280();
      float temperature = bmp.readTemperature();
      String str_temperature = String(temperature);
      str_temperature.toCharArray(msg, 50);
      Serial.print("Publish temperature message: ");
      Serial.println(msg);
      client.publish("temperature", msg);

      float pressure = bmp.readPressure() / 100.0f; //hPa
      String str_pressure = String(pressure);
      str_pressure.toCharArray(msg, 50);
      Serial.print("Publish pressure message: ");
      Serial.println(msg);
      client.publish("pressure", msg);

      int btnstate = digitalRead(BUTTONPIN);
      String str_button = String(btnstate);
      str_button.toCharArray(msg, 50);
      Serial.print("Publish button message: ");
      Serial.println(msg);
      client.publish("button", msg);

}

void loop() {
    if (!client.connected()) {
      reconnect();
    }
    client.loop();
    printValues();
    delay(10000);
}
