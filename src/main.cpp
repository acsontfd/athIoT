#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define BZR_PIN 23
#define DS18B20_PIN 12
#define MPU6050_SDA_PIN 26
#define MPU6050_SCL_PIN 27
#define POT_PIN 34

const char* ssid = "Wokwi-GUEST";
const char* password = "";
const char* mqttServer = "broker.emqx.io";

WiFiClient espClient;
PubSubClient client(espClient);

Adafruit_MPU6050 mpu;

OneWire oneWire(DS18B20_PIN);
DallasTemperature sensors(&oneWire);

unsigned long lastMsg = 0;
float steps = 0;
float lastRotation = 0;
bool buzzerState = false;
unsigned long lastBuzzerToggle = 0;

void reconnect();

void setup() {
  Serial.begin(115200);
  pinMode(BZR_PIN, OUTPUT);

  // Initialize Buzzer
  digitalWrite(BZR_PIN, HIGH);
  delay(100);
  digitalWrite(BZR_PIN, LOW);

  // Initialize WiFi
  WiFi.begin(ssid, password);
  Serial.println("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected");
  Serial.println(WiFi.localIP());

  // Initialize MQTT
  client.setServer(mqttServer, 1883);
  reconnect();

  // Initialize sensors
  sensors.begin();

  // Initialize I2C for MPU6050
  Wire.begin(MPU6050_SDA_PIN, MPU6050_SCL_PIN);
  if (!mpu.begin(0x68, &Wire)) {
    Serial.println("MPU6050 not connected!");
    while (1);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  Serial.println("Setup done");
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  sensors.requestTemperatures();
  float temperature = sensors.getTempCByIndex(0);

  sensors_event_t acc, gyro, temp;
  mpu.getEvent(&acc, &gyro, &temp);

  float accX = acc.acceleration.x;
  float accY = acc.acceleration.y;
  float accZ = acc.acceleration.z;
  float acceleration = sqrt(accX * accX + accY * accY + accZ * accZ);

  float rotationX = gyro.gyro.x * 250 / 4.36;
  if (fabs(rotationX - lastRotation) > 90) {
    steps += 0.5;
  }
  lastRotation = rotationX;

  int potValue = analogRead(POT_PIN);
  float heartRate = map(potValue, 0, 4095, 60, 240);

  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.print("°C, Steps: ");
  Serial.print(steps);
  Serial.print(", Acceleration: ");
  Serial.print(acceleration);
  Serial.print("g, Heart Rate: ");
  Serial.print(heartRate);
  Serial.println(" bpm");

  unsigned long now = millis();
  if (now - lastMsg > 1000) {
    lastMsg = now;
    client.publish("/Temperature", String(temperature).c_str());
    client.publish("/Steps", String(steps).c_str());
    client.publish("/Acceleration", String(acceleration).c_str());
    client.publish("/HeartRate", String(heartRate).c_str());
  }

  if (temperature > 40.0 || acceleration > 1.9 || heartRate > 200) {
    if (now - lastBuzzerToggle > 1000) {
      buzzerState = !buzzerState;
      digitalWrite(BZR_PIN, buzzerState ? HIGH : LOW);
      lastBuzzerToggle = now;
    }
  } else {
    digitalWrite(BZR_PIN, LOW);
    buzzerState = false;
  }

  delay(100);
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Connecting to MQTT...");
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}