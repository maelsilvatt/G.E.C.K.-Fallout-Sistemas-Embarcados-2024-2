#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

// Sensores
#include <DHTesp.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Atuadores
#include <ESP32Servo.h>
#include <LiquidCrystal_I2C.h>

// FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/**************************************
 * Definições de Hardware
 **************************************/
// BMP280 (SPI)
#define BMP_SCK   18
#define BMP_MISO  19
#define BMP_MOSI  23
#define BMP_CS    5

// Servo Motor
#define SERVO_PWM 4

// LCD I2C
#define I2C_ADDR    0x27
#define LCD_COLUMNS 20
#define LCD_LINES   4

// Sensores
#define DHT_PIN    13
#define NTC_PIN    34
#define RELAY_PIN  12

// Constantes
#define BETA       3950

/**************************************
 * Instâncias de Sensores/Atuadores
 **************************************/
Adafruit_BMP280 bmp(BMP_CS);
DHTesp dhtSensor;
Adafruit_MPU6050 mpu;
LiquidCrystal_I2C lcd(I2C_ADDR, LCD_COLUMNS, LCD_LINES);
Servo servo;

/**************************************
 * Variáveis Globais
 **************************************/
bool relayState = false;
unsigned long lastSensorRead = 0;
unsigned long lastDisplayUpdate = 0;
const unsigned long SENSOR_INTERVAL = 2500;
const unsigned long DISPLAY_INTERVAL = 2000;

// Armazenamento de leituras
float temperatureDHT = 0;
float humidityDHT = 0;
float pressureBMP = 0;
float temperatureNTC = 0;
sensors_event_t accelEvent;

// Estados de exibição
enum DisplayState { HUMIDITY, PRESSURE, ACCEL, TEMP_NTC, NUM_STATES };
DisplayState currentDisplay = HUMIDITY;

/**************************************
 * Protótipos de Funções
 **************************************/
void setupPins();
void setupSensors();
void checkSensorConnection(bool success, const char* sensorName);
void readSensors();
void updateDisplay();
void controlServo(int angle);
void toggleRelay();

/**************************************
 * Implementação
 **************************************/
void setup() {
  Serial.begin(115200);
  setupPins();
  setupSensors();
  lcd.init();
  lcd.backlight();
}

void setupPins() {
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  servo.attach(SERVO_PWM, 500, 2400);
}

void setupSensors() {
  Wire.begin();
  
  checkSensorConnection(bmp.begin(), "BMP280");
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);

  checkSensorConnection(mpu.begin(), "MPU6050");
  dhtSensor.setup(DHT_PIN, DHTesp::DHT22);
}

void checkSensorConnection(bool success, const char* sensorName) {
  int attempts = 5;
  while(!success && attempts-- > 0) {
    Serial.printf("%s não detectado! Tentativas restantes: %d\n", sensorName, attempts);
    delay(1000);
  }
  if(attempts <= 0) Serial.printf("%s falhou!\n", sensorName);
}

void loop() {
  // Leitura periódica de sensores
  if(millis() - lastSensorRead > SENSOR_INTERVAL) {
    readSensors();
    lastSensorRead = millis();
  }

  // Atualização do display
  if(millis() - lastDisplayUpdate > DISPLAY_INTERVAL) {
    updateDisplay();
    lastDisplayUpdate = millis();
  }

  controlServo(90);  // Exemplo de posição fixa
  toggleRelay();

  delay(2000);
}

void readSensors() {
  // Leitura DHT22
  auto dhtData = dhtSensor.getTempAndHumidity();
  if(dhtSensor.getStatus() == 0) {
    temperatureDHT = dhtData.temperature;
    humidityDHT = dhtData.humidity;
  }

  // Leitura BMP280
  pressureBMP = bmp.readPressure() / 100.0F;

  // Leitura MPU6050
  mpu.getAccelerometerSensor()->getEvent(&accelEvent);

  // Leitura NTC
  int adc = analogRead(NTC_PIN);
  if(adc != 0 && adc != 4095) {
    float R = 10000.0 * (4095.0 / adc - 1.0);
    temperatureNTC = 1.0/(log(R/10000.0)/BETA + 1.0/298.15) - 273.15;
  }
}

void updateDisplay() {
  lcd.clear();
  
  switch(currentDisplay) {
    case HUMIDITY:
      lcd.setCursor(0, 0);
      lcd.print("Umidade:");
      lcd.setCursor(0, 1);
      lcd.print(String(humidityDHT, 1) + " %");
      break;
      
    case PRESSURE:
      lcd.setCursor(0, 0);
      lcd.print("Pressao:");
      lcd.setCursor(0, 1);
      lcd.print(String(pressureBMP, 1) + " hPa");
      break;
      
    case ACCEL:
      lcd.setCursor(0, 0);
      lcd.print("Aceleracao:");
      lcd.setCursor(0, 1);
      lcd.print(String(accelEvent.acceleration.x, 1) + " m/s²");
      break;
      
    case TEMP_NTC:
      lcd.setCursor(0, 0);
      lcd.print("Temperatura NTC:");
      lcd.setCursor(0, 1);
      lcd.print(String(temperatureNTC, 1) + " C");
      break;
  }
  
  // Avança para o proximo estado
  currentDisplay = static_cast<DisplayState>((currentDisplay + 1) % NUM_STATES);
}

void controlServo(int angle) {
  for (int pos = 0; pos <= 180; pos++) {
    servo.write(pos);
    delay(15); // Delay necessário para movimento suave
  }

  // Movimento de 180 a 0 graus
  for (int pos = 180; pos >= 0; pos--) {
    servo.write(pos);
    delay(15); // Delay necessário para movimento suave
  }
}

void toggleRelay() {
  relayState = !relayState;
  digitalWrite(RELAY_PIN, relayState);
}