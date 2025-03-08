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
#include "freertos/semphr.h"

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
typedef struct {
  float temperatureDHT = 0;
  float humidityDHT = 0;
  float accelX = 0;
  float accelY = 0;
  float accelZ = 0;
  float rad_level  = 0;
  float temperatureNTC = 0;
} Sensor_data;

Sensor_data sensor_data;

sensors_event_t accelEvent;

// Handlers do FreeRTOS
SemaphoreHandle_t x_mutex = NULL;
TaskHandle_t task_update_LCD_handle = NULL;
TaskHandle_t task_read_DHT_handle = NULL;
TaskHandle_t task_read_MPU_handle = NULL;
TaskHandle_t task_read_BMP_handle = NULL;
TaskHandle_t task_read_NTC_handle = NULL;

/**************************************
 * Protótipos de Funções
 **************************************/
void setupPins();
void setupSensors();
void checkSensorConnection(bool success, const char* sensorName);
void controlServo();
void toggleRelay();

void vTaskUpdateDisplay(void *pvParams);
void vTaskReadDHT(void *pvParams);
void vTaskReadMPU(void *pvParams);
void vTaskReadRadScan3000(void *pvParams);
void vTaskReadNTC(void *pvParams);

/**************************************
 * Implementação
 **************************************/
void setup() {
  Serial.begin(115200);
  setupPins();
  setupSensors();

  // Inicia o display LCD
  lcd.init();
  lcd.backlight();

  // Cria o mutex para gerenciar o acesso às variáveis globais
  x_mutex = xSemaphoreCreateMutex();

  // Associa as tasks ao semáfoto e a um core
  if (x_mutex != NULL ) {    
    // Adiciona as tarefas de leitura dos sensores ao mutex
    xTaskCreatePinnedToCore(vTaskReadDHT, "task_dht", 4096, NULL, 1, &task_read_DHT_handle, 0);
    xTaskCreatePinnedToCore(vTaskReadMPU, "task_dht", 4096, NULL, 1, &task_read_MPU_handle, 0);
    xTaskCreatePinnedToCore(vTaskReadRadScan3000, "task_dht", 4096, NULL, 1, &task_read_BMP_handle, 0);
    xTaskCreatePinnedToCore(vTaskReadNTC, "task_dht", 4096, NULL, 1, &task_read_NTC_handle, 0);

    // Adiciona a tarefa de atualizar o display ao mutex
    xTaskCreatePinnedToCore(vTaskUpdateDisplay, "task_dht", 4096, NULL, 1, &task_update_LCD_handle, 1);
  }
}

// Inicializa todos os pins
void setupPins() {
  // Inicializa o pino do relé como saída e desliga inicialmente
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);  // Desliga o relé ao iniciar
  
  // Inicializa o servo motor com pino PWM, range de 500 a 2400 microssegundos (de 0º a 180º)
  servo.attach(SERVO_PWM, 500, 2400);  // Pino de controle PWM do servo
  
  // Inicializa o pino do sensor DHT como entrada
  pinMode(DHT_PIN, INPUT);
  
  // Inicializa o pino do NTC como entrada analógica (PIN 34 suporta ADC)
  pinMode(NTC_PIN, INPUT);
}

void setupSensors() {
  Wire.begin();
  
  checkSensorConnection(bmp.begin(), "BMP280");
  // bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
  //                 Adafruit_BMP280::SAMPLING_X2,
  //                 Adafruit_BMP280::SAMPLING_X16,
  //                 Adafruit_BMP280::FILTER_X16,
  //                 Adafruit_BMP280::STANDBY_MS_500);

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
}

void vTaskReadDHT(void *pvParams){
  while (true) {
    auto dhtData = dhtSensor.getTempAndHumidity();

    // Protege o acesso aos dados compartilhados
    if (xSemaphoreTake(x_mutex, portMAX_DELAY)){
      // Se o status do sensor for bom (sem erro)
      if (dhtSensor.getStatus() == 0) {        
        sensor_data.humidityDHT = dhtData.humidity;
      }
      
      // Liberar o mutex após a leitura
      xSemaphoreGive(x_mutex);
    }
    
    vTaskDelay(2000); // Delay para evitar sobrecarga da CPU (1 segundo)
  }
}

void vTaskReadRadScan3000(void *pvParams) {
  while (true) {
    if (xSemaphoreTake(x_mutex, portMAX_DELAY)) {
      // Leitura da temperatura do BMP280
      float temp = bmp.readTemperature();

      // Definição dos limites
      float tempMin = -123.5;
      float tempMax = 174.1;
      float radMin = 0.0;
      float radMax = 99.0;

      // Mapeamento linear da temperatura para radiação
      sensor_data.rad_level = (temp - tempMin) * (radMax - radMin) / (tempMax - tempMin) + radMin;

      // Garantir que o valor de radiação fique dentro dos limites 0-99
      if (sensor_data.rad_level < radMin) sensor_data.rad_level = radMin;
      if (sensor_data.rad_level > radMax) sensor_data.rad_level = radMax;
      
      // Libera o mutex
      xSemaphoreGive(x_mutex);
    }

    vTaskDelay(500); // Delay para evitar sobrecarga
  }
}

void vTaskReadMPU(void *pvParams) {
  while (true) {
    // Protege o acesso aos dados compartilhados
    if (xSemaphoreTake(x_mutex, portMAX_DELAY)) {
      // Leitura do MPU6050 (acelerômetro)
      mpu.getAccelerometerSensor()->getEvent(&accelEvent);

      // Armazenar os dados do acelerômetro
      sensor_data.accelX = accelEvent.acceleration.x;
      sensor_data.accelY = accelEvent.acceleration.y;
      sensor_data.accelZ = accelEvent.acceleration.z;
      
      // Liberar o mutex após a leitura
      xSemaphoreGive(x_mutex);
    }

    vTaskDelay(1000); // Delay para evitar sobrecarga da CPU (1 segundo)
  }
}

void vTaskReadNTC(void *pvParams) {
  while (true) {
    if (xSemaphoreTake(x_mutex, portMAX_DELAY)) {
      // Leitura do ADC
      int analogValue = analogRead(NTC_PIN);

      // Converte para Celsius
      if (analogValue > 0 && analogValue < 4095) {                         
        sensor_data.temperatureNTC = 1 / (log(1 / (4095. / analogValue - 1)) / BETA + 1.0 / 298.15) - 273.15;        
      }

      xSemaphoreGive(x_mutex);
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay de 1s
  }
}

void vTaskUpdateDisplay(void *pvParams) {
  while (true) {
    // Protege o acesso aos dados compartilhados
    if (xSemaphoreTake(x_mutex, portMAX_DELAY)) {
      // Limpar o display
      lcd.clear();
      
      // Exibir umidade
      lcd.setCursor(0, 0);
      lcd.print("Humidity: ");
      lcd.print(sensor_data.humidityDHT, 1);
      lcd.print(" %");

      // Exibir aceleração
      lcd.setCursor(0, 1);
      lcd.print("Acell Y: ");
      lcd.print(sensor_data.accelY, 1);
      lcd.print(" m/s²");

      // Exibir radiação
      lcd.setCursor(0, 2);
      lcd.print("Radiation: ");
      lcd.print(sensor_data.rad_level , 1);
      lcd.print(" Sv/h");

      // Exibir temperatura NTC
      lcd.setCursor(0, 3);
      lcd.print("Temperature: ");
      lcd.print(sensor_data.temperatureNTC, 1);
      lcd.print(" C");    

      // Liberar o mutex após a leitura
      xSemaphoreGive(x_mutex);
    }

    // Liga o relé
    toggleRelay();

    // Ativa o servo
    controlServo();

    // Desliga o relé
    toggleRelay();

    // Atrasar a atualização a cada 1 segundos
    vTaskDelay(1000);
  }
}

void controlServo() {
  for (int pos = 0; pos <= 180; pos++) {
    servo.write(pos);
    delay(5); // Delay necessário para movimento suave
  }

  // Movimento de 180 a 0 graus
  for (int pos = 180; pos >= 0; pos--) {
    servo.write(pos);
    delay(5); // Delay necessário para movimento suave
  }
}

void toggleRelay() {
  relayState = !relayState;
  digitalWrite(RELAY_PIN, relayState);
}