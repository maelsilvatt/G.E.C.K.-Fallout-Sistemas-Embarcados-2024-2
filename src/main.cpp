#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <stdbool.h>

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

// Botões
#define UP_PIN 17
#define DOWN_PIN 16
#define SELECT_PIN 0

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
  char temperatureDHT[10];  
  char humidityDHT[10];     
  char accelX[10];          
  char accelY[10];          
  char accelZ[10];          
  char rad_level[10];       
  char temperatureNTC[10];  
} Sensor_data;

Sensor_data sensor_data;

// Enum das telas do sistema
typedef enum {
  SCR_LOADING_1,
  SCR_LOADING_2,
  SCR_LOADING_3,
  SCR_HOME,
  SCR_WELCOME,
  SCR_WELCOME_CREDENTIALS,
  SCR_MENU_START,
  SCR_MENU_VERIFY,
  SCR_MENU_CREDITS,
  SCR_MENU_TURN_OFF,
  SCR_SENSORS,
  SCR_ERROR,
  SCR_CREDITS,
  SCR_PROCEDURE_1,
  SCR_PROCEDURE_2,
  SCR_PROCEDURE_3,
  SCR_ANALYZING,
  SCR_RADIATION,
  SCR_HUMIDITY,
  SCR_TEMPERATURE,
  SCR_GENERATING,
  SCR_COMPLETED
} ScreenIndex;

// Telas do sistema
typedef struct {
  char* screens[22][4] = {
    { "", "Loading system.", "", "Initializing subsystems..." }, // Iniciando sistema 1
    { "", "Loading system..", "", "Checking Vault Integrity..." }, // Iniciando sistema 2
    { "", "Loading system...", "", "Booting up protocols..." }, // Iniciando sistema 3
    { "", "G.E.C.K", "Vault-Tec", "" }, // Tela Inicial (Nome do sistema e logo da Vault-Tec)
    { "", "Welcome!", "", "" }, // Tela de boas vindas 
    { "", "Welcome!", "Engineer #22534", "" }, // Tela de boas vindas (c/ credenciais)
    { "-> Start procedure", "  Verify Sensors", "  Credits", "  Turn off" }, // Tela do menu (primeira opção)
    { "   Start procedure", "->Verify Sensors", "  Credits", "  Turn off" }, // Tela do menu (segunda opção)
    { "   Start procedure", "  Verify Sensors", "->Credits", "  Turn off" }, // Tela do menu (terceira opção)
    { "   Start procedure", "  Verify Sensors", "  Credits", "->Turn off" }, // Tela do menu (quarta opção)
    { "RAD_INFO_HERE", "TEMP_INFO_HERE", "HUMIDITY_INFO_HERE", "-> Back" }, // Verificando sensores
    { "ERROR!", "Sensor failure", "Check connections", "-> Retry" }, // Tela de erro de conexão
    { "G.E.C.K Project", "Lead Eng.: Ismael S.", "Vault 69", "-> Back" }, // Tela de Créditos
    { "", "Starting Procedure.", "", "Initializing subsystems..." }, // Iniciando varredura 1
    { "", "Starting Procedure..", "", "Calibrating sensors..." }, // Iniciando varredura 2
    { "", "Starting Procedure...", "", "Finalizing startup..." }, // Iniciando varredura 3
    { "Analyzing environment", "RAD_INFO_HERE", "TEMP_INFO_HERE", "HUMIDITY_INFO_HERE" }, // Analisando ambiente
    { "Neutralizing radiation", "Processing...", "Estimated time: 2 min", "RAD_INFO_HERE" }, // Reduzindo radiação
    { "Regulating humidity", "Adjusting moisture", "Balancing ecosystem", "HUMIDITY_INFO_HERE" }, // Regulando umidade
    { "Optimizing temperature", "Calibrating levels...", "Maintaining stability", "TEMP_INFO_HERE" }, // Ajustando temperatura
    { "Generating plant life", "Deploying seeds...", "Monitoring growth", "" }, // Criando vegetação
    { "Terraforming Complete!", "Life support stable", "Welcome to Eden", "System shutting down..." } // Processo concluído
  };
  
  ScreenIndex currentScreen; // Índice tipado
  bool in_menu;
  int buttonPressed = -1;
} Display_data;

Display_data display_data;

// Event listener do sensor MPU6050
sensors_event_t accelEvent;

// Handlers do FreeRTOS
SemaphoreHandle_t x_mutex = NULL;
TaskHandle_t task_update_LCD_handle = NULL;
TaskHandle_t task_read_buttons_handle = NULL;
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
void writeToLCD(const char* line1, const char* line2, const char* line3, const char* line4);

void vTaskUpdateDisplay(void *pvParams);
void vTaskReadButtons(void *pvParams);
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
    xTaskCreatePinnedToCore(vTaskReadDHT, "task_dht", 4096, NULL, 0, &task_read_DHT_handle, 0);
    xTaskCreatePinnedToCore(vTaskReadMPU, "task_mpu", 4096, NULL, 0, &task_read_MPU_handle, 0);
    xTaskCreatePinnedToCore(vTaskReadRadScan3000, "task_radscan", 4096, NULL, 0, &task_read_BMP_handle, 0);
    xTaskCreatePinnedToCore(vTaskReadNTC, "task_ntc", 4096, NULL, 0, &task_read_NTC_handle, 0);

    // Adiciona a tarefa de atualizar o display ao mutex
    xTaskCreatePinnedToCore(vTaskUpdateDisplay, "task_lcd", 4096, NULL, 1, &task_update_LCD_handle, 1);
    xTaskCreatePinnedToCore(vTaskUpdateDisplay, "task_buttons", 4096, NULL, 1, &task_read_buttons_handle, 1);
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

  // Inicializa os pinos dos botões
  pinMode(UP_PIN, INPUT_PULLUP);
  pinMode(DOWN_PIN, INPUT_PULLUP);
  pinMode(SELECT_PIN, INPUT_PULLUP);
}

// Inicializa todos os sensores
void setupSensors() {
  Wire.begin();
  
  checkSensorConnection(bmp.begin(), "BMP280");

  checkSensorConnection(mpu.begin(), "MPU6050");
  dhtSensor.setup(DHT_PIN, DHTesp::DHT22);
}

// Verifica a comunicação serial dos sensores
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

// Task de leitura dos dados do DHT
void vTaskReadDHT(void *pvParams){
  while (true) {
    auto dhtData = dhtSensor.getTempAndHumidity();

    // Protege o acesso aos dados compartilhados
    if (xSemaphoreTake(x_mutex, portMAX_DELAY)){
      // Se o status do sensor for bom (sem erro)
      if (dhtSensor.getStatus() == 0) {                
        // Armazena o dado do sensor em char[]
        dtostrf(dhtData.humidity, 4, 0, sensor_data.humidityDHT);
      }
      
      // Liberar o mutex após a leitura
      xSemaphoreGive(x_mutex);
    }
    
    vTaskDelay(2000 / portTICK_PERIOD_MS); // Delay para evitar sobrecarga da CPU (1 segundo)
  }
}

// Task de leitura dos dados do Rad Scan 3000 (BPM280 modificado)
void vTaskReadRadScan3000(void *pvParams) {
  while (true) {
    if (xSemaphoreTake(x_mutex, portMAX_DELAY)) {
      // Leitura da temperatura do BMP280 e interpreta como radiação
      float temp = bmp.readTemperature();
      float tempMappedRad;

      // Definição dos limites
      float tempMin = -123.5;
      float tempMax = 174.1;
      float radMin = 0.0;
      float radMax = 99.0;

      // Mapeamento linear da temperatura para radiação
      tempMappedRad = (temp - tempMin) * (radMax - radMin) / (tempMax - tempMin) + radMin;

      // Garantir que o valor de radiação fique dentro dos limites 0-99
      if (tempMappedRad < radMin) tempMappedRad = radMin;
      if (tempMappedRad > radMax) tempMappedRad = radMax;
      
      // Armazena o dado do sensor em char[]
      dtostrf(tempMappedRad, 4, 0, sensor_data.rad_level);

      // Libera o mutex
      xSemaphoreGive(x_mutex);
    }

    vTaskDelay(500 / portTICK_PERIOD_MS); // Delay para evitar sobrecarga
  }
}

// Task de leitura dos dados do MPU
void vTaskReadMPU(void *pvParams) {
  while (true) {
    // Protege o acesso aos dados compartilhados
    if (xSemaphoreTake(x_mutex, portMAX_DELAY)) {
      // Leitura do MPU6050 (acelerômetro)
      mpu.getAccelerometerSensor()->getEvent(&accelEvent);

      // Armazena o dado do sensor em char[]
      dtostrf(accelEvent.acceleration.x, 4, 0, sensor_data.accelX);
      dtostrf(accelEvent.acceleration.y, 4, 0, sensor_data.accelY);
      dtostrf(accelEvent.acceleration.z, 4, 0, sensor_data.accelZ);
      
      // Liberar o mutex após a leitura
      xSemaphoreGive(x_mutex);
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay para evitar sobrecarga da CPU (1 segundo)
  }
}

// Task de leitura dos dados do NTC
void vTaskReadNTC(void *pvParams) {
  while (true) {
    if (xSemaphoreTake(x_mutex, portMAX_DELAY)) {
      // Leitura do ADC
      int analogValue = analogRead(NTC_PIN);
      float converted_reading;

      // Converte para Celsius
      if (analogValue > 0 && analogValue < 4095) {                         
        converted_reading = 1 / (log(1 / (4095. / analogValue - 1)) / BETA + 1.0 / 298.15) - 273.15;        
      }

      // Armazena o dado do sensor em char[]
      dtostrf(converted_reading, 4, 0, sensor_data.temperatureNTC);

      xSemaphoreGive(x_mutex);
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay de 1s
  }
}

// Função auxiliar para escrever nas linhas do display LCD
void writeToLCD(const char* line1, const char* line2, const char* line3, const char* line4) {
  // Limpar o display
  lcd.clear();


  lcd.setCursor(0, 0);  // Coloca o cursor na primeira linha, coluna 0
  lcd.print(line1);     

  lcd.setCursor(0, 1);  // Coloca o cursor na segunda linha, coluna 0
  lcd.print(line2);      

  lcd.setCursor(0, 2);  // Coloca o cursor na terceira linha, coluna 0
  lcd.print(line3);   

  lcd.setCursor(0, 3);  // Coloca o cursor na quarta linha, coluna 0
  lcd.print(line4);     
}

// Task de atualização do display
void vTaskUpdateDisplay(void *pvParams) {
  while (true) {
    // Protege o acesso aos dados compartilhados
    if (xSemaphoreTake(x_mutex, portMAX_DELAY)) {
      char RAD_INFO[16]; 
      char TEMP_INFO[16]; 
      char HUMIDITY_INFO[16]; 
      
      sprintf(RAD_INFO, "Rad: %s microSv/h", sensor_data.rad_level);
      sprintf(TEMP_INFO, "Temperature: %s  C", sensor_data.temperatureNTC);
      sprintf(HUMIDITY_INFO, "Humidity: %s  %", sensor_data.humidityDHT);

      switch (display_data.currentScreen) {
        case SCR_LOADING_1:
            writeToLCD(display_data.screens[SCR_LOADING_1][0], display_data.screens[SCR_LOADING_1][1], display_data.screens[SCR_LOADING_1][2], display_data.screens[SCR_LOADING_1][3]);
            break;
        case SCR_LOADING_2:
            writeToLCD(display_data.screens[SCR_LOADING_2][0], display_data.screens[SCR_LOADING_2][1], display_data.screens[SCR_LOADING_2][2], display_data.screens[SCR_LOADING_2][3]);
            break;
        case SCR_LOADING_3:
            writeToLCD(display_data.screens[SCR_LOADING_3][0], display_data.screens[SCR_LOADING_3][1], display_data.screens[SCR_LOADING_3][2], display_data.screens[SCR_LOADING_3][3]);
            break;
        case SCR_HOME:
            writeToLCD(display_data.screens[SCR_HOME][0], display_data.screens[SCR_HOME][1], display_data.screens[SCR_HOME][2], display_data.screens[SCR_HOME][3]);
            break;
        case SCR_WELCOME:
            writeToLCD(display_data.screens[SCR_WELCOME][0], display_data.screens[SCR_WELCOME][1], display_data.screens[SCR_WELCOME][2], display_data.screens[SCR_WELCOME][3]);
            break;
        case SCR_WELCOME_CREDENTIALS:
            writeToLCD(display_data.screens[SCR_WELCOME_CREDENTIALS][0], display_data.screens[SCR_WELCOME_CREDENTIALS][1], display_data.screens[SCR_WELCOME_CREDENTIALS][2], display_data.screens[SCR_WELCOME_CREDENTIALS][3]);
            break;
        case SCR_MENU_START:
            writeToLCD(display_data.screens[SCR_MENU_START][0], display_data.screens[SCR_MENU_START][1], display_data.screens[SCR_MENU_START][2], display_data.screens[SCR_MENU_START][3]);
            break;
        case SCR_MENU_VERIFY:
            writeToLCD(display_data.screens[SCR_MENU_VERIFY][0], display_data.screens[SCR_MENU_VERIFY][1], display_data.screens[SCR_MENU_VERIFY][2], display_data.screens[SCR_MENU_VERIFY][3]);
            break;
        case SCR_MENU_CREDITS:
            writeToLCD(display_data.screens[SCR_MENU_CREDITS][0], display_data.screens[SCR_MENU_CREDITS][1], display_data.screens[SCR_MENU_CREDITS][2], display_data.screens[SCR_MENU_CREDITS][3]);
            break;
        case SCR_MENU_TURN_OFF:
            writeToLCD(display_data.screens[SCR_MENU_TURN_OFF][0], display_data.screens[SCR_MENU_TURN_OFF][1], display_data.screens[SCR_MENU_TURN_OFF][2], display_data.screens[SCR_MENU_TURN_OFF][3]);
            break;
        case SCR_SENSORS:
            writeToLCD(RAD_INFO, TEMP_INFO, HUMIDITY_INFO, display_data.screens[SCR_SENSORS][3]);
        case SCR_ERROR:
            writeToLCD(display_data.screens[SCR_ERROR][0], display_data.screens[SCR_ERROR][1], display_data.screens[SCR_ERROR][2], display_data.screens[SCR_ERROR][3]);
            break;
        case SCR_CREDITS:
            writeToLCD(display_data.screens[SCR_CREDITS][0], display_data.screens[SCR_CREDITS][1], display_data.screens[SCR_CREDITS][2], display_data.screens[SCR_CREDITS][3]);
            break;
        case SCR_PROCEDURE_1:
            writeToLCD(display_data.screens[SCR_PROCEDURE_1][0], display_data.screens[SCR_PROCEDURE_1][1], display_data.screens[SCR_PROCEDURE_1][2], display_data.screens[SCR_PROCEDURE_1][3]);
            break;
        case SCR_PROCEDURE_2:
            writeToLCD(display_data.screens[SCR_PROCEDURE_2][0], display_data.screens[SCR_PROCEDURE_2][1], display_data.screens[SCR_PROCEDURE_2][2], display_data.screens[SCR_PROCEDURE_2][3]);
            break;
        case SCR_PROCEDURE_3:
            writeToLCD(display_data.screens[SCR_PROCEDURE_3][0], display_data.screens[SCR_PROCEDURE_3][1], display_data.screens[SCR_PROCEDURE_3][2], display_data.screens[SCR_PROCEDURE_3][3]);
            break;
        case SCR_ANALYZING:
            writeToLCD(display_data.screens[SCR_ANALYZING][0], display_data.screens[SCR_ANALYZING][1], display_data.screens[SCR_ANALYZING][2], display_data.screens[SCR_ANALYZING][3]);
            break;
        case SCR_RADIATION:
            writeToLCD(display_data.screens[SCR_RADIATION][0], display_data.screens[SCR_RADIATION][1], display_data.screens[SCR_RADIATION][2], RAD_INFO);
            break;
        case SCR_HUMIDITY:
            writeToLCD(display_data.screens[SCR_HUMIDITY][0],display_data.screens[SCR_HUMIDITY][1], display_data.screens[SCR_HUMIDITY][2], HUMIDITY_INFO);
            break;
        case SCR_TEMPERATURE:
            writeToLCD(display_data.screens[SCR_TEMPERATURE][0], display_data.screens[SCR_TEMPERATURE][1], display_data.screens[SCR_TEMPERATURE][2], TEMP_INFO);
            break;
        case SCR_GENERATING:
            writeToLCD(display_data.screens[SCR_GENERATING][0],display_data.screens[SCR_GENERATING][1],display_data.screens[SCR_GENERATING][2], display_data.screens[SCR_GENERATING][3]);
            break;
        case SCR_COMPLETED:
            writeToLCD(display_data.screens[SCR_COMPLETED][0], display_data.screens[SCR_COMPLETED][1], display_data.screens[SCR_COMPLETED][2], display_data.screens[SCR_COMPLETED][3]);
            break;
        default:
            // Handle default case
            break;
    }    

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
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

// Task de leitura dos botões
void vTaskReadButtons(void *pvParams) {
  while (true) {
    if (xSemaphoreTake(x_mutex, portMAX_DELAY)) {
      // Leitura dos botões
      if (digitalRead(UP_PIN) == LOW) {
        display_data.buttonPressed = 0;
      } else if (digitalRead(DOWN_PIN) == LOW) {
        display_data.buttonPressed = 1;
      } else if (digitalRead(SELECT_PIN) == LOW) {
        display_data.buttonPressed = 2;
      } else {
        display_data.buttonPressed = -1;
      }
      
      xSemaphoreGive(x_mutex);
    }
    vTaskDelay(10 / portTICK_PERIOD_MS); // Pequeno delay para evitar polling excessivo
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