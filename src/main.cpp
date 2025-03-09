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

// LEDs de alerta
#define GREEN_LED_PIN 14
#define ORANGE_LED_PIN 27
#define RED_LED_PIN 26

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
    { "Loading system.", "", "", "" }, // Iniciando sistema 1
    { "Loading system..", "", "", "" }, // Iniciando sistema 2
    { "Loading system...", "", "", "" }, // Iniciando sistema 3
    { "", "G.E.C.K", "Vault-Tec", "" }, // Tela Inicial (Nome do sistema e logo da Vault-Tec)
    { "", "Welcome!", "", "" }, // Tela de boas vindas 
    { "", "Welcome!", "Engineer #22534", "" }, // Tela de boas vindas (c/ credenciais)
    { "->Start procedure", "  Verify Sensors", "  Credits", "  Turn off" }, // Tela do menu (primeira opção)
    { "  Start procedure", "->Verify Sensors", "  Credits", "  Turn off" }, // Tela do menu (segunda opção)
    { "  Start procedure", "  Verify Sensors", "->Credits", "  Turn off" }, // Tela do menu (terceira opção)
    { "  Start procedure", "  Verify Sensors", "  Credits", "->Turn off" }, // Tela do menu (quarta opção)
    { "RAD_INFO_HERE", "TEMP_INFO_HERE", "HUMIDITY_INFO_HERE", "->Back" }, // Verificando sensores
    { "ERROR!", "Sensor failure", "Check connections", "-> Retry" }, // Tela de erro de conexão
    { "G.E.C.K Project", "Eng.: Ismael S.", "Vault 42", "->Back" }, // Tela de Créditos
    { "Starting", "Procedure.", "", ""}, // Iniciando varredura 1
    { "Starting", "Procedure..", "", ""}, // Iniciando varredura 2
    { "Starting", "Procedure...", "", ""}, // Iniciando varredura 3
    { "Environment check:", "RAD_INFO_HERE", "TEMP_INFO_HERE", "HUMIDITY_INFO_HERE" }, // Analisando ambiente
    { "Rad. check:", "Processing", "", "RAD_INFO_HERE" }, // Reduzindo radiação
    { "Humid. check:", "Moisture", "", "HUMIDITY_INFO_HERE" }, // Regulando umidade
    { "Temp. check:", "Calibrating", "", "TEMP_INFO_HERE" }, // Ajustando temperatura
    { "Vegetation...", "Deploying seeds", "Monitoring growth", "" }, // Criando vegetação
    { "Terraforming", "Complete!", "Welcome", "to Eden!" } // Processo concluído
  };
  
  ScreenIndex currentScreen;    
} Display_data;

Display_data display_data;

// Event listener do sensor MPU6050
sensors_event_t accelEvent;

// Handlers do FreeRTOS
SemaphoreHandle_t x_mutex = NULL;
TaskHandle_t task_update_LCD_handle = NULL;
TaskHandle_t task_read_DHT_handle = NULL;
TaskHandle_t task_read_MPU_handle = NULL;
TaskHandle_t task_read_BMP_handle = NULL;
TaskHandle_t task_read_NTC_handle = NULL;

// Funções de interrupção (ISRs)
volatile bool upPressed = false;
volatile bool downPressed = false;
volatile bool selectPressed = false;
QueueHandle_t xButtonQueue; // fila para os botões

void processButtons();

// Funções de interrupção (ISRs)
void IRAM_ATTR handleUpButton() {
  bool upPressedSignal = true;
  upPressed = true;
  xQueueSendFromISR(xButtonQueue, &upPressedSignal, NULL);  // Envia um sinal para a fila
}

void IRAM_ATTR handleDownButton() {
  bool downPressedSignal = true;
  downPressed = true;
  xQueueSendFromISR(xButtonQueue, &downPressedSignal, NULL);  // Envia um sinal para a fila
}

void IRAM_ATTR handleSelectButton() {
  bool selectPressedSignal = true;
  selectPressed = true;
  xQueueSendFromISR(xButtonQueue, &selectPressedSignal, NULL);  // Envia um sinal para a fila
}

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

  // Inicia o sistema
  display_data.currentScreen = SCR_LOADING_1;

  // Cria uma fila para os botões
  xButtonQueue = xQueueCreate(10, sizeof(bool));


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
  }
}

// Inicializa todos os pins
void setupPins() {
  // Inicializa o pino do relé como saída e desliga inicialmente
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  
  // Inicializa o servo motor com pino PWM, range de 500 a 2400 microssegundos (de 0º a 180º)
  servo.attach(SERVO_PWM, 500, 2400);

  // Inicializa os pinos dos LEDs de alerta de radiação
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(ORANGE_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  
  // Inicializa o pino do sensor DHT como entrada
  pinMode(DHT_PIN, INPUT);
  
  // Inicializa o pino do NTC como entrada analógica (PIN 34 suporta ADC)
  pinMode(NTC_PIN, INPUT);

  // Inicializa os pinos dos botões como interrupções
  attachInterrupt(digitalPinToInterrupt(UP_PIN), handleUpButton, FALLING);
  attachInterrupt(digitalPinToInterrupt(DOWN_PIN), handleDownButton, FALLING);
  attachInterrupt(digitalPinToInterrupt(SELECT_PIN), handleSelectButton, FALLING);
}

// Inicializa todos os sensores
bool sensor_error = false;

void setupSensors() {
  Wire.begin();
  
  checkSensorConnection(bmp.begin(), "BMP280");

  checkSensorConnection(mpu.begin(), "MPU6050");

  try {
    dhtSensor.setup(DHT_PIN, DHTesp::DHT22);
  } catch (...) {    
    sensor_error = true;
  }
}

// Verifica a comunicação serial dos sensores
void checkSensorConnection(bool success, const char* sensorName) {
  int attempts = 5;
  while(!success && attempts-- > 0) {
    Serial.printf("%s não detectado! Tentativas restantes: %d\n", sensorName, attempts);
    delay(1000);
  }
  if(attempts <= 0) {
    Serial.printf("%s falhou!\n", sensorName);
    sensor_error = true;
  }
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

      // Ativa o relé correspondente ao nível de radiação
      digitalWrite(GREEN_LED_PIN, LOW);
      digitalWrite(ORANGE_LED_PIN, LOW);
      digitalWrite(RED_LED_PIN, LOW);

      if (tempMappedRad >= 0 && tempMappedRad < 15) {
        digitalWrite(GREEN_LED_PIN, HIGH);
      } else if (tempMappedRad >= 15 && tempMappedRad < 30) {
        digitalWrite(ORANGE_LED_PIN, HIGH);
      } else if (tempMappedRad >= 30) {
        digitalWrite(RED_LED_PIN, HIGH);
      }
      
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

// Lógica para processar os botões pressionados na atualização do display
ScreenIndex last_screen;

void processButtons() {
  if (xSemaphoreTake(x_mutex, portMAX_DELAY)){  // Armazena a última tela antes da mudança
    last_screen = display_data.currentScreen;  
    
    if (display_data.currentScreen >= SCR_MENU_START && display_data.currentScreen <= SCR_MENU_TURN_OFF) {
      if (upPressed) {
        upPressed = false;
        if (display_data.currentScreen > SCR_MENU_START) {
          display_data.currentScreen = (ScreenIndex)(display_data.currentScreen - 1);  // Sobe no menu
        } else {
          display_data.currentScreen = SCR_MENU_TURN_OFF;  // Volta para a última opção do menu
        }
      }
      if (downPressed) {
        downPressed = false;
        if (display_data.currentScreen < SCR_MENU_TURN_OFF) {
          // Navega para a próxima tela
          display_data.currentScreen = (ScreenIndex)(display_data.currentScreen + 1);
        } else {
          // Se estiver na última tela, vai para a primeira
          display_data.currentScreen = SCR_MENU_START;
        }
      }
    
      if (selectPressed) {
        selectPressed = false;
        switch (display_data.currentScreen) {
          case SCR_MENU_START:          
            display_data.currentScreen = SCR_PROCEDURE_1;  // Avança para a próxima tela do processo
            break;
    
          case SCR_MENU_VERIFY:          
            display_data.currentScreen = SCR_SENSORS;  // Vai para a tela de verificação dos sensores            
            break;
    
          case SCR_MENU_CREDITS:          
            display_data.currentScreen = SCR_CREDITS;  // Vai para a tela de créditos
            break;
    
          case SCR_MENU_TURN_OFF:          
            lcd.noDisplay();  // Apaga o display
            delay(1000);
            ESP.deepSleep(0);  // ESP32 entra em deep sleep
            break;
    
          default:        
            break;
        }
      }
    } else {
      // Navega pelos demais menus:
      if (selectPressed) {
        selectPressed = false;
      
        switch (display_data.currentScreen) {
          case SCR_SENSORS:          
            display_data.currentScreen = SCR_MENU_VERIFY;  // Volta para a tela do menu
            break;
      
          case SCR_CREDITS:          
            display_data.currentScreen = SCR_MENU_CREDITS;  // Volta para a tela do menu          
            break;

          default:        
            break;
        }
      }
    }

    // Libera o mutex
    xSemaphoreGive(x_mutex);
  }
}

// função auxiliar que simula a varredura do ambiente (ativa relé e servo)
void ambientCheck(){
    // Liga o relé
    toggleRelay();

    // Ativa o servo
    controlServo();

    // Desliga o relé
    toggleRelay();
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
  bool buttonSignal;
  int transition_delay = 1000;

  while (true) {
    if (xQueueReceive(xButtonQueue, &buttonSignal, portMAX_DELAY)) {
      if (buttonSignal) {
        processButtons();  // Chama o processamento de botões
      }
    }

    // Protege o acesso aos dados compartilhados
    if (xSemaphoreTake(x_mutex, portMAX_DELAY)) {
      char RAD_INFO[16]; 
      char TEMP_INFO[16]; 
      char HUMIDITY_INFO[16];   
      const char* MOCK_RAD = "Rad: 0 %";  
      const char* MOCK_HUMIDTY = "Humid: 60 %";    
      const char* MOCK_TEMP = "Temp: 26 C";
      
      sprintf(RAD_INFO, "Rad: %s %%", sensor_data.rad_level);
      sprintf(TEMP_INFO, "Temp: %s C", sensor_data.temperatureNTC);
      sprintf(HUMIDITY_INFO, "Humid: %s %%", sensor_data.humidityDHT);          

      switch (display_data.currentScreen) {
        case SCR_LOADING_1:
            // Exibe uma tela de carregamento do sistema
            for (int i = 0; i < 2; i++){
              writeToLCD(display_data.screens[SCR_LOADING_1][0], display_data.screens[SCR_LOADING_1][1], display_data.screens[SCR_LOADING_1][2], display_data.screens[SCR_LOADING_1][3]);
              delay(transition_delay);
              writeToLCD(display_data.screens[SCR_LOADING_2][0], display_data.screens[SCR_LOADING_2][1], display_data.screens[SCR_LOADING_2][2], display_data.screens[SCR_LOADING_2][3]);
              delay(transition_delay);
              writeToLCD(display_data.screens[SCR_LOADING_3][0], display_data.screens[SCR_LOADING_3][1], display_data.screens[SCR_LOADING_3][2], display_data.screens[SCR_LOADING_3][3]);
              delay(transition_delay);
            }

            // Atualiza a tela
            display_data.currentScreen = SCR_HOME;
            break;
        case SCR_HOME:
            writeToLCD(display_data.screens[SCR_HOME][0], display_data.screens[SCR_HOME][1], display_data.screens[SCR_HOME][2], display_data.screens[SCR_HOME][3]);
            delay(transition_delay * 2);

            // Atualiza a tela
            display_data.currentScreen = SCR_WELCOME;
            break;
        case SCR_WELCOME:
            writeToLCD(display_data.screens[SCR_WELCOME][0], display_data.screens[SCR_WELCOME][1], display_data.screens[SCR_WELCOME][2], display_data.screens[SCR_WELCOME][3]);
            delay(transition_delay * 2);

            // Atualiza a tela
            display_data.currentScreen = SCR_WELCOME_CREDENTIALS;
            break;
        case SCR_WELCOME_CREDENTIALS:
            writeToLCD(display_data.screens[SCR_WELCOME_CREDENTIALS][0], display_data.screens[SCR_WELCOME_CREDENTIALS][1], display_data.screens[SCR_WELCOME_CREDENTIALS][2], display_data.screens[SCR_WELCOME_CREDENTIALS][3]);
            delay(transition_delay * 2);

            // Atualiza a tela
            display_data.currentScreen = SCR_MENU_START;
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

            // Simula a varredura do ambiente
            ambientCheck();
            break;
        case SCR_ERROR:
            writeToLCD(display_data.screens[SCR_ERROR][0], display_data.screens[SCR_ERROR][1], display_data.screens[SCR_ERROR][2], display_data.screens[SCR_ERROR][3]);
            break;
        case SCR_CREDITS:
            writeToLCD(display_data.screens[SCR_CREDITS][0], display_data.screens[SCR_CREDITS][1], display_data.screens[SCR_CREDITS][2], display_data.screens[SCR_CREDITS][3]);
            break;
        case SCR_PROCEDURE_1:
            // Exibe as telas de inicialização do procedimento de varredura do ambiente
            writeToLCD(display_data.screens[SCR_PROCEDURE_1][0], display_data.screens[SCR_PROCEDURE_1][1], display_data.screens[SCR_PROCEDURE_1][2], display_data.screens[SCR_PROCEDURE_1][3]);
            delay(transition_delay);
            writeToLCD(display_data.screens[SCR_PROCEDURE_2][0], display_data.screens[SCR_PROCEDURE_2][1], display_data.screens[SCR_PROCEDURE_2][2], display_data.screens[SCR_PROCEDURE_2][3]);
            delay(transition_delay);
            writeToLCD(display_data.screens[SCR_PROCEDURE_3][0], display_data.screens[SCR_PROCEDURE_3][1], display_data.screens[SCR_PROCEDURE_3][2], display_data.screens[SCR_PROCEDURE_3][3]);
            delay(transition_delay);

            // Atualiza a tela
            display_data.currentScreen = SCR_ANALYZING;
            break;
        case SCR_ANALYZING:
            for (int i = 0; i < 2; i++){
              writeToLCD(display_data.screens[SCR_ANALYZING][0], RAD_INFO, TEMP_INFO, HUMIDITY_INFO);
              delay(transition_delay);

              // Simula a varredura do ambiente
              ambientCheck();
            }

            // Troca para a próxima tela
            display_data.currentScreen = SCR_RADIATION;
            break;
        case SCR_RADIATION:
            for (int i = 0; i < 2; i++){
              writeToLCD(display_data.screens[SCR_RADIATION][0], display_data.screens[SCR_RADIATION][1], display_data.screens[SCR_RADIATION][2], RAD_INFO);
              delay(transition_delay);

              // Simula a varredura do ambiente
              ambientCheck();
            }

            // Exibe um nível de radiação estabilizado (falso)            
            writeToLCD(display_data.screens[SCR_RADIATION][0], "Done!", display_data.screens[SCR_RADIATION][2], MOCK_RAD);
            delay(transition_delay);

            // Simula a varredura do ambiente
            ambientCheck();

            // Troca para a próxima tela
            display_data.currentScreen = SCR_HUMIDITY;
            break;
        case SCR_HUMIDITY:
            for (int i = 0; i < 2; i++){
              writeToLCD(display_data.screens[SCR_HUMIDITY][0], display_data.screens[SCR_HUMIDITY][1], display_data.screens[SCR_HUMIDITY][2], HUMIDITY_INFO);
              delay(transition_delay);

              // Simula a varredura do ambiente
              ambientCheck();
            }

            // Exibe uma umidade estabilizada (falsa)                        
            writeToLCD(display_data.screens[SCR_HUMIDITY][0], "Done!", display_data.screens[SCR_HUMIDITY][2], MOCK_HUMIDTY);
            delay(transition_delay);

            // Simula a varredura do ambiente
            ambientCheck();

            // Troca para a próxima tela
            display_data.currentScreen = SCR_TEMPERATURE;
            break;
        case SCR_TEMPERATURE:
            for (int i = 0; i < 2; i++){
              writeToLCD(display_data.screens[SCR_TEMPERATURE][0], display_data.screens[SCR_TEMPERATURE][1], display_data.screens[SCR_TEMPERATURE][2], TEMP_INFO);
              delay(transition_delay);

              // Simula a varredura do ambiente
              ambientCheck();
            }

            // Exibe uma temperatura estabilizada (falsa)            
            writeToLCD(display_data.screens[SCR_TEMPERATURE][0], "Done!", display_data.screens[SCR_TEMPERATURE][2], MOCK_TEMP);
            delay(transition_delay);
  
            // Simula a varredura do ambiente
            ambientCheck();            

            // Troca para a próxima tela
            display_data.currentScreen = SCR_GENERATING;
            break;
        case SCR_GENERATING:
            writeToLCD(display_data.screens[SCR_GENERATING][0],display_data.screens[SCR_GENERATING][1],display_data.screens[SCR_GENERATING][2], display_data.screens[SCR_GENERATING][3]);
            delay(transition_delay);

            // Simula a varredura do ambiente
            ambientCheck();

            // Troca para a próxima tela
            display_data.currentScreen = SCR_COMPLETED;
            break;
        case SCR_COMPLETED:
            writeToLCD(display_data.screens[SCR_COMPLETED][0], display_data.screens[SCR_COMPLETED][1], display_data.screens[SCR_COMPLETED][2], display_data.screens[SCR_COMPLETED][3]);

            // Volta para o menu inicial
            display_data.currentScreen = SCR_MENU_START;
            break;
        default:            
            break;
    }    

    // Liberar o mutex após a leitura
    xSemaphoreGive(x_mutex);
    }

    // Atrasar a atualização a cada 2 segundos
    vTaskDelay(transition_delay / portTICK_PERIOD_MS);
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

// Função para lidar/desligar o relé
void toggleRelay() {
  relayState = !relayState;
  digitalWrite(RELAY_PIN, relayState);
}