#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <DHTesp.h>
#include <ESP32Servo.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Defina os pinos SPI do BMP280
#define BMP_SCK  18    // Pino de clock SPI
#define BMP_MISO 19    // Pino MISO (Master In Slave Out)
#define BMP_MOSI 23    // Pino MOSI (Master Out Slave In)
#define BMP_CS   5     // Pino Chip Select (CS)

// Inicializa o sensor BMP280
Adafruit_BMP280 bmp(BMP_CS);

// Define o pino PWM do servo
#define SERVO_PWM 4

// Inicializa o servo
Servo servo;

// Define os pinos do painel LCD
#define I2C_ADDR    0x27
#define LCD_COLUMNS 20
#define LCD_LINES   4

// Inicializa o painel LCD
LiquidCrystal_I2C lcd(I2C_ADDR, LCD_COLUMNS, LCD_LINES);

// Inicializa o sensor MPU6050 com I2C
Adafruit_MPU6050 mpu;

// Define o pino do sensor DHT22
#define DHT_PIN 13

// Inicializa o sensor DHT22
DHTesp dhtSensor;

// Define o pino do termistor NTC
#define NTC_PIN 34

// Define o valor beta do termistor NTC
#define BETA 3950

// Define o pino do relé
#define RELAY_PIN 12

// Função para verificar se o BMP280 está conectado
void verificarBMP280() {
  int tentativas = 5;
  while (!bmp.begin() && tentativas > 0) {
    Serial.println("BMP280 não conectado! Tentando novamente...");
    delay(1000);
    tentativas--;
  }

  if (tentativas == 0) {
    Serial.println("BMP280 não detectado!");
  } else {
    Serial.println("BMP280 pronto!");
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_500);
  }
}

// Função para verificar se o MPU6050 está conectado
void verificarMPU6050() {
  int tentativas = 5;
  while (!mpu.begin() && tentativas > 0) {
    Serial.println("MPU6050 não conectado!");
    delay(1000);
    tentativas--;
  }

  if (tentativas == 0) {
    Serial.println("MPU6050 não detectado!");
  } else {
    Serial.println("MPU6050 pronto!");
  }
}
// Função para verificar se o DHT22 está conectado
void verificarDHT() {
  dhtSensor.setup(DHT_PIN, DHTesp::DHT22);  // Configura o sensor

  // Tenta fazer uma leitura para verificar se o sensor está funcionando
  TempAndHumidity data = dhtSensor.getTempAndHumidity();
  if (isnan(data.temperature) || isnan(data.humidity)) {
    Serial.println("DHT22 não detectado!");
  } else {
    Serial.println("DHT22 pronto!");
  }
}

// Função para verificar se o NTC está conectado
void verificarNTC() {
  int analogValue = analogRead(NTC_PIN);  // Leitura do valor analógico

  // Verifica se a leitura do sensor é válida
  if (analogValue == 0 || analogValue == 4095) {  
    Serial.println("Termistor NTC não detectado ou com mau contato!");
  } else {
    Serial.println("Termistor NTC pronto!");
  }
}

// Configurações ao iniciar o sistema
void setup() {
  Serial.begin(115200); // Inicializa a comunicação serial

  Wire.begin(); // Inicializa o barramento I2C

  servo.attach(SERVO_PWM, 500, 2400); // Configura o pino PWM do servo

  // Configura o pino do relé como pino de saída
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW); // Garante que sempre inicie desligado

  // Inicia o display LCD
  lcd.init();
  lcd.backlight();

  // Verifica a comunicação dos sensores do sistema
  verificarBMP280();
  verificarMPU6050();
  verificarDHT();
  verificarNTC();
}

// Lê e imprime a pressão
void ler_pressao(){
  Serial.print(F("Pressão = "));
  Serial.print(bmp.readPressure());
  Serial.println(F(" Pa"));
}

// Função para ler e exibir os valores do acelerômetro
void ler_acelerometro() {
  sensors_event_t event;
  mpu.getAccelerometerSensor()->getEvent(&event);

  Serial.print("[");
  Serial.print(millis());  // Marca o tempo em milissegundos
  Serial.print("] X: ");
  Serial.print(event.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(event.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(event.acceleration.z);
  Serial.println(" m/s^2");
}

// Função para ler a umidade
void ler_umidade() {
  TempAndHumidity  data = dhtSensor.getTempAndHumidity();
  Serial.println("Humidade do ar: " + String(data.humidity, 1) + "%");
}

// Função para ler a temperatura 
void ler_temperatura() {
  int analogValue = analogRead(NTC_PIN);

  // Verifica se a leitura do sensor é válida
  if (analogValue == 0 || analogValue >= 1023) {  
    Serial.println("Erro: Falha na leitura do termistor NTC!");
    return;
  }

  // Converte a leitura analógica para temperatura em Celsius
  float celsius = 1 / (log(1 / (1023. / analogValue - 1)) / BETA + 1.0 / 298.15) - 273.15;

  Serial.print("Temperatura: ");
  Serial.print(celsius);
  Serial.println(" ℃");
}

// Função para rotacionar o servo
int pos = 0; // Guarda a posição atual do servo
void rotacionar_servo() {
  for (pos = 0; pos <= 180; pos += 1) {
      servo.write(pos);
      delay(15);
    }
    for (pos = 180; pos >= 0; pos -= 1) {
      servo.write(pos);
      delay(15);
    }
 }

 // Função para escrever no painel LCD
 void escreve_lcd() {     
    lcd.setCursor(3, 0);
    lcd.print("Teste!");    
 }

// Função para ligar/desligar o relé
 bool relay_state = false; // Guarda o estado atual do relé

 void alterna_rele() { 
  relay_state = !relay_state;
  digitalWrite(RELAY_PIN, relay_state ? HIGH : LOW);
 }
  
void loop() {
  // Escreve no painel LCD
  escreve_lcd();

  // Ler sensores (descomente os que deseja usar)
  // ler_pressao();        // Lê a pressão atmosférica
  // ler_umidade();        // Lê a umidade do ar
  // ler_temperatura();    // Lê a temperatura do termistor NTC
  // ler_acelerometro();   // Lê os dados do acelerômetro
  // rotacionar_servo();   // Gira o servo motor
  // alterna_rele();       // Alterna o estado do relé

  delay(5000);
}
