#include <SPI.h>
#include <Arduino.h>
#include "FS.h"
#include "SPIFFS.h"
#include <esp_now.h>
#include <WiFi.h>
#include <RF24.h>
#include <RF24Network.h>
#include "Adafruit_FRAM_I2C.h"
#include <math.h>

//uint8_t ESPXIAO1[] = {0x24, 0xec, 0x4a, 0x00, 0x4f, 0x08};  // MAC do ESP1
//uint8_t ESPXIAO2[] = {0x34, 0x85, 0x18, 0x91, 0x42, 0x84};  // MAC do ESP2 com3

//Esp32 do Modulo Principal Direito
uint8_t ESPXIAO4[] = { 0x30, 0x30, 0xf9, 0x16, 0xa1, 0xb0 };  // MAC do ESP4 com10 30:30:f9:16:a1:b0d, 0xb1, 0x38};  // MAC do ESP4 com10 30:30:f9:16:a1:b0

//ESPXIAO3 e ESPXIAO4 são os dois ESP32 do Modulo Principal Esquerdo
//uint8_t ESPXIAO3[]34:85:18:91:42:84

uint8_t ESPXIAO3[] = { 0x34, 0x85, 0x18, 0x91, 0x42, 0x84 };
//uint8_t ESPXIAO1[] = {0x24, 0xec, 0x4a, 0x00, 0x4f, 0x08};  // MAC do ESP1
//uint8_t ESPXIAO2[] = {0x34, 0x85, 0x18, 0x91, 0x42, 0x84};  // MAC do ESP2 com3
//uint8_t ESPXIAO3[] = {0x64, 0xe8, 0x33, 0x7e,  0x73, 0x8c};;  // MAC do ESP3 com12  Modulo Principal Esquerdo
//uint8_t ESPXIAO4[] = {0x64, 0x08, 0x33, 0x7e,  0x73, 0x7c};  // MAC do ESP4 com10 Modulo Principal Direito
uint8_t ESPXIAO5[] = { 0xe8, 0x06, 0x90, 0xa0, 0xdf, 0x6c };  // MAC do ESP5 com9 Modulo Sensores
uint8_t ESPXIAO6[] = { 0xe8, 0x06, 0x90, 0x9D, 0x93, 0x34 };  // MAC do ESP6. com 15 Modulo Sensores
uint8_t ESPXIAO7[] = { 0xe8, 0x06, 0x90, 0x9D, 0xa0, 0x60 };  // MAC do ESP7. com16 Modulo Sensores

uint8_t ESPUNO1[] = { 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x02 };  // MAC do ESP UNO1  Modulo oculos
uint8_t ESPUNO2[] = { 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x01 };  // MAC do ESP UNO2  Modulo oculos
uint8_t ESPUNO3[] = { 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x02 };  // MAC do ESP UNO3  Modulo oculos
esp_now_peer_info_t peerInfo;
RF24 radio(D2, D1);  // nRF24L01(+) radio attached using Getting Started board

RF24Network network(radio);  // Network uses that radio


Adafruit_FRAM_I2C fram = Adafruit_FRAM_I2C();

const uint16_t this_node = 01;   // Address of our node in Octal format
const uint16_t other_node = 00;  // Address of the other node in Octal format



TaskHandle_t EnviaDadosTaskHandle, SaudeTaskHandle,logSensorTaskkHandle;
TaskHandle_t TrilpeMachineVotingTaskHandle, TrilpeMachineVotingGPSTaskHandle;
// -------------------------------------------------
// Configuration Esp Flash
// -------------------------------------------------
#define FILE_PATH "/log.txt"

// Endereço na FRAM para armazenar os valores
#define FRAM_START_ADDR 0
#define MAGIC_NUMBER 0xABCD1234  // Número que identifica se há dados válidos
//deine LOG_INTERVAL_MS     10      // 100 Hz => 10ms
#define BUFFER_FLUSH_COUNT 50  // Flush every 20 writes
File logFile;
bool fileOpen = false;
int logCounter = 0;  // used to track how many lines since last flush

#define ESPNOW_WIFI_CHANNEL 1
#define MAX_SENSORS 3
#define BUZZER_PIN D3
int NumeroPacoteTMV = 0;
bool isNovo[3] = { false, false, false };

static QueueHandle_t QueueEnvio, QueueTMV;
// Array para armazenar os MACs únicos (supondo que cada MAC tenha 6 bytes)
static uint8_t registeredMACs[MAX_SENSORS][6];
// Contador de sensores já registrados
static int sensorCount = 0;
// Definição da estrutura de dados recebida via ESP-NOW

// ENUMS

typedef enum {
  STATE_REPAIR,
  STATE_IDLE,
  STATE_TAKEOFF,
  STATE_ASCENDING,
  STATE_DESCENDING,
  STATE_LANDING,
  STATE_LANDED
} state_t;

typedef enum {
  COMANDO,
  CALIBRACAO
} MSG_Telecomando_t;

struct DataPacket {
  unsigned long timeMillis;
};

//Estruturas de dados
typedef struct {
  int numeroError;
} Erro_t;




typedef struct {
  bool isMaster;
  uint32_t timestamp;  // in milliseconds


} MasterCommand_t;

/* -----------TELECOMANDOS------------------------------------------------
ST-SincronizaTempo
CBMP-CalibraBMP
CIM-CalibraIMU
CMAG-CalibraMAG
CAC-CalibraACC
CGY-CalibraGYRO
MST-Master
SLV-Slave
STRPR-StartRepair
STIDL-Idle
STTKE-TakeOff
STLND-Landing
STHEB-Hebernate
STASC-Ascending
STDES-Descending

-----------------------------------------------------*/

typedef struct {
  uint32_t NumeroPacote;
  uint32_t timestamp;  // in milliseconds
  uint8_t ID;

  float temperature;
  float pressure;
  float altitude;
  bool ativo=false;
  float AccX;
  float AccY;
  float AccZ;
  float GyroX;
  float GyroY;
  float GyroZ;
  float MagX;
  float MagY;
  float MagZ;
  float latitude;   // in degrees * 10^6
  float longitude;  // in degrees * 10^6
  float altitudeGPS;
} __attribute__((packed)) SensorData_t;


typedef struct {
  uint32_t NumeroPacote;
  uint32_t timestamp;  // in milliseconds
  
  bool ativo1=false;
  bool ativo2=false;
  bool ativo3=false; 
  float temperature;
  float pressure;
  float altitude;
  float AccX;
  float AccY;
  float AccZ;
  float GyroX;
  float GyroY;
  float GyroZ;
  float MagX;
  float MagY;
  float MagZ;
  float latitude;   // in degrees * 10^6
  float longitude;  // in degrees * 10^6
  float altitudeGPS;
} __attribute__((packed)) TMVData_t;




typedef struct {
  bool isBMP388Calibrated;
 
  float BMPTemperatureOffset;

  float BMPAltitudeOffset;  
  
  ;
  float BMPPressureOffset;

  bool isIMUCalibrated;
  bool isAccCalibrated;
  float AccXOffset;  //
    //
  float AccYOffset;  //
;  //
  float AccZOffset;  //
  //

  bool isGyroCalibrated;
  float GyroXOffset;
  
  float GyroYOffset;  //
   //
  float GyroZOffset;  //
  //
  //
  bool isMagCalibrated;
  float MagXOffset;
   //
  float MagYOffset;  //
   //
  float MagZOffset;  //
  //
//

} __attribute__((packed)) Calibration_t;


typedef struct {
  SensorData_t sensorData1;
  SensorData_t sensorData2;
  SensorData_t sensorData3;
  TMVData_t tmvData;
} __attribute__((packed)) TMVDataBatch_t;




//Variaveis Globais


int NumeroEnvio = 0;


Calibration_t calib2, calib2_1, calib2_2, calib2_3,calib1_1, calib1_2, calib1_3;
bool isCalibrado1 = false;
bool isCalibrado2 = false;  
bool isCalibrado3 = false;

int calib[3];  //
//#endregion
unsigned long lastSendTime = 0;
bool isTempoSincronizado[4] = { false, false, false, false };
bool isCalibrado[4] = { false, false, false, false };
SensorData_t SensorData1, SensorData2, SensorData3;


// Função para salvar os valores na FRAM
// Função para recuperar os valores da FRAM



void displayCalibrationOffsets1(Calibration_t calib) {
  Serial.print("Calibração: ");
  Serial.print("AccXOffset: ");
  Serial.print(calib.AccXOffset, 2);
  Serial.print(", AccYOffset: ");
  Serial.print(calib.AccYOffset, 2);
  Serial.print(", AccZOffset: ");
  Serial.print(calib.AccZOffset, 2);
  Serial.print(", GyroXOffset: ");
  Serial.print(calib.GyroXOffset, 2);
  Serial.print(", GyroYOffset: ");
  Serial.print(calib.GyroYOffset, 2);
  Serial.print(", GyroZOffset: ");
  Serial.print(calib.GyroZOffset, 2);
  Serial.print(", MagXOffset: ");
  Serial.print(calib.MagXOffset, 2);
  Serial.print(", MagYOffset: ");
  Serial.print(calib.MagYOffset, 2);
  Serial.print(", MagZOffset: ");
  Serial.println(calib.MagZOffset, 2);
}
void mostraSensor(SensorData_t receivedData) {
  Serial.print("Dados recebidos: ");
  Serial.print(receivedData.NumeroPacote);
  // in milliseconds
  Serial.print("; ");
  Serial.print(receivedData.timestamp);  // in milliseconds
  Serial.print("; ");
  Serial.print(receivedData.ID);  
  Serial.print("; ");
  Serial.print(receivedData.temperature, 2);  // em Pa
  Serial.print("; ");
  Serial.print(receivedData.pressure, 2);  // em m
  Serial.print("; ");
  Serial.print(receivedData.altitude, 2);  // em m
  Serial.print("; ");
  Serial.print(receivedData.AccX, 2);  // em m
  Serial.print("; ");
  Serial.print(receivedData.AccY, 2);  // em m
  Serial.print("; ");
  Serial.print(receivedData.AccZ, 2);  // em m
  Serial.print("; ");
  Serial.print(receivedData.GyroX, 2);  // em m
  Serial.print("; ");
  Serial.print(receivedData.GyroY, 2);  // em m
  Serial.print("; ");
  Serial.print(receivedData.GyroZ, 2);  // em m
  Serial.print("; ");
  Serial.print(receivedData.MagX, 2);  // em m
  Serial.print("; ");
  Serial.print(receivedData.MagY, 2);  // em m
  Serial.print("; ");
  Serial.println(receivedData.MagZ, 2);  // em m
  Serial.print("; ");
  Serial.print(receivedData.latitude, 6);  // em graus * 10^6
  Serial.print("; ");
  Serial.print(receivedData.longitude, 6);  // em graus * 10^6
  Serial.print("; ");

  Serial.println(receivedData.altitudeGPS, 2);  // em m
  // em m
}
void mostraTMV(TMVData_t receivedData) {
  Serial.print("Dados TMV: ");
  Serial.print(receivedData.NumeroPacote);
  // in milliseconds
  Serial.print("; ");
  Serial.print(receivedData.timestamp);  // in milliseconds

  Serial.print("; ");
  Serial.print(receivedData.temperature, 2);  // em Pa
  Serial.print("; ");
  Serial.print(receivedData.pressure, 2);  // em m
  Serial.print("; ");
  Serial.print(receivedData.altitude, 2);  // em m
  Serial.print("; ");
  Serial.print(receivedData.AccX, 2);  // em m
  Serial.print("; ");
  Serial.print(receivedData.AccY, 2);  // em m
  Serial.print("; ");
  Serial.print(receivedData.AccZ, 2);  // em m
  Serial.print("; ");
  Serial.print(receivedData.GyroX, 2);  // em m
  Serial.print("; ");
  Serial.print(receivedData.GyroY, 2);  // em m
  Serial.print("; ");
  Serial.print(receivedData.GyroZ, 2);  // em m
  Serial.print("; ");
  Serial.print(receivedData.MagX, 2);  // em m
  Serial.print("; ");
  Serial.print(receivedData.MagY, 2);  // em m
  Serial.print("; ");
  Serial.println(receivedData.MagZ, 2);  // em m
  Serial.print("; ");
  Serial.print(receivedData.latitude, 6);  // em graus * 10^6
  Serial.print("; ");
  Serial.print(receivedData.longitude, 6);  // em graus * 10^6
  Serial.print("; ");

  Serial.println(receivedData.altitudeGPS, 2);  // em m
  // em m
}
// Função para converter a estrutura em string
String tmvDataToString(TMVData_t data) {
  String result = " TMV-";
  
  // Concatena os campos em uma única linha de string
  result += String(data.NumeroPacote) + ",";
  result += String(data.timestamp) + ",";
  result += String(data.ativo1) + ",";
  result += String(data.ativo2) + ",";
  result += String(data.ativo3) + ",";
  result += String(data.temperature) + ",";
  result += String(data.pressure) + ",";
  result += String(data.altitude) + ",";
  result += String(data.AccX) + ",";
  result += String(data.AccY) + ",";
  result += String(data.AccZ) + ",";
  result += String(data.GyroX) + ",";
  result += String(data.GyroY) + ",";
  result += String(data.GyroZ) + ",";
  result += String(data.MagX) + ",";
  result += String(data.MagY) + ",";
  result += String(data.MagZ) + ",";
  result += String(data.latitude) + ",";
  result += String(data.longitude) + ",";
  result += String(data.altitudeGPS);

  // Adiciona uma quebra de linha no final
  result += "\n";
  
  return result;
}
String sensorDataToString(SensorData_t data) {
  String result = " ";
  result += String(data.ID) + ",";
  // Concatena os campos em uma única linha de string
  result += String(data.NumeroPacote) + ",";
  result += String(data.timestamp) + ",";

  result += String(data.temperature) + ",";
  result += String(data.pressure) + ",";
  result += String(data.altitude) + ",";
  result += String(data.ativo) + ",";
  result += String(data.AccX) + ",";
  result += String(data.AccY) + ",";
  result += String(data.AccZ) + ",";
  result += String(data.GyroX) + ",";
  result += String(data.GyroY) + ",";
  result += String(data.GyroZ) + ",";
  result += String(data.MagX) + ",";
  result += String(data.MagY) + ",";
  result += String(data.MagZ) + ",";
  result += String(data.latitude) + ",";
  result += String(data.longitude) + ",";
  result += String(data.altitudeGPS)+",";
  
  return result;
}

float computeMajorityVote(bool valid1, float a, bool valid2, float b, bool valid3, float c, float offset) {
  int soma = valid1 + valid2 + valid3;
  float result = 0;

  // Se apenas um valor for válido, retorna esse valor
  if (soma == 1) {
    
    if (valid1) return a;

    if (valid2) return b;
    if (valid3) return c;
  }
  // Se dois valores forem válidos, retorna a média dos dois

  if (soma == 2) {
    if (valid1 && valid2) return (a + b) / 2;
    if (valid1 && valid3) return (a + c) / 2;
    if (valid2 && valid3) return (b + c) / 2;
  }
  // Se todos os valores forem válidos, faz TMV
  if (soma == 3) {
    int diffAB = abs(a - b);
    int diffAC = abs(a - c);
    int diffBC = abs(b - c);

    // ✅ Se todos os valores estiverem dentro do offset, retorna a média dos três
    if (diffAB <= offset && diffAC <= offset && diffBC <= offset) {
      return (a + b + c) / 3;
    }

    // 🚨 Verifica se apenas UM valor está fora do offset e descarta ele
    if (diffAB > offset && diffAC > offset) return b;  // A e C muito diferentes de B, então B é o valor confiável
    if (diffAB > offset && diffBC > offset) return c;  // A e B muito diferentes de C, então C é o valor confiável
    if (diffAC > offset && diffBC > offset) return a;  // B e C muito diferentes de A, então A é o valor confiável

    // 🔥 Se dois valores são próximos e um está distante, descarta o outlier e faz média dos dois próximos
    if (diffAB <= offset) return (a + b) / 2;  // A e B são confiáveis
    if (diffAC <= offset) return (a + c) / 2;  // A e C são confiáveis
    if (diffBC <= offset) return (b + c) / 2;  // B e C são confiáveis

    // 🚨 Se todos forem muito diferentes, retorna a média geral (última defesa)
    return (a + b + c) / 3;
  }
}


void logSensorTask(void *pvParameters) {
  (void)pvParameters;  // unused parameter
  ;
  TMVDataBatch_t receivedData;  //Nova mensagem

  for (;;) {
    // 1) Wait for data from the queue (blocking)
    if (xQueueReceive(QueueTMV, &receivedData, 0) == pdTRUE) {
      // 2) Write data to file
      if (fileOpen) {
        String logLine ;
//if(receivedData.sensorData1.ativo==true){
        // Build a line: "timestamp,temperature,pressure"
         logLine = sensorDataToString(receivedData.sensorData1) ;
       // Serial.println(logLine);
        logFile.print(logLine);
      //}
       //
        logLine= sensorDataToString(receivedData.sensorData2) ;
       // Serial.println(logLine);
        logFile.print(logLine);
     // }
       // if(receivedData.sensorData3.ativo==true){
        logLine= sensorDataToString(receivedData.sensorData3);        
        //Serial.println(logLine);
        logFile.print(logLine);
      //}

        logLine=tmvDataToString(receivedData.tmvData);
        //Serial.println(logLine);
        logFile.println(logLine);
     
        // 3) Flush every few writes to reduce flash wear
        logCounter++;
        if (logCounter >= BUFFER_FLUSH_COUNT) {
          logFile.flush();
        logCounter = 0;
       }

        // Debug print (optional)
        // Note: frequent Serial.print can slow logging
        //Serial.print("Logged: ");
        //Serial.println(logLine);
      } else {
        Serial.println("File not open, skipping write!");
      }
    }
  }
}

void calibraSensores(SensorData_t data){

  if (data.ID == 1&& isCalibrado1 == false&&data.temperature!=0) {
    Serial.println("Calibrado 1 dados");
    isCalibrado1 = true;
    calib1_1.BMPTemperatureOffset = data.temperature;
    calib1_1.BMPAltitudeOffset = data.altitude;
    calib1_1.BMPPressureOffset = data.pressure;
    calib1_1.AccXOffset = data.AccX;
    calib1_1.AccYOffset = data.AccY;
    calib1_1.AccZOffset = data.AccZ;
    calib1_1.GyroXOffset = data.GyroX;
    calib1_1.GyroYOffset = data.GyroY;
    calib1_1.GyroZOffset = data.GyroZ;
    calib1_1.MagXOffset = data.MagX;
    calib1_1.MagYOffset = data.MagY;
    calib1_1.MagZOffset = data.MagZ;
  } 
  if (data.ID == 2&& isCalibrado2 == false&&data.temperature!=0) {
    Serial.println("Calibrado 2 dados");
    isCalibrado2 = true;
    calib1_2.BMPTemperatureOffset = data.temperature;
    calib1_2.BMPAltitudeOffset = data.altitude;
    calib1_2.BMPPressureOffset = data.pressure;
    calib1_2.AccXOffset = data.AccX;
    calib1_2.AccYOffset = data.AccY;
    calib1_2.AccZOffset = data.AccZ;
    calib1_2.GyroXOffset = data.GyroX;
    calib1_2.GyroYOffset = data.GyroY;
    calib1_2.GyroZOffset = data.GyroZ;
    calib1_2.MagXOffset = data.MagX;
    calib1_2.MagYOffset = data.MagY;
    calib1_2.MagZOffset = data.MagZ;

  } 
  if (data.ID == 3 && isCalibrado3 == false&&data.temperature!=0) {
    Serial.println("Calibrado 3 dados");

    isCalibrado3 = true;
    calib1_3.BMPTemperatureOffset = data.temperature;
    calib1_3.BMPAltitudeOffset = data.altitude;
    calib1_3.BMPPressureOffset = data.pressure;
    calib1_3.AccXOffset = data.AccX;
    calib1_3.AccYOffset = data.AccY;
    calib1_3.AccZOffset = data.AccZ;
    calib1_3.GyroXOffset = data.GyroX;
    calib1_3.GyroYOffset = data.GyroY;
    calib1_3.GyroZOffset = data.GyroZ;
    calib1_3.MagXOffset = data.MagX;
    calib1_3.MagYOffset = data.MagY;
    calib1_3.MagZOffset = data.MagZ;

  }
int soma=isCalibrado1+isCalibrado2,isCalibrado3;

  if (soma==3) {
    Serial.println("Calibrado 1,2 e 3 true");
    calib2.BMPTemperatureOffset = (calib1_1.BMPTemperatureOffset + calib1_2.BMPTemperatureOffset + calib1_3.BMPTemperatureOffset) / 3;
    calib2.BMPAltitudeOffset = (calib1_1.BMPAltitudeOffset + calib1_2.BMPAltitudeOffset + calib1_3.BMPAltitudeOffset) / 3;
    calib2.BMPPressureOffset = (calib1_1.BMPPressureOffset + calib1_2.BMPPressureOffset + calib1_3.BMPPressureOffset) / 3;
    calib2.isBMP388Calibrated = true;
    calib2.AccXOffset = (calib1_1.AccXOffset + calib1_2.AccXOffset + calib1_3.AccXOffset) / 3;
    calib2.AccYOffset = (calib1_1.AccYOffset + calib1_2.AccYOffset + calib1_3.AccYOffset) / 3;
    calib2.AccZOffset = (calib1_1.AccZOffset + calib1_2.AccZOffset + calib1_3.AccZOffset) / 3;
    calib2.GyroXOffset = (calib1_1.GyroXOffset + calib1_2.GyroXOffset + calib1_3.GyroXOffset) / 3;
    calib2.GyroYOffset = (calib1_1.GyroYOffset + calib1_2.GyroYOffset + calib1_3.GyroYOffset) / 3;
    calib2.GyroZOffset = (calib1_1.GyroZOffset + calib1_2.GyroZOffset + calib1_3.GyroZOffset) / 3;
    calib2.MagXOffset = (calib1_1.MagXOffset + calib1_2.MagXOffset + calib1_3.MagXOffset) / 3;
    calib2.MagYOffset = (calib1_1.MagYOffset + calib1_2.MagYOffset + calib1_3.MagYOffset) / 3;
   // saveCalibrationToFRAM((float *)&calib);
  
  }
  if(soma==2){
    if(isCalibrado1==true&&isCalibrado2==true){
      Serial.println("Calibrado 3 false");
      calib2.BMPTemperatureOffset = (calib1_1.BMPTemperatureOffset + calib1_2.BMPTemperatureOffset) / 2;
      Serial.print("Calibrado 3 false 2-");
      Serial.print(calib1_1.BMPTemperatureOffset);
      Serial.print("-");
      Serial.print(calib1_2.BMPTemperatureOffset);
      Serial.print("-");
      Serial.println(calib2.BMPTemperatureOffset);
      calib2.BMPAltitudeOffset = (calib1_1.BMPAltitudeOffset + calib1_2.BMPAltitudeOffset) / 2;
      calib2.BMPPressureOffset = (calib1_1.BMPPressureOffset + calib1_2.BMPPressureOffset) / 2;
    calib2.AccXOffset = (calib1_1.AccXOffset + calib1_2.AccXOffset) / 2;
    calib2.AccYOffset = (calib1_1.AccYOffset + calib1_2.AccYOffset) / 2;
    calib2.AccZOffset = (calib1_1.AccZOffset + calib1_2.AccZOffset) / 2;
    calib2.GyroXOffset = (calib1_1.GyroXOffset + calib1_2.GyroXOffset) / 2;
    calib2.GyroYOffset = (calib1_1.GyroYOffset + calib1_2.GyroYOffset) / 2;
    calib2.GyroZOffset = (calib1_1.GyroZOffset + calib1_2.GyroZOffset) / 2;
    calib2.MagXOffset = (calib1_1.MagXOffset + calib1_2.MagXOffset) / 2;
    calib2.MagYOffset = (calib1_1.MagYOffset + calib1_2.MagYOffset) / 2;

    //saveCalibrationToFRAM((float *)&calib);

  }
  if(isCalibrado1==true&&isCalibrado3==true){
    Serial.println("Calibrado 2 false");
    calib2.BMPTemperatureOffset = (calib1_1.BMPTemperatureOffset + calib1_3.BMPTemperatureOffset) / 2;
    calib2.BMPAltitudeOffset = (calib1_1.BMPAltitudeOffset + calib1_3.BMPAltitudeOffset) / 2;

    calib2.BMPPressureOffset = (calib1_1.BMPPressureOffset + calib1_3.BMPPressureOffset) / 2;
    calib2.AccXOffset = (calib1_1.AccXOffset + calib1_3.AccXOffset) / 2;
    calib2.AccYOffset = (calib1_1.AccYOffset + calib1_3.AccYOffset) / 2;
    calib2.AccZOffset = (calib1_1.AccZOffset + calib1_3.AccZOffset) / 2;
    calib2.GyroXOffset = (calib1_1.GyroXOffset + calib1_3.GyroXOffset) / 2;
    calib2.GyroYOffset = (calib1_1.GyroYOffset + calib1_3.GyroYOffset) / 2;
    calib2.GyroZOffset = (calib1_1.GyroZOffset + calib1_3.GyroZOffset) / 2;
    calib2.MagXOffset = (calib1_1.MagXOffset + calib1_3.MagXOffset) / 2;
    calib2.MagYOffset = (calib1_1.MagYOffset + calib1_3.MagYOffset) / 2;
    //saveCalibrationToFRAM((float *)&calib);

  }
  if(isCalibrado2==true&&isCalibrado3==true){
    Serial.println("Calibrado 1 false");
    calib2.BMPTemperatureOffset = (calib1_3.BMPTemperatureOffset + calib1_2.BMPTemperatureOffset) / 2;
    calib2.BMPAltitudeOffset = (calib1_3.BMPAltitudeOffset + calib1_2.BMPAltitudeOffset) / 2;
    calib2.BMPPressureOffset = (calib1_3.BMPPressureOffset + calib1_2.BMPPressureOffset) / 2;
    calib2.AccXOffset = (calib1_3.AccXOffset + calib1_2.AccXOffset) / 2;
    calib2.AccYOffset = (calib1_3.AccYOffset + calib1_2.AccYOffset) / 2;
    calib2.AccZOffset = (calib1_3.AccZOffset + calib1_2.AccZOffset) / 2;
    calib2.GyroXOffset = (calib1_3.GyroXOffset + calib1_2.GyroXOffset) / 2;
    calib2.GyroYOffset = (calib1_3.GyroYOffset + calib1_2.GyroYOffset) / 2;
    calib2.GyroZOffset = (calib1_3.GyroZOffset + calib1_2.GyroZOffset) / 2;
    calib2.MagXOffset = (calib1_3.MagXOffset + calib1_2.MagXOffset) / 2;
    calib2.MagYOffset = (calib1_3.MagYOffset + calib1_2.MagYOffset) / 2;
    //saveCalibrationToFRAM((float *)&calib);

  }

}
if(soma==1)
{
calib2.BMPTemperatureOffset = data.temperature;
calib2.BMPAltitudeOffset = data.altitude;
calib2.BMPPressureOffset = data.pressure;
calib2.AccXOffset = data.AccX;
calib2.AccYOffset = data.AccY;
calib2.AccZOffset = data.AccZ;
calib2.GyroXOffset = data.GyroX;
calib2.GyroYOffset = data.GyroY;
calib2.GyroZOffset = data.GyroZ;
calib2.MagXOffset = data.MagX;
calib2.MagYOffset = data.MagY;
calib2.MagZOffset = data.MagZ;


}
Serial.print("Calibrado 1: ");
Serial.println(isCalibrado1);
Serial.print("Calibrado 2: ");
Serial.println(isCalibrado2);
Serial.print("Calibrado 3: ");
Serial.println(isCalibrado3);

if(isCalibrado1==true){
 

calib2_1.AccXOffset =calib2.AccXOffset-calib1_1.AccXOffset;


calib2_1.AccYOffset =calib2.AccYOffset-calib1_1.AccYOffset;
calib2_1.AccZOffset =calib2.AccZOffset-calib1_1.AccZOffset;
calib2_1.GyroXOffset =calib2.GyroXOffset-calib1_1.GyroXOffset;
calib2_1.GyroYOffset =calib2.GyroYOffset-calib1_1.GyroYOffset;
calib2_1.GyroZOffset =calib2.GyroZOffset-calib1_1.GyroZOffset;
calib2_1.MagXOffset =calib2.MagXOffset-calib1_1.MagXOffset;
calib2_1.MagYOffset =calib2.MagYOffset-calib1_1.MagYOffset;
calib2_1.MagZOffset =calib2.MagZOffset-calib1_1.MagZOffset;
calib2_1.BMPTemperatureOffset = calib2.BMPTemperatureOffset-calib1_1.BMPTemperatureOffset;
Serial.print("TemkOFest-");
Serial.print(calib2_1.BMPTemperatureOffset );
Serial.print("-");
Serial.print(calib2.BMPTemperatureOffset );
Serial.print("-");
Serial.print(calib1_1.BMPTemperatureOffset );

calib2_1.BMPAltitudeOffset = calib2.BMPAltitudeOffset-calib1_1.BMPAltitudeOffset;
calib2_1.BMPPressureOffset = calib2.BMPPressureOffset-calib1_1.BMPPressureOffset;
}
if(isCalibrado2==true){
  

calib2_2.AccXOffset =calib2.AccXOffset-calib1_2.AccXOffset;
calib2_2.AccYOffset =calib2.AccYOffset-calib1_2.AccYOffset;
calib2_2.AccZOffset =calib2.AccZOffset-calib1_2.AccZOffset;
calib2_2.GyroXOffset =calib2.GyroXOffset-calib1_2.GyroXOffset;
calib2_2.GyroYOffset =calib2.GyroYOffset-calib1_2.GyroYOffset;
calib2_2.GyroZOffset =calib2.GyroZOffset-calib1_2.GyroZOffset;
calib2_2.MagXOffset =calib2.MagXOffset-calib1_2.MagXOffset;
calib2_2.MagYOffset =calib2.MagYOffset-calib1_2.MagYOffset;
calib2_2.MagZOffset =calib2.MagZOffset-calib1_2.MagZOffset;
calib2_2.BMPTemperatureOffset = calib2.BMPTemperatureOffset-calib1_2.BMPTemperatureOffset;
Serial.print("TemkOFest-");
Serial.print(calib2_2.BMPTemperatureOffset );
Serial.print("-");-
Serial.print(calib2.BMPTemperatureOffset );
Serial.print("-");
Serial.print(calib1_2.BMPTemperatureOffset );
calib2_2.BMPAltitudeOffset = calib2.BMPAltitudeOffset-calib1_2.BMPAltitudeOffset;
calib2_2.BMPPressureOffset = calib2.BMPPressureOffset-calib1_2.BMPPressureOffset;
}
if(isCalibrado3==true){
 
calib2_3.AccXOffset =calib2.AccXOffset-calib1_3.AccXOffset;
calib2_3.AccYOffset =calib2.AccYOffset-calib1_3.AccYOffset;
calib2_3.AccZOffset =calib2.AccZOffset-calib1_3.AccZOffset;
calib2_3.GyroXOffset =calib2.GyroXOffset-calib1_3.GyroXOffset;
calib2_3.GyroYOffset =calib2.GyroYOffset-calib1_3.GyroYOffset;
calib2_3.GyroZOffset =calib2.GyroZOffset-calib1_3.GyroZOffset;
calib2_3.MagXOffset =calib2.MagXOffset-calib1_3.MagXOffset;
calib2_3.MagYOffset =calib2.MagYOffset-calib1_3.MagYOffset;
calib2_3.MagZOffset =calib2.MagZOffset-calib1_3.MagZOffset;
calib2_3.BMPTemperatureOffset = calib2.BMPTemperatureOffset-calib1_3.BMPTemperatureOffset;
calib2_3.BMPAltitudeOffset = calib2.BMPAltitudeOffset-calib1_3.BMPAltitudeOffset;
calib2_3.BMPPressureOffset = calib2.BMPPressureOffset-calib1_3.BMPPressureOffset;

}



}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  // Serial.println("OnDataRecv");
  SensorData_t receivedData;
 TMVData_t tripleMajorityVotingData;
 
  memcpy(&receivedData, incomingData, sizeof(SensorData_t));


  if (receivedData.ID == 1) {


    isNovo[0] = true;  
    SensorData1 = receivedData;
    SensorData1.ativo=true;
 


  }
  if (receivedData.ID == 2) {

   
    isNovo[1] = true;
  
    SensorData2 = receivedData;
    SensorData2.ativo=true;
 

  }
  if (receivedData.ID == 3) {
   
    isNovo[2] = true;
  
    SensorData3 = receivedData;
    SensorData3.ativo=true;



  }
 // Verifica se todos os dados dos sensores foram recebidos
 if ((isNovo[0] && isNovo[1] && isNovo[2]) || (millis() - lastSendTime >= 100)) {
  // Envia os dados dos sensores
  tripleMajorityVotingData.timestamp = millis();
  tripleMajorityVotingData.NumeroPacote = SensorData1.NumeroPacote;

  tripleMajorityVotingData.ativo1 = SensorData1.ativo;
  tripleMajorityVotingData.ativo2 = SensorData2.ativo;
  tripleMajorityVotingData.ativo3 = SensorData3.ativo;


  tripleMajorityVotingData.temperature = computeMajorityVote(SensorData1.ativo,SensorData1.temperature,SensorData2.ativo, SensorData2.temperature,SensorData3.ativo, SensorData3.temperature, 2);
  tripleMajorityVotingData.pressure = computeMajorityVote(SensorData1.ativo,SensorData1.pressure, SensorData2.ativo,SensorData2.pressure, SensorData3.ativo,SensorData3.pressure, 15);
  tripleMajorityVotingData.altitude = computeMajorityVote(SensorData1.ativo,SensorData1.altitude,SensorData2.ativo, SensorData2.altitude, SensorData3.ativo,SensorData3.altitude, 5); 
  tripleMajorityVotingData.latitude = computeMajorityVote(SensorData1.ativo,SensorData1.latitude,SensorData2.ativo, SensorData2.latitude,SensorData3.ativo, SensorData3.latitude, 0.00001);
  tripleMajorityVotingData.longitude = computeMajorityVote(SensorData1.ativo,SensorData1.longitude, SensorData2.ativo,SensorData2.longitude, SensorData3.ativo,SensorData3.longitude, 0.00001);
  
  tripleMajorityVotingData.AccX = computeMajorityVote(SensorData1.ativo,SensorData1.AccX,SensorData2.ativo, SensorData2.AccX, SensorData3.ativo,SensorData3.AccX, 1);
  tripleMajorityVotingData.AccY = computeMajorityVote(SensorData1.ativo,SensorData1.AccY, SensorData2.ativo,SensorData2.AccY, SensorData3.ativo,SensorData3.AccY, 1);
  tripleMajorityVotingData.AccZ = computeMajorityVote(SensorData1.ativo,SensorData1.AccZ,SensorData2.ativo, SensorData2.AccZ,SensorData3.ativo, SensorData3.AccZ, 1);
  tripleMajorityVotingData.GyroX = computeMajorityVote(SensorData1.ativo,SensorData1.GyroX, SensorData2.ativo,SensorData2.GyroX,SensorData3.ativo, SensorData3.GyroX, 1);
  tripleMajorityVotingData.GyroY = computeMajorityVote(SensorData1.ativo,SensorData1.GyroY, SensorData2.ativo,SensorData2.GyroY, SensorData3.ativo,SensorData3.GyroY, 1);
  tripleMajorityVotingData.GyroZ = computeMajorityVote(SensorData1.ativo,SensorData1.GyroZ,SensorData2.ativo, SensorData2.GyroZ,SensorData3.ativo, SensorData3.GyroZ, 1);
  tripleMajorityVotingData.MagX = computeMajorityVote(SensorData1.ativo,SensorData1.MagX, SensorData2.ativo,SensorData2.MagX, SensorData3.ativo,SensorData3.MagX, 1);
  tripleMajorityVotingData.MagY = computeMajorityVote(SensorData1.ativo,SensorData1.MagY, SensorData2.ativo,SensorData2.MagY, SensorData3.ativo,SensorData3.MagY, 1);
  tripleMajorityVotingData.MagZ = computeMajorityVote(SensorData1.ativo,SensorData1.MagZ,SensorData2.ativo, SensorData2.MagZ, SensorData3.ativo,SensorData3.MagZ, 1);
  
  tripleMajorityVotingData.NumeroPacote=NumeroPacoteTMV++;

  TMVDataBatch_t tmvDataBatch;
  tmvDataBatch.sensorData1 = SensorData1;
  tmvDataBatch.sensorData2 = SensorData2;
  tmvDataBatch.sensorData3 = SensorData3;
  tmvDataBatch.tmvData = tripleMajorityVotingData;
  // Envia os dados para a fila
  if (xQueueSend(QueueTMV, &tmvDataBatch, portMAX_DELAY) != pdTRUE) {
    Serial.println("Falha ao enviar dados para a fila de envio!");
  } else {
    
  }
  if (xQueueSend(QueueEnvio, &tripleMajorityVotingData, portMAX_DELAY) != pdTRUE) {
    Serial.println("Falha ao enviar dados para a fila de envio!");
  } else {
    
  }
  // Reseta os flags para novos dados
  isNovo[0] = false;
  isNovo[1] = false;
  isNovo[2] = false;
SensorData1.ativo=false;
SensorData2.ativo=false;
SensorData3.ativo=false;

//limpa os dados dos sensores
memset(&SensorData1, 0, sizeof(SensorData1));
memset(&SensorData2, 0, sizeof(SensorData2));
memset(&SensorData3, 0, sizeof(SensorData3));


  // Atualiza o tempo do último envio, se necessário
  lastSendTime = millis();
}

}
void enviaTempoTask(void *pvParameters) {
  (void)pvParameters;  // unused parameter
  DataPacket packet;
  for (;;) {
    
    packet.timeMillis = millis();
  
    // Enviar os dados para o peer
    esp_now_send(ESPXIAO6, (uint8_t *)&packet, sizeof(packet));
    packet.timeMillis = millis();
  
    // Enviar os dados para o peer
    esp_now_send(ESPXIAO5, (uint8_t *)&packet, sizeof(packet));
    packet.timeMillis = millis();
  
    // Enviar os dados para o peer
  esp_now_send(ESPXIAO4, (uint8_t *)&packet, sizeof(packet));

    vTaskDelay(10000 / portTICK_PERIOD_MS);
   
    }
  }
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {

}

void EnviaDadosTask(void *pvParameters) {
  (void)pvParameters;  // Evita aviso de variável não utilizada
  TMVData_t dados;
  for (;;) {
  
    //memset(&dados, 0, sizeof(dados));
    if (xQueueReceive(QueueEnvio, &dados, portMAX_DELAY) == pdTRUE) {


     
    

      //mostraSensor(dados);
      RF24NetworkHeader header(/*to node*/ other_node);

     bool ok = network.write(header, &dados, sizeof(dados));
     Serial.println(ok ? F("ok.") : F("failed."));
    }
  }

  // Aguarda antes de atualizar novamente
}


void buzzerTask(void *pvParameters) {
  pinMode(BUZZER_PIN, OUTPUT);

  while (true) {
    digitalWrite(BUZZER_PIN, HIGH);  // Liga o buzzer
    vTaskDelay(pdMS_TO_TICKS(500));  // Espera 500ms

    digitalWrite(BUZZER_PIN, LOW);   // Desliga o buzzer
    vTaskDelay(pdMS_TO_TICKS(500));  // Espera 500ms
  }
}

void setup() {
  Serial.begin(115200);
  delay(3000);
  Serial.println("Iniciando...");
  WiFi.mode(WIFI_STA);                // Modo estação
  WiFi.channel(ESPNOW_WIFI_CHANNEL);  // Canal de comunicação

  // Inicializa ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ Erro ao inicializar ESP-NOW");
    return;
  }
  Serial.println("inicializar ESP-NOW");
 //----------------------------------------------------------------
  // Inicializa FRAM
  // ---------------------

  if (!fram.begin(0x50)) {
    Serial.println("FRAM não encontrada!");
 
   
  }
  Serial.println("FRAM detectada!");
// 1) Initialize SPIFFS
Serial.println("Boa tarde");
if (!SPIFFS.begin(true)) {
  Serial.println("SPIFFS initialization failed!");
  // It's often okay to continue without returning, but no file I/O will work.
} else {
  Serial.println("SPIFFS initialized.");
}

// 2) Open the file in append mode
logFile = SPIFFS.open(FILE_PATH, FILE_APPEND);
if (logFile) {
  Serial.println("Log file opened.");
  fileOpen = true;
} else {
  Serial.println("Failed to open log file.");
  fileOpen = false;
}
//endereços do master e slave
//Esquerdo
memcpy(peerInfo.peer_addr, ESPXIAO3, 6);
if (esp_now_add_peer(&peerInfo) != ESP_OK) {
  Serial.println("Failed to add peer");
  return;
}

//Direito
memcpy(peerInfo.peer_addr, ESPXIAO4, 6);
if (esp_now_add_peer(&peerInfo) != ESP_OK) {
  Serial.println("Failed to add peer");
  return;
}



  // Register first peer
  memcpy(peerInfo.peer_addr, ESPXIAO5, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  // Register first peer
  memcpy(peerInfo.peer_addr, ESPXIAO6, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  // Register first peer
  memcpy(peerInfo.peer_addr, ESPXIAO7, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  if (!radio.begin()) {
    Serial.println(F("Radio hardware not responding!"));
   
  } else {
    Serial.println(F("Radio hardware OK!"));
    radio.setChannel(90);
    radio.setAutoAck(false);
    network.begin(/*node address*/ this_node);
   
  
  }

  // Registra callback de recepção
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
  esp_now_register_send_cb(OnDataSent);



  
  QueueEnvio = xQueueCreate(3, sizeof(SensorData_t));
  if (QueueEnvio == NULL) {
    Serial.println("Failed to create queue de ENvio!");
    // Without a queue, we cannot proceed properly
  }

  QueueTMV = xQueueCreate(10, sizeof( TMVDataBatch_t));
  if (QueueTMV == NULL) {
    Serial.println("Failed to create queue de ENvio!");
    // Without a queue, we cannot proceed properly
  }

  //xTaskCreatePinnedToCore(TrilpeMachineVotingTask, "TrilpeMAchineVOtingSensor", 4096, NULL, 2, &TrilpeMachineVotingTaskHandle, 1);

  xTaskCreatePinnedToCore(EnviaDadosTask, "EnviaDados", 4096, NULL, 2, &EnviaDadosTaskHandle, 1);
  xTaskCreatePinnedToCore(logSensorTask, "LogSensorTask", 4096, NULL, 1, &logSensorTaskkHandle, 1);
  //xTaskCreatePinnedToCore(enviaTempoTask, "EnviaTempoTask", 1024, NULL, 1, NULL, 1);

  // Cria a task do buzzer
  // xTaskCreatePinnedToCore(buzzerTask,         "Buzzer Task",      1024,  NULL,1,NULL,1);
}
void closeFile() {
  if (fileOpen) {
    
    logFile.flush();
    logFile.close();
    fileOpen = false;
    Serial.println("Log file closed.");
  } else {
    Serial.println("File already closed.");
  }
}

void readLog() {
  closeFile();  // ensure no writing while reading

  File file = SPIFFS.open(FILE_PATH, FILE_READ);
  if (!file) {
    Serial.println("Failed to open log file for reading!");
    return;
  }

  Serial.println("\\n--- Log contents ---");
  while (file.available()) {
    Serial.write(file.read());
  }
  file.close();
  Serial.println("\\n--- End of log ---");
}
void apagaFicheiro() {
  closeFile();
  SPIFFS.remove(FILE_PATH);
  Serial.println("Ficheiro apagado!");
}
void getSPIFFSInfo() {
  size_t totalBytes = SPIFFS.totalBytes();  // Tamanho total da partição
  size_t usedBytes = SPIFFS.usedBytes();    // Espaço já utilizado

  Serial.print("Espaço total: ");
  Serial.print(totalBytes);
  Serial.println(" bytes");

  Serial.print("Espaço usado: ");
  Serial.print(usedBytes);
  Serial.println(" bytes");

  Serial.print("Espaço livre: ");
  Serial.print(totalBytes - usedBytes);
  Serial.println(" bytes");
}
void loop() {

  if (Serial.available() > 0) {
    char command = Serial.read();
    switch (command) {
      case 'r':
        readLog();
        break;
      case 'c':
        closeFile();
        break;
      case 'a':
        apagaFicheiro();
        break;
      case 'e':
        getSPIFFSInfo();
        break;
      default:
        Serial.println("Unknown command. Use 'r' for read, 'c' for close.");
        break;
    }
  }


}
