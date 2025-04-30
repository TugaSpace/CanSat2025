#include <Arduino.h>
#include "FS.h"
#include "SPIFFS.h"
#include <Adafruit_BMP3XX.h>
#include <ICM20948_WE.h>
#include <TinyGPS++.h>
#include <esp_now.h>
#include <WiFi.h>
#include "Adafruit_EEPROM_I2C.h"
#include "Adafruit_FRAM_I2C.h"

// -----------------------
// Definições de pinos
// -----------------------
#define ICM_CS 1
#define BMP_CS 2  // Chip Select (CS) do BMP388
bool spi = true;
// Defina os pinos de RX e TX do GPS no Xiao ESP32
#define RX_PIN D7
#define TX_PIN D6

#define ID_SENSOR 2//2 ou 3

//---------------------------
//MAC Adresses dos ESP32
//---------------------------

//uint8_t ESPXIAO1[] = {0x24, 0xec, 0x4a, 0x00, 0x4f, 0x08};  // MAC do ESP1
//uint8_t ESPXIAO2[] = {0x34, 0x85, 0x18, 0x91, 0x42, 0x84};  // MAC do ESP2 com3

//Esp32 do Modulo Principal Direito
uint8_t ESPXIAO4[] = {0x30, 0x30, 0xf9, 0x16, 0xa1, 0xb0};  // MAC do ESP4 com10 30:30:f9:16:a1:b0d, 0xb1, 0x38};  // MAC do ESP4 com10 30:30:f9:16:a1:b0

//ESPXIAO3 e ESPXIAO4 são os dois ESP32 do Modulo Principal Esquerdo
//uint8_t ESPXIAO3[]34:85:18:91:42:84

uint8_t ESPXIAO3[] = { 0x34, 0x85, 0x18, 0x91, 0x42, 0x84 };
                                                           // MAC do ESP3 com12  Modulo Principal Esquerdo
//uint8_t ESPXIAO4[] = { 0xe8, 0x06, 0x90, 0x9d, 0xb1, 0x38 };  // MAC do ESP4 com10 Modulo Principal Direito
//uint8_t ESPXIAO5[] = {0xe8, 0x06, 0x90, 0xa0, 0xdf, 0x6c};  // MAC do ESP5 com9 Modulo Sensores
//uint8_t ESPXIAO6[] = {0xe8, 0x06, 0x90,  0x9D, 0x93, 0x34};  // MAC do ESP6. com 15 Modulo Sensores
//uint8_t ESPXIAO7[] = {0xe8, 0x06, 0x90,  0x9D, 0xa0, 0x60};  // MAC do ESP7. com16 Modulo Sensores

uint8_t ESPUNO1[] = { 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x02 };  // MAC do ESP UNO1  Modulo oculos
uint8_t ESPUNO2[] = { 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x01 };  // MAC do ESP UNO2  Modulo oculos
uint8_t ESPUNO3[] = { 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x02 };  // MAC do ESP UNO3  Modulo oculos

esp_now_peer_info_t peerInfo;
// -----------------------
// Inicializar os sensores
ICM20948_WE myIMU = ICM20948_WE(&SPI, ICM_CS, spi);  // -> uses SPI, spi is just a flag, see SPI example
Adafruit_BMP3XX bmp;                                 // Para BMP388
// Configuração da serial para o GPS
HardwareSerial GPSSerial(0);
TinyGPSPlus gps;


//Task Handles
TaskHandle_t Sensor100HZTaskHandle;
TaskHandle_t logSensorTaskkHandle;


Adafruit_FRAM_I2C fram = Adafruit_FRAM_I2C();

// -------------------------------------------------
// Configuration Esp Flash
// -------------------------------------------------
#define FILE_PATH "/log.txt"

// Endereço na FRAM para armazenar os valores
#define FRAM_START_ADDR 0
#define MAGIC_NUMBER 0xABCD1235  // Número que identifica se há dados válidos
//deine LOG_INTERVAL_MS     10      // 100 Hz => 10ms
#define BUFFER_FLUSH_COUNT 20  // Flush every 20 writes
#define ESPNOW_WIFI_CHANNEL 1

// ENUMS

struct DataPacket {
  unsigned long timeMillis;
};

typedef enum{
  STATE_REPAIR,
  STATE_IDLE,
  STATE_TAKEOFF,
  STATE_ASCENDING,
  STATE_DESCENDING,
  STATE_LANDING,
  STATE_LANDED
} state_t;











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
  uint32_t timestamp; 
  uint8_t ID;  // in milliseconds
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
  float latitude;    // in degrees * 10^6
  float longitude;   // in degrees * 10^6
  float altitudeGPS;
} __attribute__((packed)) SensorData_t;




typedef struct {
  
  uint32_t NumeroPacote;
  uint32_t timestamp; //
  bool isSensorCalibrated;
  int timeOffset; // Offset de tempo em milissegundos
  bool isBMP388Calibrated;
  float BMPTemperatureOffset; //
  float BMPAltitudeOffset; //
  float BMPPressureOffset; //
  bool isIMUCalibrated;
  bool isAccCalibrated;  
  float AccXOffset; //
  float AccYOffset; //
  float AccZOffset; //
  bool isGyroCalibrated;
  float GyroXOffset; //
  float GyroYOffset; //
  float GyroZOffset; //
  bool isMagCalibrated;
  float MagXOffset; //
  float MagYOffset; //
  float MagZOffset; //
  
} __attribute__((packed)) Calibration_t;




//Definição de queues

static QueueHandle_t sensorQueue;
static QueueHandle_t sensorEspnowQueue;
// put function declarations here:
// -------------------------------------------------
// SPIFFS File Globals
// -------------------------------------------------
File logFile;
bool fileOpen = false;
int logCounter = 0;  // used to track how many lines since last flush
 
int NumeroPacoteSensor = 0;




Calibration_t calib1;
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  DataPacket packet;
  
  // Copiar os dados recebidos para a estrutura
  memcpy(&packet, incomingData, sizeof(packet));
 calib1.timeOffset=millis()-packet.timeMillis; // Atualiza o offset de tempo
 

}
void calibrarGiroscopio() {
  const int numSamples = 1000;
  float sumX = 0.0, sumY = 0.0, sumZ = 0.0;

  Serial.println("Calibrando giroscópio... Mantenha o sensor imóvel.");

  for (int i = 0; i < numSamples; i++) {
    xyzFloat gyroValues;
    myIMU.readSensor();
    myIMU.getGValues(&gyroValues);
    sumX += gyroValues.x;
    sumY += gyroValues.y;
    sumZ += gyroValues.z;
    delay(5);
  }

  calib1.GyroXOffset = sumX / numSamples;
  calib1.GyroYOffset = sumY / numSamples;
  calib1.GyroZOffset = sumZ / numSamples;
  calib1.isGyroCalibrated = true;
  Serial.println("Calibração concluída!");
  Serial.print("Offsets - X: ");
  Serial.print(calib1.GyroXOffset);
  Serial.print(" Y: ");
  Serial.print(calib1.GyroYOffset);
  Serial.print(" Z: ");
  Serial.println(calib1.GyroZOffset);
  delay(5000);
  
}
// Função para calibrar o magnetômetro (Hard Iron Correction)
void calibrarMagnetometro() {
  float magMaxX = -1000, magMaxY = -1000, magMaxZ = -1000;
  float magMinX = 1000, magMinY = 1000, magMinZ = 1000;

  Serial.println("Calibrando magnetômetro...");
  Serial.println("Mova o sensor lentamente em todas as direções por 30 segundos.");

  unsigned long startTime = millis();
  while (millis() - startTime < 30000) {  // Coleta dados por 30 segundos
    xyzFloat magValues;
    myIMU.readSensor();
    myIMU.getGValues(&magValues);

    if (magValues.x > magMaxX) magMaxX = magValues.x;
    if (magValues.x < magMinX) magMinX = magValues.x;
    if (magValues.y > magMaxY) magMaxY = magValues.y;
    if (magValues.y < magMinY) magMinY = magValues.y;
    if (magValues.z > magMaxZ) magMaxZ = magValues.z;
    if (magValues.z < magMinZ) magMinZ = magValues.z;

    delay(100);
  }

  // Calcula os offsets como a média dos valores máximo e mínimo (Hard Iron Correction)
  calib1.MagXOffset = (magMaxX + magMinX) / 2.0;
  calib1.MagYOffset = (magMaxY + magMinY) / 2.0;
  calib1.MagZOffset = (magMaxZ + magMinZ) / 2.0;
  calib1.isMagCalibrated = true;
  Serial.println("Calibração concluída!");
  Serial.print("Offsets - X: ");
  Serial.print(calib1.MagXOffset);
  Serial.print(" Y: ");
  Serial.print(calib1.MagYOffset);
  Serial.print(" Z: ");
  Serial.println(calib1.MagZOffset);
  delay(5000);
}
void calibrarAcelerometro() {
  const int numSamples = 1000;
  float sumX = 0.0, sumY = 0.0, sumZ = 0.0;

  Serial.println("Calibrando acelerômetro... Mantenha o sensor imóvel e nivelado.");

  for (int i = 0; i < numSamples; i++) {
    xyzFloat accValues;
    myIMU.readSensor();
    myIMU.getGValues(&accValues);

    sumX += accValues.x;
    sumY += accValues.y;
    sumZ += accValues.z;
    delay(5);
  }

  calib1.AccXOffset = sumX / numSamples;
  calib1.AccYOffset = sumY / numSamples;
  calib1.AccZOffset = (sumZ / numSamples) - 1.0;  // Subtrai 1g devido à gravidade
  calib1.isAccCalibrated = true;
  Serial.println("Calibração concluída!");
  Serial.print("Offsets - X: ");
  Serial.print(calib1.AccXOffset);
  Serial.print(" Y: ");
  Serial.print(calib1.AccYOffset);
  Serial.print(" Z: ");
  Serial.println(calib1.AccZOffset);

  delay(5000);
}

void calibraBMP388() {

  float tempSum = 0, pressSum = 0;
  const int numSamples = 100;

  for (int i = 0; i < numSamples; i++) {
    if (bmp.performReading()) {
      tempSum += bmp.temperature;
      pressSum += bmp.pressure;
    } else {
      Serial.println("Falha na leitura do BMP388!");
    }
    delay(10);  // Ajuste o delay conforme sua necessidade
  }

  float avgTemp = tempSum / numSamples;
  float avgPress = pressSum / numSamples;

  Serial.print("Temperatura média: ");
  Serial.print(avgTemp);
  Serial.println(" °C");
  Serial.print("Pressão média: ");
  Serial.print(avgPress);
  Serial.println(" Pa");
calib1.BMPTemperatureOffset=-avgTemp;
calib1.BMPAltitudeOffset=bmp.readAltitude(1013.25)-avgPress;
calib1.BMPPressureOffset=1013.25-avgPress;
  delay(5000);
}
/**
 * @brief Inicializa os sensores BMP388 e ICM-20948.
 *
 * Inicializa o BMP388 e o ICM-20948, verificando se a inicialização
 * foi bem sucedida. Em caso de erro, imprime uma mensagem de erro e
 * trava o sistema.
 */
void setupSensors() {


  //----------------------------------------------------------------
  // Inicializa FRAM
  // ---------------------

  if (!fram.begin(0x50)) {
    Serial.println("FRAM não encontrada!");
 
   
  }
  Serial.println("FRAM detectada!");
  // ---------------------
  // Inicializa BMP388

  // ---------------------
  if (!bmp.begin_SPI(BMP_CS)) {  // Exemplo com BMP388 SPI no pino 2
    Serial.println("Falha ao inicializar BMP388");
    
  }

  Serial.println("BMP388 inicializado!");
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_2X);

  //bmp.setOutputDataRate(BMP3_ODR_200_HZ);
  //bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);


  // ---------------------
  // Inicializa ICM-20948 (SPI)
  if (!myIMU.init()) {
    Serial.println("ICM20948 does not respond");
  } else {
    Serial.println("ICM20948 is connected");
  }

  if (!myIMU.initMagnetometer()) {
    Serial.println("Magnetometer does not respond");
  } else {
    Serial.println("Magnetometer is connected");
  }

  // Configura a comunicação serial do GPS
  GPSSerial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);  // Ajuste o baud rate conforme necessário
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
  // put your main code here, to run repeatedly:
  // Handle serial input


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
void mostra(SensorData_t msg) {
  Serial.print("Pacote: ");
  Serial.print(msg.NumeroPacote);
  Serial.print(" | Timestamp: ");
  Serial.print(msg.timestamp);
  Serial.print(" | Temperatura: ");
  Serial.print(msg.temperature, 2);
  Serial.print(" °C | Pressão: ");
  Serial.print(msg.pressure, 2);
  Serial.print(" Pa | Altitude: ");
  Serial.print(msg.altitude, 2);
  Serial.print(" Pa | Acelerômetro: (");
  Serial.print(msg.AccX, 2);
  Serial.print(", ");
  Serial.print(msg.AccY, 2);
  Serial.print(", ");
  Serial.print(msg.AccZ, 2);
  Serial.println(")");

  Serial.print(" | Giroscópio: (");
  Serial.print(msg.GyroX, 2);
  Serial.print(", ");
  Serial.print(msg.GyroY, 2);
  Serial.print(", ");
  Serial.print(msg.GyroZ, 2);
  Serial.println(")");
  Serial.print(" | Magnetômetro: (");
  Serial.print(msg.MagX, 2);
  Serial.print(", ");
  Serial.print(msg.MagY, 2);
  Serial.print(", ");
  Serial.print(msg.MagZ, 2);
  Serial.println(")");
  Serial.print(" | Latitude: ");
  Serial.print(msg.latitude, 6);
  Serial.print(" | Longitude: ");
  Serial.print(msg.longitude, 6);
  Serial.print(" | Altitude GPS: ");
  Serial.print(msg.altitudeGPS, 2);
  Serial.println(" m");
}


void readBMP388(void *pvParameters) {
  (void)pvParameters;  // Evita aviso de variável não utilizada

  for (;;) {
    // Faz a leitura de pressão e temperatura
    /// Simulate sensor data
   SensorData_t msg;          //Nova mensagem
 

    if (!bmp.performReading()) {
      Serial.println("Falha ao ler BMP388!");
    } else {

      float temperatura = bmp.temperature;         // em °C
      float pressao = bmp.pressure;                // em Pa
      float altitude = bmp.readAltitude(1013.25);  // Ajuste a pressão ao nível do mar (hPa ou mbar)
      msg.timestamp = millis() - calib1.timeOffset;
      msg.ID = ID_SENSOR;
      msg.temperature = temperatura;
      msg.pressure = pressao;
      msg. altitude = altitude;
      msg.NumeroPacote = NumeroPacoteSensor++;
    

      //Serial.print("Tempo BMP :");
      //Serial.println(micros());
      //le imu
      xyzFloat gValue;
      xyzFloat gyr;
      xyzFloat magValue;

      myIMU.readSensor();
      myIMU.getGValues(&gValue);
      myIMU.getGyrValues(&gyr);
      myIMU.getMagValues(&magValue);
      float temp = myIMU.getTemperature();
      float resultantG = myIMU.getResultantG(&gValue);
      msg.AccX = gValue.z + calib1.AccXOffset;
      msg.AccY = gValue.y + calib1.AccYOffset;

      msg.AccZ = gValue.x + calib1.AccZOffset;
      msg.GyroX = gyr.z + calib1.GyroXOffset;
      msg.GyroY = gyr.y + calib1.GyroYOffset;
      msg.GyroZ = gyr.x + calib1.GyroZOffset;
      msg.MagX = magValue.z +calib1.MagXOffset;
      msg.MagY = magValue.y +calib1.MagYOffset;
      msg.MagZ = magValue.x + calib1.MagZOffset;
      while (GPSSerial.available() > 0) {
        if (gps.encode(GPSSerial.read())) {
          if (gps.location.isUpdated()) {
            msg.latitude = gps.location.lat() ;  // em graus * 10^6
            msg.longitude = gps.location.lng() ; // em graus * 10^6
            msg.altitudeGPS=gps.altitude.meters();        // em metros
      
          }
        }}

    mostra(msg);
        esp_err_t result1 = esp_now_send(0, (uint8_t *)&msg, sizeof(msg));



    
    }

    // Atraso de 1 segundo até a próxima leitura
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
// -------------------------------------------------
// Consumer Task
//    - Blocks on the queue waiting for sensor data
//    - Writes each incoming item to the file
//    - Flushes periodically
// -------------------------------------------------
void logSensorTask(void *pvParameters) {
  (void)pvParameters;  // unused parameter
  ;
  SensorData_t receivedData;  //Nova mensagem

  for (;;) {
    // 1) Wait for data from the queue (blocking)
    if (xQueueReceive(sensorQueue, &receivedData, 0) == pdTRUE) {
      // 2) Write data to file
      if (fileOpen) {

        // Build a line: "timestamp,temperature,pressure"
        String logLine = String(receivedData. NumeroPacote) + "," + String(receivedData.temperature, 2) + "," + String(receivedData.pressure, 2) + "," + String(receivedData.altitude, 2) + "," + String(receivedData. AccX, 2) + "," + String(receivedData. AccY, 2) + "," + String(receivedData. AccZ, 2) + "," + String(receivedData. GyroX, 2) + "," + String(receivedData. GyroY, 2) + "," + String(receivedData. GyroZ, 2) + "," + String(receivedData. MagX, 2) + "," + String(receivedData. MagY, 2) + 
        "," + String(receivedData. MagZ, 2) + "," +  String(receivedData. MagZ, 2) + "," + String(receivedData.latitude, 2) + "," + String(receivedData.longitude, 2) + "," + String(receivedData.altitude, 2) + "," + "/n";
                          
Serial.println(logLine);
        logFile.print(logLine);

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

// Função para calcular o CRC16 (CRC16-CCITT) com polinômio 0x1021 e valor inicial 0xFFFF
uint16_t calcularCRC16(const uint8_t *dados, size_t tamanho) {
  uint16_t crc = 0xFFFF;  // Valor inicial
  for (size_t i = 0; i < tamanho; i++) {
    crc ^= (uint16_t)dados[i] << 8;
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x8000)
        crc = (crc << 1) ^ 0x1021;
      else
        crc <<= 1;
    }
  }
  return crc;
}




// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //Serial.print(" send status:\t");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

/**
 * Creates FreeRTOS tasks for reading sensor data.
 * 
 * This function initializes two tasks:
 * 1. ReadBMP388: A task for reading data from the BMP388 sensor.
 * 2. ReadICM20948: A task for reading data from the ICM20948 sensor.
 * 
 * Each task is created with a stack size of 2048 bytes, a priority of 1, 
 * and no parameters or task handles.
 */

void createTasks() {
  xTaskCreatePinnedToCore(readBMP388, "ReadBMP388", 3048, NULL, 1, &Sensor100HZTaskHandle, 1);
  //xTaskCreate(readICM20948, "ReadICM20948", 3048, NULL, 1, NULL);
 //TaskCreatePinnedToCore(readGPS, "ReadGPS", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(logSensorTask, "LogSensorTask", 4096, NULL, 1, &logSensorTaskkHandle, 1);

  //askCreatePinnedToCore(EspNowSensorTask, "EspNowSenderTask", 4096, NULL, 2, NULL, 0);
}

// Função para salvar os valores na FRAM
void saveCalibrationToFRAM(float *data) {
  uint16_t addr = FRAM_START_ADDR;
  uint32_t magic = MAGIC_NUMBER; 
    // Grava o Magic Number
    fram.write(addr, (uint8_t *)&magic, sizeof(MAGIC_NUMBER));
    addr += sizeof(MAGIC_NUMBER);
  for (int i = 0; i < 9; i++) {   
      fram.write(addr, (uint8_t *)&data[i], sizeof(float));     
      addr += sizeof(float);
  }
  Serial.println("Dados de calibração salvos na FRAM!");
}
// Função para recuperar os valores da FRAM
void loadCalibrationFromFRAM(float *data) {
  uint16_t addr = FRAM_START_ADDR;
  for (int i = 0; i < 9; i++) {
      fram.read(addr, (uint8_t *)&data[i], sizeof(float));
      addr += sizeof(float);
  }
}

bool isCalibrationFromFRAM(float *data) {
  uint16_t addr = FRAM_START_ADDR;
  uint32_t magic;

  // Ler o Magic Number da FRAM
  fram.read(addr, (uint8_t *)&magic, sizeof(magic));
  addr += sizeof(magic);

  // Se o Magic Number não for igual ao esperado, os dados são inválidos
  if (magic != MAGIC_NUMBER) {
      return false;
  }
return true;
}
// Função para salvar os valores na FRAM
// -------------------------------------------------
// Close File
// -------------------------------------------------

/**
 * Setup function
 * 
 * This function is called once at the beginning of the program and is used to
 * initialize the ESP32, the sensors, and the FreeRTOS tasks.
 * 
 * The following steps are performed in this function:
 * 1. Initialize SPIFFS 
 * 2. Open the file in append mode
 * 3. Initialize the ESP-NOW
 * 4. Create the queue for sensor data
 * 5. Initialize the sensors
 * 6. Create the FreeRTOS tasks
 */
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(2000);
  //initializeFileSystem();
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

  //Espnow init
  WiFi.mode(WIFI_STA);                // Modo Station para ESP-NOW
  WiFi.channel(ESPNOW_WIFI_CHANNEL);  // Canal de comunicação
  if (esp_now_init() != ESP_OK) {
    Serial.println("Erro ao iniciar ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  // Register first peer
  memcpy(peerInfo.peer_addr, ESPXIAO3, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }



  // Register first peer
  memcpy(peerInfo.peer_addr, ESPXIAO4, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }


  // 3) Create the queue for sensor data
  //    Max 64 items in the queue, each of size SensorData_t
  sensorQueue = xQueueCreate(5, sizeof(SensorData_t));
  if (sensorQueue == NULL) {
    Serial.println("Failed to create sensor queue!");
    // Without a queue, we cannot proceed properly
  }

  // Inicializar sensores
  setupSensors();

  float recoveredData[9];

  if (isCalibrationFromFRAM(recoveredData)){
    loadCalibrationFromFRAM(recoveredData);
    Serial.println("Dados de calibração recuperados da FRAM!");
    calib1.AccXOffset = recoveredData[0];
    calib1.AccYOffset = recoveredData[1]; 
    calib1.AccZOffset = recoveredData[2];
    calib1.GyroXOffset = recoveredData[3];
    calib1.GyroYOffset = recoveredData[4];
    calib1.GyroZOffset = recoveredData[5];
    calib1.MagXOffset = recoveredData[6];
    calib1.MagYOffset = recoveredData[7];
    calib1.MagZOffset = recoveredData[8];
    Serial.print("Offsets - X: ");
    Serial.print(calib1.AccXOffset);
    Serial.print(" Y: ");
    Serial.print(calib1.AccYOffset);
    Serial.print(" Z: ");
    Serial.print(calib1.AccZOffset);
    Serial.print(" X: ");
    Serial.print(calib1.GyroXOffset);
    Serial.print(" Y: ");
    Serial.print(calib1.GyroYOffset);
    Serial.print(" Z: ");
    Serial.print(calib1.GyroZOffset);
    Serial.print(" X: ");
    Serial.print(calib1.MagXOffset);
    Serial.print(" Y: ");
    Serial.print(calib1.MagYOffset);
    Serial.print(" Z: ");
    Serial.println(calib1.MagZOffset);




  }else{

  calibraBMP388();
  calibrarAcelerometro();
 calibrarGiroscopio();
  calibrarMagnetometro();
  Serial.println("Is Acc Calibrated: " + String(calib1.isAccCalibrated));
  Serial.println("Is Gyro Calibrated: " + String(calib1.isGyroCalibrated));
  Serial.println("Is Mag Calibrated: " + String(calib1.isMagCalibrated));
 
  if ((calib1.isAccCalibrated==true && calib1.isGyroCalibrated==true) && calib1.isMagCalibrated==true) {
    
    calib1.isIMUCalibrated = true;
    //envia por EspNow
    SensorData_t calibData;
    //calibData.data.calibrado = calib1;


    //esp_now_send(ESPXIAO4, (uint8_t *)&calibData, sizeof(message_t));
    Serial.println("Calibração concluída!");
    float calibData1[9]={calib1.AccXOffset,calib1.AccYOffset,calib1.AccZOffset,calib1.GyroXOffset,calib1.GyroYOffset,calib1.GyroZOffset,calib1.MagXOffset,calib1.MagYOffset,calib1.MagZOffset};
    saveCalibrationToFRAM(calibData1);
    delay(2000);


  } else {
    Serial.println("Calibração incompleta!");
   // calib1.isICMCalibrated = false;
  }}
  
  createTasks();
}
