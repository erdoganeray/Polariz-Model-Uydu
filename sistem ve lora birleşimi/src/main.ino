/*
 * ESP32 Entegre Sensör-LoRa Sistemi
 * 
 * Bu sistem sensor_system.ino'daki sensör veri toplama yeteneklerini
 * lora_1.ino'daki LoRa iletişim protokolü ile birleştirir.
 * 
 * Sensörler:
 * - MPU6050: I2C üzerinden pitch, roll, yaw (GPIO21-SDA, GPIO22-SCL)
 * - BMP280: I2C üzerinden basınç ve sıcaklık (GPIO21-SDA, GPIO22-SCL)
 * - GY-GPS6MV2: UART1 üzerinden GPS verileri (GPIO14-RX, GPIO15-TX)
 * - DS3231 RTC: I2C üzerinden zaman bilgisi (GPIO21-SDA, GPIO22-SCL)
 * - SD Kart: SPI üzerinden veri kaydetme (GPIO18-SCK, GPIO19-MISO, GPIO23-MOSI, GPIO5-CS)
 * - ADC: Gerilim ölçümü
 * - SG90 Servo Motorlar: PWM ile RHRH pozisyon kontrolü (GPIO33, GPIO25)
 * - DC Motor: Ayrılma/birleşme sistemi (GPIO26-ayrılma, GPIO32-birleşme)
 * 
 * LoRa E22 Modülü:
 * - UART2 üzerinden iletişim (GPIO16-TX, GPIO17-RX)
 * - Kontrol pinleri: M0=19, M1=18, AUX=4
 * - Kanal 10, Adres 0x00:0x0A
 * 
 * Komut formatları USB Serial üzerinden:
 * - "GET_DATA" -> Manuel veri toplama
 * - "RESET_REF" -> Referans basıncını ve açıları sıfırla (mevcut yükseklik ve açılar 0 yapılır)
 * - "RHRH:2M6K" -> Servo pozisyon kontrol (2sn M=0°, 6sn K=100°)
 * - "AYRIL" -> DC Motor ayrılma yönünde çalıştır
 * - "BIRLES" -> DC Motor birleşme yönünde çalıştır
 * - "STATUS" -> Sistem durumu
 * - "SERVO_TEST" -> Servo ve DC motor test hareketi
 * - "LORA_TEST" -> LoRa bağlantı testi
 */

// Kütüphaneler - Sensörler
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <TinyGPS++.h>
#include <RTClib.h>
#include <SD.h>
#include <FS.h>

// LoRa kütüphaneleri
#include <LoRa_E22.h>
#include "../binary_protocol.h"

// GPS UART1 Pin tanımlamaları
#define GPS_RX 14
#define GPS_TX 15
#define GPS_BAUD 9600
HardwareSerial gpsSerial(1);

// LoRa E22 Pin tanımlamaları
#define LORA_M0 13
#define LORA_M1 12
#define LORA_AUX 4
#define LORA_TX 16
#define LORA_RX 17

#define LORA_CHANNEL 10
#define LORA_ADDR_H 0x00
#define LORA_ADDR_L 0x0A

// SD Kart SPI pin tanımlamaları
#define SD_SCK 18
#define SD_MISO 19
#define SD_MOSI 23
#define SD_CS 5

// ADC pin tanımlaması
#define pil_gerilimi_PIN A0

// Servo motor pin tanımlamaları (RHRH sistemi için)
#define SERVO1_PIN 33
#define SERVO2_PIN 25

// DC Motor pin tanımlamaları (ayrılma/birleşme sistemi için)
#define DC_MOTOR_AYRILMA_PIN 26  // Ayrılma için ileri yön
#define DC_MOTOR_BIRLESME_PIN 32 // Birleşme için geri yön

// Acil ayrılma butonu pin tanımlaması
#define EMERGENCY_BUTTON_PIN 35  // GPIO35 - Acil ayrılma butonu

// Acil uydu status butonu pin tanımlaması  
#define EMERGENCY_UYDU_STATUS_PIN 34  // GPIO34 - Acil uydu status butonu

// Buzzer pin tanımlaması
#define BUZZER_PIN 27  // GPIO27 - Uydu Status 5 için buzzer

// Servo PWM kanalları
#define SERVO1_CHANNEL 0
#define SERVO2_CHANNEL 1
#define SERVO_FREQ 50
#define SERVO_RESOLUTION 16

// Servo pozisyon açı değerleri
// Yeni RHRH servo pozisyonları
#define ANGLE_A 40   // A - default pozisyon (servo1: 40°, servo2: 40°)
#define ANGLE_M 80   // M - servo1: 80°, servo2: 80°
#define ANGLE_F 0    // F - servo1: 0°, servo2: 0°
#define ANGLE_N 120  // N - servo1: 120°, servo2: 120°
#define ANGLE_R 40   // R - servo1: 40°, servo2: 80° (servo1 için)
#define ANGLE_R2 80  // R - servo2: 80° (servo2 için)
#define ANGLE_G 0    // G - servo1: 0°, servo2: 40° (servo1 için)
#define ANGLE_G2 40  // G - servo2: 40° (servo2 için)
#define ANGLE_B 40   // B - servo1: 40°, servo2: 120° (servo1 için)
#define ANGLE_B2 120 // B - servo2: 120° (servo2 için)
#define ANGLE_P 80   // P - servo1: 80°, servo2: 120° (servo1 için)
#define ANGLE_P2 120 // P - servo2: 120° (servo2 için)
#define ANGLE_Y 0    // Y - servo1: 0°, servo2: 80° (servo1 için)
#define ANGLE_Y2 80  // Y - servo2: 80° (servo2 için)
#define ANGLE_C 0    // C - servo1: 0°, servo2: 120° (servo1 için)
#define ANGLE_C2 120 // C - servo2: 120° (servo2 için)

// Sensör nesneleri
Adafruit_MPU6050 mpu;
Adafruit_BMP280 bmp;
TinyGPSPlus gps;
RTC_DS3231 rtc;

// LoRa nesnesi
LoRa_E22 E22(&Serial2, LORA_AUX, LORA_M0, LORA_M1);

// Servo kontrol değişkenleri - Yeni RHRH Sistemi
String current_rhrh = "0A0A";  // Default: 0A0A (servo1: 40°, servo2: 40°)
unsigned long rhrh_start_time = 0;
unsigned long rhrh_phase2_start_time = 0;  // Faz 2 için ayrı başlangıç zamanı
unsigned long rhrh_total_duration = 0;
unsigned long rhrh_phase1_duration = 0;
unsigned long rhrh_phase2_duration = 0;
bool rhrh_active = false;
bool rhrh_phase1_active = false;
bool rhrh_phase2_active = false;
bool servo3_separated = false;

// RHRH fazları için
char rhrh_phase1_pos = 'A';  // İlk harfin pozisyonu
char rhrh_phase2_pos = 'A';  // İkinci harfin pozisyonu

// LoRa kontrol değişkenleri
uint16_t paket_sayisi_lora2 = 1;
uint32_t current_rhrh_encoded = encodeRHRH('0', 'A', '0', 'A');  // Büyük harflerle

// LoRa'dan gelen basınç verisi (Pressure Container)
float lora_basinc2 = 0.0;
bool lora_basinc2_received = false;

// LoRa'dan gelen IoT verileri (L4 ve L5 Data)
int16_t lora_iot_s1_data = 0;
int16_t lora_iot_s2_data = 0;
bool lora_iot_s1_received = false;
bool lora_iot_s2_received = false;

// Sensör durumları
bool bmp_status = false;
bool mpu_status = false;
bool rtc_status = false;
bool gps_status = false;
bool sd_status = false;
bool adc_status = false;
bool lora_status = false;

// Paket numarası
uint16_t packet_counter = 1;

// İniş hızı hesaplama değişkenleri
float previous_altitude = 0.0;
unsigned long previous_time = 0;
bool first_altitude_reading = true;

// Referans basınç değişkeni (sistem başlangıcındaki basınç = 0 metre yükseklik)
float reference_pressure = 101325.0; // Standart deniz seviyesi basıncı (başlangıç değeri)
bool reference_pressure_set = false;

// Referans açısal değişkenler (sistem başlangıcındaki açılar = 0 referans)
float reference_pitch = 0.0;
float reference_roll = 0.0;
float reference_yaw = 0.0;
bool reference_angles_set = false;

// Uydu status takibi için değişkenler
float previous_altitude_for_status = 0.0;
bool first_status_reading = true;
uint8_t previous_uydu_status = 255; // Önceki uydu status'unu takip etmek için (-1 başlangıç değeri)

// Uydu status algoritması değişkenleri
bool start_button = false;
bool ayrilma_status = false;
bool ayrilma_basla = false;
bool emergency_uydu_status_triggered = false;  // GPIO34 butonu tetiklenme durumu (bir kez tetiklendikten sonra değişmez)
uint8_t manual_ayrilma = 0;  // 0: default, 1: ayrılma, 2: birleşme
float ayrilma_toleransi = 50.0;
float tolerans0 = 2.0;
float tolerans1 = 2.0;
float tolerans2 = 2.0;
float tolerans3 = 2.0;
float tolerans4 = 2.0;  // tolerans4 eksikti algoritma dosyasında
float tolerans5 = 2.0;  // tolerans5 eksikti algoritma dosyasında

// Buzzer kontrol değişkenleri (uydu_status 5 için)
unsigned long buzzer_last_toggle = 0;
unsigned long buzzer_interval = 500;  // 500ms aralıklarla on/off
bool buzzer_state = false;
uint8_t current_uydu_status = 0;  // Mevcut uydu status'unu takip etmek için

// DC Motor kontrol değişkenleri (millis tabanlı)
unsigned long dc_motor_start_time = 0;
bool dc_motor_ayrilma_active = false;
bool dc_motor_birlesme_active = false;
const unsigned long DC_MOTOR_AYRILMA_DURATION = 10000;  // 10 saniye
const unsigned long DC_MOTOR_BIRLESME_DURATION = 1000;   // 1 saniye

// Fonksiyon prototipleri
void initSensors();
void initServos();
void initSDCard();
void initLoRa();
void initDCMotor();
void dcMotorAyrilma();
void dcMotorBirlesme();
void dcMotorStop();
void updateDCMotorControl();
void collectSensorData(TelemetryPacket* data);
void sendTelemetryToLora2();
void processSerialCommands();
void updateServoControl();
void processRHRHCommand(String command);
void setServoPosition(int channel, int angle);
int angleToDutyCycle(int angle);
int getServoPosition(char position);
int getServoAngle(char position, int servo_number);
void updateSensorStatus();
void saveToSD(TelemetryPacket data);
bool appendFile(fs::FS &fs, const char *path, const char *message);
void writeFile(fs::FS &fs, const char *path, const char *message);
void setRTCTime(String command);
float calculateAltitude(float P, float T_celsius);
void printLoRaModuleInfo();
void printLoRaConfig();
bool waitForLoRaMessage(unsigned long timeout_ms);
void loadLastStateFromSD();  // SD karttan en son durum bilgisini yükle
void updateBuzzerControl();  // Buzzer kontrolü (uydu_status 5 için)

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("ESP32 Entegre Sensör-LoRa Sistemi Başlatılıyor...");
  Serial.println("==============================================");
  
  // GPS UART başlatma
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  
  // I2C başlatma
  Wire.begin();
  
  // Acil ayrılma butonu GPIO35'ü input olarak ayarla (Pull-down ile)
  pinMode(EMERGENCY_BUTTON_PIN, INPUT);
  
  // Acil uydu status butonu GPIO34'ü input olarak ayarla (Pull-down ile)
  pinMode(EMERGENCY_UYDU_STATUS_PIN, INPUT);
  
  // Buzzer GPIO27'yi output olarak ayarla
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);  // Başlangıçta kapalı
  
  // Sistemleri başlatma
  initSensors();
  initServos();
  initDCMotor();
  initSDCard();
  
  // SD karttan en son durum bilgilerini yükle
  if (sd_status) {
    loadLastStateFromSD();
  }
  
  initLoRa();

  configureLoRa();
  
  Serial.println("==============================================");
  Serial.println("Sistem hazır! Otomatik veri gönderimi ve LoRa iletişimi (1Hz)");
  Serial.println("Komutlar: GET_DATA, RESET_REF, RHRH:2M6K, AYRIL, BIRLES, STATUS, SERVO_TEST, LORA_TEST");
  Serial.println("==============================================");
}

void loop() {
  // ACİL DURUM: GPIO35 butonu kontrolü (loop'un en başında - en yüksek öncelik)
  if (digitalRead(EMERGENCY_BUTTON_PIN) == HIGH) {
    ayrilma_status = true;
    Serial.println("*** ACİL DURUM: GPIO35 Butonu Aktif! Ayrılma Status TRUE yapıldı! ***");
  }
  
  // ACİL DURUM: GPIO34 butonu kontrolü - Uydu Status 0'a zorla (sadece bir kez tetiklenir)
  if (!emergency_uydu_status_triggered && digitalRead(EMERGENCY_UYDU_STATUS_PIN) == HIGH) {
    emergency_uydu_status_triggered = true;  // Tetiklenme flag'ini set et
    Serial.println("*** ACİL DURUM: GPIO34 Butonu Aktif! Uydu Status bir sonraki döngüde 0'a zorlanacak! (Tek seferlik) ***");
  }
  
  // GPS verilerini sürekli güncelle
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }
  
  // Serial komutları işle
  processSerialCommands();
  
  // Servo kontrollerini güncelle
  updateServoControl();
  
  // DC Motor kontrollerini güncelle
  updateDCMotorControl();
  
  // Buzzer kontrolünü güncelle (uydu_status 5 için)
  updateBuzzerControl();
  
  // Sensör durumlarını periyodik kontrol et
  updateSensorStatus();
  
  // Her saniye otomatik LoRa telemetry gönder
  static unsigned long lastLoRaTransmission = 0;
  if (millis() - lastLoRaTransmission >= 950) {
    lastLoRaTransmission = millis();
    
    Serial.println("=== LORA COMM CYCLE BAŞLADI ===");
    sendTelemetryToLora2();
    
    // LoRa mesajları dinle
    unsigned long cycle_start = millis();
    while (millis() - cycle_start < 900) { // 900ms dinle
      if (E22.available() > 1) {
        waitForLoRaMessage(100);
      }
      delay(10);
    }
    Serial.println("=== LORA COMM CYCLE TAMAMLANDI ===");
  }
  
  delay(50);
}

void initSensors() {
  Serial.println("Sensörler başlatılıyor...");
  
  // MPU6050 başlatma
  Serial.print("MPU6050... ");
  if (!mpu.begin(0x69)) {
    Serial.println("HATA!");
    mpu_status = false;
  } else {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    delay(100);
    
    sensors_event_t a, g, temp;
    if (mpu.getEvent(&a, &g, &temp)) {
      mpu_status = true;
      
      // Sistem başlangıcındaki açıları referans açılar olarak ayarla (0 derece referans)
      delay(100); // Sensörün kararlı hale gelmesi için kısa bekleme
      
      // Referans açıları belirlemek için 5 okuma yapıp ortalamasını al
      float pitch_sum = 0, roll_sum = 0, yaw_sum = 0;
      int valid_readings = 0;
      
      for(int i = 0; i < 5; i++) {
        sensors_event_t ref_a, ref_g, ref_temp;
        if (mpu.getEvent(&ref_a, &ref_g, &ref_temp)) {
          pitch_sum += ref_g.gyro.y * 180.0 / PI;
          roll_sum += ref_g.gyro.x * 180.0 / PI;
          yaw_sum += ref_g.gyro.z * 180.0 / PI;
          valid_readings++;
        }
        delay(20);
      }
      
      if (valid_readings >= 3) { // En az 3 geçerli okuma varsa
        reference_pitch = pitch_sum / valid_readings;
        reference_roll = roll_sum / valid_readings;
        reference_yaw = yaw_sum / valid_readings;
        reference_angles_set = true;
        Serial.printf("OK (Referans açılar: P=%.1f°, R=%.1f°, Y=%.1f° - Bu açılar 0° kabul edildi)\n", 
                      reference_pitch, reference_roll, reference_yaw);
      } else {
        reference_pitch = 0.0;
        reference_roll = 0.0;
        reference_yaw = 0.0;
        reference_angles_set = false;
        Serial.println("OK (Uyarı: Referans açılar okunamadı, 0° değeri kullanılıyor)");
      }
    } else {
      mpu_status = false;
      Serial.println("Test HATASI");
    }
  }
  
  // BMP280 başlatma
  Serial.print("BMP280... ");
  if (!bmp.begin(0x76)) {
    Serial.println("HATA!");
    bmp_status = false;
  } else {
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_500);
    bmp_status = true;
    
    // Sistem başlangıcındaki basıncı referans basınç olarak ayarla (0 metre yükseklik)
    delay(100); // Sensörün kararlı hale gelmesi için kısa bekleme
    reference_pressure = bmp.readPressure();
    if (!isnan(reference_pressure) && reference_pressure > 50000.0) { // Geçerli basınç kontrolü
      reference_pressure_set = true;
      Serial.printf("OK (Referans basınç: %.1f Pa - Bu seviye 0m kabul edildi)\n", reference_pressure);
    } else {
      reference_pressure = 101325.0; // Hata durumunda standart basınç kullan
      reference_pressure_set = false;
      Serial.println("OK (Uyarı: Referans basınç okunamadı, standart değer kullanılıyor)");
    }
  }
  
  // RTC başlatma
  Serial.print("DS3231 RTC... ");
  if (!rtc.begin()) {
    Serial.println("HATA!");
    rtc_status = false;
  } else {
    rtc_status = true;
    DateTime now = rtc.now();
    if (now.year() < 2020 || now.year() > 2035) {
      Serial.printf("OK (Zaman ayarlanması gerekiyor - Mevcut: %04d/%02d/%02d %02d:%02d:%02d)\n", 
                    now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
      Serial.println("Komut: SET:2025,8,5,14,30,0");
    } else {
      Serial.printf("OK (Mevcut zaman: %04d/%02d/%02d %02d:%02d:%02d)\n", 
                    now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
    }
  }
  
  Serial.println("GPS modülü başlatıldı (UART1)");
}

void initServos() {
  Serial.println("Servo motorlar başlatılıyor...");
  
  ledcSetup(SERVO1_CHANNEL, SERVO_FREQ, SERVO_RESOLUTION);
  ledcSetup(SERVO2_CHANNEL, SERVO_FREQ, SERVO_RESOLUTION);
  
  ledcAttachPin(SERVO1_PIN, SERVO1_CHANNEL);
  ledcAttachPin(SERVO2_PIN, SERVO2_CHANNEL);
  
  delay(500);
  
  setServoPosition(SERVO1_CHANNEL, ANGLE_A);
  setServoPosition(SERVO2_CHANNEL, ANGLE_A);
  
  delay(1000);
  
  Serial.println("Servo motorlar OK");
  Serial.printf("Servo1: GPIO%d, Servo2: GPIO%d\n", SERVO1_PIN, SERVO2_PIN);
}

void initSDCard() {
  Serial.print("SD Kart... ");
  
  if (!SD.begin()) {
    Serial.println("HATA!");
    sd_status = false;
    return;
  }
  
  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("Kart bulunamadı!");
    sd_status = false;
    return;
  }
  
  sd_status = true;
  Serial.println("OK");
  
  if (!SD.exists("/sensor_data.csv")) {
    writeFile(SD, "/sensor_data.csv", "Packet_Number,Uydu_Statusu,Hata_Kodu,Gonderme_Saati,Pressure1,Pressure2,Yukseklik1,Yukseklik2,Irtifa_Farki,Inis_Hizi,Temperature,Pil_Gerilimi,GPS1_Latitude,GPS1_Longitude,GPS1_Altitude,Pitch,Roll,Yaw,RHRH,IoT_S1_Data,IoT_S2_Data,Takim_No,Sensor_Status\n");
  }
}

void initLoRa() {
  Serial.print("LoRa E22... ");
  
  // LoRa pinlerini output olarak ayarla
  pinMode(LORA_M0, OUTPUT);
  pinMode(LORA_M1, OUTPUT);
  pinMode(LORA_AUX, INPUT);
  
  // Normal moda geç (M0=LOW, M1=LOW)
  digitalWrite(LORA_M0, LOW);
  digitalWrite(LORA_M1, LOW);
  delay(100);
  
  Serial2.begin(9600, SERIAL_8N1, LORA_RX, LORA_TX);
  delay(500);
  
  E22.begin();
  delay(1000);
  
  // LoRa modül test
  ResponseStructContainer c = E22.getModuleInformation();
  if (c.status.code == 1) {
    lora_status = true;
    Serial.println("OK");
    
    ModuleInformation moduleInfo = *(ModuleInformation*) c.data;
    Serial.printf("LoRa Model: %d, Version: %d\n", moduleInfo.model, moduleInfo.version);
    
    // Konfigürasyonu kontrol et
    ResponseStructContainer config = E22.getConfiguration();
    if (config.status.code == 1) {
      Configuration configuration = *(Configuration*) config.data;
      Serial.printf("LoRa Adres: 0x%02X:0x%02X, Kanal: %d\n", 
                    configuration.ADDH, configuration.ADDL, configuration.CHAN);
    }
    config.close();
  } else {
    lora_status = false;
    Serial.printf("HATA! Status Code: %d\n", c.status.code);
    Serial.printf("Status: %s\n", c.status.getResponseDescription().c_str());
    
    // LoRa troubleshooting
    Serial.println("LoRa Troubleshooting:");
    Serial.printf("M0: GPIO%d, M1: GPIO%d, AUX: GPIO%d\n", LORA_M0, LORA_M1, LORA_AUX);
    Serial.printf("TX: GPIO%d, RX: GPIO%d\n", LORA_TX, LORA_RX);
    Serial.println("Kontrol edilecekler:");
    Serial.println("- LoRa modül bağlantıları");
    Serial.println("- 5V/3.3V güç besleme");
    Serial.println("- Pin bağlantılarının doğruluğu");
  }
  c.close();
}

void collectSensorData(TelemetryPacket* data) {
  // Paket başlığı
  data->packet_type = PACKET_TYPE_TELEMETRY;
  data->paket_sayisi = paket_sayisi_lora2;
  data->takim_no = TAKIM_NO;
  data->rhrh = current_rhrh_encoded;
  
  // Hata kodunu sıfırla ve 6 bit hata durumunu kontrol et
  clearAllErrors(&data->hata_kodu);
  
  // Varsayılan değerler
  data->basinc2 = 0.0;
  data->yukseklik2 = -500.0;
  data->irtifa_farki = 0.0;
  data->inis_hizi = 0.0;
  data->iot_s1_data = 0;
  data->iot_s2_data = 0;
  
  // LoRa'dan gelen basınç verisi (Pressure Container)
  if (lora_basinc2_received) {
    data->basinc2 = lora_basinc2;
    
    Serial.printf("🔵 LoRa Basınç2: %.2f Pa\n", lora_basinc2);
    
    // Yükseklik2 hesaplaması - basınç2'den hesapla
    if (!isnan(lora_basinc2) && lora_basinc2 > 50000.0) {
      // data.sicaklik değerini kullan (Celsius * 10 formatında, bu yüzden /10 yap)
      float temperature_for_altitude2 = 15.0; // Varsayılan sıcaklık
      if (data->sicaklik != 0) {
        temperature_for_altitude2 = data->sicaklik / 10.0; // int16_t'den float'a çevir
      } else {
        // Eğer sicaklik verisi yoksa BMP280'den oku
        if (bmp_status) {
          float bmp_temp = bmp.readTemperature();
          if (!isnan(bmp_temp)) {
            temperature_for_altitude2 = bmp_temp;
          }
        }
      }
      
      // calculateAltitude fonksiyonunu kullanarak yükseklik2'yi hesapla
      data->yukseklik2 = calculateAltitude(lora_basinc2, temperature_for_altitude2);
      
      Serial.printf("🔵 Yükseklik2 Hesaplandı: %.4f m (Basınç: %.2f Pa, Sıcaklık: %.1f°C, Referans: %.2f Pa)\n", 
                   data->yukseklik2, lora_basinc2, temperature_for_altitude2, reference_pressure);
    } else {
      data->yukseklik2 = -500.0; // Geçersiz basınç verisi
      Serial.printf("❌ Basınç2 geçersiz: %.2f Pa, Yükseklik2 = -500.0\n", lora_basinc2);
    }
  } else {
    // LoRa verisi yoksa varsayılan değerler
    data->basinc2 = 0.0;
    data->yukseklik2 = -500.0; // Veri yok demek
    Serial.println("❌ LoRa Basınç2 verisi YOK");
  }
  
  // LoRa'dan gelen IoT verileri (L4 ve L5 Data)
  if (lora_iot_s1_received) {
    data->iot_s1_data = lora_iot_s1_data;
  }
  if (lora_iot_s2_received) {
    data->iot_s2_data = lora_iot_s2_data;
  }
  
  // RTC zamanı
  if (rtc_status) {
    DateTime now = rtc.now();
    if (now.year() > 2020 && now.year() < 2035) {
      data->gonderme_saati = now.unixtime();
    } else {
      data->gonderme_saati = 0; // RTC geçersiz zaman veriyorsa 0
      rtc_status = false;
    }
  } else {
    data->gonderme_saati = 0; // RTC çalışmıyorsa 0
  }
  
  // MPU6050 verileri
  if (mpu_status) {
    sensors_event_t a, g, temp;
    if (mpu.getEvent(&a, &g, &temp)) {
      // Ham açı değerlerini oku
      float raw_pitch = g.gyro.y * 180.0 / PI;
      float raw_roll = g.gyro.x * 180.0 / PI;
      float raw_yaw = g.gyro.z * 180.0 / PI;
      
      // Referans açıları çıkararak relatif açıları hesapla
      float relative_pitch = raw_pitch - reference_pitch;
      float relative_roll = raw_roll - reference_roll;
      float relative_yaw = raw_yaw - reference_yaw;
      
      // 10 ile çarparak int16_t formatına çevir
      data->pitch = (int16_t)(relative_pitch * 10);
      data->roll = (int16_t)(relative_roll * 10);
      data->yaw = (int16_t)(relative_yaw * 10);
    } else {
      mpu_status = false;
      data->pitch = 0;
      data->roll = 0;
      data->yaw = 0;
    }
  } else {
    data->pitch = 0;
    data->roll = 0;
    data->yaw = 0;
  }
  
  // BMP280 verileri
  if (bmp_status) {
    float pressure = bmp.readPressure();
    float temperature = bmp.readTemperature();
    
    if (!isnan(pressure) && !isnan(temperature)) {
      data->basinc1 = pressure;
      data->sicaklik = (int16_t)(temperature * 10);
      data->yukseklik1 = calculateAltitude(pressure, temperature);
      
      // İniş hızı hesaplama
      unsigned long current_time = millis();
      if (!first_altitude_reading && (current_time - previous_time) > 0) {
        float altitude_diff = data->yukseklik1 - previous_altitude;
        float time_diff = (current_time - previous_time) / 1000.0;
        data->inis_hizi = altitude_diff / time_diff;
        
        if (data->inis_hizi < 0) {
          data->inis_hizi = -data->inis_hizi;
        } else {
          data->inis_hizi = 0.0;
        }
      } else {
        data->inis_hizi = 0.0;
        first_altitude_reading = false;
      }
      
      previous_altitude = data->yukseklik1;
      previous_time = current_time;
    } else {
      bmp_status = false;
      data->basinc1 = 0;
      data->sicaklik = 0;
      data->yukseklik1 = -500.0;
      data->inis_hizi = 0.0;
    }
  } else {
    data->basinc1 = 0;
    data->sicaklik = 0;
    data->yukseklik1 = -500.0;
    data->inis_hizi = -1;
  }
  
  // GPS verileri
  if (gps.location.isValid()) {
    data->gps1_latitude = gps.location.lat();
    data->gps1_longitude = gps.location.lng();
    gps_status = true;
  } else {
    data->gps1_latitude = 0.0;
    data->gps1_longitude = 0.0;
    gps_status = false;
  }
  
  if (gps.altitude.isValid()) {
    data->gps1_altitude = gps.altitude.meters();
  } else {
    data->gps1_altitude = 0.0;
  }
  
  // ADC gerilim
  int adcValue = analogRead(pil_gerilimi_PIN);
  if (adcValue >= 0 && adcValue <= 4095) {
    float voltage = (adcValue * 3.3) / 4096.0;
    data->pil_gerilimi = (uint16_t)(voltage * 100);
    adc_status = true;
  } else {
    data->pil_gerilimi = 330;
    adc_status = false;
  }
  
  // Uydu statusunu geçici olarak belirle (hata kodları için)
  float current_altitude_for_status = data->yukseklik1;
  if (current_altitude_for_status < 0) {
    current_altitude_for_status = 0; // Negatif yükseklik varsa 0 kabul et
  }
  
  // İlk okuma ise önceki yüksekliği mevcut yükseklik yap
  if (first_status_reading) {
    previous_altitude_for_status = current_altitude_for_status;
    first_status_reading = false;
  }
  
  // Start button durumunu belirle (sensörler hazırsa ve yer seviyesindeyse)
  start_button = (bmp_status && mpu_status && rtc_status && gps_status && sd_status && adc_status && lora_status);
  
  // Ayrılma durumunu servo3'ün pozisyonuna göre belirle
  ayrilma_status = servo3_separated;
  
  // Geçici status belirleme (hata kodları için)
  uint8_t temp_uydu_statusu = determineUyduStatusAlgorithm(
    current_altitude_for_status,    // yukseklik1
    data->inis_hizi,               // inis_hizi  
    previous_uydu_status,          // current_uydu_statusu
    start_button,                  // start_button
    ayrilma_status,                // ayrilma_status
    manual_ayrilma,                // manual_ayrilma
    tolerans0, tolerans1, tolerans2, tolerans3, tolerans4, tolerans5,  // toleranslar
    ayrilma_toleransi              // ayrilma_toleransi
  );
  
  // 6-bit hata kodu kontrolleri - STATUS kodlarına göre
  // Varsayılan hata kodu: 000000 (tüm bitler temiz)
  clearAllErrors(&data->hata_kodu);
  
  // Status 2: Model Uydu İniş
  if (temp_uydu_statusu == STATUS_MODEL_INIS) {
    // Önce tüm bitleri 1 yap (bit 0 hariç - o hız şartına göre)
    setErrorBit(&data->hata_kodu, ERROR_BIT_GOREV_INIS_HIZI);     // Bit 1: 1
    setErrorBit(&data->hata_kodu, ERROR_BIT_TASIYICI_BASINC);     // Bit 2: 1
    setErrorBit(&data->hata_kodu, ERROR_BIT_GOREV_KONUM);         // Bit 3: 1
    setErrorBit(&data->hata_kodu, ERROR_BIT_AYRILMA);             // Bit 4: 1
    setErrorBit(&data->hata_kodu, ERROR_BIT_SPEKTRAL_FILTRE);     // Bit 5: 1
    
    // Bit 0: Model iniş hızına göre (12-14 m/s) - şarta göre belirlenir
    updateModelInisHiziError(&data->hata_kodu, data->inis_hizi);
  }
  // Status 3: Ayrılma
  else if (temp_uydu_statusu == STATUS_AYRILMA) {
    // Bit 0: Model uydu iniş hızı (hala kontrol edilebilir)
    updateModelInisHiziError(&data->hata_kodu, data->inis_hizi);
    
    // Bit 1: Görev yükü iniş hızı (henüz ayrılma aşamasında, 0 olmalı)
    clearErrorBit(&data->hata_kodu, ERROR_BIT_GOREV_INIS_HIZI);
    
    // Bit 2: Taşıyıcı basınç verisi
    updateTasiyiciBasincError(&data->hata_kodu, bmp_status && data->basinc1 > 0);
    
    // Bit 3: Görev yükü konum verisi  
    updateGorevKonumError(&data->hata_kodu, gps_status && gps.location.isValid());
    
    // Bit 4: Ayrılma durumu
    bool ayrilma_gerceklesti = servo3_separated;
    updateAyrilmaError(&data->hata_kodu, ayrilma_gerceklesti);
    
    // Bit 5: Multi-spektral mekanik filtreleme sistemi (duruma göre güncellenmeli)
    bool spektral_filtre_calisiyor = !rhrh_active;
    updateSpektralFiltreError(&data->hata_kodu, spektral_filtre_calisiyor);
  }
  // Status 4: Görev Yükü İniş
  else if (temp_uydu_statusu == STATUS_GOREV_INIS) {
    // Bit 0: Model iniş hızı artık 0 olmalı (görev yükü aşamasında)
    clearErrorBit(&data->hata_kodu, ERROR_BIT_MODEL_INIS_HIZI);
    
    // Bit 1: Görev yükü iniş hızı (6-8 m/s)
    updateGorevInisHiziError(&data->hata_kodu, data->inis_hizi);
    
    // Bit 2,3,4: Duruma göre 0 veya 1
    updateTasiyiciBasincError(&data->hata_kodu, bmp_status && data->basinc1 > 0);
    updateGorevKonumError(&data->hata_kodu, gps_status && gps.location.isValid());
    
    // Ayrılma gerçekleşmiş olmalı
    bool ayrilma_gerceklesti = servo3_separated;
    updateAyrilmaError(&data->hata_kodu, ayrilma_gerceklesti);
    
    // Bit 5: Multi-spektral sistem
    bool spektral_filtre_calisiyor = !rhrh_active;
    updateSpektralFiltreError(&data->hata_kodu, spektral_filtre_calisiyor);
  }
  // Status 5: Kurtarma ve diğer durumlar
  else if (temp_uydu_statusu >= STATUS_KURTARMA) {
    // Normal kontroller
    updateModelInisHiziError(&data->hata_kodu, data->inis_hizi);
    updateGorevInisHiziError(&data->hata_kodu, data->inis_hizi);
    updateTasiyiciBasincError(&data->hata_kodu, bmp_status && data->basinc1 > 0);
    updateGorevKonumError(&data->hata_kodu, gps_status && gps.location.isValid());
    
    bool ayrilma_gerceklesti = servo3_separated;
    updateAyrilmaError(&data->hata_kodu, ayrilma_gerceklesti);
    
    bool spektral_filtre_calisiyor = !rhrh_active;
    updateSpektralFiltreError(&data->hata_kodu, spektral_filtre_calisiyor);
  }
  // Status 0-1 ve diğer durumlar: Varsayılan 000000 kalır
  
  // Manuel ayrılma komutu LoRa'dan gelir (şimdilik false)
  // manual_ayrilma değişkeni LoRa mesajlarında güncellenecek
  
  // Algoritma tabanlı status belirleme - ana status ataması
  data->uydu_statusu = temp_uydu_statusu;
  
  // ACİL DURUM: GPIO34 butonu tetiklendiyse ve uydu status 255 (-1) ise bir kereliğine 0 yap
  if (emergency_uydu_status_triggered && previous_uydu_status == 255) {
    data->uydu_statusu = 0;
    Serial.println("*** ACİL DURUM: GPIO34 Butonu - Uydu Status bir kereliğine 0'a zorlandı! ***");
  }
  
  // Mevcut uydu status'unu global değişkende güncelle (buzzer kontrolü için)
  current_uydu_status = data->uydu_statusu;
  
  // Ayrılma kontrolü - algoritma ayrılma statusu verirse DC motoru aktif et
  if (data->uydu_statusu == STATUS_AYRILMA && !servo3_separated) {
    Serial.println("ALGORITMA: Ayrılma statusu - DC Motor ayrılma yönünde");
    dcMotorAyrilma();
    servo3_separated = true;
    ayrilma_basla = true;
  }
  
  // Ayrılma başladıysa ayrilma_status'u güncelle
  if (ayrilma_basla) {
    ayrilma_status = true;
  }
  
  // İrtifa farkı hesapla: yukseklik1 ve yukseklik2 arasındaki mutlak fark
  Serial.printf("📐 İRTİFA FARKI HESABI:\n");
  Serial.printf("   Yükseklik1: %.4f m\n", data->yukseklik1);
  Serial.printf("   Yükseklik2: %.4f m\n", data->yukseklik2);
  
  if (data->yukseklik1 > -500.0 && data->yukseklik2 > -500.0) {
    data->irtifa_farki = fabs(data->yukseklik1 - data->yukseklik2);
    Serial.printf("   ✅ İRTİFA FARKI = |%.4f - %.4f| = %.4f m\n", 
                 data->yukseklik1, data->yukseklik2, data->irtifa_farki);
  } else {
    data->irtifa_farki = 0.0;
    Serial.printf("   ❌ İrtifa farkı hesaplanamadı - Yük1: %.4f, Yük2: %.4f (geçersiz veri)\n", 
                 data->yukseklik1, data->yukseklik2);
  }
  
  // Bir sonraki döngü için önceki yüksekliği ve status'u güncelle
  previous_altitude_for_status = current_altitude_for_status;
  previous_uydu_status = data->uydu_statusu;
}

void sendTelemetryToLora2() {
  if (!lora_status) {
    Serial.println("LoRa hatası - veri gönderilemedi (durum kontrol ediliyor)");
    
    // LoRa durumunu yeniden kontrol et
    ResponseStructContainer c = E22.getModuleInformation();
    if (c.status.code == 1) {
      lora_status = true;
      Serial.println("LoRa yeniden bağlandı!");
    } else {
      Serial.printf("LoRa hala çevrimdışı - Status Code: %d\n", c.status.code);
      c.close();
      return;
    }
    c.close();
  }
  
  TelemetryPacket telemetry;
  collectSensorData(&telemetry);
  
  // Gönderilecek paketi serial monitöre yazdır
  char rhrh_str[5];
  decodeRHRH(current_rhrh_encoded, rhrh_str);


  //Debug
  Serial.println("=== LORA PAKETİ İÇERİĞİ ===");
  Serial.printf("Paket Tipi: 0x%02X\n", telemetry.packet_type);
  Serial.printf("Paket Sayısı: %d\n", telemetry.paket_sayisi);
  
  // Uydu statusunu açıklamasıyla birlikte göster
  const char* status_names[] = {"Uçuşa Hazır", "Yükselme", "Model Uydu İniş", "Ayrılma", "Görev Yükü İniş", "Kurtarma"};
  if (telemetry.uydu_statusu == 255) {
    Serial.printf("Uydu Statüsü: -1 (Default/Belirsiz)\n");
  } else {
    Serial.printf("Uydu Statüsü: %d (%s)\n", telemetry.uydu_statusu, 
                  (telemetry.uydu_statusu <= 5) ? status_names[telemetry.uydu_statusu] : "Bilinmiyor");
  }
  
  // Hata kodunu binary olarak manuel yazdır (6 bit)
  char hata_binary[7];
  for(int i = 5; i >= 0; i--) {
    hata_binary[5-i] = (telemetry.hata_kodu & (1 << i)) ? '1' : '0';
  }
  hata_binary[6] = '\0';
  Serial.printf("Hata Kodu: 0b%s (0x%02X)\n", hata_binary, telemetry.hata_kodu);
  
  // RTC değerini formatlanmış olarak yazdır
  if (rtc_status && telemetry.gonderme_saati > 0) {
    DateTime now = rtc.now();
    Serial.printf("Gönderme Saati: %d (RTC: %04d/%02d/%02d %02d:%02d:%02d)\n", 
                  telemetry.gonderme_saati, 
                  now.year(), now.month(), now.day(),
                  now.hour(), now.minute(), now.second());
  } else {
    Serial.printf("Gönderme Saati: %d (RTC: Hata/Geçersiz)\n", telemetry.gonderme_saati);
  }
  
  Serial.printf("Basınç1: %.1f Pa\n", telemetry.basinc1);
  Serial.printf("Basınç2: %.1f Pa\n", telemetry.basinc2);
  Serial.printf("Yükseklik1: %.1f m (Ref: %.1f Pa%s)\n", telemetry.yukseklik1, reference_pressure, 
                reference_pressure_set ? "" : " - Uyarı!");
  Serial.printf("Yükseklik2: %.1f m\n", telemetry.yukseklik2);
  Serial.printf("İrtifa Farkı: %.1f m\n", telemetry.irtifa_farki);
  Serial.printf("İniş Hızı: %.1f m/s\n", telemetry.inis_hizi);
  Serial.printf("Sıcaklık: %.1f°C\n", telemetry.sicaklik / 10.0);
  Serial.printf("Pil Gerilimi: %.2fV\n", telemetry.pil_gerilimi / 100.0);
  Serial.printf("GPS Latitude: %.6f\n", telemetry.gps1_latitude);
  Serial.printf("GPS Longitude: %.6f\n", telemetry.gps1_longitude);
  Serial.printf("GPS Altitude: %.1f m\n", telemetry.gps1_altitude);
  Serial.printf("Pitch: %.1f° (Ref: %.1f°%s)\n", telemetry.pitch / 10.0, reference_pitch, 
                reference_angles_set ? "" : " - Uyarı!");
  Serial.printf("Roll: %.1f° (Ref: %.1f°%s)\n", telemetry.roll / 10.0, reference_roll, 
                reference_angles_set ? "" : " - Uyarı!");
  Serial.printf("Yaw: %.1f° (Ref: %.1f°%s)\n", telemetry.yaw / 10.0, reference_yaw, 
                reference_angles_set ? "" : " - Uyarı!");
  Serial.printf("RHRH: %s (0x%08X)\n", rhrh_str, telemetry.rhrh);
  Serial.printf("IoT S1 Data: %d\n", telemetry.iot_s1_data);
  Serial.printf("IoT S2 Data: %d\n", telemetry.iot_s2_data);
  Serial.printf("Takım No: %d\n", telemetry.takim_no);
  Serial.printf("Toplam Boyut: %d bytes\n", sizeof(telemetry));
  Serial.println("===========================");
  //Debug End



  // AUX pininin hazır olup olmadığını kontrol et
  if (digitalRead(LORA_AUX) == LOW) {
    Serial.println("LoRa modülü meşgul - bekliyor");
    unsigned long waitStart = millis();
    while (digitalRead(LORA_AUX) == LOW && (millis() - waitStart < 1000)) {
      delay(10);
    }
    
    if (digitalRead(LORA_AUX) == LOW) {
      Serial.println("LoRa modülü zaman aşımı - veri gönderilemedi");
      return;
    }
  }
  
  ResponseStatus rs = E22.sendFixedMessage(0x00, 0x0A, 20, (uint8_t*)&telemetry, sizeof(telemetry));
  
  if (rs.code == 1) {
    Serial.printf("LoRa2'ye telemetry gönderildi (%d bytes), Paket#: %d, RHRH: %s\n", 
                  sizeof(telemetry), paket_sayisi_lora2, rhrh_str);
    
    // SD'ye de kaydet
    if (sd_status) {
      saveToSD(telemetry);
    }
    
    paket_sayisi_lora2++;
    packet_counter++;
  } else {
    Serial.printf("LoRa gönderme hatası - Status Code: %d\n", rs.code);
    Serial.printf("Hata açıklaması: %s\n", rs.getResponseDescription().c_str());
    lora_status = false; // Durum güncelleme
  }
}

bool waitForLoRaMessage(unsigned long timeout_ms) {
  unsigned long start_time = millis();
  while (millis() - start_time < timeout_ms) {
    if (E22.available() > 1) {
      ResponseContainer rs = E22.receiveMessage();
      
      uint8_t* data = (uint8_t*)rs.data.c_str();
      int length = rs.data.length();
      
      if (length > 0) {
        uint8_t packet_type = data[0];
        Serial.printf("LoRa paketi alındı - Tip: 0x%02X, Boyut: %d bytes\n", packet_type, length);
        
        switch (packet_type) {
          case PACKET_TYPE_BUTTON_CONTROL:
            if (length >= sizeof(ButtonControlPacket)) {
              ButtonControlPacket* btn = (ButtonControlPacket*)data;
              Serial.printf("Button Control - Paket#: %d, Manuel Ayrılma: %s\n", 
                            btn->paket_sayisi, btn->manuel_ayrilma ? "true" : "false");
              
              uint32_t eski_rhrh = current_rhrh_encoded;
              current_rhrh_encoded = btn->rhrh;
              
              char rhrh_str[5];
              decodeRHRH(btn->rhrh, rhrh_str);
              Serial.printf("LoRa'dan RHRH güncellendi: %s\n", rhrh_str);
              
              // Yeni RHRH sistemi ile komut işle
              String rhrhCommand = String(rhrh_str);
              if (rhrhCommand.length() == 4) {
                processRHRHCommand(rhrhCommand);
              } else {
                Serial.println("LoRa'dan geçersiz RHRH formatı alındı");
              }
              
              // Manuel ayrılma/birleşme komutu varsa DC motor kontrolü ve algoritma değişkenini güncelle
              if (btn->manuel_ayrilma == 1) {
                manual_ayrilma = 1; // Algoritma için manuel ayrılma değişkenini aktif et
                
                if (!servo3_separated) {
                  Serial.println("LoRa'dan manuel ayrılma komutu - DC Motor ayrılma yönünde");
                  dcMotorAyrilma();
                  servo3_separated = true;
                  ayrilma_basla = true;
                }
              } else if (btn->manuel_ayrilma == 2) {
                manual_ayrilma = 2; // Manuel birleşme komutu
                Serial.println("LoRa'dan manuel birleşme komutu - DC Motor birleşme yönünde");
                dcMotorBirlesme();
                // servo3_separated durumunu değiştirmiyoruz, sadece motor çalıştırıyoruz
              }
            }
            break;
            
          case PACKET_TYPE_PRESSURE_CONTAINER:
            if (length >= sizeof(PressureContainerPacket)) {
              PressureContainerPacket* prs = (PressureContainerPacket*)data;
              Serial.printf("Pressure Container - Paket#: %d, Basınç: %.1f Pa\n", 
                            prs->paket_sayisi, prs->basinc1);
              
              // LoRa'dan gelen basınç verisini basinc2 olarak kaydet
              lora_basinc2 = prs->basinc1;
              lora_basinc2_received = true;
              
              Serial.printf("LoRa basınç verisi güncellendi: basinc2 = %.1f Pa\n", lora_basinc2);
            }
            break;
            
          case PACKET_TYPE_L4_DATA:
            if (length >= sizeof(LoraDataPacket)) {
              LoraDataPacket* ldata = (LoraDataPacket*)data;
              
              if (validatePacket(data, length, packet_type)) {
                // L4 Data'yı IoT S1 Data'ya kaydet
                lora_iot_s1_data = ldata->temperature; // temperature alanını kullan
                lora_iot_s1_received = true;
                Serial.printf("L4 Data - Paket#: %d, IoT S1: %d (LoRa'dan güncellendi)\n", 
                              ldata->paket_sayisi, lora_iot_s1_data);
              } else {
                Serial.println("L4 Data - Checksum hatası!");
              }
            } else {
              Serial.printf("L4 Data - Yetersiz paket boyutu: %d < %d\n", length, sizeof(LoraDataPacket));
            }
            break;
            
          case PACKET_TYPE_L5_DATA:
            if (length >= sizeof(LoraDataPacket)) {
              LoraDataPacket* ldata = (LoraDataPacket*)data;
              
              if (validatePacket(data, length, packet_type)) {
                // L5 Data'yı IoT S2 Data'ya kaydet
                lora_iot_s2_data = ldata->temperature; // temperature alanını kullan
                lora_iot_s2_received = true;
                Serial.printf("L5 Data - Paket#: %d, IoT S2: %d (LoRa'dan güncellendi)\n", 
                              ldata->paket_sayisi, lora_iot_s2_data);
              } else {
                Serial.println("L5 Data - Checksum hatası!");
              }
            } else {
              Serial.printf("L5 Data - Yetersiz paket boyutu: %d < %d\n", length, sizeof(LoraDataPacket));
            }
            break;          default:
            Serial.printf("Bilinmeyen paket tipi: 0x%02X\n", packet_type);
            break;
        }
        return true;
      }
    }
    delay(10);
  }
  return false;
}

void processSerialCommands() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command.startsWith("RHRH:")) {
      String rhrhValue = command.substring(5);
      processRHRHCommand(rhrhValue);
    }
    else if (command.startsWith("SET:")) {
      setRTCTime(command);
    }
    else if (command == "GET_DATA") {
      TelemetryPacket data;
      collectSensorData(&data);
      
      Serial.println("=== MANUEL VERİ TOPLAMA ===");
      Serial.printf("Paket#: %d\n", data.paket_sayisi);
      Serial.printf("Basınç1 (BMP280): %.1f Pa\n", data.basinc1);
      Serial.printf("Basınç2 (LoRa): %.1f Pa%s\n", data.basinc2, lora_basinc2_received ? "" : " (veri yok)");
      Serial.printf("Sıcaklık: %.1f°C\n", data.sicaklik / 10.0);
      Serial.printf("🎯 Yükseklik1: %.4f m\n", data.yukseklik1);
      Serial.printf("🎯 Yükseklik2: %.4f m\n", data.yukseklik2);
      Serial.printf("🎯 İrtifa Farkı: %.4f m\n", data.irtifa_farki);
      Serial.printf("🎯 İniş Hızı: %.4f m/s\n", data.inis_hizi);
      Serial.printf("GPS: %.6f, %.6f (%.1f m)\n", data.gps1_latitude, data.gps1_longitude, data.gps1_altitude);
      Serial.printf("MPU: P=%.1f°, R=%.1f°, Y=%.1f°\n", data.pitch/10.0, data.roll/10.0, data.yaw/10.0);
      Serial.printf("Pil: %.2fV\n", data.pil_gerilimi / 100.0);
    }
    else if (command == "RESET_REF") {
      Serial.println("=== REFERANS SIFIRLAMA ===");
      bool pressure_success = true; // Basınç referansına dokunmuyoruz
      bool angles_success = false;
      
      // Basınç referansını KORUMA - sadece sistem başlangıcında ayarlanır
      Serial.printf("✓ Basınç referansı korundu: %.1f Pa (sistem başlangıcından)\n", reference_pressure);
      Serial.println("ℹ️  Basınç referansı sadece sistem başlangıcında ayarlanır!");
      
      // Sadece İniş hızı hesaplamasını sıfırla
      first_altitude_reading = true;
      previous_altitude = 0.0;
      
      // Status takibini de sıfırla
      first_status_reading = true;
      previous_altitude_for_status = 0.0;
      
      // Açı referanslarını sıfırla
      if (mpu_status) {
        // Referans açıları belirlemek için 5 okuma yapıp ortalamasını al
        float pitch_sum = 0, roll_sum = 0, yaw_sum = 0;
        int valid_readings = 0;
        
        for(int i = 0; i < 5; i++) {
          sensors_event_t ref_a, ref_g, ref_temp;
          if (mpu.getEvent(&ref_a, &ref_g, &ref_temp)) {
            pitch_sum += ref_g.gyro.y * 180.0 / PI;
            roll_sum += ref_g.gyro.x * 180.0 / PI;
            yaw_sum += ref_g.gyro.z * 180.0 / PI;
            valid_readings++;
          }
          delay(20);
        }
        
        if (valid_readings >= 3) { // En az 3 geçerli okuma varsa
          reference_pitch = pitch_sum / valid_readings;
          reference_roll = roll_sum / valid_readings;
          reference_yaw = yaw_sum / valid_readings;
          reference_angles_set = true;
          Serial.printf("✓ Referans açılar sıfırlandı: P=%.1f°, R=%.1f°, Y=%.1f°\n", 
                        reference_pitch, reference_roll, reference_yaw);
          Serial.println("✓ Mevcut açılar artık 0° kabul edildi!");
          angles_success = true;
        } else {
          Serial.println("✗ Hata: MPU6050'den geçerli açı okunamadı!");
        }
      } else {
        Serial.println("✗ Hata: MPU6050 sensörü çalışmıyor!");
      }
      
      if (pressure_success && angles_success) {
        Serial.println("✓ Tüm referanslar başarıyla sıfırlandı!");
      } else if (pressure_success || angles_success) {
        Serial.println("⚠ Referanslar kısmen sıfırlandı!");
      } else {
        Serial.println("✗ Hiçbir referans sıfırlanamadı!");
      }
    }
    else if (command == "AYRIL") {
      if (!servo3_separated) {
        Serial.println("AYRIL komutu - DC Motor ayrılma yönünde çalışıyor");
        dcMotorAyrilma();
        servo3_separated = true;
        Serial.println("Ayrılma tamamlandı! DC Motor aktif");
      } else {
        Serial.println("Ayrılma zaten gerçekleştirilmiş!");
      }
    }
    else if (command == "BIRLES") {
      Serial.println("BIRLES komutu - DC Motor birleşme yönünde çalışıyor");
      dcMotorBirlesme();
      Serial.println("Birleşme komutu gönderildi! DC Motor aktif");
    }
    else if (command == "STATUS") {
      Serial.println("=== SİSTEM DURUMU ===");
      Serial.printf("BMP280: %s\n", bmp_status ? "OK" : "HATA");
      Serial.printf("MPU6050: %s\n", mpu_status ? "OK" : "HATA");
      
      // RTC durumunu detaylı göster
      if (rtc_status) {
        DateTime now = rtc.now();
        Serial.printf("RTC: OK (%04d/%02d/%02d %02d:%02d:%02d)\n", 
                      now.year(), now.month(), now.day(),
                      now.hour(), now.minute(), now.second());
      } else {
        Serial.printf("RTC: HATA\n");
      }
      
      Serial.printf("GPS: %s\n", gps_status ? "OK" : "HATA");
      Serial.printf("SD Kart: %s\n", sd_status ? "OK" : "HATA");
      Serial.printf("ADC: %s\n", adc_status ? "OK" : "HATA");
      Serial.printf("LoRa: %s\n", lora_status ? "OK" : "HATA");
      
      // Referans basınç bilgisini göster
      if (reference_pressure_set) {
        Serial.printf("Referans Basınç: %.1f Pa (Bu seviye 0m kabul edildi)\n", reference_pressure);
      } else {
        Serial.printf("Referans Basınç: %.1f Pa (Standart değer - uyarı!)\n", reference_pressure);
      }
      
      // Referans açı bilgilerini göster
      if (reference_angles_set) {
        Serial.printf("Referans Açılar: P=%.1f°, R=%.1f°, Y=%.1f° (Bu açılar 0° kabul edildi)\n", 
                      reference_pitch, reference_roll, reference_yaw);
      } else {
        Serial.printf("Referans Açılar: P=%.1f°, R=%.1f°, Y=%.1f° (Standart değer - uyarı!)\n", 
                      reference_pitch, reference_roll, reference_yaw);
      }
      
      Serial.printf("RHRH: %s\n", current_rhrh.c_str());
      Serial.printf("Ayrılma Durumu: %s\n", servo3_separated ? "Ayrık (DC Motor)" : "Birleşik");
      Serial.printf("Paket sayısı: %d\n", packet_counter);
    }
    else if (command == "SERVO_TEST") {
      Serial.println("=== SERVO TEST (Yeni RHRH Sistemi) ===");
      Serial.println("Servo pozisyon testi başlıyor...");
      
      Serial.printf("Default A pozisyonu: Servo1=%d°, Servo2=%d° - 2sn\n", ANGLE_A, ANGLE_A);
      setServoPosition(SERVO1_CHANNEL, ANGLE_A);
      setServoPosition(SERVO2_CHANNEL, ANGLE_A);
      delay(2000);
      
      Serial.printf("M pozisyonu: Servo1=%d°, Servo2=%d° - 2sn\n", ANGLE_M, ANGLE_M);
      setServoPosition(SERVO1_CHANNEL, ANGLE_M);
      setServoPosition(SERVO2_CHANNEL, ANGLE_M);
      delay(2000);
      
      Serial.printf("F pozisyonu: Servo1=%d°, Servo2=%d° - 2sn\n", ANGLE_F, ANGLE_F);
      setServoPosition(SERVO1_CHANNEL, ANGLE_F);
      setServoPosition(SERVO2_CHANNEL, ANGLE_F);
      delay(2000);
      
      Serial.printf("C pozisyonu: Servo1=%d°, Servo2=%d° - 2sn\n", ANGLE_C, ANGLE_C2);
      setServoPosition(SERVO1_CHANNEL, ANGLE_C);
      setServoPosition(SERVO2_CHANNEL, ANGLE_C2);
      delay(2000);
      
      Serial.println("DC Motor Test: Ayrılma yönü - 2sn");
      dcMotorAyrilma();
      delay(2000);
      
      Serial.println("DC Motor Test: Birleşme yönü - 2sn");
      dcMotorBirlesme();
      delay(2000);
      
      Serial.println("Default pozisyonlara dönülüyor (0A0A)");
      setServoPosition(SERVO1_CHANNEL, ANGLE_A);
      setServoPosition(SERVO2_CHANNEL, ANGLE_A);
      servo3_separated = false;
      current_rhrh = "0A0A";
      current_rhrh_encoded = encodeRHRH('0', 'A', '0', 'A');
      
      Serial.println("Servo test tamamlandı");
    }
    else if (command == "RHRH_TEST") {
      Serial.println("=== RHRH TEST (Yeni Sistem) ===");
      Serial.println("Test komutları:");
      Serial.println("  6M3C - 6s M pozisyonu, sonra 3s C pozisyonu");
      Serial.println("  4F2N - 4s F pozisyonu, sonra 2s N pozisyonu");
      Serial.println("  2R5Y - 2s R pozisyonu, sonra 5s Y pozisyonu");
      Serial.println("");
      Serial.println("Pozisyon açıları:");
      Serial.println("  A (default): 40°, 40°");
      Serial.println("  M: 80°, 80°    F: 0°, 0°     N: 120°, 120°");
      Serial.println("  R: 40°, 80°    G: 0°, 40°    B: 40°, 120°");
      Serial.println("  P: 80°, 120°   Y: 0°, 80°    C: 0°, 120°");
      Serial.println("");
      Serial.println("Örnek test için RHRH:6M3C yazın");
    }
    else if (command == "LORA_TEST") {
      Serial.println("=== LORA TEST ===");
      printLoRaModuleInfo();
      printLoRaConfig();
      
      Serial.println("Test telemetry paketi gönderiliyor...");
      sendTelemetryToLora2();
      
      Serial.println("5 saniye mesaj dinleniyor...");
      unsigned long start = millis();
      while (millis() - start < 5000) {
        if (waitForLoRaMessage(100)) {
          Serial.println("Mesaj alındı!");
        }
        delay(100);
      }
      Serial.println("LoRa test tamamlandı");
    }
  }
}

void processRHRHCommand(String command) {
  if (command.length() != 4) {
    Serial.println("HATA: RHRH formatı yanlış! Format: RHSH (örn: 6M3C)");
    return;
  }
  
  char dur1 = command.charAt(0);  // İlk rakam (süre)
  char pos1 = command.charAt(1);  // İlk harf (pozisyon)
  char dur2 = command.charAt(2);  // İkinci rakam (süre)
  char pos2 = command.charAt(3);  // İkinci harf (pozisyon)
  
  // Pozisyon harflerini büyük harfe çevir
  if (pos1 >= 'a' && pos1 <= 'z') pos1 = pos1 - 'a' + 'A';
  if (pos2 >= 'a' && pos2 <= 'z') pos2 = pos2 - 'a' + 'A';
  
  // Geçerli pozisyonları kontrol et
  if ((pos1 != 'A' && pos1 != 'M' && pos1 != 'F' && pos1 != 'N' && 
       pos1 != 'R' && pos1 != 'G' && pos1 != 'B' && pos1 != 'P' && 
       pos1 != 'Y' && pos1 != 'C') ||
      (pos2 != 'A' && pos2 != 'M' && pos2 != 'F' && pos2 != 'N' && 
       pos2 != 'R' && pos2 != 'G' && pos2 != 'B' && pos2 != 'P' && 
       pos2 != 'Y' && pos2 != 'C')) {
    Serial.println("HATA: Geçersiz pozisyon! Kullanılabilir: A,M,F,N,R,G,B,P,Y,C");
    return;
  }
  
  if (!isdigit(dur1) || !isdigit(dur2)) {
    Serial.println("HATA: Süre değerleri rakam olmalı! (0-9)");
    return;
  }
  
  // Yeni RHRH sistemini başlat
  current_rhrh = String(dur1) + String(pos1) + String(dur2) + String(pos2);
  current_rhrh_encoded = encodeRHRH(dur1, pos1, dur2, pos2);
  
  rhrh_phase1_duration = (dur1 - '0') * 1000; // milisaniye
  rhrh_phase2_duration = (dur2 - '0') * 1000; // milisaniye
  rhrh_total_duration = rhrh_phase1_duration + rhrh_phase2_duration;
  rhrh_phase1_pos = pos1;
  rhrh_phase2_pos = pos2;
  
  rhrh_start_time = millis();
  rhrh_active = true;
  rhrh_phase1_active = (rhrh_phase1_duration > 0);
  rhrh_phase2_active = false; // İkinci faz sonra başlayacak
  
  // İlk fazı başlat
  if (rhrh_phase1_active) {
    int servo1_angle = getServoAngle(pos1, 1); // 1. servo için açı
    int servo2_angle = getServoAngle(pos1, 2); // 2. servo için açı
    
    setServoPosition(SERVO1_CHANNEL, servo1_angle);
    setServoPosition(SERVO2_CHANNEL, servo2_angle);
    
    Serial.printf("RHRH Başladı: %s\n", current_rhrh.c_str());
    Serial.printf("Faz 1: %c pozisyonu (%d°,%d°) - %ds\n", pos1, servo1_angle, servo2_angle, (dur1 - '0'));
    Serial.printf("Faz 2: %c pozisyonu - %ds (bekliyor)\n", pos2, (dur2 - '0'));
  } else {
    // İlk faz 0 saniye ise direkt ikinci faza geç
    rhrh_phase1_active = false;
    rhrh_phase2_active = (rhrh_phase2_duration > 0);
    
    if (rhrh_phase2_active) {
      rhrh_phase2_start_time = millis(); // Faz 2 için başlangıç zamanı
      
      int servo1_angle = getServoAngle(pos2, 1);
      int servo2_angle = getServoAngle(pos2, 2);
      
      setServoPosition(SERVO1_CHANNEL, servo1_angle);
      setServoPosition(SERVO2_CHANNEL, servo2_angle);
      
      Serial.printf("RHRH Başladı (Faz1=0s): %s\n", current_rhrh.c_str());
      Serial.printf("Faz 2: %c pozisyonu (%d°,%d°) - %ds\n", pos2, servo1_angle, servo2_angle, (dur2 - '0'));
    }
  }
}

void updateServoControl() {
  if (!rhrh_active) return; // RHRH aktif değilse çık
  
  unsigned long currentTime = millis();
  unsigned long elapsed_time = currentTime - rhrh_start_time;
  
  // Debug: Her 500ms'de bir durum yazdır
  static unsigned long lastDebugTime = 0;
  if (currentTime - lastDebugTime >= 500) {
    lastDebugTime = currentTime;
    Serial.printf("RHRH Debug - Elapsed: %lums, Phase1: %s, Phase2: %s\n", 
                  elapsed_time, 
                  rhrh_phase1_active ? "AKTIF" : "PASIF",
                  rhrh_phase2_active ? "AKTIF" : "PASIF");
  }
  
  // Faz 1 aktifse ve süresi dolmuşsa
  if (rhrh_phase1_active && elapsed_time >= rhrh_phase1_duration) {
    rhrh_phase1_active = false;
    Serial.printf("RHRH Faz 1 tamamlandı (%lums geçti)\n", elapsed_time);
    
    // Faz 2'yi başlat
    if (rhrh_phase2_duration > 0) {
      rhrh_phase2_active = true;
      rhrh_phase2_start_time = millis(); // Faz 2 için yeni başlangıç zamanı
      
      int servo1_angle = getServoAngle(rhrh_phase2_pos, 1);
      int servo2_angle = getServoAngle(rhrh_phase2_pos, 2);
      
      setServoPosition(SERVO1_CHANNEL, servo1_angle);
      setServoPosition(SERVO2_CHANNEL, servo2_angle);
      
      Serial.printf("RHRH Faz 2 başladı: %c pozisyonu (%d°,%d°) - %dms süre\n", 
                    rhrh_phase2_pos, servo1_angle, servo2_angle, rhrh_phase2_duration);
    } else {
      // Faz 2 süresi 0 ise direkt defaulta dön
      rhrh_active = false;
      setServoPosition(SERVO1_CHANNEL, ANGLE_A);
      setServoPosition(SERVO2_CHANNEL, ANGLE_A);
      current_rhrh = "0A0A";
      current_rhrh_encoded = encodeRHRH('0', 'A', '0', 'A');
      Serial.println("RHRH tamamlandı - Default pozisyona döndü (0A0A)");
    }
  }
  
  // Faz 2 aktifse ve süresi dolmuşsa (kendi başlangıç zamanına göre)
  if (rhrh_phase2_active) {
    unsigned long phase2_elapsed = currentTime - rhrh_phase2_start_time;
    if (phase2_elapsed >= rhrh_phase2_duration) {
      rhrh_phase2_active = false;
      rhrh_active = false;
      
      Serial.printf("RHRH Faz 2 tamamlandı (%lums geçti)\n", phase2_elapsed);
      
      // Default pozisyona dön
      setServoPosition(SERVO1_CHANNEL, ANGLE_A);
      setServoPosition(SERVO2_CHANNEL, ANGLE_A);
      current_rhrh = "0A0A";
      current_rhrh_encoded = encodeRHRH('0', 'A', '0', 'A');
      
      Serial.println("RHRH tamamlandı - Default pozisyona döndü (0A0A)");
    }
  }
}

void setServoPosition(int channel, int angle) {
  int dutyCycle = angleToDutyCycle(angle);
  ledcWrite(channel, dutyCycle);
}

int angleToDutyCycle(int angle) {
  angle = constrain(angle, 0, 180);
  int pulseWidth = map(angle, 0, 180, 500, 2500);
  const int PWM_PERIOD = 20000;
  int dutyCycle = (pulseWidth * 65535) / PWM_PERIOD;
  return dutyCycle;
}

int getServoAngle(char position, int servo_number) {
  // servo_number: 1=Servo1, 2=Servo2
  switch(position) {
    case 'A': return ANGLE_A;        // A: servo1=40°, servo2=40° (default)
    case 'M': return ANGLE_M;        // M: servo1=80°, servo2=80°
    case 'F': return ANGLE_F;        // F: servo1=0°, servo2=0°
    case 'N': return ANGLE_N;        // N: servo1=120°, servo2=120°
    case 'R': 
      return (servo_number == 1) ? ANGLE_R : ANGLE_R2;  // R: servo1=40°, servo2=80°
    case 'G': 
      return (servo_number == 1) ? ANGLE_G : ANGLE_G2;  // G: servo1=0°, servo2=40°
    case 'B': 
      return (servo_number == 1) ? ANGLE_B : ANGLE_B2;  // B: servo1=40°, servo2=120°
    case 'P': 
      return (servo_number == 1) ? ANGLE_P : ANGLE_P2;  // P: servo1=80°, servo2=120°
    case 'Y': 
      return (servo_number == 1) ? ANGLE_Y : ANGLE_Y2;  // Y: servo1=0°, servo2=80°
    case 'C': 
      return (servo_number == 1) ? ANGLE_C : ANGLE_C2;  // C: servo1=0°, servo2=120°
    default: 
      return ANGLE_A; // Default pozisyon
  }
}

int getServoPosition(char position) {
  switch(position) {
    case 'A': return ANGLE_A;
    case 'B':return ANGLE_B;
    case 'R': return ANGLE_R;
    case 'G': return ANGLE_G;
    default: return ANGLE_A;
  }
}

void updateSensorStatus() {
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck >= 10000) { // 10 saniyede bir kontrol
    lastCheck = millis();
    
    if (!mpu_status && mpu.begin(0x69)) {
      mpu_status = true;
      Serial.println("MPU6050 yeniden bağlandı");
    }
    
    if (!bmp_status && bmp.begin(0x76)) {
      bmp_status = true;
      Serial.println("BMP280 yeniden bağlandı");
    }
    
    if (!rtc_status && rtc.begin()) {
      rtc_status = true;
      Serial.println("RTC yeniden bağlandı");
    }
    
    if (!sd_status && SD.begin()) {
      sd_status = true;
      Serial.println("SD kart yeniden bağlandı");
    }
    
    // LoRa durumunu da kontrol et
    if (!lora_status) {
      Serial.println("LoRa yeniden bağlanmaya çalışılıyor...");
      
      // M0 ve M1 pinlerini yeniden ayarla
      digitalWrite(LORA_M0, LOW);
      digitalWrite(LORA_M1, LOW);
      delay(100);
      
      ResponseStructContainer c = E22.getModuleInformation();
      if (c.status.code == 1) {
        lora_status = true;
        Serial.println("LoRa yeniden bağlandı");
      } else {
        Serial.printf("LoRa bağlanma başarısız - Code: %d\n", c.status.code);
      }
      c.close();
    }
  }
}

void saveToSD(TelemetryPacket data) {
  if (!sd_status) return;
  
  String csvLine = String(data.paket_sayisi) + "," +
                   String(data.uydu_statusu) + "," +
                   String(data.hata_kodu) + "," +
                   String(data.gonderme_saati) + "," +
                   String(data.basinc1, 1) + "," +
                   String(data.basinc2, 1) + "," +
                   String(data.yukseklik1, 1) + "," +
                   String(data.yukseklik2, 1) + "," +
                   String(data.irtifa_farki, 1) + "," +
                   String(data.inis_hizi, 1) + "," +
                   String(data.sicaklik / 10.0, 1) + "," +
                   String(data.pil_gerilimi / 100.0, 3) + "," +
                   String(data.gps1_latitude, 6) + "," +
                   String(data.gps1_longitude, 6) + "," +
                   String(data.gps1_altitude, 1) + "," +
                   String(data.pitch / 10.0, 1) + "," +
                   String(data.roll / 10.0, 1) + "," +
                   String(data.yaw / 10.0, 1) + "," +
                   current_rhrh + "," +
                   String(data.iot_s1_data) + "," +
                   String(data.iot_s2_data) + "," +
                   String(data.takim_no) + "," +
                   String(bmp_status ? "1" : "0") +
                   String(mpu_status ? "1" : "0") +
                   String(rtc_status ? "1" : "0") +
                   String(gps_status ? "1" : "0") +
                   String(sd_status ? "1" : "0") +
                   String(adc_status ? "1" : "0") + "\n";
  
  if (!appendFile(SD, "/sensor_data.csv", csvLine.c_str())) {
    sd_status = false;
  }
}

bool appendFile(fs::FS &fs, const char *path, const char *message) {
  File file = fs.open(path, FILE_APPEND);
  if (!file) return false;
  
  size_t bytesWritten = file.print(message);
  file.close();
  return bytesWritten > 0;
}

void writeFile(fs::FS &fs, const char *path, const char *message) {
  File file = fs.open(path, FILE_WRITE);
  if (file) {
    file.print(message);
    file.close();
  }
}

void setRTCTime(String command) {
  command = command.substring(4);
  
  int values[6];
  int index = 0;
  
  while (command.length() > 0 && index < 6) {
    int commaIndex = command.indexOf(',');
    if (commaIndex == -1) {
      values[index] = command.toInt();
      break;
    }
    values[index] = command.substring(0, commaIndex).toInt();
    command = command.substring(commaIndex + 1);
    index++;
  }
  
  if (index == 5) {
    rtc.adjust(DateTime(values[0], values[1], values[2], values[3], values[4], values[5]));
    Serial.printf("RTC zamanı ayarlandı: %02d/%02d/%04d %02d:%02d:%02d\n", 
                  values[2], values[1], values[0], values[3], values[4], values[5]);
    rtc_status = true;
  } else {
    Serial.println("Hatalı format! Kullanım: SET:2025,8,3,14,30,0");
  }
}

float calculateAltitude(float P, float T_celsius) {
  // Referans basınç yerine sistem başlangıcındaki basıncı kullan
  const float L = 0.0065;
  const float R = 287.05;
  const float g = 9.80665;
  
  float T = T_celsius + 273.15;
  float h = (T / L) * (1 - pow(P / reference_pressure, (R * L) / g));
  return h;
}

void printLoRaModuleInfo() {
  ResponseStructContainer c = E22.getModuleInformation();
  ModuleInformation moduleInfo = *(ModuleInformation*) c.data;
  Serial.printf("LoRa Model: %d\n", moduleInfo.model);
  Serial.printf("LoRa Version: %d\n", moduleInfo.version);
  Serial.printf("LoRa Features: %d\n", moduleInfo.features);
  Serial.printf("Status: %s\n", c.status.getResponseDescription().c_str());
  c.close();
}

void printLoRaConfig() {
  ResponseStructContainer c = E22.getConfiguration();
  Configuration configuration = *(Configuration*) c.data;
  Serial.printf("LoRa Adres: 0x%02X:0x%02X\n", configuration.ADDH, configuration.ADDL);
  Serial.printf("LoRa Kanal: %d\n", configuration.CHAN);
  Serial.printf("UART Baud: %d\n", configuration.SPED.uartBaudRate);
  Serial.printf("Air Data Rate: %d\n", configuration.SPED.airDataRate);
  Serial.printf("Status: %s\n", c.status.getResponseDescription().c_str());
  c.close();
}


void configureLoRa() {
  Serial.println("\n=== LoRa Konfigürasyonu ===");
  
  // Konfigürasyon yapısını oluştur
  Configuration configuration;
  
  // *** 1. ADDRESS AYARLARI ***
  configuration.ADDH = 0x00;        // Yüksek adres byte'ı (0x00-0xFF)
  configuration.ADDL = 0x0A;        // Düşük adres byte'ı (0x00-0xFF)
  configuration.NETID = 0x00;       // Network ID (0x00-0xFF)
  
  // *** 2. CHANNEL AYARI ***
  configuration.CHAN = 10;          // Kanal numarası (0-80)
                                    // Frekans = 410.125 + CHAN MHz
                                    // Kanal 23 = 433.125 MHz
  
  // *** 3. AIR DATA RATE (Hava Veri Hızı) ***
  configuration.SPED.airDataRate = AIR_DATA_RATE_011_48;  // 4.8kbps
  // Diğer seçenekler:
  // AIR_DATA_RATE_000_03  -> 0.3kbps  (en uzun menzil)
  // AIR_DATA_RATE_001_12  -> 1.2kbps
  // AIR_DATA_RATE_010_24  -> 2.4kbps  (varsayılan)
  // AIR_DATA_RATE_011_48  -> 4.8kbps  *** SEÇİLDİ ***
  // AIR_DATA_RATE_100_96  -> 9.6kbps
  // AIR_DATA_RATE_101_192 -> 19.2kbps
  // AIR_DATA_RATE_110_384 -> 38.4kbps
  // AIR_DATA_RATE_111_625 -> 62.5kbps (en kısa menzil, en hızlı)
  
  // *** 4. PACKET SIZE (Paket Boyutu) ***
  configuration.OPTION.subPacketSetting = SPS_128_01;  // 240 byte
  // Diğer seçenekler:
  // SPS_240_00 -> 240 bytes (varsayılan)
  // SPS_128_01 -> 128 bytes
  // SPS_064_10 -> 64 bytes
  // SPS_032_11 -> 32 bytes
  
  // *** 5. TRANSMISSION MODE (İletim Modu) ***
  configuration.TRANSMISSION_MODE.fixedTransmission = FT_FIXED_TRANSMISSION;  // Sabit iletim
  // Seçenekler:
  // FT_TRANSPARENT_TRANSMISSION -> Şeffaf iletim (varsayılan)
  // FT_FIXED_TRANSMISSION       -> Sabit iletim (adres ve kanal belirtilerek) *** SEÇİLDİ ***
  
  // Diğer gerekli parametreler (değiştirmeyin)
  configuration.SPED.uartBaudRate = UART_BPS_9600;
  configuration.SPED.uartParity = MODE_00_8N1;
  configuration.OPTION.transmissionPower = POWER_22;
  configuration.OPTION.RSSIAmbientNoise = RSSI_AMBIENT_NOISE_DISABLED;
  configuration.TRANSMISSION_MODE.enableRSSI = RSSI_DISABLED;
  configuration.TRANSMISSION_MODE.enableRepeater = REPEATER_DISABLED;
  configuration.TRANSMISSION_MODE.enableLBT = LBT_DISABLED;
  configuration.TRANSMISSION_MODE.WORTransceiverControl = WOR_RECEIVER;
  configuration.TRANSMISSION_MODE.WORPeriod = WOR_2000_011;
  
  // Konfigürasyonu LoRa modülüne gönder
  ResponseStatus rs = E22.setConfiguration(configuration, WRITE_CFG_PWR_DWN_SAVE);
  printConfig();
}

void printConfig() {
  ResponseStructContainer c = E22.getConfiguration();
  Configuration configuration = *(Configuration*) c.data;
  Serial.print("Adres: "); Serial.print(configuration.ADDH, HEX); Serial.print(" "); Serial.println(configuration.ADDL, HEX);
  Serial.print("Kanal: "); Serial.println(configuration.CHAN);
  Serial.print("UART Baudrate: "); Serial.println(configuration.SPED.uartBaudRate);
  Serial.print("Air Data Rate: "); Serial.println(configuration.SPED.airDataRate);
  Serial.print("Status: "); Serial.println(c.status.getResponseDescription());
  c.close();
}

void loadLastStateFromSD() {
  Serial.print("SD karttan en son durum yükleniyor... ");
  
  if (!SD.exists("/sensor_data.csv")) {
    Serial.println("CSV dosyası bulunamadı - varsayılan değerler kullanılıyor");
    return;
  }
  
  File file = SD.open("/sensor_data.csv", FILE_READ);
  if (!file) {
    Serial.println("Dosya açılamadı - varsayılan değerler kullanılıyor");
    return;
  }
  
  String lastLine = "";
  String currentLine = "";
  
  // Dosyayı satır satır okuyarak en son satırı bul
  while (file.available()) {
    char c = file.read();
    if (c == '\n') {
      if (currentLine.length() > 0 && !currentLine.startsWith("Packet_Number")) {
        lastLine = currentLine;
      }
      currentLine = "";
    } else if (c != '\r') {
      currentLine += c;
    }
  }
  
  // Son satır newline ile bitmiyorsa
  if (currentLine.length() > 0 && !currentLine.startsWith("Packet_Number")) {
    lastLine = currentLine;
  }
  
  file.close();
  
  if (lastLine.length() == 0) {
    Serial.println("Geçerli veri bulunamadı - varsayılan değerler kullanılıyor");
    return;
  }
  
  // CSV satırını parse et
  // Format: Packet_Number,Uydu_Statusu,Hata_Kodu,Gonderme_Saati,Pressure1,Pressure2,Yukseklik1,Yukseklik2,Irtifa_Farki,Inis_Hizi,Temperature,Pil_Gerilimi,GPS1_Latitude,GPS1_Longitude,GPS1_Altitude,Pitch,Roll,Yaw,RHRH,IoT_S1_Data,IoT_S2_Data,Takim_No,Sensor_Status
  
  int commaIndex = 0;
  int fieldIndex = 0;
  String currentField = "";
  
  for (int i = 0; i <= lastLine.length(); i++) {
    char c = (i < lastLine.length()) ? lastLine.charAt(i) : ','; // Son field için virgül ekle
    
    if (c == ',' || i == lastLine.length()) {
      switch (fieldIndex) {
        case 0: // Packet_Number
          {
            uint16_t lastPacketNumber = currentField.toInt();
            if (lastPacketNumber > 0) {
              paket_sayisi_lora2 = lastPacketNumber + 1;
              packet_counter = lastPacketNumber + 1;
              Serial.printf("Paket sayısı: %d (önceki: %d) ", paket_sayisi_lora2, lastPacketNumber);
            }
          }
          break;
        case 1: // Uydu_Statusu
          {
            uint8_t lastStatus = currentField.toInt();
            if (lastStatus <= 5 || lastStatus == 255) {
              previous_uydu_status = lastStatus;
              Serial.printf("Status: %d ", previous_uydu_status);
            }
          }
          break;
        case 18: // RHRH
          {
            if (currentField.length() == 4) {
              current_rhrh = currentField;
              current_rhrh.toUpperCase(); // Büyük harfe çevir
              
              // RHRH'yi encode et
              char r1 = current_rhrh.charAt(0);
              char h1 = current_rhrh.charAt(1);
              char r2 = current_rhrh.charAt(2);
              char h2 = current_rhrh.charAt(3);
              current_rhrh_encoded = encodeRHRH(r1, h1, r2, h2);
              
              Serial.printf("RHRH: %s ", current_rhrh.c_str());
            }
          }
          break;
      }
      currentField = "";
      fieldIndex++;
    } else {
      currentField += c;
    }
  }
  
  Serial.println("- Yüklendi!");
}

// DC Motor fonksiyonları
void initDCMotor() {
  Serial.println("DC Motor pinleri başlatılıyor...");
  
  pinMode(DC_MOTOR_AYRILMA_PIN, OUTPUT);
  pinMode(DC_MOTOR_BIRLESME_PIN, OUTPUT);
  
  // Başlangıçta tüm pinleri LOW yap
  digitalWrite(DC_MOTOR_AYRILMA_PIN, LOW);
  digitalWrite(DC_MOTOR_BIRLESME_PIN, LOW);
  
  Serial.printf("DC Motor - Ayrılma: GPIO%d, Birleşme: GPIO%d\n", DC_MOTOR_AYRILMA_PIN, DC_MOTOR_BIRLESME_PIN);
  Serial.println("DC Motor pinleri OK");
}

void dcMotorAyrilma() {
  if (!dc_motor_ayrilma_active && !dc_motor_birlesme_active) {
    Serial.println("DC Motor: Ayrılma yönünde başlatılıyor... (10 saniye)");
    digitalWrite(DC_MOTOR_BIRLESME_PIN, LOW);   // Önce diğer yönü kapat
    digitalWrite(DC_MOTOR_AYRILMA_PIN, HIGH);   // Ayrılma yönünde çalıştır
    
    dc_motor_start_time = millis();
    dc_motor_ayrilma_active = true;
    dc_motor_birlesme_active = false;
  } else {
    Serial.println("DC Motor: Zaten çalışıyor - yeni komut iptal edildi");
  }
}

void dcMotorBirlesme() {
  if (!dc_motor_ayrilma_active && !dc_motor_birlesme_active) {
    Serial.println("DC Motor: Birleşme yönünde başlatılıyor... (1 saniye)");
    digitalWrite(DC_MOTOR_AYRILMA_PIN, LOW);    // Önce diğer yönü kapat
    digitalWrite(DC_MOTOR_BIRLESME_PIN, HIGH);  // Birleşme yönünde çalıştır
    
    dc_motor_start_time = millis();
    dc_motor_birlesme_active = true;
    dc_motor_ayrilma_active = false;
  } else {
    Serial.println("DC Motor: Zaten çalışıyor - yeni komut iptal edildi");
  }
}

void dcMotorStop() {
  Serial.println("DC Motor: Durduruldu");
  digitalWrite(DC_MOTOR_AYRILMA_PIN, LOW);
  digitalWrite(DC_MOTOR_BIRLESME_PIN, LOW);
  dc_motor_ayrilma_active = false;
  dc_motor_birlesme_active = false;
}

void updateDCMotorControl() {
  unsigned long currentTime = millis();
  
  // Ayrılma motoru kontrolü (10 saniye)
  if (dc_motor_ayrilma_active) {
    unsigned long elapsed = currentTime - dc_motor_start_time;
    if (elapsed >= DC_MOTOR_AYRILMA_DURATION) {
      Serial.printf("DC Motor: Ayrılma tamamlandı (%lu ms)\n", elapsed);
      dcMotorStop();
    }
  }
  
  // Birleşme motoru kontrolü (1 saniye)
  if (dc_motor_birlesme_active) {
    unsigned long elapsed = currentTime - dc_motor_start_time;
    if (elapsed >= DC_MOTOR_BIRLESME_DURATION) {
      Serial.printf("DC Motor: Birleşme tamamlandı (%lu ms)\n", elapsed);
      dcMotorStop();
    }
  }
}

// Buzzer kontrol fonksiyonu (uydu_status 5 için millis() tabanlı)
void updateBuzzerControl() {
  // Eğer uydu_status 5 ise buzzer'ı on/off yap
  if (current_uydu_status == 5) {
    // 500ms aralıklarla buzzer'ı toggle et
    if (millis() - buzzer_last_toggle >= buzzer_interval) {
      buzzer_state = !buzzer_state;
      digitalWrite(BUZZER_PIN, buzzer_state ? HIGH : LOW);
      buzzer_last_toggle = millis();
      
      // Debug için (opsiyonel - istenirse kaldırılabilir)
      if (buzzer_state) {
        Serial.println("BUZZER: ON (Uydu Status 5)");
      }
    }
  } else {
    // Uydu status 5 değilse buzzer'ı kapat
    if (buzzer_state) {
      digitalWrite(BUZZER_PIN, LOW);
      buzzer_state = false;
      Serial.println("BUZZER: OFF (Uydu Status ≠ 5)");
    }
  }
}