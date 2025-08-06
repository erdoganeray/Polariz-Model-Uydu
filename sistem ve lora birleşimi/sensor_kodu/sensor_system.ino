/*
 * ESP32 Sensör Veri Toplama Sistemi
 * 
 * Sensörler:
 * - MPU6050: I2C üzerinden pitch, roll, yaw (GPIO21-SDA, GPIO22-SCL)
 * - BMP280: I2C üzerinden basınç ve sıcaklık (GPIO21-SDA, GPIO22-SCL)
 * - GY-GPS6MV2: UART1 üzerinden GPS verileri (GPIO14-RX, GPIO15-TX)
 * - DS3231 RTC: I2C üzerinden zaman bilgisi (GPIO21-SDA, GPIO22-SCL)
 * - SD Kart: SPI üzerinden veri kaydetme (GPIO18-SCK, GPIO19-MISO, GPIO23-MOSI, GPIO5-CS)
 * - ADC: Gerilim ölçümü
 * - SG90 Servo Motorlar: PWM ile pozisyon kontrolü (GPIO33, GPIO25, GPIO26)
 * 
 * Komut formatları USB Serial üzerinden:
 * - "GET_DATA" -> Manuel veri toplama
 * - "RHRH:m2k6" -> Servo pozisyon kontrol (m=65° 2sn, k=115° 6sn)
 * - "AYRIL" -> Servo3'ü 0°'den 90°'ye döndür (ayrılma)
 * - "BIRLES" -> Servo3'ü 90°'den 0°'ye döndür (birleşme)
 * - "STATUS" -> Sistem durumu
 * - "SERVO_TEST" -> Servo test hareketi
 */

// Kütüphaneler
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <TinyGPS++.h>
#include <RTClib.h>
#include <SD.h>
#include <FS.h>
// MG90S servo için PWM kullanacağız (ESP32Servo yerine)

// GPS UART1 Pin tanımlamaları
#define GPS_RX 14
#define GPS_TX 15
#define GPS_BAUD 9600
HardwareSerial gpsSerial(1);

// SD Kart SPI pin tanımlamaları (ESP32 Default SPI)
#define SD_SCK 18
#define SD_MISO 19
#define SD_MOSI 23
#define SD_CS 5

// ADC pin tanımlaması
#define pil_gerilimi_PIN A0

// Servo motor pin tanımlamaları (MG90S için PWM pinleri)
#define SERVO1_PIN 33
#define SERVO2_PIN 25
#define SERVO3_PIN 26

// MG90S servo PWM kanalları (pozisyon kontrolü için)
#define SERVO1_CHANNEL 0
#define SERVO2_CHANNEL 1
#define SERVO3_CHANNEL 2
#define SERVO_FREQ 50      // 50Hz PWM frekansı
#define SERVO_RESOLUTION 16 // 16-bit çözünürlük

// SG90 servo pozisyon kontrol değerleri (pulse width in microseconds)
#define SERVO_0_DEG 500     // 0° pozisyon (m)
#define SERVO_50_DEG 1250   // 50° pozisyon (b)
#define SERVO_100_DEG 1750  // 100° pozisyon (k)
#define SERVO_150_DEG 2250  // 150° pozisyon (y)

// Servo pozisyon açı değerleri (kolay değiştirim için)
#define ANGLE_B 50   // b pozisyonu (varsayılan)
#define ANGLE_M 0    // m pozisyonu
#define ANGLE_K 100  // k pozisyonu
#define ANGLE_Y 150 // y pozisyonu

// Sensör nesneleri
Adafruit_MPU6050 mpu;
Adafruit_BMP280 bmp;
TinyGPSPlus gps;
RTC_DS3231 rtc;

// SG90 Servo motor PWM kontrol fonksiyonları (180° pozisyon kontrolü)
void setServoPosition(int channel, int angle);
int angleToDutyCycle(int angle);
void moveServoToNeutral(int channel);

// RHRH servo kontrol değişkenleri
String current_rhrh = "b0b0"; // Default değer (b=ANGLE_B°)
unsigned long servo1_start_time = 0;
unsigned long servo2_start_time = 0;
int servo1_duration = 0;
int servo2_duration = 0;
bool servo1_active = false;
bool servo2_active = false;

// Servo3 AYRIL komutu için değişkenler
bool servo3_separated = false; // Ayrılma durumu

// Veri yapısı
struct __attribute__((packed)) TelemetryPacket {
  // Paket numarası
  uint16_t packet_number;

  // Uydu statüsü (İlerleyen zamanlarda kullanılacak)
  char  uydu_statusu;      // 1-6 arası durum kodu

  // Hata Kodu (İlerleyen zamanlarda kullanılacak)
  char hata_kodu;          // 5 haneli hata kodu
  
  // RTC verileri
  String gönderme_saati;  //Daha sonra değişecek. //gönderme_saati

  //Görev yükünün basıncı
  float pressure1; //pressure1 olarak değiştir 
  
  //Taşıyıcı basıncı
  float pressure2; //Daha sonra kullanılacak

  float yukseklik1;        // Birincil yükseklik (m).
  float yukseklik2;        // İkincil yükseklik (m). /Daha sonra kullanılacak
  float irtifa_farki;      // Yükseklik farkı (m). /Daha sonra kullanılacak
  float inis_hizi;         // Model Uydunun yere iniş hızı.

  float temperature; //Görev yükündeki sıcaklık verisi

  // ADC verisi
  float pil_gerilimi;  // Pil gerilimi

  // GPS verileri
  double gps1_latitude;
  double gps1_longitude;
  float gps1_altitude;
  //bool gps_valid; // GPS verilerinin geçerli olup olmadığını gösterir

  // MPU6050 verileri
  float pitch;
  float roll;
  float yaw;
  
  //RHRH
  String rhrh; // RHRH servo kontrol komutu (örn: m2k6)

  float iot_s1_data;       // IoT sensor 1 verisi /Daha sonra kullanılacak
  float iot_s2_data;       // IoT sensor 2 verisi /Daha sonra kullanılacak

  //Takım numarası
  uint32_t takim_no; // Takım numarası (626541)

  bool gps_valid;
  // Sensör durumları (BMP280, MPU6050, RTC, GPS, SD, ADC)
  String sensor_status; // Daha sonra silinecek.
};

// Sensör durumları
bool bmp_status = false;
bool mpu_status = false;
bool rtc_status = false;
bool gps_status = false;
bool sd_status = false;
bool adc_status = false;

// Paket numarası
uint16_t packet_counter = 1;

// İniş hızı hesaplama için değişkenler
float previous_gps1_altitude = 0.0;
unsigned long previous_time = 0;
bool first_gps1_altitude_reading = true;

String createDataPacket(TelemetryPacket data);
void saveToSD(TelemetryPacket data);
void updateSensorStatus();
void writeFile(fs::FS &fs, const char *path, const char *message);
bool appendFile(fs::FS &fs, const char *path, const char *message);
void setRTCTime(String command);
void loadLastPacketNumber();
float calculategps1_altitude(float P, float T_celsius);
void collectAndSendData();
void initSDCard();
void initSensors();
void initServos();
void processRHRHCommand(String command);
int getServoPosition(char position);
void updateServoControl();
void processSerialCommands();


void setup() {
  Serial.begin(115200);
  delay(1000);
  
  // GPS UART başlatma
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  
  Serial.println("ESP32 Sensör Sistemi Başlatılıyor...");
  Serial.println("====================================");
  
  // I2C başlatma
  Wire.begin();
  
  // Sensörleri başlatma
  initSensors();
  
  // Servo motorları başlatma
  initServos();
  
  // SD kart başlatma
  initSDCard();
  
  Serial.println("Sistem hazır! Otomatik veri gönderimi (1Hz)");
}

void loop() {
  // GPS verilerini sürekli güncelle
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }
  
  // Serial komutları işle
  processSerialCommands();
  
  // Servo kontrollerini güncelle
  updateServoControl();
  
  // Her saniye otomatik veri topla ve gönder
  static unsigned long lastDataCollection = 0;
  if (millis() - lastDataCollection >= 1000) {
    lastDataCollection = millis();
    collectAndSendData();
  }
  
  delay(100);
}

void initSensors() {
  // MPU6050 başlatma
  Serial.println("MPU6050 başlatılıyor...");
  delay(100);
  
  if (!mpu.begin(0x69)) {
    Serial.println("HATA: MPU6050 bulunamadı!");
    mpu_status = false;
  } else {
    Serial.println("MPU6050 başarıyla başlatıldı");
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    delay(100);
    
    // Test okuma yaparak çalışıp çalışmadığını kontrol et
    sensors_event_t a, g, temp;
    if (mpu.getEvent(&a, &g, &temp)) {
      mpu_status = true;
      Serial.println("MPU6050 test okuması başarılı");
    } else {
      mpu_status = false;
      Serial.println("MPU6050 test okuması başarısız");
    }
  }
  
  // BMP280 başlatma
  if (!bmp.begin(0x76)) {
    Serial.println("HATA: BMP280 bulunamadı!");
    bmp_status = false;
  } else {
    Serial.println("BMP280 başarıyla başlatıldı");
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_500);
    bmp_status = true;
  }
  
  // RTC başlatma
  Serial.println("RTC başlatılıyor...");
  delay(100);
  
  if (!rtc.begin()) {
    Serial.println("HATA: DS3231 RTC bulunamadı! (0x68)");
    
    // Alternatif adres dene
    Wire.beginTransmission(0x57);
    if (Wire.endTransmission() == 0) {
      Serial.println("RTC 0x57 adresinde bulundu, tekrar deneniyor...");
      delay(100);
      if (rtc.begin()) {
        rtc_status = true;
        Serial.println("DS3231 RTC başarıyla başlatıldı (0x57)");
      } else {
        rtc_status = false;
      }
    } else {
      rtc_status = false;
      Serial.println("RTC hiçbir adreste bulunamadı");
    }
  } else {
    Serial.println("DS3231 RTC başarıyla başlatıldı (0x68)");
    rtc_status = true;
    
    // RTC çalışıyor mu test et
    DateTime now = rtc.now();
    if (now.year() < 2020) {
      Serial.println("RTC zaman geçersiz - ayarlanması gerekiyor");
      Serial.println("Komut: SET:2025,7,18,14,30,0");
    } else {
      Serial.printf("RTC Zamanı: %d-%d-%d %d:%d:%d\n", 
                    now.year(), now.month(), now.day(),
                    now.hour(), now.minute(), now.second());
    }
  }
  
  Serial.println("GPS modülü başlatıldı (UART1)");
}

void initSDCard() {
  Serial.println("SD kart başlatılıyor...");
  delay(100);
  
  Serial.printf("SPI Pinler - SCK:%d, MISO:%d, MOSI:%d, CS:%d\n", SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  
  // Basit SD kart başlatma (çalışan kod)
  if (!SD.begin()) {
    Serial.println("HATA: SD kart başlatılamadı!");
    Serial.println("Kontrol edilecekler:");
    Serial.println("- SD kart takılı mı?");
    Serial.println("- SD kart FAT32 formatında mı?");
    Serial.println("- Pin bağlantıları doğru mu?");
    Serial.println("- SD kart kapasitesi 32GB'den küçük mü?");
    Serial.printf("- CS: %d, MOSI: %d, MISO: %d, SCK: %d\n", SD_CS, SD_MOSI, SD_MISO, SD_SCK);
    Serial.println("PIN KONFIGÜRASYONU:");
    Serial.println("SD CS   -> GPIO5");
    Serial.println("SD MOSI -> GPIO23"); 
    Serial.println("SD MISO -> GPIO19");
    Serial.println("SD SCK  -> GPIO18");
    sd_status = false;
    return;
  }
  
  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("HATA: SD kart bulunamadı!");
    Serial.println("SD kart yuvaya düzgün takıldığından emin olun");
    sd_status = false;
    return;
  }
  
  Serial.println("SD kart başarıyla başlatıldı");
  Serial.print("SD kart tipi: ");
  if (cardType == CARD_MMC) {
    Serial.println("MMC");
  } else if (cardType == CARD_SD) {
    Serial.println("SDSC");
  } else if (cardType == CARD_SDHC) {
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }
  
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD kart boyutu: %lluMB\n", cardSize);
  
  sd_status = true;
  
  // Log dosyası başlığı oluştur
  if (!SD.exists("/sensor_data.csv")) {
    Serial.println("CSV dosyası oluşturuluyor...");
    writeFile(SD, "/sensor_data.csv", "Packet_Number,Uydu_Statusu,Hata_Kodu,gönderme_saati,Pressure1,Pressure2,Yukseklik1,Yukseklik2,Irtifa_Farki,Inis_Hizi,Temperature,Pil_Gerilimi,gps1_latitude,gps1_longitude,gps1_altitude,Pitch,Roll,Yaw,RHRH,IoT_S1_Data,IoT_S2_Data,Takim_No,Sensor_Status\n");
    packet_counter = 1;
  } else {
    Serial.println("CSV dosyası mevcut");
    // Son paket numarasını oku
    loadLastPacketNumber();
  }
}

void collectAndSendData() {
  TelemetryPacket data;
  
  // Paket numarasını ata
  data.packet_number = packet_counter;
  
  // Varsayılan değerleri initialize et
  data.uydu_statusu = 0;
  data.hata_kodu = 0;
  data.pressure2 = 0.0;
  data.yukseklik2 = 0.0;
  data.irtifa_farki = 0.0;
  data.inis_hizi = 0.0;
  data.rhrh = current_rhrh;
  data.iot_s1_data = 0.0;
  data.iot_s2_data = 0.0;
  data.takim_no = 626541;
  
  // Sensör durumlarını güncelle
  updateSensorStatus();
  
  // MPU6050 verilerini oku
  if (mpu_status) {
    sensors_event_t a, g, temp;
    if (mpu.getEvent(&a, &g, &temp)) {
      // Gyro verilerini al (rad/s'den derece/s'ye çevir)
      data.pitch = g.gyro.y * 180.0 / PI;
      data.roll = g.gyro.x * 180.0 / PI;
      data.yaw = g.gyro.z * 180.0 / PI;
      
      // Değerlerin geçerli olup olmadığını kontrol et
      if (isnan(data.pitch) || isnan(data.roll) || isnan(data.yaw)) {
        mpu_status = false;
        data.pitch = 0.0;
        data.roll = 0.0;
        data.yaw = 0.0;
      }
    } else {
      mpu_status = false;
      data.pitch = 0.0;
      data.roll = 0.0;
      data.yaw = 0.0;
    }
  } else {
    data.pitch = 0.0;
    data.roll = 0.0;
    data.yaw = 0.0;
  }
  
  // BMP280 verilerini oku
  if (bmp_status) {
    float pressure1 = bmp.readPressure();
    float temperature = bmp.readTemperature();
    if (!isnan(pressure1) && !isnan(temperature)) {
      data.pressure1 = pressure1;
      data.temperature = temperature;
      
      // Yükseklik hesapla (barometrik formül ile)
      data.yukseklik1 = calculategps1_altitude(pressure1, temperature);
      
      // İniş hızı hesapla
      unsigned long current_time = millis();
      if (!first_gps1_altitude_reading && (current_time - previous_time) > 0) {
        // Yükseklik farkı (m)
        float gps1_altitude_diff = data.yukseklik1 - previous_gps1_altitude;
        // Zaman farkı (saniye)
        float time_diff = (current_time - previous_time) / 1000.0;
        // İniş hızı (m/s) - negatif değer iniş anlamına gelir
        data.inis_hizi = gps1_altitude_diff / time_diff;
        
        // İniş hızını pozitif yapabiliriz (iniş hızının büyüklüğü)
        if (data.inis_hizi < 0) {
          data.inis_hizi = -data.inis_hizi; // Mutlak değer al
        } else {
          data.inis_hizi = 0.0; // Yükseliyorsa iniş hızı 0
        }
      } else {
        data.inis_hizi = 0.0; // İlk okuma veya zaman farkı yok
        first_gps1_altitude_reading = false;
      }
      
      // Bir sonraki hesaplama için değerleri sakla
      previous_gps1_altitude = data.yukseklik1;
      previous_time = current_time;
      
    } else {
      bmp_status = false;
      data.pressure1 = 0.0;
      data.temperature = 0.0;
      data.yukseklik1 = 0.0;
      data.inis_hizi = 0.0;
    }
  } else {
    data.pressure1 = 0.0;
    data.temperature = 0.0;
    data.yukseklik1 = 0.0;
    data.inis_hizi = 0.0;
  }
  
  // GPS verilerini oku
  if (gps.location.isValid()) {
    data.gps1_latitude = gps.location.lat();
    data.gps1_longitude = gps.location.lng();
    data.gps_valid = true;
    gps_status = true;
  } else {
    data.gps1_latitude = 0.0;
    data.gps1_longitude = 0.0;
    data.gps_valid = false;
    gps_status = false;
  }
  
  if (gps.altitude.isValid()) {
    data.gps1_altitude = gps.altitude.meters();
  } else {
    data.gps1_altitude = 0.0;
  }
  
  // RTC zamanını oku
  if (rtc_status) {
    DateTime now = rtc.now();
    if (now.year() > 2020 && now.year() < 2035) {  // Makul tarih aralığı kontrolü
      data.gönderme_saati = String(now.year()) + "-" + 
                       String(now.month()) + "-" + 
                       String(now.day()) + " " + 
                       String(now.hour()) + ":" + 
                       String(now.minute()) + ":" + 
                       String(now.second());
    } else {
      rtc_status = false;
      data.gönderme_saati = "0000-00-00 00:00:00";
      Serial.println("RTC tarih geçersiz, zaman ayarlaması gerekiyor");
    }
  } else {
    data.gönderme_saati = "0000-00-00 00:00:00";
  }
  
  // ADC gerilim oku
  int adcValue = analogRead(pil_gerilimi_PIN);
  if (adcValue >= 0 && adcValue <= 4095) { // Geçerli ADC aralığı
    data.pil_gerilimi = (adcValue * 3.3) / 4096.0; // ESP32 ADC referansı 3.3V, 12-bit
    adc_status = true;
  } else {
    data.pil_gerilimi = 0.0;
    adc_status = false;
  }
  
  // Sensör durumlarını string olarak oluştur (BMP280, MPU6050, RTC, GPS, SD, ADC)
  data.sensor_status = String(bmp_status ? "1" : "0") +
                       String(mpu_status ? "1" : "0") +
                       String(rtc_status ? "1" : "0") +
                       String(gps_status ? "1" : "0") +
                       String(sd_status ? "1" : "0") +
                       String(adc_status ? "1" : "0");
  
  // Veriyi formatla
  String dataPacket = createDataPacket(data);
  
  // SD karta kaydet (mümkünse)
  if (sd_status) {
    saveToSD(data);
  }
  
  // USB Serial üzerinden gönder
  Serial.println("DATA_PACKET_START");
  Serial.println(dataPacket);
  Serial.println("DATA_PACKET_END");
  
  // Paket numarasını artır
  packet_counter++;
}

String createDataPacket(TelemetryPacket data) {
  String packet = "";
  /*
  packet += String(data.packet_number) + ",";
  packet += data.gönderme_saati + ",";
  packet += String(data.pitch, 3) + ",";
  packet += String(data.roll, 3) + ",";
  packet += String(data.yaw, 3) + ",";
  packet += String(data.pressure1, 2) + ",";
  packet += String(data.temperature, 2) + ",";
  packet += String(data.yukseklik1, 2) + ",";
  packet += String(data.gps1_latitude, 6) + ",";
  packet += String(data.gps1_longitude, 6) + ",";
  packet += String(data.gps1_altitude, 2) + ",";
  packet += String(data.gps_valid ? "1" : "0") + ",";
  packet += String(data.pil_gerilimi, 3) + ",";
  packet += data.sensor_status + "\n";
  */
  packet += String(data.packet_number) + ",";
  packet += data.uydu_statusu + ",";
  packet += data.hata_kodu + ",";
  packet += data.gönderme_saati + ",";
  packet += String(data.pressure1, 1) + ",";
  packet += String(data.pressure2, 1) + ",";
  packet += String(data.yukseklik1, 1) + ",";
  packet += String(data.yukseklik2, 1) + ",";
  packet += String(data.irtifa_farki, 1) + ",";
  packet += String(data.inis_hizi, 1) + ",";
  packet += String(data.temperature, 1) + ",";
  packet += String(data.pil_gerilimi, 3) + ",";
  packet += String(data.gps1_latitude, 6) + ",";
  packet += String(data.gps1_longitude, 6) + ",";
  packet += String(data.gps1_altitude, 1) + ",";
  //packet += String(data.gps_valid ? "1" : "0") + ",";
  packet += String(data.pitch, 1) + ",";
  packet += String(data.roll, 1) + ",";
  packet += String(data.yaw, 1) + ",";
  packet += data.rhrh + ",";
  packet += String(data.iot_s1_data, 1) + ",";
  packet += String(data.iot_s2_data, 1) + ",";
  packet += String(data.takim_no) + ",";

  packet += data.sensor_status + "\n";

  
  return packet;
}

void saveToSD(TelemetryPacket data) {
  if (!sd_status) {
    return; // SD kart çalışmıyorsa kaydetmeye çalışma
  }
  
  // Telemetri paketine uygun CSV formatı
  String csvLine = String(data.packet_number) + "," +
                   String(data.uydu_statusu) + "," +
                   String(data.hata_kodu) + "," +
                   data.gönderme_saati + "," +
                   String(data.pressure1, 1) + "," +
                   String(data.pressure2, 1) + "," +
                   String(data.yukseklik1, 1) + "," +
                   String(data.yukseklik2, 1) + "," +
                   String(data.irtifa_farki, 1) + "," +
                   String(data.inis_hizi, 1) + "," +
                   String(data.temperature, 1) + "," +
                   String(data.pil_gerilimi, 3) + "," +
                   String(data.gps1_latitude, 6) + "," +
                   String(data.gps1_longitude, 6) + "," +
                   String(data.gps1_altitude, 1) + "," +
                   String(data.pitch, 1) + "," +
                   String(data.roll, 1) + "," +
                   String(data.yaw, 1) + "," +
                   data.rhrh + "," +
                   String(data.iot_s1_data, 1) + "," +
                   String(data.iot_s2_data, 1) + "," +
                   String(data.takim_no) + "," +
                   data.sensor_status + "\n";
  
  if (!appendFile(SD, "/sensor_data.csv", csvLine.c_str())) {
    Serial.println("SD kart yazma hatası - durum güncelleniyor");
    sd_status = false; // SD kart yazma hatası durumunda durumu güncelle
  }
}

void updateSensorStatus() {
  // Periyodik olarak sensör durumlarını kontrol et
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck >= 5000) { // 5 saniyede bir kontrol et
    lastCheck = millis();
    
    // MPU6050 kontrolü
    if (!mpu_status) {
      if (mpu.begin(0x69)) {
        mpu_status = true;
        Serial.println("MPU6050 yeniden bağlandı");
      }
    }
    
    // BMP280 kontrolü
    if (!bmp_status) {
      if (bmp.begin(0x76)) {
        bmp_status = true;
        Serial.println("BMP280 yeniden bağlandı");
      }
    }
    
    // RTC kontrolü
    if (!rtc_status) {
      if (rtc.begin()) {
        rtc_status = true;
        Serial.println("RTC yeniden bağlandı");
      }
    }
    
    // SD kart kontrolü
    if (!sd_status) {
      Serial.println("SD kart yeniden bağlanmaya çalışılıyor...");
      
      // Basit SD kart yeniden başlatma
      if (SD.begin()) {
        sd_status = true;
        Serial.println("SD kart yeniden bağlandı");
        
        // CSV dosyası varsa kontrol et, yoksa oluştur
        if (!SD.exists("/sensor_data.csv")) {
          writeFile(SD, "/sensor_data.csv", "Packet_Number,Uydu_Statusu,Hata_Kodu,gönderme_saati,Pressure1,Pressure2,Yukseklik1,Yukseklik2,Irtifa_Farki,Inis_Hizi,Temperature,Pil_Gerilimi,gps1_latitude,gps1_longitude,gps1_altitude,Pitch,Roll,Yaw,RHRH,IoT_S1_Data,IoT_S2_Data,Takim_No,Sensor_Status\n");
        }
      } else {
        Serial.println("SD kart yeniden bağlanma başarısız");
      }
    }
  }
}

void writeFile(fs::FS &fs, const char *path, const char *message) {
  Serial.printf("SD'ye yazılıyor: %s\n", path);
  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("SD dosya yazma hatası - dosya açılamadı");
    sd_status = false;
    return;
  }
  if (file.print(message)) {
    Serial.println("SD dosya başarıyla yazıldı");
  } else {
    Serial.println("SD yazma başarısız - yazma hatası");
    sd_status = false;
  }
  file.close();
}

bool appendFile(fs::FS &fs, const char *path, const char *message) {
  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("SD dosya ekleme hatası - dosya açılamadı");
    return false;
  }
  
  size_t bytesWritten = file.print(message);
  if (bytesWritten > 0) {
    file.close();
    return true;
  } else {
    Serial.println("SD ekleme başarısız - yazma hatası");
    file.close();
    return false;
  }
}

void setRTCTime(String command) {
  // Format: SET:2025,7,18,14,30,0 (Yıl,Ay,Gün,Saat,Dakika,Saniye)
  command = command.substring(4); // "SET:" kısmını çıkar
  
  int values[6];
  int index = 0;
  
  // String'i parçala
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
  
  if (index == 5) { // 6 değer alındıysa
    rtc.adjust(DateTime(values[0], values[1], values[2], values[3], values[4], values[5]));
    Serial.printf("RTC zamanı ayarlandı: %02d/%02d/%04d %02d:%02d:%02d\n", 
                  values[2], values[1], values[0], values[3], values[4], values[5]);
    rtc_status = true;
  } else {
    Serial.println("Hatalı format! Kullanım: SET:2025,7,18,14,30,0");
  }
}

void loadLastPacketNumber() {
  Serial.println("Son paket numarası okunuyor...");
  
  File file = SD.open("/sensor_data.csv", FILE_READ);
  if (!file) {
    Serial.println("CSV dosyası açılamadı, paket numarası 1'den başlayacak");
    packet_counter = 1;
    return;
  }
  
  String lastLine = "";
  String currentLine = "";
  
  // Dosyayı okuyup son satırı bul
  while (file.available()) {
    char c = file.read();
    if (c == '\n') {
      if (currentLine.length() > 0) {
        lastLine = currentLine;
        currentLine = "";
      }
    } else {
      currentLine += c;
    }
  }
  
  // Son satır da ekle (dosya \n ile bitmiyorsa)
  if (currentLine.length() > 0) {
    lastLine = currentLine;
  }
  
  file.close();
  
  // Son satırdan paket numarasını çıkar
  if (lastLine.length() > 0 && lastLine != "Packet_Number,Uydu_Statusu,Hata_Kodu,gönderme_saati,Pressure1,Pressure2,Yukseklik1,Yukseklik2,Irtifa_Farki,Inis_Hizi,Temperature,Pil_Gerilimi,gps1_latitude,gps1_longitude,gps1_altitude,Pitch,Roll,Yaw,RHRH,IoT_S1_Data,IoT_S2_Data,Takim_No,Sensor_Status") {
    int commaIndex = lastLine.indexOf(',');
    if (commaIndex > 0) {
      String packetNumStr = lastLine.substring(0, commaIndex);
      unsigned long lastPacketNum = packetNumStr.toInt();
      if (lastPacketNum > 0) {
        packet_counter = lastPacketNum + 1;
        Serial.printf("Son paket numarası: %lu, devam edecek: %lu\n", lastPacketNum, packet_counter);
      } else {
        packet_counter = 1;
        Serial.println("Paket numarası okunamadı, 1'den başlayacak");
      }
    } else {
      packet_counter = 1;
      Serial.println("CSV formatı hatalı, paket numarası 1'den başlayacak");
    }
  } else {
    packet_counter = 1;
    Serial.println("CSV dosyası boş, paket numarası 1'den başlayacak");
  }
}

float calculategps1_altitude(float P, float T_celsius) {
  const float P0 = 101325.0;     // Deniz seviyesi basıncı (Pa)
  const float L = 0.0065;        // Sıcaklık gradyanı (K/m)
  const float R = 287.05;        // Gaz sabiti (J/kg·K)
  const float g = 9.80665;       // Yerçekimi (m/s²)

  float T = T_celsius + 273.15;  // Celsius -> Kelvin dönüşümü

  // Barometrik formül:
  float h = (T / L) * (1 - pow(P / P0, (R * L) / g));
  return h;
}

void initServos() {
  Serial.println("SG90 180° Servo motorlar başlatılıyor...");
  
  // PWM kanallarını ayarla (SG90 180° için 50Hz)
  ledcSetup(SERVO1_CHANNEL, SERVO_FREQ, SERVO_RESOLUTION);
  ledcSetup(SERVO2_CHANNEL, SERVO_FREQ, SERVO_RESOLUTION);
  ledcSetup(SERVO3_CHANNEL, SERVO_FREQ, SERVO_RESOLUTION);
  
  // PWM kanallarını pin'lere bağla
  ledcAttachPin(SERVO1_PIN, SERVO1_CHANNEL);
  ledcAttachPin(SERVO2_PIN, SERVO2_CHANNEL);
  ledcAttachPin(SERVO3_PIN, SERVO3_CHANNEL);
  
  delay(500); // PWM'in hazır olması için bekle
  
  // Başlangıç durumunda servolar b pozisyonunda
  setServoPosition(SERVO1_CHANNEL, ANGLE_B);
  setServoPosition(SERVO2_CHANNEL, ANGLE_B);
  setServoPosition(SERVO3_CHANNEL, 0); // Servo3 0° pozisyonunda (ayrılmamış)
  
  delay(1000); // Servo motorların pozisyona gelmesi için bekle
  
  Serial.println("SG90 180° Servo motorlar başarıyla başlatıldı");
  Serial.println("Servo1 Pin: " + String(SERVO1_PIN) + " (PWM Kanal " + String(SERVO1_CHANNEL) + ")");
  Serial.println("Servo2 Pin: " + String(SERVO2_PIN) + " (PWM Kanal " + String(SERVO2_CHANNEL) + ")");
  Serial.println("Servo3 Pin: " + String(SERVO3_PIN) + " (PWM Kanal " + String(SERVO3_CHANNEL) + ") - AYRIL/BIRLES Servo");
  Serial.println("RHRH varsayılan değer: " + current_rhrh);
  Serial.println("Pozisyon kodları (SG90): b=" + String(ANGLE_B) + "°, m=" + String(ANGLE_M) + "°, k=" + String(ANGLE_K) + "°, y=" + String(ANGLE_Y) + "°");
  Serial.println("Servo3 durumu: 0° (Birleşik)");
  
  // Test hareketi
  Serial.println("SG90 Servo pozisyon test hareketi yapılıyor...");
  
  // Servo1 pozisyon test
  Serial.println("Servo1: " + String(ANGLE_B) + "° pozisyon (b)");
  setServoPosition(SERVO1_CHANNEL, ANGLE_B);
  delay(2000);
  
  Serial.println("Servo1: " + String(ANGLE_M) + "° pozisyon (m)");
  setServoPosition(SERVO1_CHANNEL, ANGLE_M);
  delay(2000);
  
  Serial.println("Servo1: " + String(ANGLE_K) + "° pozisyon (k)");
  setServoPosition(SERVO1_CHANNEL, ANGLE_K);
  delay(2000);
  
  // Servo2 pozisyon test
  Serial.println("Servo2: " + String(ANGLE_B) + "° pozisyon (b)");
  setServoPosition(SERVO2_CHANNEL, ANGLE_B);
  delay(2000);
  
  // Servo3 test hareketi
  Serial.println("Servo3: 0° pozisyon (Birleşik)");
  setServoPosition(SERVO3_CHANNEL, 0);
  delay(2000);
  
  Serial.println("SG90 Servo pozisyon test hareketi tamamlandı - varsayılan pozisyonlara dönülüyor");
  setServoPosition(SERVO1_CHANNEL, ANGLE_B);
  setServoPosition(SERVO2_CHANNEL, ANGLE_B);
  setServoPosition(SERVO3_CHANNEL, 0);
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
      collectAndSendData();
    }
    else if (command == "AYRIL") {
      if (!servo3_separated) {
        Serial.println("AYRIL komutu alındı - Servo3 90° pozisyonuna dönüyor");
        setServoPosition(SERVO3_CHANNEL, 90);
        servo3_separated = true;
        Serial.println("Ayrılma işlemi tamamlandı! Servo3: 90°");
      } else {
        Serial.println("Ayrılma işlemi zaten gerçekleştirilmiş!");
      }
    }
    else if (command == "BIRLES") {
      if (servo3_separated) {
        Serial.println("BIRLES komutu alındı - Servo3 0° pozisyonuna dönüyor");
        setServoPosition(SERVO3_CHANNEL, 0);
        servo3_separated = false;
        Serial.println("Birleşme işlemi tamamlandı! Servo3: 0°");
      } else {
        Serial.println("Birleşme işlemi zaten gerçekleştirilmiş!");
      }
    }
    else if (command == "STATUS") {
      Serial.println("=== SİSTEM DURUMU ===");
      Serial.println("BMP280: " + String(bmp_status ? "OK" : "HATA"));
      Serial.println("MPU6050: " + String(mpu_status ? "OK" : "HATA"));
      Serial.println("RTC: " + String(rtc_status ? "OK" : "HATA"));
      Serial.println("GPS: " + String(gps_status ? "OK" : "HATA"));
      Serial.println("SD Kart: " + String(sd_status ? "OK" : "HATA"));
      Serial.println("ADC: " + String(adc_status ? "OK" : "HATA"));
      Serial.println("Güncel RHRH: " + current_rhrh);
      Serial.println("Servo3 Durumu: " + String(servo3_separated ? "Ayrık (90°)" : "Birleşik (0°)"));
      Serial.println("Paket sayısı: " + String(packet_counter));
    }
    else if (command == "SERVO_TEST") {
      Serial.println("=== SG90 180° SERVO TEST ===");
      Serial.println("SG90 180° Servo pozisyon test hareketi başlıyor...");
      
      // Test dizisi (SG90 pozisyon kontrolü için)
      Serial.println("Servo1: " + String(ANGLE_B) + "° pozisyon (b) - (2sn)");
      setServoPosition(SERVO1_CHANNEL, ANGLE_B);
      delay(2000);
      
      Serial.println("Servo1: " + String(ANGLE_M) + "° pozisyon (m) - (2sn)");
      setServoPosition(SERVO1_CHANNEL, ANGLE_M);
      delay(2000);
      
      Serial.println("Servo1: " + String(ANGLE_K) + "° pozisyon (k) - (2sn)");
      setServoPosition(SERVO1_CHANNEL, ANGLE_K);
      delay(2000);
      
      Serial.println("Servo1: " + String(ANGLE_Y) + "° pozisyon (y) - (2sn)");
      setServoPosition(SERVO1_CHANNEL, ANGLE_Y);
      delay(2000);
      
      Serial.println("Servo2: " + String(ANGLE_B) + "° pozisyon (b) - (2sn)");
      setServoPosition(SERVO2_CHANNEL, ANGLE_B);
      delay(2000);
      
      Serial.println("Servo2: " + String(ANGLE_M) + "° pozisyon (m) - (2sn)");
      setServoPosition(SERVO2_CHANNEL, ANGLE_M);
      delay(2000);
      
      Serial.println("Servo2: " + String(ANGLE_K) + "° pozisyon (k) - (2sn)");
      setServoPosition(SERVO2_CHANNEL, ANGLE_K);
      delay(2000);
      
      Serial.println("Servo2: " + String(ANGLE_Y) + "° pozisyon (y) - (2sn)");
      setServoPosition(SERVO2_CHANNEL, ANGLE_Y);
      delay(2000);
      
      // Servo3 test hareketi
      Serial.println("Servo3: 0° pozisyon (Ayrılmamış) - (2sn)");
      setServoPosition(SERVO3_CHANNEL, 0);
      delay(2000);
      
      Serial.println("Servo3: 90° pozisyon (Ayrılmış) - (2sn)");
      setServoPosition(SERVO3_CHANNEL, 90);
      delay(2000);
      
      // Varsayılan pozisyonlara getir
      Serial.println("Servolar varsayılan pozisyonlara getiriliyor");
      setServoPosition(SERVO1_CHANNEL, ANGLE_B);
      setServoPosition(SERVO2_CHANNEL, ANGLE_B);
      setServoPosition(SERVO3_CHANNEL, 0); // Servo3'ü 0° pozisyonuna getir (birleşik durum)
      servo3_separated = false; // Ayrılma durumunu sıfırla (birleşik durum)
      current_rhrh = "b0b0"; // RHRH değerini sıfırla
      
      Serial.println("SG90 180° Servo test tamamlandı (Servo3 dahil)");
    }
  }
}

void processRHRHCommand(String command) {
  // RHRH formatı: HSHD (Harf Sayı Harf Sayı) örneğin: m2k6
  if (command.length() != 4) {
    Serial.println("HATA: RHRH formatı yanlış! Format: HSHS (örn: m2k6)");
    return;
  }
  
  char pos1 = command.charAt(0);  // İlk pozisyon harfi
  char dur1 = command.charAt(1);  // İlk süre rakamı
  char pos2 = command.charAt(2);  // İkinci pozisyon harfi
  char dur2 = command.charAt(3);  // İkinci süre rakamı
  
  // Geçerli pozisyon kontrolü
  if ((pos1 != 'b' && pos1 != 'm' && pos1 != 'k' && pos1 != 'y') ||
      (pos2 != 'b' && pos2 != 'm' && pos2 != 'k' && pos2 != 'y')) {
    Serial.println("HATA: Geçersiz pozisyon! Kullanılabilir: b, m, k, y");
    return;
  }
  
  // Geçerli süre kontrolü (0-9 arası)
  if (!isdigit(dur1) || !isdigit(dur2)) {
    Serial.println("HATA: Süre değerleri rakam olmalı! (0-9)");
    return;
  }
  
  // RHRH komutunu uygula
  current_rhrh = command;
  
  // Servo pozisyonlarını ayarla
  int servo1_pos = getServoPosition(pos1);
  int servo2_pos = getServoPosition(pos2);
  
  Serial.println("RHRH komutu uygulandı: " + command);
  Serial.println("Servo1: " + String(servo1_pos) + "° - " + String((dur1 - '0')) + "s");
  Serial.println("Servo2: " + String(servo2_pos) + "° - " + String((dur2 - '0')) + "s");
  
  // Servo pozisyonlarını ayarla
  Serial.println("Servo1 pozisyonu ayarlanıyor: " + String(servo1_pos) + "°");
  setServoPosition(SERVO1_CHANNEL, servo1_pos);
  delay(100); // Servo hareket etmeye başlaması için bekle
  
  Serial.println("Servo2 pozisyonu ayarlanıyor: " + String(servo2_pos) + "°");
  setServoPosition(SERVO2_CHANNEL, servo2_pos);
  delay(100); // Servo hareket etmeye başlaması için bekle
  
  // Süreleri ayarla
  servo1_duration = (dur1 - '0') * 1000; // Saniyeyi milisaniyeye çevir
  servo2_duration = (dur2 - '0') * 1000;
  
  // Zamanlayıcıları başlat
  servo1_start_time = millis();
  servo2_start_time = millis();
  servo1_active = (servo1_duration > 0);
  servo2_active = (servo2_duration > 0);
  
  Serial.println("Servo pozisyonları ayarlandı ve zamanlayıcı başlatıldı");
}

int getServoPosition(char position) {
  switch(position) {
    case 'b': return ANGLE_B;   // b pozisyonu (varsayılan)
    case 'm': return ANGLE_M;   // m pozisyonu
    case 'k': return ANGLE_K;   // k pozisyonu
    case 'y': return ANGLE_Y;   // y pozisyonu
    default: return ANGLE_B;    // Varsayılan b pozisyonu
  }
}

void updateServoControl() {
  unsigned long currentTime = millis();
  bool rhrh_changed = false;
  
  // Servo1 süre kontrolü
  if (servo1_active && (currentTime - servo1_start_time >= servo1_duration)) {
    Serial.println("Servo1 süre doldu, b pozisyonuna (" + String(ANGLE_B) + "°) dönüyor");
    setServoPosition(SERVO1_CHANNEL, ANGLE_B); // Servo b pozisyonuna
    delay(100); // Servo hareket etmesi için kısa bekle
    servo1_active = false;
    Serial.println("Servo1 b pozisyonuna getirildi");
    
    // RHRH değerini güncelle
    current_rhrh[0] = 'b';
    current_rhrh[1] = '0';
    rhrh_changed = true;
  }
  
  // Servo2 süre kontrolü  
  if (servo2_active && (currentTime - servo2_start_time >= servo2_duration)) {
    Serial.println("Servo2 süre doldu, b pozisyonuna (" + String(ANGLE_B) + "°) dönüyor");
    setServoPosition(SERVO2_CHANNEL, ANGLE_B); // Servo b pozisyonuna
    delay(100); // Servo hareket etmesi için kısa bekle
    servo2_active = false;
    Serial.println("Servo2 b pozisyonuna getirildi");
    
    // RHRH değerini güncelle
    current_rhrh[2] = 'b';
    current_rhrh[3] = '0';
    rhrh_changed = true;
  }
  
  // RHRH değeri değiştiyse bilgilendir
  if (rhrh_changed) {
    Serial.println("RHRH güncellendi: " + current_rhrh);
  }
}

// SG90 servo motor için PWM pozisyon kontrol fonksiyonları
void setServoPosition(int channel, int angle) {
  // Açıyı duty cycle'a çevir
  int dutyCycle = angleToDutyCycle(angle);
  
  // PWM sinyali gönder
  ledcWrite(channel, dutyCycle);
  
  String positionName = "";
  if (angle == ANGLE_B) positionName = String(ANGLE_B) + "° (b)";
  else if (angle == ANGLE_M) positionName = String(ANGLE_M) + "° (m)";
  else if (angle == ANGLE_K) positionName = String(ANGLE_K) + "° (k)";
  else if (angle == ANGLE_Y) positionName = String(ANGLE_Y) + "° (y)";
  else positionName = String(angle) + "°";
  
  Serial.println("Kanal " + String(channel) + " -> " + positionName + " (Angle: " + String(angle) + "°, Duty: " + String(dutyCycle) + ")");
}

void moveServoToNeutral(int channel) {
  setServoPosition(channel, ANGLE_B); // b pozisyonu (varsayılan durum)
}

int angleToDutyCycle(int angle) {
  // SG90 Servo özellikleri - PWM period = 20ms (50Hz)
  // Standart servo aralığı: 0-180°
  
  // Açıyı standart servo aralığında sınırla
  angle = constrain(angle, 0, 180);
  
  // Pulse width hesapla (500-2500µs aralığında lineer interpolasyon)
  // 0° = 500µs, 180° = 2500µs
  int pulseWidth = map(angle, 0, 180, 500, 2500);
  
  // 16-bit çözünürlük (0-65535)
  // 20ms period için full range
  const int PWM_PERIOD = 20000; // 20ms in microseconds
  
  // Duty cycle hesapla (16-bit)
  int dutyCycle = (pulseWidth * 65535) / PWM_PERIOD;
  
  return dutyCycle;
}