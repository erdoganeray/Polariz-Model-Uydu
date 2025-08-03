/*
 * BERKAYIN KODU
 * SON HALİ
 * ESP32 Sensör Veri Toplama Sistemi
 * 
 * Sensörler:
 * - MPU6050: I2C üzerin  //RHRH
  uint32_t rhrh; // Hex veri (4 byte) /Daha sonra kullanılacak

  float iot_s1_data;       // IoT sensor 1 verisi /Daha sonra kullanılacak
  float iot_s2_data;       // IoT sensor 2 verisi /Daha sonra kullanılacak

  //Takım numarası
  uint32_t takim_no; // Takım numarası (626541)h, roll, yaw (GPIO21-SDA, GPIO22-SCL)
 * - BMP280: I2C üzerinden basınç ve sıcaklık (GPIO21-SDA, GPIO22-SCL)
 * - GY-GPS6MV2: UART1 üzerinden GPS verileri (GPIO16-RX, GPIO17-TX)
 * - DS3231 RTC: I2C üzerinden zaman bilgisi (GPIO21-SDA, GPIO22-SCL)
 * - SD Kart: SPI üzerinden veri kaydetme (GPIO18-SCK, GPIO19-MISO, GPIO23-MOSI, GPIO5-CS)
 * - ADC: Gerilim ölçümü
 * 
 * Komut formatı USB Serial üzerinden: "GET_DATA"
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

// GPS UART1 Pin tanımlamaları
#define GPS_RX 16
#define GPS_TX 17
#define GPS_BAUD 9600
HardwareSerial gpsSerial(1);

// SD Kart SPI pin tanımlamaları (ESP32 Default SPI)
#define SD_SCK 18
#define SD_MISO 19
#define SD_MOSI 23
#define SD_CS 5

// ADC pin tanımlaması
#define pil_gerilimi_PIN A0

// Sensör nesneleri
Adafruit_MPU6050 mpu;
Adafruit_BMP280 bmp;
TinyGPSPlus gps;
RTC_DS3231 rtc;

// Veri yapısı
struct __attribute__((packed)) SensorData {
  // Paket numarası
  uint16_t packet_number;

  // Uydu statüsü (İlerleyen zamanlarda kullanılacak)
  uint8_t  uydu_statusu;      // 1-6 arası durum kodu

  // Hata Kodu (İlerleyen zamanlarda kullanılacak)
  uint8_t hata_kodu;          // 5 haneli hata kodu
  
  // RTC verileri
  String timestamp;  //Daha sonra değişecek.

  //Görev yükünün basıncı
  float pressure1; //pressure1 olarak değiştir 
  
  //Taşıyıcı basıncı
  float pressure2; //Daha sonra kullanılacak

  float yukseklik1;        // Birincil yükseklik (m).
  float yukseklik2;        // İkincil yükseklik (m). /Daha sonra kullanılacak
  float irtifa_farki;      // Yükseklik farkı (m). /Daha sonra kullanılacak

  float temperature; //Görev yükündeki sıcaklık verisi

  // ADC verisi
  float pil_gerilimi;  // Pil gerilimi

  // GPS verileri
  double latitude;
  double longitude;
  float altitude;
  bool gps_valid; // GPS verilerinin geçerli olup olmadığını gösterir

  // MPU6050 verileri
  float pitch;
  float roll;
  float yaw;
  
  //RHRH
  uint32_t rhrh; // Hex veri (4 byte) /Daha sonra kullanılacak

  float iot_s1_data;       // IoT sensor 1 verisi /Daha sonra kullanılacak
  float iot_s2_data;       // IoT sensor 2 verisi /Daha sonra kullanılacak

  //Takım numarası
  uint32_t takim_no; // Takım numarası (626541)

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
  
  // SD kart başlatma
  initSDCard();
  
  Serial.println("Sistem hazır! Otomatik veri gönderimi (1Hz) ve GET_DATA komutu aktif...");
}

void loop() {
  // USB Serial üzerinden komut kontrolü
  if (Serial.available()) {
    String command = Serial.readString();
    command.trim();
    
    if (command == "GET_DATA") {
      collectAndSendData();
    } else if (command.startsWith("SET:")) {
      setRTCTime(command);
    }
  }
  
  // GPS verilerini sürekli güncelle
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }
  
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
    writeFile(SD, "/sensor_data.csv", "Packet_Number,Timestamp,Pitch,Roll,Yaw,Pressure,Temperature,Yukseklik1,Latitude,Longitude,Altitude,GPS_Valid,pil_gerilimi,Sensor_Status\n");
    packet_counter = 1;
  } else {
    Serial.println("CSV dosyası mevcut");
    // Son paket numarasını oku
    loadLastPacketNumber();
  }
}

void collectAndSendData() {
  SensorData data;
  
  // Paket numarasını ata
  data.packet_number = packet_counter;
  
  // Varsayılan değerleri initialize et
  data.uydu_statusu = 0;
  data.hata_kodu = 0;
  data.pressure2 = 0.0;
  data.yukseklik2 = 0.0;
  data.irtifa_farki = 0.0;
  data.rhrh = 0;
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
      data.yukseklik1 = calculateAltitude(pressure1, temperature);
    } else {
      bmp_status = false;
      data.pressure1 = 0.0;
      data.temperature = 0.0;
      data.yukseklik1 = 0.0;
    }
  } else {
    data.pressure1 = 0.0;
    data.temperature = 0.0;
    data.yukseklik1 = 0.0;
  }
  
  // GPS verilerini oku
  if (gps.location.isValid()) {
    data.latitude = gps.location.lat();
    data.longitude = gps.location.lng();
    data.gps_valid = true;
    gps_status = true;
  } else {
    data.latitude = 0.0;
    data.longitude = 0.0;
    data.gps_valid = false;
    gps_status = false;
  }
  
  if (gps.altitude.isValid()) {
    data.altitude = gps.altitude.meters();
  } else {
    data.altitude = 0.0;
  }
  
  // RTC zamanını oku
  if (rtc_status) {
    DateTime now = rtc.now();
    if (now.year() > 2020 && now.year() < 2035) {  // Makul tarih aralığı kontrolü
      data.timestamp = String(now.year()) + "-" + 
                       String(now.month()) + "-" + 
                       String(now.day()) + " " + 
                       String(now.hour()) + ":" + 
                       String(now.minute()) + ":" + 
                       String(now.second());
    } else {
      rtc_status = false;
      data.timestamp = "0000-00-00 00:00:00";
      Serial.println("RTC tarih geçersiz, zaman ayarlaması gerekiyor");
    }
  } else {
    data.timestamp = "0000-00-00 00:00:00";
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

String createDataPacket(SensorData data) {
  String packet = "";
  
  packet += "PACKET_NUMBER:" + String(data.packet_number) + "\n";
  packet += "TIMESTAMP:" + data.timestamp + "\n";
  packet += "PITCH:" + String(data.pitch, 3) + "\n";
  packet += "ROLL:" + String(data.roll, 3) + "\n";
  packet += "YAW:" + String(data.yaw, 3) + "\n";
  packet += "PRESSURE:" + String(data.pressure1, 2) + "\n";
  packet += "TEMPERATURE:" + String(data.temperature, 2) + "\n";
  packet += "YUKSEKLIK1:" + String(data.yukseklik1, 2) + "\n";
  packet += "LATITUDE:" + String(data.latitude, 6) + "\n";
  packet += "LONGITUDE:" + String(data.longitude, 6) + "\n";
  packet += "ALTITUDE:" + String(data.altitude, 2) + "\n";
  packet += "GPS_VALID:" + String(data.gps_valid ? "1" : "0") + "\n";
  packet += "pil_gerilimi:" + String(data.pil_gerilimi, 3) + "\n";
  packet += "SENSOR_STATUS:" + data.sensor_status + "\n";
  
  return packet;
}

void saveToSD(SensorData data) {
  if (!sd_status) {
    return; // SD kart çalışmıyorsa kaydetmeye çalışma
  }
  
  String csvLine = String(data.packet_number) + "," +
                   data.timestamp + "," +
                   String(data.pitch, 3) + "," +
                   String(data.roll, 3) + "," +
                   String(data.yaw, 3) + "," +
                   String(data.pressure1, 2) + "," +
                   String(data.temperature, 2) + "," +
                   String(data.yukseklik1, 2) + "," +
                   String(data.latitude, 6) + "," +
                   String(data.longitude, 6) + "," +
                   String(data.altitude, 2) + "," +
                   String(data.gps_valid ? "1" : "0") + "," +
                   String(data.pil_gerilimi, 3) + "," +
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
      Serial.println("SD kart yeniden bağlanmaya çalışılıyor..."