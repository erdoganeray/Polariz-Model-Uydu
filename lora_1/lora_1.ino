#include <Arduino.h>
#include <LoRa_E22.h>
#include <Adafruit_BMP280.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <RTClib.h>
#include "../binary_protocol.h"

#define LORA_M0 13
#define LORA_M1 12
#define LORA_AUX 4
#define LORA_TX 16
#define LORA_RX 17

#define LORA_CHANNEL 10
#define LORA_ADDR_H 0x00
#define LORA_ADDR_L 0x0A             

LoRa_E22 E22(&Serial2, LORA_AUX, LORA_M0, LORA_M1);

// BMP280 sensor
Adafruit_BMP280 bmp;

// MPU6050 sensor
Adafruit_MPU6050 mpu;

// DS3231 RTC modülü
RTC_DS3231 rtc;

uint16_t paket_sayisi_lora2 = 1;
uint16_t paket_sayisi_lora3 = 1;

// RHRH değeri - başlangıçta 0000, lora_2'den gelen değerle güncellenecek
uint32_t current_rhrh = encodeRHRH('0', '0', '0', '0');

// Lora 3'ten gelen basınç değeri
float basinc_lora3 = 0.00; // Varsayılan değer

// Lora 4 ve 5'ten gelen sıcaklık değerleri 
int16_t sicaklik_lora4 = 0; // 0°C * 100
int16_t sicaklik_lora5 = 0; // 0°C * 100

// Yükseklik hesaplama için gerekli değişkenler
float previous_altitude = 0.0;
unsigned long previous_time = 0;

// Referans basınç değerleri (ilk okuma)
float basinc1_referans = 0.0;
float basinc2_referans = 0.0;
bool basinc_kalibrasyonu = false;

// MPU6050 için açı değişkenleri
float pitch = 0.0;
float roll = 0.0;
float yaw = 0.0;

// MPU6050 referans değerleri (ilk okuma)
float pitch_offset = 0.0;
float roll_offset = 0.0;
float yaw_offset = 0.0;
bool mpu_calibrated = false;

// Basınç değerinden yükseklik hesaplama fonksiyonu
float calculateAltitude(float pressure, float reference_pressure) {
  // Referans basınç yoksa veya geçersizse, hesaplama yapma
  if (reference_pressure <= 0) {
    return 0.0;
  }
  
  // Barometrik formül kullanarak yükseklik hesapla
  // h = 44330 * (1 - (P/P_ref)^(1/5.255))
  float altitude = 44330.0 * (1.0 - pow(pressure / reference_pressure, 1.0/5.255));
  
  return altitude;
}

// İniş hızı hesaplama fonksiyonu
float calculateDescentRate(float current_altitude, unsigned long current_time) {
  if (previous_time == 0) {
    // İlk ölçüm, hız hesaplanamaz
    previous_altitude = current_altitude;
    previous_time = current_time;
    return 0.0;
  }
  
  float time_diff = (current_time - previous_time) / 1000.0; // saniye cinsinden
  if (time_diff > 0) {
    float altitude_diff = current_altitude - previous_altitude;
    float descent_rate = altitude_diff / time_diff; // m/s
    
    // Değerleri güncelle
    previous_altitude = current_altitude;
    previous_time = current_time;
    
    return descent_rate;
  }
  
  return 0.0;
}

// RTC'den Unix timestamp alma fonksiyonu
uint32_t getRTCUnixTime() {
  DateTime now = rtc.now();
  return now.unixtime();
}

// RTC tarih/saat bilgisi alma fonksiyonu
void printRTCDateTime() {
  DateTime now = rtc.now();
  Serial.print("RTC Tarih/Saat: ");
  Serial.print(now.day(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.year(), DEC);
  Serial.print(" ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  if (now.minute() < 10) Serial.print('0');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  if (now.second() < 10) Serial.print('0');
  Serial.print(now.second(), DEC);
  Serial.print(" (Unix: ");
  Serial.print(now.unixtime());
  Serial.println(")");
}

// Serial monitörden gelen SET komutunu işleme fonksiyonu
void processSerialCommand() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    // SET komutunu kontrol et (format: SET DD,MM,YY,HH,MM,SS)
    if (command.startsWith("SET ")) {
      String dateTime = command.substring(4); // "SET " kısmını atla
      
      // Virgüllerle ayrılmış değerleri parse et
      int day, month, year, hour, minute, second;
      if (sscanf(dateTime.c_str(), "%d,%d,%d,%d,%d,%d", &day, &month, &year, &hour, &minute, &second) == 6) {
        // 2 haneli yılı 4 haneli yıla çevir (25 -> 2025)
        if (year < 100) {
          year += 2000;
        }
        
        // RTC'yi ayarla
        rtc.adjust(DateTime(year, month, day, hour, minute, second));
        
        Serial.print("RTC tarih/saat ayarlandi: ");
        Serial.print(day);
        Serial.print("/");
        Serial.print(month);
        Serial.print("/");
        Serial.print(year);
        Serial.print(" ");
        Serial.print(hour);
        Serial.print(":");
        if (minute < 10) Serial.print("0");
        Serial.print(minute);
        Serial.print(":");
        if (second < 10) Serial.print("0");
        Serial.println(second);
        
        // Kontrol için ayarlanan zamanı oku
        delay(100);
        printRTCDateTime();
      } else {
        Serial.println("Hata: Gecersiz format! Kullanim: SET DD,MM,YY,HH,MM,SS");
        Serial.println("Ornek: SET 06,08,25,17,25,00");
      }
    }
  }
}

// MPU6050'den açı değerlerini oku
void readMPU6050() {
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);
  
  // Accelerometer verilerinden ham pitch ve roll hesapla
  float raw_pitch = atan2(accel.acceleration.y, sqrt(accel.acceleration.x * accel.acceleration.x + accel.acceleration.z * accel.acceleration.z)) * 180.0 / PI;
  float raw_roll = atan2(-accel.acceleration.x, accel.acceleration.z) * 180.0 / PI;
  
  // Gyroscope verilerinden ham yaw değerini al (basit integrasyon)
  static float yaw_rate = 0.0;
  static unsigned long last_time = 0;
  unsigned long current_time = millis();
  
  if (last_time != 0) {
    float dt = (current_time - last_time) / 1000.0; // saniye cinsinden
    yaw_rate += gyro.gyro.z * dt * 180.0 / PI; // rad/s'yi derece/s'ye çevir
  }
  
  last_time = current_time;
  
  // İlk okuma ise offset değerlerini kaydet
  if (!mpu_calibrated) {
    pitch_offset = raw_pitch;
    roll_offset = raw_roll;
    yaw_offset = yaw_rate;
    mpu_calibrated = true;
    Serial.println("MPU6050 referans degerler kaydedildi (ilk okuma sifir kabul edildi)");
    Serial.print("Pitch offset: "); Serial.println(pitch_offset);
    Serial.print("Roll offset: "); Serial.println(roll_offset);
    Serial.print("Yaw offset: "); Serial.println(yaw_offset);
  }
  
  // Offset değerlerini çıkararak gerçek açıları hesapla
  pitch = raw_pitch - pitch_offset;
  roll = raw_roll - roll_offset;
  float raw_yaw = yaw_rate - yaw_offset;
  
  // Yaw değerini 0-360 derece arasında tut
  yaw = raw_yaw;
  if (yaw < 0) yaw += 360.0;
  if (yaw >= 360) yaw -= 360.0;
}

void configureLoRa() {
  Serial.println("\n=== LoRa Konfigürasyonu ===");
  
  // Konfigürasyon yapısını oluştur
  Configuration configuration;
  
  // *** ADDRESS AYARLARI ***
  configuration.ADDH = 0x00;        // Yüksek adres byte'ı
  configuration.ADDL = 0x0A;        // Düşük adres byte'ı (10 decimal = 0x0A hex)
  configuration.NETID = 0x00;       // Network ID
  
  // *** CHANNEL AYARI ***
  configuration.CHAN = 10;          // Kanal 10
  
  // *** AIR DATA RATE ***
  configuration.SPED.airDataRate = AIR_DATA_RATE_011_48;  // 4.8kbps
  
  // *** PACKET SIZE ***
  configuration.OPTION.subPacketSetting = SPS_240_00;  // 240 bytes
  
  // *** TRANSMISSION MODE ***
  configuration.TRANSMISSION_MODE.fixedTransmission = FT_FIXED_TRANSMISSION;  // Sabit iletim
  
  // Diğer gerekli parametreler
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
  if (rs.code == 1) {
    Serial.println("✓ LoRa konfigürasyonu başarıyla uygulandı!");
    Serial.println("--- Uygulanan Ayarlar ---");
    Serial.println("Address: 0x00:0x0A (10)");
    Serial.println("Channel: 10");
    Serial.println("Air Rate: 4.8kbps");
    Serial.println("Packet Size: 240 bytes");
    Serial.println("Mode: Fixed Transmission");
    Serial.println("-------------------------");
  } else {
    Serial.println("✗ LoRa konfigürasyonu uygulanamadı!");
  }
}

void printModuleInfo() {
  ResponseStructContainer c = E22.getModuleInformation();
  ModuleInformation moduleInfo = *(ModuleInformation*) c.data;
  Serial.print("Model: "); Serial.println(moduleInfo.model);
  Serial.print("Version: "); Serial.println(moduleInfo.version);
  Serial.print("Features: "); Serial.println(moduleInfo.features);
  Serial.print("Status: "); Serial.println(c.status.getResponseDescription());
  c.close();
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

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial2.begin(9600, SERIAL_8N1, LORA_RX, LORA_TX);
  
  // BMP280 sensörünü başlat
  Wire.begin(21, 22); // SDA=21, SCL=22 (ESP32 default)
  if (!bmp.begin(0x76)) {  // BMP280 I2C adresi genellikle 0x76 veya 0x77
    Serial.println("BMP280 sensor bulunamadi!");
    while(1); // Program dur
  }
  Serial.println("BMP280 sensor baslatildi");
  
  // MPU6050 sensörünü başlat
  Serial.println("MPU6050 baslatiluyor...");
  if (!mpu.begin(0x69)) {  // Test kodunda çalışan adres
    Serial.println("MPU6050 sensor bulunamadi! I2C adreslerini kontrol ediliyor...");
    
    // I2C adreslerini tara
    Serial.println("I2C adres taramasi:");
    for (byte address = 1; address < 127; address++) {
      Wire.beginTransmission(address);
      if (Wire.endTransmission() == 0) {
        Serial.print("I2C cihaz bulundu: 0x");
        if (address < 16) Serial.print("0");
        Serial.println(address, HEX);
      }
    }
    
    // MPU6050'yi varsayılan adresle dene
    Serial.println("MPU6050 0x68 adresi ile deneniyor...");
    if (!mpu.begin(0x68)) {
      Serial.println("MPU6050 hala bulunamadi!");
      while(1); // Program dur
    }
  }
  Serial.println("MPU6050 sensor baslatildi");
  
  // DS3231 RTC modülünü başlat
  Serial.println("DS3231 RTC baslatiluyor...");
  if (!rtc.begin()) {
    Serial.println("DS3231 RTC bulunamadi!");
    while(1); // Program dur
  }
  Serial.println("DS3231 RTC baslatildi");
  
  // RTC zamanını kontrol et
  if (rtc.lostPower()) {
    Serial.println("RTC guc kaybetti, varsayilan zaman ayarlaniyor...");
    // Varsayılan olarak derleme zamanını ayarla
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  
  // Mevcut RTC zamanını göster
  printRTCDateTime();
  Serial.println("RTC tarih/saat ayarlamak icin 'SET DD,MM,YY,HH,MM,SS' komutunu kullanin");
  Serial.println("Ornek: SET 06,08,25,17,25,00 (6 Agustos 2025, 17:25:00)");
  
  // MPU6050 ayarları
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("MPU6050 konfigurasyonu tamamlandi");
  
  // MPU6050 ilk kalibrasyonu için bekleme
  Serial.println("MPU6050 kalibrasyon icin 1 saniye bekleniyor...");
  delay(1000);
  
  // İlk okuma yaparak referans değerleri ayarla
  readMPU6050();
  Serial.println("MPU6050 referans kalibrasyonu tamamlandi");
  
  E22.begin();
  configureLoRa();
  Serial.println("LORA 1 - Ana Kontrol Merkezi baslatildi");
  
  // Başlangıç RHRH değerini göster
  char rhrh_str[5];
  decodeRHRH(current_rhrh, rhrh_str);
  Serial.print("Baslangic RHRH degeri: ");
  Serial.println(rhrh_str);
  Serial.println("RHRH degeri lora_2'den gelen Button Control paketleri ile guncellenecek.");
  Serial.println();
  
  printModuleInfo();
  printConfig();
  delay(500);
}

void sendTelemetryToLora2() {
  // MPU6050'den açı değerlerini oku
  readMPU6050();
  
  // Binary telemetry paketi oluştur
  TelemetryPacket telemetry;

  // BMP280'den basınç ve sıcaklık verilerini oku
  float bmp_basinc = bmp.readPressure(); // Pascal cinsinden
  float bmp_sicaklik = bmp.readTemperature(); // Celsius cinsinden

  // İlk okuma ise referans basınç değerlerini kaydet
  if (!basinc_kalibrasyonu) {
    basinc1_referans = bmp_basinc;
    if (basinc_lora3 > 0) { // Lora3'ten veri gelmişse onu da referans olarak kaydet
      basinc2_referans = basinc_lora3;
    } else {
      basinc2_referans = bmp_basinc; // Henüz veri gelmemişse aynı değeri kullan
    }
    basinc_kalibrasyonu = true;
    Serial.println("Basinc referans degerleri kaydedildi (ilk okuma sifir yukseklik kabul edildi)");
    Serial.print("Basinc1 referans: "); Serial.print(basinc1_referans); Serial.println(" Pa");
    Serial.print("Basinc2 referans: "); Serial.print(basinc2_referans); Serial.println(" Pa");
  }

  telemetry.packet_type = PACKET_TYPE_TELEMETRY;
  telemetry.paket_sayisi = paket_sayisi_lora2;
  telemetry.uydu_statusu = 5;
  telemetry.hata_kodu = 0b010101; // 010101 binary (6 bit)
  telemetry.gonderme_saati = getRTCUnixTime(); // RTC'den gerçek zaman
  telemetry.basinc1 = bmp_basinc; // BMP280'den gelen basınç
  telemetry.basinc2 = basinc_lora3; // Lora 3'ten gelen basınç
  telemetry.sicaklik = (int16_t)(bmp_sicaklik * 100); // BMP280'den gelen sıcaklık * 100
  telemetry.pil_gerilimi = 377; // 3.77V * 100
  telemetry.gps1_latitude = 39.123456;
  telemetry.gps1_longitude = 32.654321;
  telemetry.gps1_altitude = 155.20;
  telemetry.pitch = (int16_t)(pitch * 10); // MPU6050'den gelen pitch değeri * 10
  telemetry.roll = (int16_t)(roll * 10); // MPU6050'den gelen roll değeri * 10
  telemetry.yaw = (int16_t)(yaw * 10); // MPU6050'den gelen yaw değeri * 10
  telemetry.rhrh = current_rhrh; // Güncel RHRH değerini kullan
  telemetry.iot_s1_data = sicaklik_lora4; // Lora 4'ten gelen sıcaklık
  telemetry.iot_s2_data = sicaklik_lora5; // Lora 5'ten gelen sıcaklık
  telemetry.takim_no = TAKIM_NO;
  
  // Basınç değerlerinden yükseklikleri hesapla (referans değerlere göre)
  float yukseklik1 = calculateAltitude(bmp_basinc, basinc1_referans);
  float yukseklik2 = calculateAltitude(basinc_lora3, basinc2_referans);
  
  // İrtifa farkını hesapla (mutlak değer)
  float irtifa_farki = abs(yukseklik1 - yukseklik2);
  
  // İniş hızını hesapla
  unsigned long current_time = millis();
  float inis_hizi = calculateDescentRate(yukseklik1, current_time);

  telemetry.yukseklik1 = yukseklik1;
  telemetry.yukseklik2 = yukseklik2;
  telemetry.irtifa_farki = irtifa_farki;
  telemetry.inis_hizi = inis_hizi;
  
  ResponseStatus rs = E22.sendFixedMessage(0x00, 0x0A, 20, (uint8_t*)&telemetry, sizeof(telemetry));
  
  // Gönderilen RHRH değerini göster
  char rhrh_str[5];
  decodeRHRH(current_rhrh, rhrh_str);
  
  Serial.print("Lora2'ye binary telemetry gonderildi (");
  Serial.print(sizeof(telemetry));
  Serial.print(" bytes), Paket#: ");
  Serial.print(paket_sayisi_lora2);
  Serial.print(", RHRH: ");
  Serial.print(rhrh_str);
  Serial.print(", BMP280 Basinc: ");
  Serial.print(bmp_basinc);
  Serial.print(" Pa, BMP280 Sicaklik: ");
  Serial.print(bmp_sicaklik);
  Serial.print("°C");
  Serial.print(", MPU6050 P/R/Y: ");
  Serial.print(pitch);
  Serial.print("/");
  Serial.print(roll);
  Serial.print("/");
  Serial.print(yaw);
  Serial.println("°");
  Serial.print("Durum: ");
  Serial.println(rs.getResponseDescription());
  
  paket_sayisi_lora2++;
}

bool waitForMessage(unsigned long timeout_ms) {
  unsigned long start_time = millis();
  while (millis() - start_time < timeout_ms) {
    if (E22.available() > 1) {
      ResponseContainer rs = E22.receiveMessage();
      
      // Binary veri olarak işle
      uint8_t* data = (uint8_t*)rs.data.c_str();
      int length = rs.data.length();
      
      if (length > 0) {
        uint8_t packet_type = data[0];
        Serial.print("Binary paket alindi - Tip: 0x");
        Serial.print(packet_type, HEX);
        Serial.print(", Boyut: ");
        Serial.print(length);
        Serial.println(" bytes");
        
        // Paket tipine göre işle
        switch (packet_type) {
          case PACKET_TYPE_BUTTON_CONTROL:
            if (length >= sizeof(ButtonControlPacket)) {
              ButtonControlPacket* btn = (ButtonControlPacket*)data;
              Serial.print("\n2222222222 Button Control - Paket#: ");
              Serial.print(btn->paket_sayisi);
              Serial.print(", Manuel Ayrilma: ");
              Serial.print(btn->manuel_ayrilma ? "true" : "false");
              
              // RHRH değerini güncelle
              uint32_t eski_rhrh = current_rhrh;
              current_rhrh = btn->rhrh;
              
              char rhrh_str[5];
              decodeRHRH(btn->rhrh, rhrh_str);
              Serial.print(", RHRH: ");
              Serial.print(rhrh_str);
              
              // Eğer RHRH değişmişse bilgi ver
              if (eski_rhrh != current_rhrh) {
                char eski_rhrh_str[5];
                decodeRHRH(eski_rhrh, eski_rhrh_str);
                Serial.print(" (RHRH guncellendi: ");
                Serial.print(eski_rhrh_str);
                Serial.print(" -> ");
                Serial.print(rhrh_str);
                Serial.print(")");
              }
              Serial.println();
            }
            break;
            
          case PACKET_TYPE_PRESSURE_CONTAINER:
            if (length >= sizeof(PressureContainerPacket)) {
              PressureContainerPacket* prs = (PressureContainerPacket*)data;
              Serial.print("\n3333333333 Pressure Container - Paket#: ");
              Serial.print(prs->paket_sayisi);
              Serial.print(", Basinc: ");
              Serial.println(prs->basinc1);
              
              // Lora 3'ten gelen basınç değerini güncelle
              basinc_lora3 = prs->basinc1;
              
              // Eğer henüz basınç kalibrasyonu yapılmamışsa ve ilk lora3 verisi gelmişse
              // referans değeri güncelle
              if (basinc_kalibrasyonu && basinc2_referans == basinc1_referans) {
                basinc2_referans = prs->basinc1;
                Serial.print("Basinc2 referans guncellendi: "); 
                Serial.print(basinc2_referans); 
                Serial.println(" Pa");
              }
            }
            break;
            
          case PACKET_TYPE_L4_DATA:
            if (length >= sizeof(LoraDataPacket)) {
              LoraDataPacket* l4data = (LoraDataPacket*)data;
              if (validatePacket(data, length, PACKET_TYPE_L4_DATA)) {
                Serial.print("\n4444444444 L4 Data - Paket#: ");
                Serial.print(l4data->paket_sayisi);
                Serial.print(", Sicaklik: ");
                Serial.println(l4data->temperature / 100.0);
                
                // Lora 4'ten gelen sıcaklık değerini güncelle
                sicaklik_lora4 = l4data->temperature;
              } else {
                Serial.println("L4 Data - Checksum hatasi!");
              }
            }
            break;
            
          case PACKET_TYPE_L5_DATA:
            if (length >= sizeof(LoraDataPacket)) {
              LoraDataPacket* l5data = (LoraDataPacket*)data;
              if (validatePacket(data, length, PACKET_TYPE_L5_DATA)) {
                Serial.print("\n5555555555 L5 Data - Paket#: ");
                Serial.print(l5data->paket_sayisi);
                Serial.print(", Sicaklik: ");
                Serial.println(l5data->temperature / 100.0);
                
                // Lora 5'ten gelen sıcaklık değerini güncelle
                sicaklik_lora5 = l5data->temperature;
              } else {
                Serial.println("L5 Data - Checksum hatasi!");
              }
            }
            break;
            
          default:
            Serial.print("Bilinmeyen paket tipi: 0x");
            Serial.println(packet_type, HEX);
            break;
        }
        return true;
      }
    }
    delay(10);
  }
  Serial.println("Timeout - veri alinamadi");
  return false;
}

void loop() {
  Serial.println("=== COMM LOOP BASLADI ===");
  
  // Serial komutları kontrol et
  processSerialCommand();
  
  // Zamanlayıcı başlat
  unsigned long loop_start_time = millis();
  
  // 1. Lora 2'ye telemetry gönder
  sendTelemetryToLora2();
  
  // 2. 1 saniye dolana kadar dinleme modunda ol
  while (millis() - loop_start_time < 950) {
    // Serial komutları kontrol et
    processSerialCommand();
    
    if (E22.available() > 1) {
      waitForMessage(100); // Kısa timeout ile mesaj işle
    }
    delay(10); // Buffer yönetimi için kısa bekleme
  }
  
  Serial.println("=== COMM LOOP TAMAMLANDI ===");
}
