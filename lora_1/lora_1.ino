#include <Arduino.h>
#include <LoRa_E22.h>
#include <Adafruit_BMP280.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <RTClib.h>
#include <SD.h>
#include <FS.h>
#include <SPI.h>
#include <ESP32Servo.h>
#include "../binary_protocol.h"

#define LORA_M0 13
#define LORA_M1 12
#define LORA_AUX 4
#define LORA_TX 16
#define LORA_RX 17

#define LORA_CHANNEL 10
#define LORA_ADDR_H 0x00
#define LORA_ADDR_L 0x0A             

#define PIL_GERILIMI_PIN A0
#define SEPERATION 35
#define UYDU_STATU 34

// SD Kart SPI pin tanımlamaları
#define SD_SCK 18
#define SD_MISO 19
#define SD_MOSI 23
#define SD_CS 5             

#define SERVO1_PIN 33
#define SERVO2_PIN 25

// DC Motor pin tanımlamaları
#define DC_ILERI 26
#define DC_GERI 32

// Renk pozisyonları (derece cinsinden)
#define POS_BOS 40      // Boş pozisyon
#define POS_KIRMIZI 80 // Kırmızı pozisyon
#define POS_MAVI 120   // Mavi pozisyon
#define POS_YESIL 0  // Yeşil pozisyon             

LoRa_E22 E22(&Serial2, LORA_AUX, LORA_M0, LORA_M1);

// BMP280 sensor
Adafruit_BMP280 bmp;

// MPU6050 sensor
Adafruit_MPU6050 mpu;

// DS3231 RTC modülü
RTC_DS3231 rtc;

// Servo motorlar
Servo servo1;
Servo servo2;

uint16_t paket_sayisi_lora2 = 1;
uint16_t paket_sayisi_lora3 = 1;

// RHRH değeri - başlangıçta 0A0A, lora_2'den gelen değerle güncellenecek
uint32_t current_rhrh = encodeRHRH('0', 'A', '0', 'A');

// RHRH işleme için değişkenler
bool rhrh_processing = false;
unsigned long rhrh_start_time = 0;
uint8_t rhrh_step = 0;
String rhrh_command = "";

// Lora 2'den gelen manuel ayrılma değeri
uint8_t current_manuel_ayrilma = 0; // 0=normal, 1=ayrılma, 2=birleşme

// DC Motor kontrolü için değişkenler
bool dc_motor_running = false;
unsigned long dc_motor_start_time = 0;
unsigned long dc_motor_duration = 0;
uint8_t dc_motor_direction = 0; // 0=durdur, 1=ileri, 2=geri

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

// Sensör durumları
bool bmp_status = false;
bool mpu_status = false;
bool rtc_status = false;
bool gps_status = false;
bool sd_status = false;
bool adc_status = false;
bool lora_status = false;

// Servo kontrolü için yardımcı fonksiyonlar
void initServos() {
  // Servo motorları attach et
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
}

// DC Motor kontrolü için yardımcı fonksiyonlar
void initDCMotor() {
  // DC motor pinlerini output olarak ayarla
  pinMode(DC_ILERI, OUTPUT);
  pinMode(DC_GERI, OUTPUT);
  
  // Başlangıçta motoru durdur
  digitalWrite(DC_ILERI, LOW);
  digitalWrite(DC_GERI, LOW);
  Serial.println("DC Motor pinleri baslatildi");
}

// DC Motor hareket fonksiyonları
void startDCMotor(uint8_t direction, unsigned long duration_ms) {
  // Önce motoru durdur
  digitalWrite(DC_ILERI, LOW);
  digitalWrite(DC_GERI, LOW);
  
  dc_motor_direction = direction;
  dc_motor_duration = duration_ms;
  dc_motor_start_time = millis();
  dc_motor_running = true;
  
  // Yönü ayarla
  if (direction == 1) { // İleri
    digitalWrite(DC_ILERI, HIGH);
    digitalWrite(DC_GERI, LOW);
    Serial.print("DC Motor ILERI basladi - Sure: ");
    Serial.print(duration_ms);
    Serial.println(" ms");
  } else if (direction == 2) { // Geri
    digitalWrite(DC_ILERI, LOW);
    digitalWrite(DC_GERI, HIGH);
    Serial.print("DC Motor GERI basladi - Sure: ");
    Serial.print(duration_ms);
    Serial.println(" ms");
  }
}

void stopDCMotor() {
  digitalWrite(DC_ILERI, LOW);
  digitalWrite(DC_GERI, LOW);
  dc_motor_running = false;
  Serial.println("DC Motor durduruldu");
}

// DC Motor durumunu güncelleme fonksiyonu (loop'ta çağrılacak)
void updateDCMotor() {
  if (!dc_motor_running) {
    return;
  }
  
  // Süre doldu mu kontrol et
  if (millis() - dc_motor_start_time >= dc_motor_duration) {
    stopDCMotor();
  }
}

// Servo pozisyon ayarlama fonksiyonu
void writeServo(int pin, int angle) {
  // Açıyı sınırla (0-180 derece)
  angle = constrain(angle, 0, 180);
  
  // İlgili servo motoruna açıyı yaz
  if (pin == SERVO1_PIN) {
    servo1.write(angle);
  } else if (pin == SERVO2_PIN) {
    servo2.write(angle);
  }
}

// Servo pozisyon ayarlama fonksiyonu
void setServoPositions(char color_code) {
  int servo1_pos = POS_BOS;
  int servo2_pos = POS_BOS;
  
  switch (color_code) {
    case 'M': // kırmızı + kırmızı
      servo1_pos = POS_KIRMIZI;
      servo2_pos = POS_KIRMIZI;
      break;
    case 'F': // yeşil + yeşil
      servo1_pos = POS_YESIL;
      servo2_pos = POS_YESIL;
      break;
    case 'N': // mavi + mavi
      servo1_pos = POS_MAVI;
      servo2_pos = POS_MAVI;
      break;
    case 'R': // kırmızı + boş
      servo1_pos = POS_KIRMIZI;
      servo2_pos = POS_BOS;
      break;
    case 'G': // yeşil + boş
      servo1_pos = POS_YESIL;
      servo2_pos = POS_BOS;
      break;
    case 'B': // mavi + boş
      servo1_pos = POS_MAVI;
      servo2_pos = POS_BOS;
      break;
    case 'P': // kırmızı + mavi
      servo1_pos = POS_KIRMIZI;
      servo2_pos = POS_MAVI;
      break;
    case 'Y': // kırmızı + yeşil
      servo1_pos = POS_KIRMIZI;
      servo2_pos = POS_YESIL;
      break;
    case 'C': // mavi + yeşil
      servo1_pos = POS_MAVI;
      servo2_pos = POS_YESIL;
      break;
    case 'A': // boş + boş (varsayılan)
    default:
      servo1_pos = POS_BOS;
      servo2_pos = POS_BOS;
      break;
  }
  
  // Servo pozisyonlarını ayarla
  writeServo(SERVO1_PIN, servo1_pos);
  writeServo(SERVO2_PIN, servo2_pos);
  
  Serial.print("Servo pozisyonları ayarlandı - Renk: ");
  Serial.print(color_code);
  Serial.print(", Servo1: ");
  Serial.print(servo1_pos);
  Serial.print("°, Servo2: ");
  Serial.print(servo2_pos);
  Serial.println("°");
}

// RHRH komutunu başlatma fonksiyonu
void startRHRHCommand(uint32_t rhrh_value) {
  char rhrh_str[5];
  decodeRHRH(rhrh_value, rhrh_str);
  
  // 0A0A ise işlem yapma
  if (strcmp(rhrh_str, "0A0A") == 0) {
    return;
  }
  
  Serial.print("RHRH komutu başlatılıyor: ");
  Serial.println(rhrh_str);
  
  rhrh_command = String(rhrh_str);
  rhrh_processing = true;
  rhrh_start_time = millis();
  rhrh_step = 0;
  
  // İlk adımı başlat
  processRHRHStep();
}

// RHRH adımını işleme fonksiyonu
void processRHRHStep() {
  if (!rhrh_processing || rhrh_command.length() < 4) {
    return;
  }
  
  if (rhrh_step >= 2) {
    // Tüm adımlar tamamlandı
    Serial.println("RHRH komutu tamamlandı - 0A0A pozisyonuna dönülüyor");
    setServoPositions('A'); // Boş pozisyona dön
    current_rhrh = encodeRHRH('0', 'A', '0', 'A');
    rhrh_processing = false;
    return;
  }
  
  // Mevcut adımın süresini ve rengini al
  char duration_char = rhrh_command.charAt(rhrh_step * 2);
  char color_char = rhrh_command.charAt(rhrh_step * 2 + 1);
  
  int duration = duration_char - '0'; // Karakter rakamı integer'a çevir
  
  Serial.print("RHRH Adım ");
  Serial.print(rhrh_step + 1);
  Serial.print(": ");
  Serial.print(duration);
  Serial.print(" saniye ");
  Serial.print(color_char);
  Serial.println(" rengi");
  
  // Servo pozisyonlarını ayarla
  setServoPositions(color_char);
  
  // Süreyi kaydet (milisaniye cinsinden)
  rhrh_start_time = millis();
  rhrh_step++;
}

// RHRH işlemini güncelleme fonksiyonu (loop'ta çağrılacak)
void updateRHRH() {
  if (!rhrh_processing) {
    return;
  }
  
  // Mevcut adımın süresini kontrol et
  char duration_char = rhrh_command.charAt((rhrh_step - 1) * 2);
  int duration = duration_char - '0';
  unsigned long duration_ms = duration * 1000; // Saniyeyi milisaniyeye çevir
  
  if (millis() - rhrh_start_time >= duration_ms) {
    // Mevcut adım tamamlandı, bir sonrakine geç
    processRHRHStep();
  }
}

// Manuel işlem fonksiyonları
void manuelAyrilma() {
  Serial.println("MANUEL AYRILMA islemi baslatildi");
  
  // DC motoru 10 saniye ileri döndür
  startDCMotor(1, 10000); // 1=ileri, 10000ms=10 saniye
  
  // Manuel ayrılma değerini sıfırla
  current_manuel_ayrilma = 0;
}

void manuelBirlesme() {
  Serial.println("MANUEL BIRLESME islemi baslatildi");
  
  // DC motoru 1 saniye geri döndür
  startDCMotor(2, 1000); // 2=geri, 1000ms=1 saniye
  
  // Manuel ayrılma değerini sıfırla
  current_manuel_ayrilma = 0;
}

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
  if (!rtc_status) {
    // RTC aktif değilse millis() bazlı bir timestamp döndür
    return (millis() / 1000) + 1751328000; // 6 Temmuz 2025'ten itibaren saniye
  }
  
  DateTime now = rtc.now();
  uint32_t timestamp = now.unixtime();
  
  // Mantıklı bir tarih aralığında olup olmadığını kontrol et
  if (timestamp < 1751328000 || timestamp > 1782864000) { // 2025-2026 arası
    Serial.println("RTC zaman hatasi - millis() bazli zaman kullaniliyor");
    rtc_status = false;
    return (millis() / 1000) + 1751328000; // 6 Temmuz 2025'ten itibaren
  }
  
  return timestamp;
}

// RTC tarih/saat bilgisi alma fonksiyonu
void printRTCDateTime() {
  if (!rtc_status) {
    Serial.println("RTC aktif degil - zaman bilgisi alinamadi");
    return;
  }
  
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
    // SD durum komutunu kontrol et
    else if (command == "SD_STATUS") {
      Serial.print("SD Kart Durumu: ");
      Serial.println(sd_status ? "AKTIF" : "HATA");
      if (sd_status) {
        Serial.print("Dosya: /lora1_data.csv - ");
        if (SD.exists("/lora1_data.csv")) {
          File file = SD.open("/lora1_data.csv", FILE_READ);
          if (file) {
            Serial.print("Boyut: ");
            Serial.print(file.size());
            Serial.println(" bytes");
            file.close();
          }
        } else {
          Serial.println("Dosya bulunamadı");
        }
      }
    }
    // SD test komutunu kontrol et
    else if (command == "SD_TEST") {
      if (sd_status) {
        String testData = "Test verisi: " + String(millis()) + "\n";
        if (appendFile(SD, "/lora1_test.txt", testData.c_str())) {
          Serial.println("SD test yazma başarılı");
        } else {
          Serial.println("SD test yazma hatası");
        }
      } else {
        Serial.println("SD kart aktif değil");
      }
    }
    // SERVO komutunu kontrol et (format: SERVO angle1 angle2)
    else if (command.startsWith("SERVO ")) {
      String servoParams = command.substring(6); // "SERVO " kısmını atla
      
      // Boşluklarla ayrılmış değerleri parse et
      int spaceIndex = servoParams.indexOf(' ');
      if (spaceIndex > 0) {
        int angle1 = servoParams.substring(0, spaceIndex).toInt();
        int angle2 = servoParams.substring(spaceIndex + 1).toInt();
        
        // Açı değerlerini kontrol et (0-180 derece arası)
        if (angle1 >= 0 && angle1 <= 180 && angle2 >= 0 && angle2 <= 180) {
          // Servo pozisyonlarını ayarla
          writeServo(SERVO1_PIN, angle1);
          writeServo(SERVO2_PIN, angle2);
          
          Serial.print("Servo pozisyonları manuel olarak ayarlandı - Servo1: ");
          Serial.print(angle1);
          Serial.print("°, Servo2: ");
          Serial.print(angle2);
          Serial.println("°");
        } else {
          Serial.println("Hata: Servo açı değerleri 0-180 derece arasında olmalıdır!");
          Serial.println("Kullanım: SERVO angle1 angle2");
          Serial.println("Örnek: SERVO 90 45");
        }
      } else {
        Serial.println("Hata: Geçersiz format! Kullanım: SERVO angle1 angle2");
        Serial.println("Örnek: SERVO 90 45");
      }
    }
  }
}

// Sensörlerin durumunu kontrol eden fonksiyon
void sensorControl() {
  static unsigned long lastSensorCheck = 0;
  
  // Her 5 saniyede bir sensör kontrolü yap
  if (millis() - lastSensorCheck < 5000) {
    return;
  }
  
  lastSensorCheck = millis();
  bool status_changed = false;
  
  // BMP280 durumunu kontrol et
  bool old_bmp = bmp_status;
  float test_pressure = bmp.readPressure();
  bmp_status = (test_pressure > 0 && test_pressure < 200000); // Mantıklı basınç aralığı (0-200kPa)
  if (old_bmp != bmp_status) {
    Serial.print("BMP280 durum degisikliği: ");
    Serial.println(bmp_status ? "AKTIF" : "HATA");
    status_changed = true;
  }
  
  // MPU6050 durumunu kontrol et
  bool old_mpu = mpu_status;
  sensors_event_t accel, gyro, temp;
  bool mpu_read_success = mpu.getEvent(&accel, &gyro, &temp);
  // Accelerometer değerlerinin mantıklı olup olmadığını kontrol et
  mpu_status = (mpu_read_success && 
                abs(accel.acceleration.x) < 50 && 
                abs(accel.acceleration.y) < 50 && 
                abs(accel.acceleration.z) < 50);
  if (old_mpu != mpu_status) {
    Serial.print("MPU6050 durum degisikliği: ");
    Serial.println(mpu_status ? "AKTIF" : "HATA");
    status_changed = true;
  }
  
  // RTC durumunu kontrol et
  bool old_rtc = rtc_status;
  DateTime now = rtc.now();
  // RTC'nin mantıklı bir tarih verip vermediğini kontrol et
  rtc_status = (now.year() >= 2025 && now.year() <= 2027 && 
                now.month() >= 1 && now.month() <= 12 &&
                now.day() >= 1 && now.day() <= 31);
  if (old_rtc != rtc_status) {
    Serial.print("RTC durum degisikliği: ");
    Serial.println(rtc_status ? "AKTIF" : "HATA");
    status_changed = true;
  }
  
  // SD Kart durumunu kontrol et
  bool old_sd = sd_status;
  if (sd_status) {
    // SD kart aktifse, dosya erişimi test et
    if (!SD.exists("/lora1_data.csv")) {
      Serial.println("SD kart dosyası bulunamadı - yeniden bağlanmaya çalışılıyor...");
      sd_status = false;
    }
  }
  
  if (!sd_status) {
    // SD kart aktif değilse yeniden başlatmaya çalış
    if (SD.begin(SD_CS)) {
      sd_status = true;
      
      // CSV dosyası yoksa oluştur
      if (!SD.exists("/lora1_data.csv")) {
        writeFile(SD, "/lora1_data.csv", "paket_sayisi,uydu_statusu,hata_kodu,gonderme_saati,basinc1,basinc2,yukseklik1,yukseklik2,irtifa_farki,inis_hizi,sicaklik,pil_gerilimi,gps1_latitude,gps1_longitud,gps1_altitude,pitch,roll,yaw,rhrh,iot_s1_data,iot_s2_data,takim_no\n");
      }
    }
  }
  
  if (old_sd != sd_status) {
    Serial.print("SD Kart durum degisikli: ");
    Serial.println(sd_status ? "AKTIF" : "HATA");
    status_changed = true;
  }
  
  // LoRa durumunu kontrol et
  bool old_lora = lora_status;
  // LoRa modülünün haberleşme durumunu kontrol et
  ResponseStructContainer c = E22.getConfiguration();
  lora_status = (c.status.code == 1); // Başarılı yanıt alındıysa LoRa aktif
  c.close();
  
  if (old_lora != lora_status) {
    Serial.print("LoRa durum degisikli: ");
    Serial.println(lora_status ? "AKTIF" : "HATA");
    status_changed = true;
  }
  
  // Durum değişikliği olduysa genel sensor durumunu yazdır
  if (status_changed) {
    Serial.println("\n=== SENSOR DURUMLARI ===");
    Serial.print("BMP280: "); Serial.println(bmp_status ? "AKTIF" : "HATA");
    Serial.print("MPU6050: "); Serial.println(mpu_status ? "AKTIF" : "HATA");
    Serial.print("RTC: "); Serial.println(rtc_status ? "AKTIF" : "HATA");
    Serial.print("SD Kart: "); Serial.println(sd_status ? "AKTIF" : "HATA");
    Serial.print("LoRa: "); Serial.println(lora_status ? "AKTIF" : "HATA");
    Serial.print("GPS: "); Serial.println(gps_status ? "AKTIF" : "DEVRE DISI");
    Serial.print("ADC: "); Serial.println(adc_status ? "AKTIF" : "DEVRE DISI");
    Serial.println("========================\n");
  }
}

void checkSDStatus() {
  // Bu fonksiyon artık sensorControl() içinde çalışıyor
  // Geriye uyumluluk için boş bırakıldı
}

// SD Kart fonksiyonları
void initSDCard() {
  Serial.print("SD Kart... ");
  
  if (!SD.begin(SD_CS)) {
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
  
  if (!SD.exists("/lora1_data.csv")) {
    writeFile(SD, "/lora1_data.csv", "paket_sayisi,uydu_statusu,hata_kodu,gonderme_saati,basinc1,basinc2,yukseklik1,yukseklik2,irtifa_farki,inis_hizi,sicaklik,pil_gerilimi,gps1_latitude,gps1_longitud,gps1_altitude,pitch,roll,yaw,rhrh,iot_s1_data,iot_s2_data,takim_no\n");
  }
}

void writeFile(fs::FS &fs, const char *path, const char *message) {
  File file = fs.open(path, FILE_WRITE);
  if (file) {
    file.print(message);
    file.close();
  }
}

bool appendFile(fs::FS &fs, const char *path, const char *message) {
  File file = fs.open(path, FILE_APPEND);
  if (!file) return false;
  
  size_t bytesWritten = file.print(message);
  file.close();
  return bytesWritten > 0;
}

void saveToSD(TelemetryPacket data, float bmp_pressure, float bmp_temperature) {
  if (!sd_status) return;
  
  // RHRH değerini string'e çevir
  char rhrh_str[5];
  decodeRHRH(data.rhrh, rhrh_str);
  
  // Timestamp'i okunabilir formata çevir
  char timestamp_str[20];
  if (rtc_status) {
    DateTime now = rtc.now();
    sprintf(timestamp_str, "%02d.%02d.%04d:%02d.%02d.%02d", 
            now.day(), now.month(), now.year(), 
            now.hour(), now.minute(), now.second());
  } else {
    // RTC aktif değilse millis() bazlı timestamp kullan
    unsigned long mil = millis();
    sprintf(timestamp_str, "MILLIS:%lu", mil);
  }
  
  // CSV satırını belirtilen sırayla oluştur
  String csvLine = String(data.paket_sayisi) + "," +                    // paket_sayisi
                   String(data.uydu_statusu) + "," +                    // uydu_statusu
                   String(data.hata_kodu) + "," +                       // hata_kodu
                   String(timestamp_str) + "," +                        // gonderme_saati (formatted)
                   String(data.basinc1, 1) + "," +                      // basinc1
                   String(data.basinc2, 1) + "," +                      // basinc2
                   String(data.yukseklik1, 1) + "," +                   // yukseklik1
                   String(data.yukseklik2, 1) + "," +                   // yukseklik2
                   String(data.irtifa_farki, 1) + "," +                 // irtifa_farki
                   String(data.inis_hizi, 1) + "," +                    // inis_hizi
                   String(data.sicaklik / 100.0, 1) + "," +             // sicaklik
                   String(data.pil_gerilimi / 100.0, 3) + "," +         // pil_gerilimi
                   String(data.gps1_latitude, 6) + "," +                // gps1_latitude
                   String(data.gps1_longitude, 6) + "," +               // gps1_longitud
                   String(data.gps1_altitude, 1) + "," +                // gps1_altitude
                   String(data.pitch / 10.0, 1) + "," +                 // pitch
                   String(data.roll / 10.0, 1) + "," +                  // roll
                   String(data.yaw / 10.0, 1) + "," +                   // yaw
                   String(rhrh_str) + "," +                             // rhrh
                   String(data.iot_s1_data / 100.0, 1) + "," +          // iot_s1_data
                   String(data.iot_s2_data / 100.0, 1) + "," +          // iot_s2_data
                   String(data.takim_no) + "\n";                        // takim_no
  
  if (!appendFile(SD, "/lora1_data.csv", csvLine.c_str())) {
    sd_status = false;
  }
}

void loadLastPacketNumber() {
  if (!sd_status || !SD.exists("/lora1_data.csv")) {
    return;
  }
  
  File file = SD.open("/lora1_data.csv", FILE_READ);
  if (!file) return;
  
  String lastLine = "";
  String currentLine = "";
  
  // Dosyayı satır satır okuyarak en son satırı bul
  while (file.available()) {
    char c = file.read();
    if (c == '\n') {
      if (currentLine.length() > 0 && !currentLine.startsWith("paket_sayisi")) {
        lastLine = currentLine;
      }
      currentLine = "";
    } else if (c != '\r') {
      currentLine += c;
    }
  }
  
  // Son satır newline ile bitmiyorsa
  if (currentLine.length() > 0 && !currentLine.startsWith("paket_sayisi")) {
    lastLine = currentLine;
  }
  
  file.close();
  
  if (lastLine.length() > 0) {
    int commaIndex = lastLine.indexOf(',');
    if (commaIndex > 0) {
      uint16_t lastPacketNumber = lastLine.substring(0, commaIndex).toInt();
      if (lastPacketNumber > 0) {
        paket_sayisi_lora2 = lastPacketNumber + 1;
        Serial.printf("SD karttan son paket numarası yüklendi: %d (yeni: %d)\n", lastPacketNumber, paket_sayisi_lora2);
      }
    }
  }
}

// MPU6050'den açı değerlerini oku
void readMPU6050() {
  // MPU6050 aktif değilse varsayılan değerleri kullan
  if (!mpu_status) {
    pitch = 0.0;
    roll = 0.0;
    yaw = 0.0;
    return;
  }
  
  sensors_event_t accel, gyro, temp;
  bool read_success = mpu.getEvent(&accel, &gyro, &temp);
  
  // Okuma başarısızsa varsayılan değerleri kullan
  if (!read_success) {
    Serial.println("MPU6050 okuma hatasi - varsayilan degerler kullaniliyor");
    pitch = 0.0;
    roll = 0.0;
    yaw = 0.0;
    mpu_status = false; // Status'u güncelle
    return;
  }
  
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

// Hata kodu hesaplama fonksiyonu
uint8_t calculateHataKodu(int uydu_statusu, float inis_hizi, float tasiyici_basinc) {
  uint8_t hata_kodu = 0;
  
  // 1. basamak (en soldaki): uydu_status == 2 ve inis_hizi 12-14 arası
  bool basamak1_hatasiz = (uydu_statusu == 2 && inis_hizi >= 12.0 && inis_hizi <= 14.0);
  if (!basamak1_hatasiz) {
    hata_kodu |= (1 << 5); // 6. bitten başlayarak, 1. basamak = 6. bit
  }
  
  // 2. basamak: uydu_status == 4 ve inis_hizi 6-8 arası
  bool basamak2_hatasiz = (uydu_statusu == 4 && inis_hizi >= 6.0 && inis_hizi <= 8.0);
  if (!basamak2_hatasiz) {
    hata_kodu |= (1 << 4); // 2. basamak = 5. bit
  }
  
  // 3. basamak: basinc2 değeri sıfıra eşitse hatalı
  bool basamak3_hatalik = (basinc_lora3 == 0.0);
  if (basamak3_hatalik) {
    hata_kodu |= (1 << 3); // 3. basamak = 4. bit
  }
  
  // Debug için 3. basamak durumunu yazdır
  Serial.print("3. Basamak Debug - Basinc2 (basinc_lora3): ");
  Serial.print(basinc_lora3);
  Serial.print(", Basamak3 Hatalik: ");
  Serial.println(basamak3_hatalik ? "EVET" : "HAYIR");
  
  // 4. basamak: şimdilik default olarak hatalı (1)
  hata_kodu |= (1 << 2); // 4. basamak = 3. bit
  
  // 5. basamak: SEPERATION değeri kontrol
  bool basamak5_hatasiz = (digitalRead(SEPERATION) == HIGH); // SEPERATION = 1 (HIGH) ise hatasız
  if (!basamak5_hatasiz) {
    hata_kodu |= (1 << 1); // 5. basamak = 2. bit
  }
  
  // 6. basamak: şimdilik default olarak hatalı (1)
  hata_kodu |= (1 << 0); // 6. basamak = 1. bit
  
  return hata_kodu;
}

// Buton durumlarını okuma ve yazdırma fonksiyonu
void checkButtonStates() {
  static unsigned long lastButtonCheck = 0;
  static bool lastSeparationState = HIGH;
  static bool lastUyduStatuState = HIGH;
  
  // Her 500ms'de bir buton durumlarını kontrol et
  if (millis() - lastButtonCheck >= 500) {
    lastButtonCheck = millis();
    
    bool separationState = digitalRead(SEPERATION);
    bool uyduStatuState = digitalRead(UYDU_STATU);
    
    // Durum değişikliği olduysa yazdır
    if (separationState != lastSeparationState || uyduStatuState != lastUyduStatuState) {
      Serial.print("BUTON DURUMLARI - SEPERATION: ");
      Serial.print(separationState == LOW ? "BASILDI" : "BIRAKILDI");
      Serial.print(", UYDU_STATU: ");
      Serial.println(uyduStatuState == LOW ? "BASILDI" : "BIRAKILDI");
      
      lastSeparationState = separationState;
      lastUyduStatuState = uyduStatuState;
    }
  }
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
  
  // ESP32Servo için timer ayarla
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  
  // Servo motorları başlat
  initServos();
  
  // DC motorları başlat
  initDCMotor();
  
  // Pin modlarını ayarla
  pinMode(SEPERATION, INPUT);
  pinMode(UYDU_STATU, INPUT);
  
  // Servo motorları başlangıç pozisyonuna getir (boş pozisyon)
  writeServo(SERVO1_PIN, POS_BOS);
  writeServo(SERVO2_PIN, POS_BOS);
  delay(500); // Servo motorların pozisyona gelmesi için bekle
  Serial.println("Servo motorlar baslatildi - Baslangic pozisyonu: BOS");
  
  // BMP280 sensörünü başlat
  Wire.begin(21, 22); // SDA=21, SCL=22 (ESP32 default)
  if (!bmp.begin(0x76)) {  // BMP280 I2C adresi genellikle 0x76 veya 0x77
    Serial.println("BMP280 sensor bulunamadi! - Program devam ediyor...");
    bmp_status = false;
  } else {
    bmp_status = true;
    Serial.println("BMP280 sensor baslatildi");
  }
  
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
      Serial.println("MPU6050 hala bulunamadi! - Program devam ediyor...");
      mpu_status = false;
    } else {
      mpu_status = true;
    }
  } else {
    mpu_status = true;
  }
  
  if (mpu_status) {
    Serial.println("MPU6050 sensor baslatildi");
  } else {
    Serial.println("MPU6050 sensoru devre disi");
  }
  
  // DS3231 RTC modülünü başlat
  Serial.println("DS3231 RTC baslatiluyor...");
  if (!rtc.begin()) {
    Serial.println("DS3231 RTC bulunamadi! - Program devam ediyor...");
    rtc_status = false;
  } else {
    rtc_status = true;
    Serial.println("DS3231 RTC baslatildi");
  }
  
  // RTC zamanını kontrol et
  if (rtc.lostPower()) {
    Serial.println("RTC guc kaybetti, varsayilan zaman ayarlaniyor...");
    // Varsayılan olarak derleme zamanını ayarla
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  
  // Mevcut RTC zamanını göster
  printRTCDateTime();
  Serial.println("RTC tarih/saat ayarlamak icin 'SET DD,MM,YY,HH,MM,SS' komutunu kullanin");
  Serial.println("Ornek: SET 06,07,25,17,25,00 (6 Temmuz 2025, 17:25:00)");
  
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
  
  // SD kartı başlat
  initSDCard();
  
  // SD karttan son paket numarasını yükle
  if (sd_status) {
    loadLastPacketNumber();
  }
  
  E22.begin();
  configureLoRa();
  lora_status = true; // LoRa başarıyla başlatıldı
  Serial.println("LORA 1 - Ana Kontrol Merkezi baslatildi");
  
  // Başlangıç RHRH değerini göster
  char rhrh_str[5];
  decodeRHRH(current_rhrh, rhrh_str);
  Serial.print("Baslangic RHRH degeri: ");
  Serial.println(rhrh_str);
  Serial.println("RHRH degeri lora_2'den gelen Button Control paketleri ile guncellenecek.");
  Serial.println();
  
  // Kullanılabilir komutları göster
  Serial.println("=== KULLANILABILIR KOMUTLAR ===");
  Serial.println("SET DD,MM,YY,HH,MM,SS - RTC tarih/saat ayarla");
  Serial.println("SD_STATUS - SD kart durumu goruntule");
  Serial.println("SD_TEST - SD kart test dosyası yaz");
  Serial.println("SERVO angle1 angle2 - Servo motor acılarını ayarla (0-180°)");
  Serial.println("===============================");
  Serial.println();
  
  printModuleInfo();
  printConfig();
  
  // Başlangıç sensör durumlarını göster
  Serial.println("\n=== BASLANGIC SENSOR DURUMLARI ===");
  Serial.print("BMP280: "); Serial.println(bmp_status ? "AKTIF" : "HATA");
  Serial.print("MPU6050: "); Serial.println(mpu_status ? "AKTIF" : "HATA");
  Serial.print("RTC: "); Serial.println(rtc_status ? "AKTIF" : "HATA");
  Serial.print("SD Kart: "); Serial.println(sd_status ? "AKTIF" : "HATA");
  Serial.print("LoRa: "); Serial.println(lora_status ? "AKTIF" : "HATA");
  Serial.print("GPS: "); Serial.println(gps_status ? "AKTIF" : "DEVRE DISI");
  Serial.print("ADC: "); Serial.println(adc_status ? "AKTIF" : "DEVRE DISI");
  Serial.println("==================================\n");
  
  delay(500);
}

void sendTelemetryToLora2() {
  // MPU6050'den açı değerlerini oku (hata kontrolü içinde)
  readMPU6050();
  
  // Binary telemetry paketi oluştur
  TelemetryPacket telemetry;

  // BMP280'den basınç ve sıcaklık verilerini oku (hata kontrolü ile)
  float bmp_basinc = 0.0;
  float bmp_sicaklik = 0.0;
  
  if (bmp_status) {
    bmp_basinc = bmp.readPressure(); // Pascal cinsinden
    bmp_sicaklik = bmp.readTemperature(); // Celsius cinsinden
    
    // Okunan değerlerin mantıklı olup olmadığını kontrol et
    if (bmp_basinc <= 0 || bmp_basinc > 200000 || isnan(bmp_basinc)) {
      Serial.println("BMP280 basinc okuma hatasi - varsayilan deger kullaniliyor");
      bmp_basinc = 101325.0; // Deniz seviyesi basıncı
      bmp_status = false; // Status'u güncelle
    }
    
    if (isnan(bmp_sicaklik) || bmp_sicaklik < -50 || bmp_sicaklik > 100) {
      Serial.println("BMP280 sicaklik okuma hatasi - varsayilan deger kullaniliyor");
      bmp_sicaklik = 25.0; // Oda sıcaklığı
    }
  } else {
    // BMP280 aktif değilse varsayılan değerleri kullan
    bmp_basinc = 101325.0; // Deniz seviyesi basıncı
    bmp_sicaklik = 25.0; // Oda sıcaklığı
  }

  // İlk okuma ise referans basınç değerlerini kaydet
  if (!basinc_kalibrasyonu) {
    basinc1_referans = bmp_basinc;
    // Taşıyıcı basınç referansını sadece gerçek veri geldiğinde kaydet
    if (basinc_lora3 > 0) {
      basinc2_referans = basinc_lora3;
      Serial.println("Basinc referans degerleri kaydedildi (ilk okuma sifir yukseklik kabul edildi)");
      Serial.print("Basinc1 referans: "); Serial.print(basinc1_referans); Serial.println(" Pa");
      Serial.print("Basinc2 referans: "); Serial.print(basinc2_referans); Serial.println(" Pa");
      basinc_kalibrasyonu = true;
    } else {
      // Henüz taşıyıcı verisi gelmemiş, sadece BMP280 referansını ayarla
      basinc2_referans = 0; // Geçici olarak sıfır
      Serial.println("BMP280 referans kaydedildi, tasiyici verisi bekleniyor...");
      Serial.print("Basinc1 referans: "); Serial.print(basinc1_referans); Serial.println(" Pa");
    }
  } else if (basinc2_referans == 0 && basinc_lora3 > 0) {
    // Kalibrasyon sonrasında ilk taşıyıcı verisi geldi
    basinc2_referans = basinc_lora3;
    Serial.print("Basinc2 referans guncellendi (ilk tasiyici verisi): "); 
    Serial.print(basinc2_referans); 
    Serial.println(" Pa");
  }

  telemetry.packet_type = PACKET_TYPE_TELEMETRY;
  telemetry.paket_sayisi = paket_sayisi_lora2;
  telemetry.uydu_statusu = -1; // Default değer
  telemetry.gonderme_saati = getRTCUnixTime(); // RTC'den gerçek zaman
  telemetry.basinc1 = bmp_basinc; // BMP280'den gelen basınç
  telemetry.basinc2 = basinc_lora3; // Lora 3'ten gelen basınç
  telemetry.sicaklik = (int16_t)(bmp_sicaklik * 100); // BMP280'den gelen sıcaklık * 100
  
  // ADC gerilim
  int adcValue = analogRead(PIL_GERILIMI_PIN);
  if (adcValue >= 0 && adcValue <= 4095) {
    float voltage = (adcValue * 3.3) / 4096.0;
    telemetry.pil_gerilimi = (uint16_t)(voltage * 100);
    adc_status = true;
  } else {
    telemetry.pil_gerilimi = 0;
    adc_status = false;
  }
  
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
  float yukseklik2 = 0.0; // Varsayılan değer
  
  // Taşıyıcı yükseklik hesaplama (sadece referans varsa)
  if (basinc2_referans > 0 && basinc_lora3 > 0) {
    yukseklik2 = calculateAltitude(basinc_lora3, basinc2_referans);
  } else {
    // Referans henüz yok, yükseklik hesaplanamaz
    Serial.println("Tasiyici yukseklik hesaplanamaz - referans bekleniyor");
  }
  
  // İrtifa farkını hesapla (mutlak değer)
  float irtifa_farki = abs(yukseklik1 - yukseklik2);
  
  // İniş hızını hesapla
  unsigned long current_time = millis();
  float inis_hizi = calculateDescentRate(yukseklik1, current_time);

  telemetry.yukseklik1 = yukseklik1;
  telemetry.yukseklik2 = yukseklik2;
  telemetry.irtifa_farki = irtifa_farki;
  telemetry.inis_hizi = inis_hizi;
  
  // İniş hızı hesaplandıktan sonra hata kodu hesapla
  telemetry.hata_kodu = calculateHataKodu(telemetry.uydu_statusu, inis_hizi, basinc_lora3);
  
  // SD karta kaydet (mümkünse)
  if (sd_status) {
    saveToSD(telemetry, bmp_basinc, bmp_sicaklik);
  }
  
  ResponseStatus rs = E22.sendFixedMessage(0x00, 0x0A, 20, (uint8_t*)&telemetry, sizeof(telemetry));
  
  // Gönderilen RHRH değerini göster
  char rhrh_str[5];
  decodeRHRH(current_rhrh, rhrh_str);
  
  // Hata kodu bilgisini yazdır
  Serial.print("Hata Kodu: 0b");
  Serial.print(telemetry.hata_kodu, BIN);
  Serial.print(" (0x");
  Serial.print(telemetry.hata_kodu, HEX);
  Serial.print(") - Uydu Status: ");
  Serial.print(telemetry.uydu_statusu);
  Serial.print(", Inis Hizi: ");
  Serial.print(inis_hizi);
  Serial.print(", Tasiyici Basinc: ");
  Serial.print(basinc_lora3);
  Serial.print(", SEPERATION: ");
  Serial.println(digitalRead(SEPERATION) == HIGH ? "HIGH" : "LOW");
  
  Serial.print("Lora2'ye binary telemetry gonderildi (");
  Serial.print(sizeof(telemetry));
  Serial.print(" bytes), Paket#: ");
  Serial.print(paket_sayisi_lora2);
  Serial.print(", RHRH: ");
  Serial.print(rhrh_str);
  Serial.print(", Manuel: ");
  Serial.print(current_manuel_ayrilma);
  Serial.print(", Durum: ");
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
              Serial.print(btn->manuel_ayrilma);
              
              // Manuel ayrılma değerini güncelle
              current_manuel_ayrilma = btn->manuel_ayrilma;
              
              // Manuel işlem kontrol et
              if (btn->manuel_ayrilma == 1) {
                manuelAyrilma();
              } else if (btn->manuel_ayrilma == 2) {
                manuelBirlesme();
              }
              
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
                
                // RHRH komutu başlat (eğer 0A0A değilse)
                startRHRHCommand(btn->rhrh);
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
  
  // Sensör durumlarını kontrol et
  sensorControl();
  
  // Buton durumlarını kontrol et
  checkButtonStates();
  
  // Serial komutları kontrol et
  processSerialCommand();
  
  // SD kart durumunu kontrol et (artık sensorControl içinde)
  checkSDStatus();
  
  // RHRH işlemini güncelle
  updateRHRH();
  
  // DC Motor işlemini güncelle
  updateDCMotor();
  
  // Zamanlayıcı başlat
  unsigned long loop_start_time = millis();
  
  // 1. Lora 2'ye telemetry gönder
  sendTelemetryToLora2();
  
  // 2. 1 saniye dolana kadar dinleme modunda ol
  while (millis() - loop_start_time < 950) {
    // Serial komutları kontrol et
    processSerialCommand();
    
    // Buton durumlarını kontrol et
    checkButtonStates();
    
    // RHRH işlemini güncelle
    updateRHRH();
    
    // DC Motor işlemini güncelle
    updateDCMotor();
    
    if (E22.available() > 1) {
      waitForMessage(100); // Kısa timeout ile mesaj işle
    }
    delay(10); // Buffer yönetimi için kısa bekleme
  }
  
  // Pil gerilimini göster
  int adcValue = analogRead(PIL_GERILIMI_PIN);
  if (adcValue >= 0 && adcValue <= 4095) {
    float voltage = (adcValue * 3.3) / 4096.0;
    uint16_t pil_gerilimi = (uint16_t)(voltage * 100);
    Serial.print("Pil Gerilimi: ");
    Serial.println(pil_gerilimi);
  } else {
    Serial.println("Pil Gerilimi: ADC Okuma Hatasi");
  }
  
  Serial.println("=== COMM LOOP TAMAMLANDI ===");
}
