/*
 * BMP280, MPU6050, DS3231 RTC, SD Kart ve GPS Sensör Test Kodu
 * ESP32 ile BMP280 sensöründen sıcaklık, basınç ve yükseklik okuma
 * ESP32 ile MPU6050 sensöründen ivme, gyro ve açı okuma
 * ESP32 ile DS3231 RTC'den zaman bilgisi okuma
 * ESP32 ile SD Kart modülü okuma/yazma testi
 * ESP32 ile GY-GPS6MV2 GPS modülü konum testi
 * 
 * Bağlantılar:
 * BMP280 VCC -> 3.3V
 * BMP280 GND -> GND
 * BMP280 SCL -> GPIO 22 (I2C Clock)
 * BMP280 SDA -> GPIO 21 (I2C Data)
 * 
 * MPU6050 VCC -> 3.3V
 * MPU6050 GND -> GND
 * MPU6050 SCL -> GPIO 22 (I2C Clock)
 * MPU6050 SDA -> GPIO 21 (I2C Data)
 * 
 * DS3231 RTC VCC -> 3.3V
 * DS3231 RTC GND -> GND
 * DS3231 RTC SCL -> GPIO 22 (I2C Clock)
 * DS3231 RTC SDA -> GPIO 21 (I2C Data)
 * 
 * SD Kart VCC -> 3.3V
 * SD Kart GND -> GND
 * SD Kart CS  -> GPIO 5  (Chip Select)
 * SD Kart MOSI-> GPIO 23 (Master Out Slave In)
 * SD Kart MISO-> GPIO 19 (Master In Slave Out)
 * SD Kart SCK -> GPIO 18 (Serial Clock)
 * 
 * GPS VCC -> 3.3V veya 5V
 * GPS GND -> GND
 * GPS RX  -> GPIO 17 (ESP32 TX)
 * GPS TX  -> GPIO 16 (ESP32 RX)
 */

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <RTClib.h>
#include <SD.h>
#include <FS.h>
#include <TinyGPS++.h>

// Sensör objelerini oluştur
Adafruit_BMP280 bmp;
Adafruit_MPU6050 mpu;
RTC_DS3231 rtc;
TinyGPSPlus gps;

// GPS UART1 Pin tanımlamaları (main.ino'daki gibi)
#define GPS_RX 15
#define GPS_TX 14
#define GPS_BAUD 9600
HardwareSerial gpsSerial(1);

// Buzzer pin tanımlaması
#define BUZZER_PIN 27

// Buton pin tanımlamaları
#define BUTTON1_PIN 35
#define BUTTON2_PIN 34

// SD Kart SPI pin tanımlamaları (main.ino'daki gibi)
#define SD_SCK 18
#define SD_MISO 19
#define SD_MOSI 23
#define SD_CS 5

// Test dosyası adları
#define TEST_FILE "/test_file.txt"
#define CSV_FILE "/sensor_test.csv"
#define MONITORING_FILE "/monitoring.log"

// Deniz seviyesi basıncı (hPa) - kalibre etmek için kullanılır
#define SEALEVELPRESSURE_HPA (1013.25)

// Sensör durumları
bool bmp_status = false;
bool mpu_status = false;
bool rtc_status = false;
bool sd_status = false;
bool gps_status = false;

// Buzzer kontrol değişkenleri
unsigned long buzzer_last_time = 0;
bool buzzer_state = false;
const unsigned long BUZZER_INTERVAL = 1000; // 1 saniye = 1000ms

// Buton durumları
bool button1_state = false;
bool button2_state = false;

void setup() {
  Serial.begin(115200);
  Serial.println("BMP280, MPU6050, DS3231 RTC, SD Kart ve GPS Test Başlatılıyor...");
  
  // Pin modlarını ayarla
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  pinMode(BUTTON2_PIN, INPUT_PULLUP);
  
  // Buzzer başlangıç sesi
  digitalWrite(BUZZER_PIN, HIGH);
  delay(100);
  digitalWrite(BUZZER_PIN, LOW);
  
  // I2C başlat
  Wire.begin();
  
  // GPS UART başlatma (main.ino'daki gibi)
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  
  Serial.println("========================================");
  
  // RTC başlatma (main.ino'daki gibi)
  Serial.println("DS3231 RTC başlatılıyor...");
  delay(100);
  
  if (!rtc.begin()) {
    Serial.println("HATA: DS3231 RTC bulunamadı! (0x68)");
    
    // Alternatif adres dene (main.ino'daki gibi)
    Wire.beginTransmission(0x57);
    if (Wire.endTransmission() == 0) {
      Serial.println("RTC 0x57 adresinde bulundu, tekrar deneniyor...");
      delay(100);
      if (rtc.begin()) {
        rtc_status = true;
        Serial.println("DS3231 RTC başarıyla başlatıldı (0x57) ✓");
      } else {
        rtc_status = false;
        Serial.println("RTC 0x57 adresinde başlatılamadı!");
      }
    } else {
      rtc_status = false;
      Serial.println("RTC hiçbir adreste bulunamadı");
    }
  } else {
    Serial.println("DS3231 RTC başarıyla başlatıldı (0x68) ✓");
    rtc_status = true;
    
    // RTC çalışıyor mu test et (main.ino'daki gibi)
    DateTime now = rtc.now();
    if (now.year() < 2020) {
      Serial.println("RTC zaman geçersiz - ayarlanması gerekiyor");
      Serial.println("Serial Monitor'dan zaman ayarlayabilirsiniz:");
      Serial.println("Komut: SET:2025,8,5,14,30,0");
    } else {
      Serial.printf("RTC Zamanı: %02d-%02d-%04d %02d:%02d:%02d\n", 
                    now.day(), now.month(), now.year(),
                    now.hour(), now.minute(), now.second());
    }
  }
  
  // GPS modülü başlatma (main.ino'daki gibi)
  Serial.println("GPS modülü başlatıldı (UART1)");
  Serial.printf("GPS Pin Konfigürasyonu:\n");
  Serial.printf("- RX (ESP32): GPIO%d\n", GPS_RX);
  Serial.printf("- TX (ESP32): GPIO%d\n", GPS_TX);
  Serial.printf("- Baud Rate: %d\n", GPS_BAUD);
  Serial.println("GPS sinyali aranıyor...");
  gps_status = false; // Başlangıçta false, sinyal gelince true olacak
  
  // MPU6050 sensörünü başlat (main.ino'daki gibi 0x69 adresi)
  Serial.println("MPU6050 başlatılıyor...");
  delay(100);
  
  if (!mpu.begin(0x69)) {
    Serial.println("HATA: MPU6050 bulunamadı! Bağlantıları kontrol edin.");
    Serial.println("Alternatif olarak I2C adresini 0x68 deneyin.");
    mpu_status = false;
  } else {
    Serial.println("MPU6050 başarıyla başlatıldı!");
    
    // main.ino'daki ayarlarla aynı konfigürasyon
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    delay(100);
    
    // Test okuma yaparak çalışıp çalışmadığını kontrol et
    sensors_event_t a, g, temp;
    if (mpu.getEvent(&a, &g, &temp)) {
      mpu_status = true;
      Serial.println("MPU6050 test okuması başarılı ✓");
    } else {
      mpu_status = false;
      Serial.println("MPU6050 test okuması başarısız!");
    }
  }
  
  // BMP280 sensörünü başlat
  Serial.println("BMP280 başlatılıyor...");
  if (!bmp.begin(0x76)) { // 0x76 varsayılan I2C adresi, bazı modüllerde 0x77 olabilir
    Serial.println("BMP280 sensörü bulunamadı! Bağlantıları kontrol edin.");
    Serial.println("Alternatif olarak I2C adresini 0x77 deneyin.");
    bmp_status = false;
  } else {
    Serial.println("BMP280 sensörü başarıyla başlatıldı!");
    
    // BMP280 ayarları (main.ino'daki gibi)
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     // İşletim modu
                    Adafruit_BMP280::SAMPLING_X2,     // Sıcaklık oversampling
                    Adafruit_BMP280::SAMPLING_X16,    // Basınç oversampling
                    Adafruit_BMP280::FILTER_X16,      // Filtreleme
                    Adafruit_BMP280::STANDBY_MS_500); // Standby süresi
    bmp_status = true;
  }
  
  // SD Kart başlatma (main.ino'daki gibi)
  Serial.println("SD Kart başlatılıyor...");
  delay(100);
  
  Serial.printf("SPI Pin Konfigürasyonu:\n");
  Serial.printf("- SCK (Clock):  GPIO%d\n", SD_SCK);
  Serial.printf("- MISO (Data):  GPIO%d\n", SD_MISO);
  Serial.printf("- MOSI (Data):  GPIO%d\n", SD_MOSI);
  Serial.printf("- CS (Select):  GPIO%d\n", SD_CS);
  
  if (!SD.begin(SD_CS)) {
    Serial.println("HATA: SD kart başlatılamadı!");
    Serial.println("Kontrol Listesi:");
    Serial.println("✗ SD kart takılı mı?");
    Serial.println("✗ SD kart FAT32 formatında mı?");
    Serial.println("✗ Pin bağlantıları doğru mu?");
    Serial.println("✗ SD kart kapasitesi 32GB'den küçük mü?");
    Serial.println("✗ 3.3V besleme kullanılıyor mu?");
    sd_status = false;
  } else {
    uint8_t cardType = SD.cardType();
    if (cardType == CARD_NONE) {
      Serial.println("HATA: SD kart bulunamadı!");
      Serial.println("SD kart yuvaya düzgün takıldığından emin olun");
      sd_status = false;
    } else {
      Serial.println("SD kart başarıyla başlatıldı ✓");
      
      // SD kart bilgilerini göster
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
      
      // Test CSV dosyası oluştur
      if (!SD.exists(CSV_FILE)) {
        Serial.println("Test CSV dosyası oluşturuluyor...");
        String csvHeader = "Zaman,MPU_Pitch,MPU_Roll,MPU_Yaw,BMP_Temp,BMP_Pressure,BMP_Altitude,RTC_Timestamp,RTC_Temp,GPS_Latitude,GPS_Longitude,GPS_Altitude,GPS_Valid,Durum\n";
        if (writeFile(SD, CSV_FILE, csvHeader.c_str())) {
          Serial.println("✓ CSV dosyası oluşturuldu");
        }
      }
    }
  }
  
  Serial.println("----------------------------------------");
  Serial.print("MPU6050 Durumu: ");
  Serial.println(mpu_status ? "OK ✓" : "HATA ✗");
  Serial.print("BMP280 Durumu: ");
  Serial.println(bmp_status ? "OK ✓" : "HATA ✗");
  Serial.print("DS3231 RTC Durumu: ");
  Serial.println(rtc_status ? "OK ✓" : "HATA ✗");
  Serial.print("GPS Durumu: ");
  Serial.println("SINYAL ARANIYOR...");
  Serial.print("SD Kart Durumu: ");
  Serial.println(sd_status ? "OK ✓" : "HATA ✗");
  Serial.println("----------------------------------------");
  
  if (sd_status) {
    Serial.println("SD Kart testleri başlatılıyor...");
    // Başlangıç testleri
    testSDCardOperations();
  }
  
  delay(2000); // Sensörlerin stabilize olması için bekle
}

void loop() {
  // Buzzer kontrolü (millis tabanlı)
  unsigned long current_time = millis();
  if (current_time - buzzer_last_time >= BUZZER_INTERVAL) {
    buzzer_state = !buzzer_state;
    digitalWrite(BUZZER_PIN, buzzer_state ? HIGH : LOW);
    buzzer_last_time = current_time;
  }
  
  // Buton durumlarını oku
  bool new_button1_state = !digitalRead(BUTTON1_PIN); // INPUT_PULLUP olduğu için ters
  bool new_button2_state = !digitalRead(BUTTON2_PIN); // INPUT_PULLUP olduğu için ters
  
  // Buton durumu değiştiğinde serial'a yazdır
  if (new_button1_state != button1_state) {
    button1_state = new_button1_state;
    Serial.print("BUTTON1 (Pin 35): ");
    Serial.println(button1_state ? "BASILDI" : "BIRAKILDI");
  }
  
  if (new_button2_state != button2_state) {
    button2_state = new_button2_state;
    Serial.print("BUTTON2 (Pin 34): ");
    Serial.println(button2_state ? "BASILDI" : "BIRAKILDI");
  }
  
  // GPS verilerini sürekli güncelle (main.ino'daki gibi)
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }
  
  // Serial komut kontrolü
  if (Serial.available()) {
    String command = Serial.readString();
    command.trim();
    
    if (command.startsWith("SET:")) {
      setRTCTime(command);
    } else if (command == "SD_INIT") {
      Serial.println("SD kart yeniden başlatılıyor...");
      testSDCardInit();
    } else if (command == "SD_INFO") {
      displaySDCardInfo();
    } else if (command == "SD_LIST") {
      listAllFiles();
    } else if (command == "SD_TEST") {
      if (sd_status) {
        testSDCardOperations();
      } else {
        Serial.println("SD kart çalışmıyor!");
      }
    } else if (command == "GPS_INFO") {
      displayGPSInfo();
    } else if (command == "HELP") {
      showHelp();
    }
  }
  
  Serial.println("=== SENSÖR VERİLERİ ===");
  
  // MPU6050 verilerini oku (main.ino'daki gibi)
  float pitch = 0.0, roll = 0.0, yaw = 0.0;
  float accel_x = 0.0, accel_y = 0.0, accel_z = 0.0;
  float temp_mpu = 0.0;
  
  if (mpu_status) {
    sensors_event_t a, g, temp;
    if (mpu.getEvent(&a, &g, &temp)) {
      // Gyro verilerini al (rad/s'den derece/s'ye çevir) - main.ino'daki gibi
      pitch = g.gyro.y * 180.0 / PI;
      roll = g.gyro.x * 180.0 / PI;
      yaw = g.gyro.z * 180.0 / PI;
      
      // İvme verilerini al (m/s²)
      accel_x = a.acceleration.x;
      accel_y = a.acceleration.y;
      accel_z = a.acceleration.z;
      
      // Sıcaklık (°C)
      temp_mpu = temp.temperature;
      
      // Değerlerin geçerli olup olmadığını kontrol et
      if (isnan(pitch) || isnan(roll) || isnan(yaw)) {
        mpu_status = false;
        pitch = roll = yaw = 0.0;
        Serial.println("UYARI: MPU6050 gyro verisi okunamadı!");
      }
    } else {
      mpu_status = false;
      Serial.println("UYARI: MPU6050 sensörü okunamadı!");
    }
  }
  
  // BMP280 verilerini oku
  float temperature = 0.0, pressure = 0.0, altitude = 0.0, pressureHPa = 0.0;
  
  if (bmp_status) {
    temperature = bmp.readTemperature();    // Sıcaklık (°C)
    pressure = bmp.readPressure();          // Basınç (Pa)
    altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA); // Yükseklik (m)
    
    // Basıncı hPa'ya çevir
    pressureHPa = pressure / 100.0F;
    
    // Sensör durumunu kontrol et
    if (isnan(temperature) || isnan(pressure)) {
      bmp_status = false;
      Serial.println("UYARI: BMP280 sensör verisi okunamadı!");
    }
  }
  
  // === MPU6050 VERİLERİ ===
  Serial.println("--- MPU6050 (Gyro/Accel) ---");
  if (mpu_status) {
    Serial.print("Pitch (Y-ekseni): ");
    Serial.print(pitch, 2);
    Serial.println(" °/s");
    
    Serial.print("Roll (X-ekseni): ");
    Serial.print(roll, 2);
    Serial.println(" °/s");
    
    Serial.print("Yaw (Z-ekseni): ");
    Serial.print(yaw, 2);
    Serial.println(" °/s");
    
    Serial.print("İvme X: ");
    Serial.print(accel_x, 2);
    Serial.println(" m/s²");
    
    Serial.print("İvme Y: ");
    Serial.print(accel_y, 2);
    Serial.println(" m/s²");
    
    Serial.print("İvme Z: ");
    Serial.print(accel_z, 2);
    Serial.println(" m/s²");
    
    Serial.print("MPU Sıcaklık: ");
    Serial.print(temp_mpu, 1);
    Serial.println(" °C");
    
    Serial.println("MPU6050 çalışıyor ✓");
  } else {
    Serial.println("MPU6050 HATALI ✗");
  }
  
  // === RTC VERİLERİ ===
  String timestamp = "0000-00-00 00:00:00";
  float rtc_temp = 0.0;
  
  if (rtc_status) {
    DateTime now = rtc.now();
    if (now.year() > 2020 && now.year() < 2035) {  // Makul tarih aralığı kontrolü (main.ino'daki gibi)
      timestamp = String(now.year()) + "-" + 
                  addLeadingZero(now.month()) + "-" + 
                  addLeadingZero(now.day()) + " " + 
                  addLeadingZero(now.hour()) + ":" + 
                  addLeadingZero(now.minute()) + ":" + 
                  addLeadingZero(now.second());
      
      // RTC'nin dahili sıcaklık sensörünü oku
      rtc_temp = rtc.getTemperature();
    } else {
      rtc_status = false;
      timestamp = "0000-00-00 00:00:00";
      Serial.println("UYARI: RTC tarih geçersiz, zaman ayarlaması gerekiyor");
    }
  }
  
  // === GPS VERİLERİ ===
  double latitude = 0.0, longitude = 0.0;
  float gps_altitude = 0.0;
  bool gps_valid = false;
  int satellites = 0;
  
  // GPS verilerini oku (main.ino'daki gibi)
  if (gps.location.isValid()) {
    latitude = gps.location.lat();
    longitude = gps.location.lng();
    gps_valid = true;
    gps_status = true;
  } else {
    latitude = 0.0;
    longitude = 0.0;
    gps_valid = false;
    gps_status = false;
  }
  
  if (gps.altitude.isValid()) {
    gps_altitude = gps.altitude.meters();
  } else {
    gps_altitude = 0.0;
  }
  
  if (gps.satellites.isValid()) {
    satellites = gps.satellites.value();
  }
  
  // === BMP280 VERİLERİ ===
  Serial.println("--- BMP280 (Basınç/Sıcaklık) ---");
  if (bmp_status) {
    Serial.print("Sıcaklık: ");
    Serial.print(temperature, 2);
    Serial.println(" °C");
    
    Serial.print("Basınç: ");
    Serial.print(pressureHPa, 2);
    Serial.println(" hPa");
    
    Serial.print("Yükseklik: ");
    Serial.print(altitude, 2);
    Serial.println(" m");
    
    Serial.println("BMP280 çalışıyor ✓");
  } else {
    Serial.println("BMP280 HATALI ✗");
  }
  
  // === DS3231 RTC VERİLERİ ===
  Serial.println("--- DS3231 RTC (Zaman/Sıcaklık) ---");
  if (rtc_status) {
    Serial.print("Zaman: ");
    Serial.println(timestamp);
    
    Serial.print("RTC Sıcaklık: ");
    Serial.print(rtc_temp, 2);
    Serial.println(" °C");
    
    Serial.println("DS3231 RTC çalışıyor ✓");
  } else {
    Serial.println("DS3231 RTC HATALI ✗");
    Serial.println("Zaman ayarlamak için: SET:2025,8,5,14,30,0");
  }
  
  // === GPS VERİLERİ ===
  Serial.println("--- GPS (Konum/Uydu) ---");
  if (gps_valid) {
    Serial.print("Enlem (Latitude): ");
    Serial.println(latitude, 6);
    
    Serial.print("Boylam (Longitude): ");
    Serial.println(longitude, 6);
    
    Serial.print("GPS Yükseklik: ");
    Serial.print(gps_altitude, 2);
    Serial.println(" m");
    
    Serial.print("Uydu Sayısı: ");
    Serial.println(satellites);
    
    // Konum doğruluğu bilgisi
    if (gps.hdop.isValid()) {
      Serial.print("HDOP (Doğruluk): ");
      Serial.println(gps.hdop.hdop(), 2);
    }
    
    // GPS zamanı
    if (gps.time.isValid() && gps.date.isValid()) {
      Serial.printf("GPS Zamanı: %02d/%02d/%04d %02d:%02d:%02d UTC\n",
                    gps.date.day(), gps.date.month(), gps.date.year(),
                    gps.time.hour(), gps.time.minute(), gps.time.second());
    }
    
    Serial.println("GPS çalışıyor ✓");
  } else {
    Serial.println("GPS sinyali bulunamadı ✗");
    Serial.print("Uydu Sayısı: ");
    Serial.println(satellites);
    
    // GPS'den gelen karakterleri göster
    if (gpsSerial.available()) {
      Serial.println("GPS'den veri geliyor ama geçersiz");
    } else {
      Serial.println("GPS'den hiç veri gelmiyor - bağlantı kontrolü");
    }
  }
  
  // === DURUM ÖZETİ ===
  Serial.println("--- Sistem Durumu ---");
  Serial.print("MPU6050: ");
  Serial.print(mpu_status ? "OK" : "HATA");
  Serial.print(" | BMP280: ");
  Serial.print(bmp_status ? "OK" : "HATA");
  Serial.print(" | RTC: ");
  Serial.print(rtc_status ? "OK" : "HATA");
  Serial.print(" | GPS: ");
  Serial.print(gps_status ? "OK" : "HATA");
  Serial.print(" | SD: ");
  Serial.println(sd_status ? "OK" : "HATA");
  
  // === BUTON VE BUZZER DURUMU ===
  Serial.println("--- Kontrol Durumu ---");
  Serial.print("BUTTON1 (Pin 35): ");
  Serial.print(button1_state ? "BASILDI" : "SERBEST");
  Serial.print(" | BUTTON2 (Pin 34): ");
  Serial.print(button2_state ? "BASILDI" : "SERBEST");
  Serial.print(" | BUZZER (Pin 27): ");
  Serial.println(buzzer_state ? "AÇIK" : "KAPALI");
  
  // === SD KART'A VERİ KAYDETME ===
  if (sd_status) {
    saveDataToSD(timestamp, pitch, roll, yaw, temperature, pressureHPa, altitude, rtc_temp, latitude, longitude, gps_altitude, gps_valid);
  } else {
    Serial.println("SD kart çalışmıyor - veri kaydedilemiyor");
  }
  
  Serial.println("========================");
  Serial.println();
  
  delay(2000); // 2 saniye bekle
}

// Yardımcı fonksiyon - sayının önüne sıfır ekler (iki haneli yapmak için)
String addLeadingZero(int number) {
  if (number < 10) {
    return "0" + String(number);
  }
  return String(number);
}

// RTC zaman ayarlama fonksiyonu (main.ino'daki gibi)
void setRTCTime(String command) {
  // Format: SET:2025,8,5,14,30,0 (Yıl,Ay,Gün,Saat,Dakika,Saniye)
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
    Serial.println("Hatalı format! Kullanım: SET:2025,8,5,14,30,0");
  }
}

// === SD KART FONKSİYONLARI ===

void testSDCardInit() {
  Serial.println("--- SD Kart Başlatma Testi ---");
  
  if (!SD.begin(SD_CS)) {
    Serial.println("HATA: SD kart başlatılamadı!");
    Serial.println("\nKontrol Listesi:");
    Serial.println("✗ SD kart takılı mı?");
    Serial.println("✗ SD kart FAT32 formatında mı?");
    Serial.println("✗ Pin bağlantıları doğru mu?");
    Serial.println("✗ SD kart kapasitesi 32GB'den küçük mü?");
    Serial.println("✗ 3.3V besleme kullanılıyor mu?");
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
  
  Serial.println("SD kart başarıyla başlatıldı ✓");
  sd_status = true;
}

void displaySDCardInfo() {
  if (!sd_status) {
    Serial.println("SD kart çalışmıyor - bilgi alınamıyor");
    return;
  }
  
  Serial.println("--- SD Kart Bilgileri ---");
  
  // Kart tipi
  uint8_t cardType = SD.cardType();
  Serial.print("Kart Tipi: ");
  switch (cardType) {
    case CARD_MMC:
      Serial.println("MMC");
      break;
    case CARD_SD:
      Serial.println("SDSC (Standard Capacity)");
      break;
    case CARD_SDHC:
      Serial.println("SDHC (High Capacity)");
      break;
    default:
      Serial.println("UNKNOWN");
  }
  
  // Kart boyutu
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("Toplam Kapasite: %llu MB\n", cardSize);
  
  // Kullanılan/boş alan
  uint64_t usedBytes = SD.usedBytes() / (1024 * 1024);
  uint64_t totalBytes = SD.totalBytes() / (1024 * 1024);
  uint64_t freeBytes = totalBytes - usedBytes;
  
  Serial.printf("Kullanılan Alan: %llu MB\n", usedBytes);
  Serial.printf("Boş Alan: %llu MB\n", freeBytes);
  Serial.printf("Kullanım Oranı: %.1f%%\n", (float)usedBytes / totalBytes * 100);
  
  Serial.println("✓ SD kart bilgileri alındı");
}

void testSDCardOperations() {
  Serial.println("--- SD Kart Dosya İşlemleri Testi ---");
  
  if (!sd_status) {
    Serial.println("SD kart çalışmıyor!");
    return;
  }
  
  // Test 1: Dosya yazma
  Serial.println("Test 1: Dosya yazma...");
  String testData = "ESP32 Sensör Test\n";
  testData += "Zaman: " + String(millis()) + " ms\n";
  testData += "Test verisi: 12345\n";
  
  if (writeFile(SD, TEST_FILE, testData.c_str())) {
    Serial.println("✓ Dosya yazma başarılı");
  } else {
    Serial.println("✗ Dosya yazma başarısız");
    return;
  }
  
  // Test 2: Dosya okuma
  Serial.println("Test 2: Dosya okuma...");
  String readData = readFile(SD, TEST_FILE);
  if (readData.length() > 0) {
    Serial.println("✓ Dosya okuma başarılı");
    Serial.println("Okunan içerik:");
    Serial.println(readData);
  } else {
    Serial.println("✗ Dosya okuma başarısız");
  }
  
  // Test 3: Dosyaya ekleme
  Serial.println("Test 3: Dosyaya ekleme...");
  String appendData = "Eklenen veri: " + String(millis()) + "\n";
  if (appendFile(SD, TEST_FILE, appendData.c_str())) {
    Serial.println("✓ Dosyaya ekleme başarılı");
  } else {
    Serial.println("✗ Dosyaya ekleme başarısız");
  }
  
  // Test 4: Dosya silme
  Serial.println("Test 4: Dosya silme...");
  if (SD.remove(TEST_FILE)) {
    Serial.println("✓ Dosya silme başarılı");
  } else {
    Serial.println("✗ Dosya silme başarısız");
  }
}

void listAllFiles() {
  Serial.println("--- SD Kart Dosya Listesi ---");
  
  if (!sd_status) {
    Serial.println("SD kart çalışmıyor!");
    return;
  }
  
  File root = SD.open("/");
  if (!root) {
    Serial.println("✗ Root dizin açılamadı");
    return;
  }
  
  if (!root.isDirectory()) {
    Serial.println("✗ Root bir dizin değil");
    return;
  }
  
  Serial.println("SD Kart İçeriği:");
  int fileCount = 0;
  
  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      Serial.printf("  [DIR]  %s\n", file.name());
    } else {
      Serial.printf("  [FILE] %s (%d bytes)\n", file.name(), file.size());
      fileCount++;
    }
    file = root.openNextFile();
  }
  
  root.close();
  Serial.printf("Toplam %d dosya bulundu\n", fileCount);
}

void saveDataToSD(String timestamp, float pitch, float roll, float yaw, 
                  float temperature, float pressure, float altitude, float rtc_temp,
                  double latitude, double longitude, float gps_altitude, bool gps_valid) {
  if (!sd_status) {
    return;
  }
  
  static unsigned long lastSave = 0;
  if (millis() - lastSave < 5000) { // Her 5 saniyede bir kaydet
    return;
  }
  lastSave = millis();
  
  // CSV satırı oluştur
  String csvLine = String(millis()) + "," +
                   String(pitch, 2) + "," +
                   String(roll, 2) + "," +
                   String(yaw, 2) + "," +
                   String(temperature, 2) + "," +
                   String(pressure, 2) + "," +
                   String(altitude, 2) + "," +
                   timestamp + "," +
                   String(rtc_temp, 2) + "," +
                   String(latitude, 6) + "," +
                   String(longitude, 6) + "," +
                   String(gps_altitude, 2) + "," +
                   String(gps_valid ? "1" : "0") + "," +
                   (mpu_status && bmp_status && rtc_status && gps_status ? "ALL_OK" : "PARTIAL") + "\n";
  
  if (appendFile(SD, CSV_FILE, csvLine.c_str())) {
    Serial.println("✓ Veri SD karta kaydedildi");
  } else {
    Serial.println("✗ SD kart yazma hatası");
    sd_status = false; // SD durumunu güncelle
  }
}

// Yardımcı fonksiyonlar (main.ino'daki gibi)
bool writeFile(fs::FS &fs, const char *path, const char *message) {
  Serial.printf("Dosya yazılıyor: %s\n", path);
  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("✗ Dosya açma hatası");
    return false;
  }
  
  if (file.print(message)) {
    Serial.println("✓ Dosya yazma başarılı");
    file.close();
    return true;
  } else {
    Serial.println("✗ Dosya yazma hatası");
    file.close();
    return false;
  }
}

bool appendFile(fs::FS &fs, const char *path, const char *message) {
  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("✗ Dosya ekleme hatası - açılamadı");
    return false;
  }
  
  size_t bytesWritten = file.print(message);
  file.close();
  
  if (bytesWritten > 0) {
    return true;
  } else {
    Serial.println("✗ Dosya ekleme hatası - yazılamadı");
    return false;
  }
}

String readFile(fs::FS &fs, const char *path) {
  Serial.printf("Dosya okunuyor: %s\n", path);
  File file = fs.open(path);
  if (!file) {
    Serial.println("✗ Dosya okuma hatası - açılamadı");
    return "";
  }
  
  String content = "";
  while (file.available()) {
    content += char(file.read());
  }
  file.close();
  
  return content;
}

void displayGPSInfo() {
  Serial.println("--- GPS Detaylı Bilgiler ---");
  
  Serial.printf("GPS Pin Konfigürasyonu:\n");
  Serial.printf("- RX (ESP32): GPIO%d -> GPS TX\n", GPS_RX);
  Serial.printf("- TX (ESP32): GPIO%d -> GPS RX\n", GPS_TX);
  Serial.printf("- Baud Rate: %d\n", GPS_BAUD);
  
  Serial.println("\nGPS Durum Bilgileri:");
  Serial.print("Konum Geçerliliği: ");
  Serial.println(gps.location.isValid() ? "✓ GEÇERLİ" : "✗ GEÇERSİZ");
  
  Serial.print("Zaman Geçerliliği: ");
  Serial.println(gps.time.isValid() ? "✓ GEÇERLİ" : "✗ GEÇERSİZ");
  
  Serial.print("Tarih Geçerliliği: ");
  Serial.println(gps.date.isValid() ? "✓ GEÇERLİ" : "✗ GEÇERSİZ");
  
  if (gps.satellites.isValid()) {
    Serial.print("Görünen Uydu Sayısı: ");
    Serial.println(gps.satellites.value());
  } else {
    Serial.println("Uydu bilgisi alınamıyor");
  }
  
  if (gps.hdop.isValid()) {
    Serial.print("HDOP (Konum Doğruluğu): ");
    Serial.println(gps.hdop.hdop(), 2);
    Serial.println("  (< 2: Mükemmel, 2-5: İyi, 5-10: Orta, >10: Zayıf)");
  }
  
  Serial.print("İşlenen Karakter Sayısı: ");
  Serial.println(gps.charsProcessed());
  
  Serial.print("Başarısız Cümle Sayısı: ");
  Serial.println(gps.failedChecksum());
  
  // Ham GPS verisini göster
  Serial.println("\nHam GPS Verisi (10 saniye):");
  Serial.println("--- GPS RAW DATA START ---");
  unsigned long startTime = millis();
  while (millis() - startTime < 10000) {
    if (gpsSerial.available()) {
      Serial.write(gpsSerial.read());
    }
  }
  Serial.println("\n--- GPS RAW DATA END ---");
}

void showHelp() {
  Serial.println("\n=== KULLANILABİLİR KOMUTLAR ===");
  Serial.println("SET:2025,8,5,14,30,0 - RTC zaman ayarla");
  Serial.println("SD_INIT   - SD kartı yeniden başlat");
  Serial.println("SD_INFO   - SD kart bilgilerini göster");
  Serial.println("SD_LIST   - Dosya listesini göster");
  Serial.println("SD_TEST   - SD dosya işlemleri testini çalıştır");
  Serial.println("GPS_INFO  - GPS detaylı bilgilerini göster");
  Serial.println("HELP      - Bu yardım menüsünü göster");
  Serial.println("====================================");
  Serial.println("\n=== PİN KONFIGÜRASYONU ===");
  Serial.println("Buzzer: Pin 27 (Otomatik 1s açık/kapalı)");
  Serial.println("Button1: Pin 35 (INPUT_PULLUP)");
  Serial.println("Button2: Pin 34 (INPUT_PULLUP)");
  Serial.println("====================================");
}

/*
 * Troubleshooting:
 * 
 * === BMP280 ===
 * 1. Eğer "BMP280 sensörü bulunamadı" hatası alıyorsanız:
 *    - Kablo bağlantılarını kontrol edin
 *    - I2C adresini 0x77 olarak değiştirmeyi deneyin: bmp.begin(0x77)
 *    - 3.3V besleme kullandığınızdan emin olun (5V değil)
 * 
 * === MPU6050 ===
 * 2. Eğer "MPU6050 bulunamadı" hatası alıyorsanız:
 *    - Kablo bağlantılarını kontrol edin (SDA-21, SCL-22)
 *    - I2C adresini 0x68 olarak değiştirmeyi deneyin: mpu.begin(0x68)
 *    - 3.3V besleme kullandığınızdan emin olun
 *    - AD0 pininin durumunu kontrol edin (GND=0x68, VCC=0x69)
 * 
 * === DS3231 RTC ===
 * 3. Eğer "DS3231 RTC bulunamadı" hatası alıyorsanız:
 *    - Kablo bağlantılarını kontrol edin (SDA-21, SCL-22)
 *    - 3.3V besleme kullandığınızdan emin olun
 *    - Coin battery (CR2032) takılı olduğundan emin olun
 *    - I2C adresi genellikle 0x68'dir, bazı modüllerde 0x57 olabilir
 *    - RTC zamanı ayarlamak için: SET:2025,8,5,14,30,0
 * 
 * === SD Kart ===
 * 4. Eğer "SD kart başlatılamadı" hatası alıyorsanız:
 *    - SD kart yuvaya düzgün takıldığından emin olun
 *    - SD kart FAT32 formatında olmalı (NTFS/exFAT değil)
 *    - Maksimum 32GB kapasiteli SD kart kullanın
 *    - Pin bağlantıları: CS-5, MOSI-23, MISO-19, SCK-18
 *    - 3.3V besleme kullanın (5V değil)
 *    - SD kart yazma korumalı değil mi kontrol edin
 * 
 * === GPS Modülü ===
 * 5. Eğer "GPS sinyali bulunamadı" sorunu yaşıyorsanız:
 *    - GPS modülü dışarıda mı? (Binada GPS sinyali almayabilir)
 *    - Pin bağlantıları: GPS RX->GPIO17, GPS TX->GPIO16
 *    - GPS modülü 3.3V veya 5V ile beslenebilir
 *    - Uydu görüş açısı açık mı? (Cam kenarı, balkon, dış mekan)
 *    - İlk sinyal için 30 saniye - 5 dakika bekleyin (Cold Start)
 *    - GPS_INFO komutu ile detaylı bilgi alın
 *    - Ham veri gelip gelmediğini kontrol edin
 * 
 * === Genel ===
 * 6. I2C Scanner kodu ile sensörlerin adreslerini bulabilirsiniz:
 *    Wire.beginTransmission(address);
 *    if (Wire.endTransmission() == 0) // sensör bulundu
 * 
 * 7. Kütüphane kurulumu:
 *    Arduino IDE -> Tools -> Manage Libraries:
 *    - "Adafruit BMP280" aratın ve kurun
 *    - "Adafruit MPU6050" aratın ve kurun
 *    - "Adafruit Unified Sensor" kütüphanesi de gerekli
 *    - "RTClib" by Adafruit aratın ve kurun
 *    - "TinyGPS++" by Mikal Hart aratın ve kurun
 *    - SD ve SPI kütüphaneleri ESP32 ile birlikte gelir
 * 
 * === main.ino Uyumluluğu ===
 * 8. Bu kod main.ino ile uyumlu olacak şekilde yazılmıştır:
 *    - MPU6050 adresi: 0x69 (main.ino'daki gibi)
 *    - BMP280 adresi: 0x76 (standart)
 *    - DS3231 RTC adresi: 0x68 (alternatif 0x57)
 *    - SD kart pinleri: CS-5, MOSI-23, MISO-19, SCK-18
 *    - GPS pinleri: RX-16, TX-17, Baud-9600
 *    - Gyro verileri derece/saniye olarak (main.ino'daki gibi)
 *    - RTC zaman formatı main.ino ile aynı
 *    - Aynı sensör ayarları kullanılmıştır
 * 
 * === Komut Kullanımı ===
 * 9. Serial Monitor komutları:
 *    - RTC zaman ayarlama: SET:2025,8,5,14,30,0
 *    - SD kart yeniden başlatma: SD_INIT
 *    - SD kart bilgisi: SD_INFO
 *    - Dosya listesi: SD_LIST
 *    - SD test: SD_TEST
 *    - GPS detaylı bilgi: GPS_INFO
 *    - Yardım: HELP
 * 
 * === Veri Kaydetme ===
 * 10. SD kart üzerine otomatik veri kaydetme:
 *     - Her 5 saniyede bir sensör verileri CSV formatında kaydedilir
 *     - Dosya adı: /sensor_test.csv
 *     - Format: Zaman,MPU_Pitch,MPU_Roll,MPU_Yaw,BMP_Temp,BMP_Pressure,BMP_Altitude,RTC_Timestamp,RTC_Temp,GPS_Latitude,GPS_Longitude,GPS_Altitude,GPS_Valid,Durum
 *     - SD kart çalışmıyorsa veri kaydetme atlanır
 * 
 * === GPS İpuçları ===
 * 11. GPS performansını artırmak için:
 *     - Açık alanda test edin (dış mekan ideal)
 *     - Metal cisimlerden uzak tutun
 *     - İlk bağlantı (Cold Start) 30 saniye - 5 dakika sürebilir
 *     - HDOP değeri <5 olmalı (GPS_INFO ile kontrol edin)
 *     - En az 4 uydu gerekli konum tespiti için
 *     - Anten yönü yukarı bakmalı
 */
