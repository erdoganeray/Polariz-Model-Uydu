/*
 * DS3231 RTC Modülü Test Kodu
 * ESP32 ile DS3231 RTC modülü testi
 * I2C protokolü üzerinden RTC zaman okuma/ayarlama işlemleri
 * 
 * Bağlantılar (ESP32 ile DS3231 RTC Modülü):
 * RTC VCC -> 3.3V
 * RTC GND -> GND
 * RTC SCL -> GPIO22 (I2C Clock)
 * RTC SDA -> GPIO21 (I2C Data)
 * 
 * Önemli Notlar:
 * - DS3231 modülünde coin battery (CR2032) olmalı
 * - I2C adresi genellikle 0x68'dir
 * - Bazı modüllerde 0x57 adresi kullanılabilir
 * - 3.3V besleme kullanın
 */

#include <Wire.h>
#include <RTClib.h>

RTC_DS3231 rtc;

bool rtc_status = false;

void setup() {
  Serial.begin(115200);
  Serial.println("DS3231 RTC Test Başlatılıyor...");
  
  // I2C başlat
  Wire.begin();
  
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
}

void loop() {
  // Serial komut kontrolü
  if (Serial.available()) {
    String command = Serial.readString();
    command.trim();
    
    if (command.startsWith("SET:")) {
      setRTCTime(command);
    } else if (command == "HELP") {
      showHelp();
    } else if (command == "INFO") {
      showRTCInfo();
    }
  }
  
  // RTC'nin zamanını her saniye güncelle
  if (rtc_status) {
    DateTime now = rtc.now();
    
    // Zaman geçerliliğini kontrol et
    if (now.year() > 2020 && now.year() < 2035) {
      Serial.printf("Güncel Zaman: %02d/%02d/%04d %02d:%02d:%02d", 
                    now.day(), now.month(), now.year(),
                    now.hour(), now.minute(), now.second());
      
      // RTC sıcaklığını da göster
      float rtc_temp = rtc.getTemperature();
      Serial.printf(" | Sıcaklık: %.2f°C\n", rtc_temp);
    } else {
      Serial.println("RTC zaman geçersiz! Zaman ayarlaması gerekiyor.");
      Serial.println("Kullanım: SET:2025,8,6,15,30,0");
    }
  } else {
    Serial.println("RTC çalışmıyor! Bağlantıları kontrol edin.");
  }
  
  delay(1000); // 1 saniye bekle
}

// RTC zaman ayarlama fonksiyonu
void setRTCTime(String command) {
  // Format: SET:2025,8,6,15,30,0 (Yıl,Ay,Gün,Saat,Dakika,Saniye)
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
    Serial.printf("✓ RTC zamanı ayarlandı: %02d/%02d/%04d %02d:%02d:%02d\n", 
                  values[2], values[1], values[0], values[3], values[4], values[5]);
    rtc_status = true;
  } else {
    Serial.println("✗ Hatalı format! Kullanım: SET:2025,8,6,15,30,0");
  }
}

// RTC detaylı bilgiler
void showRTCInfo() {
  Serial.println("\n=== DS3231 RTC Detaylı Bilgiler ===");
  
  if (rtc_status) {
    DateTime now = rtc.now();
    
    Serial.println("✓ RTC Durumu: ÇALIŞIYOR");
    Serial.printf("Tarih: %02d/%02d/%04d (%s)\n", 
                  now.day(), now.month(), now.year(),
                  getDayName(now.dayOfTheWeek()));
    Serial.printf("Saat: %02d:%02d:%02d\n", 
                  now.hour(), now.minute(), now.second());
    Serial.printf("Unix Timestamp: %lu\n", now.unixtime());
    Serial.printf("Dahili Sıcaklık: %.2f°C\n", rtc.getTemperature());
    
    // Güç kaybı kontrolü
    if (rtc.lostPower()) {
      Serial.println("⚠️  UYARI: RTC güç kaybı yaşamış, zaman geçersiz olabilir");
    } else {
      Serial.println("✓ RTC güç durumu: Normal");
    }
  } else {
    Serial.println("✗ RTC Durumu: ÇALIŞMIYOR");
    Serial.println("Kontrol Listesi:");
    Serial.println("- I2C bağlantıları (SDA-21, SCL-22)");
    Serial.println("- 3.3V besleme");
    Serial.println("- Coin battery (CR2032)");
    Serial.println("- I2C adresi (0x68 veya 0x57)");
  }
}

// Gün adlarını döndür
String getDayName(int dayOfWeek) {
  String days[] = {"Pazar", "Pazartesi", "Salı", "Çarşamba", "Perşembe", "Cuma", "Cumartesi"};
  if (dayOfWeek >= 0 && dayOfWeek <= 6) {
    return days[dayOfWeek];
  }
  return "Bilinmiyor";
}

// Yardım menüsü
void showHelp() {
  Serial.println("\n=== DS3231 RTC TEST KOMUTLARI ===");
  Serial.println("SET:2025,8,6,15,30,0 - RTC zamanını ayarla");
  Serial.println("                     (Yıl,Ay,Gün,Saat,Dakika,Saniye)");
  Serial.println("INFO                 - RTC detaylı bilgilerini göster");
  Serial.println("HELP                 - Bu yardım menüsünü göster");
  Serial.println("=====================================");
  Serial.println("Pin Bağlantıları:");
  Serial.println("RTC SDA -> GPIO21 (I2C Data)");
  Serial.println("RTC SCL -> GPIO22 (I2C Clock)");
  Serial.println("RTC VCC -> 3.3V");
  Serial.println("RTC GND -> GND");
  Serial.println("=====================================");
}