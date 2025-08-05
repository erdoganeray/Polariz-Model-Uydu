/*
 * SD Kart Modülü Test Kodu
 * ESP32 ile SD kart modülü testi
 * SPI protokolü üzerinden SD kart okuma/yazma işlemleri
 * 
 * Bağlantılar (ESP32 ile SD Kart Modülü):
 * SD CS   -> GPIO5  (Chip Select)
 * SD MOSI -> GPIO23 (Master Out Slave In)
 * SD MISO -> GPIO19 (Master In Slave Out)
 * SD SCK  -> GPIO18 (Serial Clock)
 * SD VCC  -> 3.3V
 * SD GND  -> GND
 * 
 * Önemli Notlar:
 * - SD kart FAT32 formatında olmalı
 * - Maksimum 32GB kapasiteli SD kart kullanın
 * - 3.3V besleme kullanın (5V değil)
 * - SD kart yuvaya düzgün takılmalı
 */

#include <SPI.h>
#include <SD.h>
#include <FS.h>

// SD Kart SPI pin tanımlamaları (main.ino'daki gibi)
#define SD_SCK 18
#define SD_MISO 19
#define SD_MOSI 23
#define SD_CS 5

// Test dosyası adları
#define TEST_FILE "/test_file.txt"
#define CSV_FILE "/sensor_test.csv"
#define BINARY_FILE "/binary_test.bin"

// SD kart durumu
bool sd_status = false;

void setup() {
  Serial.begin(115200);
  Serial.println("SD Kart Modülü Test Başlatılıyor...");
  
  delay(1000); // Sistem stabilize olsun
  
  Serial.println("========================================");
  Serial.printf("SPI Pin Konfigürasyonu:\n");
  Serial.printf("- SCK (Clock):  GPIO%d\n", SD_SCK);
  Serial.printf("- MISO (Data):  GPIO%d\n", SD_MISO);
  Serial.printf("- MOSI (Data):  GPIO%d\n", SD_MOSI);
  Serial.printf("- CS (Select):  GPIO%d\n", SD_CS);
  Serial.println("========================================");
  
  // SD kart başlatma testi
  testSDCardInit();
  
  if (sd_status) {
    // SD kart bilgilerini görüntüle
    displaySDCardInfo();
    
    // Dosya işlemleri testleri
    testFileOperations();
    
    // CSV dosyası testi
    testCSVOperations();
    
    // Binary dosya testi
    testBinaryOperations();
    
    // Dosya listesi göster
    listAllFiles();
  } else {
    Serial.println("SD kart başlatılamadığı için testler atlanıyor.");
    Serial.println("Bağlantıları kontrol edin ve sistemi yeniden başlatın.");
  }
  
  Serial.println("========================================");
  Serial.println("Test tamamlandı!");
  Serial.println("Loop'ta sürekli monitoring başlıyor...");
}

void loop() {
  // Her 10 saniyede bir SD kart durumunu kontrol et
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck >= 10000) {
    lastCheck = millis();
    
    Serial.println("\n=== SD KART DURUM KONTROLÜ ===");
    checkSDCardStatus();
    
    if (sd_status) {
      // Test yazma işlemi
      String currentTime = String(millis() / 1000) + " saniye";
      String logData = "Zaman: " + currentTime + ", Durum: OK\n";
      
      if (appendFile(SD, "/monitoring.log", logData.c_str())) {
        Serial.println("Monitoring log'u güncellendi ✓");
      } else {
        Serial.println("Monitoring log'u güncellenemedi ✗");
      }
    }
    
    Serial.println("==============================");
  }
  
  // Serial komut kontrolü
  if (Serial.available()) {
    String command = Serial.readString();
    command.trim();
    
    if (command == "INIT") {
      Serial.println("SD kart yeniden başlatılıyor...");
      testSDCardInit();
    } else if (command == "INFO") {
      displaySDCardInfo();
    } else if (command == "LIST") {
      listAllFiles();
    } else if (command == "TEST") {
      if (sd_status) {
        testFileOperations();
      } else {
        Serial.println("SD kart çalışmıyor!");
      }
    } else if (command == "FORMAT") {
      Serial.println("UYARI: Bu işlev güvenlik nedeniyle devre dışı!");
      Serial.println("SD kartı bilgisayarda FAT32 olarak formatlayın.");
    } else if (command == "HELP") {
      showHelp();
    }
  }
  
  delay(100);
}

void testSDCardInit() {
  Serial.println("--- SD Kart Başlatma Testi ---");
  
  // SD kart başlatma (main.ino'daki gibi)
  if (!SD.begin(SD_CS)) {
    Serial.println("HATA: SD kart başlatılamadı!");
    Serial.println("\nKontrol Listesi:");
    Serial.println("✗ SD kart takılı mı?");
    Serial.println("✗ SD kart FAT32 formatında mı?");
    Serial.println("✗ Pin bağlantıları doğru mu?");
    Serial.println("✗ SD kart kapasitesi 32GB'den küçük mü?");
    Serial.println("✗ 3.3V besleme kullanılıyor mu?");
    Serial.println("\nPin Bağlantıları:");
    Serial.printf("✗ CS: GPIO%d\n", SD_CS);
    Serial.printf("✗ MOSI: GPIO%d\n", SD_MOSI);
    Serial.printf("✗ MISO: GPIO%d\n", SD_MISO);
    Serial.printf("✗ SCK: GPIO%d\n", SD_SCK);
    sd_status = false;
    return;
  }
  
  // SD kart tipi kontrolü
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

void testFileOperations() {
  Serial.println("--- Dosya İşlemleri Testi ---");
  
  if (!sd_status) {
    Serial.println("SD kart çalışmıyor!");
    return;
  }
  
  // Test 1: Dosya yazma
  Serial.println("Test 1: Dosya yazma...");
  String testData = "ESP32 SD Kart Test\n";
  testData += "Zaman: " + String(millis()) + " ms\n";
  testData += "Test verisi: 12345\n";
  testData += "Özel karakterler: ÇĞıİÖŞÜ çğıİöşü\n";
  
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
  
  // Test 4: Dosya boyutu kontrolü
  Serial.println("Test 4: Dosya boyutu...");
  File file = SD.open(TEST_FILE);
  if (file) {
    Serial.printf("✓ Dosya boyutu: %d byte\n", file.size());
    file.close();
  } else {
    Serial.println("✗ Dosya boyutu alınamadı");
  }
  
  // Test 5: Dosya silme
  Serial.println("Test 5: Dosya silme...");
  if (SD.remove(TEST_FILE)) {
    Serial.println("✓ Dosya silme başarılı");
  } else {
    Serial.println("✗ Dosya silme başarısız");
  }
}

void testCSVOperations() {
  Serial.println("--- CSV Dosyası Testi ---");
  
  if (!sd_status) {
    Serial.println("SD kart çalışmıyor!");
    return;
  }
  
  // CSV başlığı oluştur (main.ino'daki gibi)
  String csvHeader = "Zaman,Sıcaklık,Basınç,Nem,Durum\n";
  if (!writeFile(SD, CSV_FILE, csvHeader.c_str())) {
    Serial.println("✗ CSV başlığı yazılamadı");
    return;
  }
  
  Serial.println("✓ CSV dosyası oluşturuldu");
  
  // Test verileri ekle
  for (int i = 1; i <= 5; i++) {
    String csvLine = String(millis()) + "," +
                     String(20.0 + i * 0.5, 1) + "," +
                     String(1013.25 + i, 2) + "," +
                     String(45 + i) + "," +
                     "OK\n";
    
    if (appendFile(SD, CSV_FILE, csvLine.c_str())) {
      Serial.printf("✓ CSV satır %d eklendi\n", i);
    } else {
      Serial.printf("✗ CSV satır %d eklenemedi\n", i);
      break;
    }
    
    delay(100); // Kısa bekleme
  }
  
  // CSV dosyasını oku ve göster
  Serial.println("CSV Dosyası İçeriği:");
  String csvContent = readFile(SD, CSV_FILE);
  Serial.println(csvContent);
}

void testBinaryOperations() {
  Serial.println("--- Binary Dosya Testi ---");
  
  if (!sd_status) {
    Serial.println("SD kart çalışmıyor!");
    return;
  }
  
  // Binary veri oluştur
  uint8_t binaryData[16];
  for (int i = 0; i < 16; i++) {
    binaryData[i] = i * 16 + i; // 0x00, 0x11, 0x22, ... 0xFF
  }
  
  // Binary dosya yazma
  File file = SD.open(BINARY_FILE, FILE_WRITE);
  if (file) {
    size_t written = file.write(binaryData, 16);
    file.close();
    
    if (written == 16) {
      Serial.println("✓ Binary dosya yazma başarılı");
    } else {
      Serial.printf("✗ Binary yazma hatası: %d/16 byte yazıldı\n", written);
    }
  } else {
    Serial.println("✗ Binary dosya açılamadı");
    return;
  }
  
  // Binary dosya okuma
  file = SD.open(BINARY_FILE, FILE_READ);
  if (file) {
    uint8_t readData[16];
    size_t readBytes = file.read(readData, 16);
    file.close();
    
    if (readBytes == 16) {
      Serial.println("✓ Binary dosya okuma başarılı");
      Serial.print("Okunan data: ");
      for (int i = 0; i < 16; i++) {
        Serial.printf("0x%02X ", readData[i]);
      }
      Serial.println();
      
      // Veri doğrulama
      bool dataOK = true;
      for (int i = 0; i < 16; i++) {
        if (readData[i] != binaryData[i]) {
          dataOK = false;
          break;
        }
      }
      
      if (dataOK) {
        Serial.println("✓ Binary veri doğrulaması başarılı");
      } else {
        Serial.println("✗ Binary veri doğrulaması başarısız");
      }
    } else {
      Serial.printf("✗ Binary okuma hatası: %d/16 byte okundu\n", readBytes);
    }
  } else {
    Serial.println("✗ Binary dosya okunamadı");
  }
}

void listAllFiles() {
  Serial.println("--- Dosya Listesi ---");
  
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

void checkSDCardStatus() {
  // SD kart hala çalışıyor mu kontrol et
  File testFile = SD.open("/", FILE_READ);
  if (testFile) {
    testFile.close();
    if (!sd_status) {
      Serial.println("SD kart yeniden çalışmaya başladı ✓");
      sd_status = true;
    } else {
      Serial.println("SD kart durumu: OK ✓");
    }
  } else {
    if (sd_status) {
      Serial.println("SD kart bağlantısı kesildi ✗");
      sd_status = false;
    } else {
      Serial.println("SD kart durumu: HATA ✗");
    }
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

void showHelp() {
  Serial.println("\n=== KULLANILABİLİR KOMUTLAR ===");
  Serial.println("INIT   - SD kartı yeniden başlat");
  Serial.println("INFO   - SD kart bilgilerini göster");
  Serial.println("LIST   - Dosya listesini göster");
  Serial.println("TEST   - Dosya işlemleri testini çalıştır");
  Serial.println("FORMAT - Format uyarısı göster");
  Serial.println("HELP   - Bu yardım menüsünü göster");
  Serial.println("================================");
}

/*
 * Troubleshooting Rehberi:
 * 
 * === SD Kart Bulunamıyor ===
 * 1. Fiziksel kontroller:
 *    - SD kart yuvaya düzgün takılı mı?
 *    - SD kart hasarlı değil mi?
 *    - Pin bağlantıları sağlam mı?
 *    
 * 2. Besleme kontrolleri:
 *    - 3.3V kullanılıyor mu? (5V değil)
 *    - GND bağlantısı var mı?
 *    
 * 3. Pin bağlantıları:
 *    - CS: GPIO5
 *    - MOSI: GPIO23
 *    - MISO: GPIO19
 *    - SCK: GPIO18
 * 
 * === SD Kart Format Sorunları ===
 * 4. SD kart formatı:
 *    - FAT32 formatında olmalı
 *    - NTFS, exFAT desteklenmez
 *    - Maksimum 32GB kapasiteli olmalı
 *    
 * === Yazma/Okuma Hataları ===
 * 5. Dosya sistemi sorunları:
 *    - SD kart yazma korumalı değil mi?
 *    - Dosya adı geçerli mi? (8.3 format önerilir)
 *    - Yeterli boş alan var mı?
 *    
 * === Performans Sorunları ===
 * 6. SPI hızı:
 *    - SD.begin(CS_PIN, SPI, 4000000) ile hızı düşürün
 *    - Uzun kablo kullanmayın
 *    - Kapasitör ile besleme filtreleyin
 *    
 * === Kütüphane Sorunları ===
 * 7. ESP32 kütüphaneleri:
 *    - ESP32 Board Package güncel mi?
 *    - SD kütüphanesi dahil mi?
 *    - SPI kütüphanesi dahil mi?
 */
