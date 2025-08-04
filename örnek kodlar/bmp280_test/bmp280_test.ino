/*
 * BMP280 ve MPU6050 Sensör Test Kodu
 * ESP32 ile BMP280 sensöründen sıcaklık, basınç ve yükseklik okuma
 * ESP32 ile MPU6050 sensöründen ivme, gyro ve açı okuma
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
 */

#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Sensör objelerini oluştur
Adafruit_BMP280 bmp;
Adafruit_MPU6050 mpu;

// Deniz seviyesi basıncı (hPa) - kalibre etmek için kullanılır
#define SEALEVELPRESSURE_HPA (1013.25)

// Sensör durumları
bool bmp_status = false;
bool mpu_status = false;

void setup() {
  Serial.begin(115200);
  Serial.println("BMP280 ve MPU6050 Test Başlatılıyor...");
  
  // I2C başlat
  Wire.begin();
  
  Serial.println("========================================");
  
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
  
  Serial.println("----------------------------------------");
  Serial.print("MPU6050 Durumu: ");
  Serial.println(mpu_status ? "OK ✓" : "HATA ✗");
  Serial.print("BMP280 Durumu: ");
  Serial.println(bmp_status ? "OK ✓" : "HATA ✗");
  Serial.println("----------------------------------------");
  
  delay(2000); // Sensörlerin stabilize olması için bekle
}

void loop() {
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
  
  // === DURUM ÖZETİ ===
  Serial.println("--- Sistem Durumu ---");
  Serial.print("MPU6050: ");
  Serial.print(mpu_status ? "OK" : "HATA");
  Serial.print(" | BMP280: ");
  Serial.println(bmp_status ? "OK" : "HATA");
  
  Serial.println("========================");
  Serial.println();
  
  delay(2000); // 2 saniye bekle
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
 * === Genel ===
 * 3. I2C Scanner kodu ile sensörlerin adreslerini bulabilirsiniz:
 *    Wire.beginTransmission(address);
 *    if (Wire.endTransmission() == 0) // sensör bulundu
 * 
 * 4. Kütüphane kurulumu:
 *    Arduino IDE -> Tools -> Manage Libraries:
 *    - "Adafruit BMP280" aratın ve kurun
 *    - "Adafruit MPU6050" aratın ve kurun
 *    - "Adafruit Unified Sensor" kütüphanesi de gerekli
 * 
 * === main.ino Uyumluluğu ===
 * 5. Bu kod main.ino ile uyumlu olacak şekilde yazılmıştır:
 *    - MPU6050 adresi: 0x69 (main.ino'daki gibi)
 *    - BMP280 adresi: 0x76 (standart)
 *    - Gyro verileri derece/saniye olarak (main.ino'daki gibi)
 *    - Aynı sensör ayarları kullanılmıştır
 */
