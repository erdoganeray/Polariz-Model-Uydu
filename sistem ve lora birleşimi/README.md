# ESP32 Sensör Veri Toplama Sistemi

## 📋 Genel Bakış
Bu proje, ESP32 mikrodenetleyicisi kullanarak çoklu sensör verilerini toplayan, SD karta kaydeden ve servo motorları kontrol eden bir sistemdir.

## 🔧 Pin Bağlantı Şeması

### 📍 ESP32 Pin Konfigürasyonu

#### 🌐 **I2C Sensörler (Varsayılan Pinler)**
| Pin | Fonksiyon | Açıklama |
|-----|-----------|----------|
| GPIO21 | SDA | I2C Veri Hattı |
| GPIO22 | SCL | I2C Saat Hattı |

#### 📡 **GPS Modülü (UART1)**
| ESP32 Pin | GPS Pin | Açıklama |
|-----------|---------|----------|
| GPIO14 | TX | GPS Veri Çıkışı → ESP32 RX |
| GPIO15 | RX | GPS Veri Girişi ← ESP32 TX |
| 3.3V | VCC | Güç Kaynağı |
| GND | GND | Toprak |

#### 💾 **SD Kart Modülü (SPI)**
| ESP32 Pin | SD Kart Pin | Açıklama            |
|-----------|-------------|---------------------|
| GPIO5     | CS          | Chip Select         |
| GPIO18    | SCK         | SPI Saat            |
| GPIO19    | MISO        | Master In Slave Out |
| GPIO23    | MOSI        | Master Out Slave In |
| 3.3V      | VCC         | Güç Kaynağı         |
| GND       | GND         | Toprak              |

#### 🎛️ **Servo Motorlar (PWM)**
| ESP32 Pin | Servo | Açıklama |
|-----------|-------|----------|
| GPIO33 | Servo1 | RHRH Kontrol Servo 1 |
| GPIO25 | Servo2 | RHRH Kontrol Servo 2 |
| GPIO26 | Servo3 | AYRIL Servo (0°→90°) |
| 5V | VCC | Servo Güç Kaynağı |
| GND | GND | Toprak |

#### ⚡ **ADC (Analog Girişler)**
| ESP32 Pin | Fonksiyon | Açıklama |
|-----------|-----------|----------|
| A0 (GPIO36) | Pil Gerilimi | Analog gerilim ölçümü |

---

## 🧩 Sensör Detayları

### 📊 **MPU6050 (Gyro/Accelerometer)**
- **I2C Adresi:** 0x69
- **Bağlantı:** I2C Bus (GPIO21-SDA, GPIO22-SCL)
- **Veri:** Pitch, Roll, Yaw
- **Güç:** 3.3V

### 🌡️ **BMP280 (Basınç/Sıcaklık)**
- **I2C Adresi:** 0x76
- **Bağlantı:** I2C Bus (GPIO21-SDA, GPIO22-SCL)
- **Veri:** Basınç, Sıcaklık, Yükseklik
- **Güç:** 3.3V

### 🕐 **DS3231 RTC (Gerçek Zaman Saati)**
- **I2C Adresi:** 0x68 (alternatif: 0x57)
- **Bağlantı:** I2C Bus (GPIO21-SDA, GPIO22-SCL)
- **Veri:** Tarih, Saat
- **Güç:** 3.3V (pil destekli)

### 🛰️ **GY-GPS6MV2 (GPS Modülü)**
- **Baud Rate:** 9600
- **Bağlantı:** UART1 (GPIO14-RX, GPIO15-TX)
- **Veri:** Latitude, Longitude, Altitude
- **Güç:** 3.3V

---

## 🔌 Bağlantı Diyagramı

```
ESP32                    Sensörler
┌─────────────────┐     ┌─────────────────┐
│     GPIO21(SDA) │────▶│ MPU6050         │
│     GPIO22(SCL) │────▶│ BMP280          │
│                 │     │ DS3231 RTC      │
│                 │     └─────────────────┘
│                 │     
│     GPIO14(RX)  │────▶│ GPS TX          │
│     GPIO15(TX)  │◄────│ GPS RX          │
│                 │     
│     GPIO5(CS)   │────▶│ SD Kart CS      │
│     GPIO18(SCK) │────▶│ SD Kart SCK     │
│     GPIO19(MISO)│◄────│ SD Kart MISO    │
│     GPIO23(MOSI)│────▶│ SD Kart MOSI    │
│                 │     
│     GPIO33      │────▶│ Servo1 Signal   │
│     GPIO25      │────▶│ Servo2 Signal   │
│     GPIO26      │────▶│ Servo3 Signal   │
│                 │     
│     A0(GPIO36)  │────▶│ Pil Gerilimi    │
└─────────────────┘
```

---

## ⚙️ Güç Bağlantıları

### 🔋 **3.3V Rail**
- ESP32 3.3V → MPU6050 VCC
- ESP32 3.3V → BMP280 VCC  
- ESP32 3.3V → DS3231 VCC
- ESP32 3.3V → GPS VCC
- ESP32 3.3V → SD Kart VCC

### 🔌 **5V Rail (Servo Motorlar)**
- Harici 5V Kaynağı → Servo1 VCC
- Harici 5V Kaynağı → Servo2 VCC
- Harici 5V Kaynağı → Servo3 VCC

### 🌍 **Toprak (GND)**
- Tüm modüller ve ESP32 GND'si ortak bağlanmalı

---

## 🎮 Komut Sistemi

### 📡 **Serial Komutlar (115200 baud)**
| Komut | Açıklama | Örnek |
|-------|----------|-------|
| `GET_DATA` | Manuel veri toplama | `GET_DATA` |
| `RHRH:` | Filtre pozisyon kontrolü | `RHRH:m2k6` |
| `AYRIL` | Servo3 ayrılma (0°→90°) | `AYRIL` |
| `STATUS` | Sistem durumu | `STATUS` |
| `SERVO_TEST` | Servo test hareketi | `SERVO_TEST` |
| `SET:` | RTC zaman ayarı | `SET:2025,8,3,14,30,0` |

### 🎯 **RHRH Servo Kontrol**
Format: `RHRH:X#Y#`
- **X, Y:** Pozisyon harfi (b, m, k, y)
- **#:** Süre (0-9 saniye)

**Servo Pozisyonları:**
- **b:** 50° (varsayılan)
- **m:** 0°
- **k:** 100°
- **y:** 150°

**Örnek:** `RHRH:m2k6`
- Servo1: 0°'ye git, 2 saniye bekle, sonra 50°'ye dön
- Servo2: 100°'ye git, 6 saniye bekle, sonra 50°'ye dön

---

## 📊 Veri Formatı

### 📈 **Telemetri Paketi (CSV)**
```
Packet_Number,Uydu_Statusu,Hata_Kodu,Timestamp,Pressure1,Pressure2,
Yukseklik1,Yukseklik2,Irtifa_Farki,Inis_Hizi,Temperature,Pil_Gerilimi,
Latitude,Longitude,Altitude,Pitch,Roll,Yaw,RHRH,IoT_S1_Data,IoT_S2_Data,
Takim_No,Sensor_Status
```

### 🔍 **Sensör Durumu (6-bit)**
- Bit 0: BMP280 (1=OK, 0=Hata)
- Bit 1: MPU6050 (1=OK, 0=Hata)
- Bit 2: RTC (1=OK, 0=Hata)
- Bit 3: GPS (1=OK, 0=Hata)
- Bit 4: SD Kart (1=OK, 0=Hata)
- Bit 5: ADC (1=OK, 0=Hata)

**Örnek:** `111111` = Tüm sensörler çalışıyor

---

## 🛠️ Kurulum ve Kullanım

### 📦 **Gerekli Kütüphaneler**
```cpp
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <TinyGPS++.h>
#include <RTClib.h>
#include <SD.h>
#include <FS.h>
```

### 🔧 **Bağlantı Kontrol Listesi**
- [ ] I2C sensörler (GPIO21, GPIO22)
- [ ] GPS modülü (GPIO14, GPIO15)
- [ ] SD kart (GPIO5, GPIO18, GPIO19, GPIO23)
- [ ] Servo motorlar (GPIO33, GPIO25, GPIO26)
- [ ] ADC pil gerilimi (A0)
- [ ] Güç bağlantıları (3.3V, 5V, GND)

### 🚀 **İlk Çalıştırma**
1. Bağlantıları kontrol edin
2. SD kartı FAT32 formatında hazırlayın
3. GPS modülünü açık alana çıkarın
4. RTC zamanını ayarlayın: `SET:2025,8,3,14,30,0`
5. `STATUS` komutuyla sistem durumunu kontrol edin

---

## 🐛 Sorun Giderme

### ❌ **Yaygın Sorunlar**

**GPS Veri Alamıyor:**
- GPS modülünü açık alana çıkarın
- 4-5 dakika bekleyin (cold start)
- Pin bağlantılarını kontrol edin

**SD Kart Hata:**
- FAT32 formatında olduğundan emin olun
- Kapasiteyi 32GB altında tutun
- CS pin bağlantısını kontrol edin

**I2C Sensör Bulunamıyor:**
- Pullup dirençleri kontrol edin
- I2C adreslerini tarayın
- Güç bağlantılarını kontrol edin

---

## 📝 Notlar

- Sistem otomatik olarak 1Hz frekansta veri toplar
- SD karta CSV formatında kayıt yapar
- Sensör durumları 5 saniyede bir güncellenir
- Servo motorlar süre dolunca otomatik olarak varsayılan pozisyona döner

## 👥 Takım
**Takım No:** 626541

---

> 📧 **İletişim:** Herhangi bir sorun durumunda sistem durumunu `STATUS` komutuyla kontrol edebilirsiniz.
