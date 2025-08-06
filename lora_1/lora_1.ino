#include <Arduino.h>
#include <LoRa_E22.h>
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

uint16_t paket_sayisi_lora2 = 1;
uint16_t paket_sayisi_lora3 = 1;

// RHRH değeri - başlangıçta 0000, lora_2'den gelen değerle güncellenecek
uint32_t current_rhrh = encodeRHRH('0', '0', '0', '0');

// Yükseklik hesaplama için gerekli değişkenler
float previous_altitude = 0.0;
unsigned long previous_time = 0;

// Basınç değerinden yükseklik hesaplama fonksiyonu
float calculateAltitude(float pressure) {
  // Deniz seviyesi basıncı (Pa)
  const float SEA_LEVEL_PRESSURE = 101325.0;
  
  // Barometrik formül kullanarak yükseklik hesapla
  // h = 44330 * (1 - (P/P0)^(1/5.255))
  float altitude = 44330.0 * (1.0 - pow(pressure / SEA_LEVEL_PRESSURE, 1.0/5.255));
  
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
  // Binary telemetry paketi oluştur
  TelemetryPacket telemetry;

  telemetry.packet_type = PACKET_TYPE_TELEMETRY;
  telemetry.paket_sayisi = paket_sayisi_lora2;
  telemetry.uydu_statusu = 5;
  telemetry.hata_kodu = 0b010101; // 010101 binary (6 bit)
  telemetry.gonderme_saati = 1722426600; // Unix timestamp for 31/07/2025-14:30:00
  telemetry.basinc1 = 101325.00;
  telemetry.basinc2 = 101300.00;
  telemetry.sicaklik = 2540; // 25.40°C * 100
  telemetry.pil_gerilimi = 377; // 3.77V * 100
  telemetry.gps1_latitude = 39.123456;
  telemetry.gps1_longitude = 32.654321;
  telemetry.gps1_altitude = 155.20;
  telemetry.pitch = 152; // 15.2° * 10
  telemetry.roll = 87; // 8.7° * 10
  telemetry.yaw = 1805; // 180.5° * 10
  telemetry.rhrh = current_rhrh; // Güncel RHRH değerini kullan
  telemetry.iot_s1_data = 2250; // 22.50°C * 100
  telemetry.iot_s2_data = 2310; // 23.10°C * 100
  telemetry.takim_no = TAKIM_NO;
  
  // Basınç değerlerinden yükseklikleri hesapla
  float yukseklik1 = calculateAltitude(101325.00);
  float yukseklik2 = calculateAltitude(101300.00);
  
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
  Serial.println(rhrh_str);
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
  
  // Zamanlayıcı başlat
  unsigned long loop_start_time = millis();
  
  // 1. Lora 2'ye telemetry gönder
  sendTelemetryToLora2();
  
  // 2. 1 saniye dolana kadar dinleme modunda ol
  while (millis() - loop_start_time < 950) {
    if (E22.available() > 1) {
      waitForMessage(100); // Kısa timeout ile mesaj işle
    }
    delay(10); // Buffer yönetimi için kısa bekleme
  }
  
  Serial.println("=== COMM LOOP TAMAMLANDI ===");
}
