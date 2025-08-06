/*
 * LoRa E22 Basit Konfigürasyon
 * Sadece temel parametreleri konfigure eder:
 * - Air Data Rate (Hava veri hızı)
 * - Packet Size (Paket boyutu)
 * - Transmission Mode (İletim modu)
 * - Address (Adres)
 * - Channel (Kanal)
 * 
 * ESP32 Pin Bağlantıları:
 * LoRa E22    ESP32
 * VCC         3.3V
 * GND         GND
 * TX          GPIO 4 (RX2)
 * RX          GPIO 2 (TX2)
 * AUX         GPIO 18
 * M0          GPIO 21
 * M1          GPIO 19
 */

#include "LoRa_E22.h"

// LoRa E22 pin tanımlamaları
#define LORA_TX_PIN 4     // ESP32 RX2 -> LoRa TX
#define LORA_RX_PIN 2     // ESP32 TX2 -> LoRa RX
#define LORA_AUX_PIN 18   // AUX pin
#define LORA_M0_PIN 21    // M0 pin
#define LORA_M1_PIN 19    // M1 pin

// LoRa E22 nesnesi oluştur
LoRa_E22 lora(LORA_TX_PIN, LORA_RX_PIN, &Serial2, LORA_AUX_PIN, LORA_M0_PIN, LORA_M1_PIN, UART_BPS_RATE_9600);

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("LoRa E22 Basit Konfigürasyon Başlatılıyor...");
  
  // LoRa modülünü başlat
  if (lora.begin()) {
    Serial.println("LoRa E22 başarıyla başlatıldı!");
  } else {
    Serial.println("LoRa E22 başlatılamadı!");
    while(1);
  }
  
  delay(1000);
  
  // Basit konfigürasyon uygula
  configureLoRa();
  
  Serial.println("Konfigürasyon tamamlandı!");
}

void loop() {
  // Test mesajı gönder ve al
  testCommunication();
  delay(5000);
}

void configureLoRa() {
  Serial.println("\n=== LoRa Konfigürasyonu ===");
  
  // Konfigürasyon yapısını oluştur
  Configuration configuration;
  
  // *** 1. ADDRESS AYARLARI ***
  configuration.ADDH = 0x00;        // Yüksek adres byte'ı (0x00-0xFF)
  configuration.ADDL = 0x01;        // Düşük adres byte'ı (0x00-0xFF)
  configuration.NETID = 0x00;       // Network ID (0x00-0xFF)
  
  // *** 2. CHANNEL AYARI ***
  configuration.CHAN = 23;          // Kanal numarası (0-80)
                                    // Frekans = 410.125 + CHAN MHz
                                    // Kanal 23 = 433.125 MHz
  
  // *** 3. AIR DATA RATE (Hava Veri Hızı) ***
  configuration.SPED.airDataRate = AIR_DATA_RATE_011_48;  // 4.8kbps
  // Diğer seçenekler:
  // AIR_DATA_RATE_000_03  -> 0.3kbps  (en uzun menzil)
  // AIR_DATA_RATE_001_12  -> 1.2kbps
  // AIR_DATA_RATE_010_24  -> 2.4kbps  (varsayılan)
  // AIR_DATA_RATE_011_48  -> 4.8kbps  *** SEÇİLDİ ***
  // AIR_DATA_RATE_100_96  -> 9.6kbps
  // AIR_DATA_RATE_101_192 -> 19.2kbps
  // AIR_DATA_RATE_110_384 -> 38.4kbps
  // AIR_DATA_RATE_111_625 -> 62.5kbps (en kısa menzil, en hızlı)
  
  // *** 4. PACKET SIZE (Paket Boyutu) ***
  configuration.OPTION.subPacketSetting = SPS_240_00;  // 240 byte
  // Diğer seçenekler:
  // SPS_240_00 -> 240 bytes (varsayılan)
  // SPS_128_01 -> 128 bytes
  // SPS_064_10 -> 64 bytes
  // SPS_032_11 -> 32 bytes
  
  // *** 5. TRANSMISSION MODE (İletim Modu) ***
  configuration.TRANSMISSION_MODE.fixedTransmission = FT_FIXED_TRANSMISSION;  // Sabit iletim
  // Seçenekler:
  // FT_TRANSPARENT_TRANSMISSION -> Şeffaf iletim (varsayılan)
  // FT_FIXED_TRANSMISSION       -> Sabit iletim (adres ve kanal belirtilerek) *** SEÇİLDİ ***
  
  // Diğer gerekli parametreler (değiştirmeyin)
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
  ResponseStatus rs = lora.setConfiguration(configuration, WRITE_CFG_PWR_DWN_SAVE);
  if (rs.code == 1) {
    Serial.println("✓ Konfigürasyon başarıyla uygulandı!");
    printConfigurationSummary();
  } else {
    Serial.println("✗ Konfigürasyon uygulanamadı!");
  }
}

void printConfigurationSummary() {
  Serial.println("\n--- Uygulanan Konfigürasyon ---");
  Serial.println("Address H: 0x00");
  Serial.println("Address L: 0x01");
  Serial.println("Network ID: 0x00");
  Serial.println("Kanal: 23 (433.125 MHz)");
  Serial.println("Hava Veri Hızı: 4.8kbps");
  Serial.println("Paket Boyutu: 240 bytes");
  Serial.println("İletim Modu: Sabit İletim (Fixed)");
  Serial.println("-------------------------------");
}

void testCommunication() {
  // Normal modda test mesajı gönder
  lora.setMode(MODE_0_NORMAL);
  
  String testMessage = "Test-" + String(millis());
  
  ResponseStatus rs = lora.sendMessage(testMessage);
  if (rs.code == 1) {
    Serial.println("📤 Mesaj gönderildi: " + testMessage);
  } else {
    Serial.println("❌ Mesaj gönderilemedi!");
  }
  
  // Gelen mesajları kontrol et
  if (lora.available() > 0) {
    ResponseContainer rc = lora.receiveMessage();
    if (rc.status.code == 1) {
      Serial.println("📥 Mesaj alındı: " + rc.data);
    }
  }
}

// Farklı konfigürasyon örnekleri
void configureLongRange() {
  // Uzun menzil için konfigürasyon
  Serial.println("Uzun menzil konfigürasyonu uygulanıyor...");
  
  Configuration config;
  config.ADDH = 0x00;
  config.ADDL = 0x02;
  config.NETID = 0x00;
  config.CHAN = 23;
  config.SPED.airDataRate = AIR_DATA_RATE_000_03;  // En düşük hız = En uzun menzil
  config.OPTION.subPacketSetting = SPS_240_00;     // Büyük paket
  config.TRANSMISSION_MODE.fixedTransmission = FT_TRANSPARENT_TRANSMISSION;
  
  // Diğer ayarlar...
  config.SPED.uartBaudRate = UART_BPS_9600;
  config.SPED.uartParity = MODE_00_8N1;
  config.OPTION.transmissionPower = POWER_22;
  config.OPTION.RSSIAmbientNoise = RSSI_AMBIENT_NOISE_DISABLED;
  config.TRANSMISSION_MODE.enableRSSI = RSSI_DISABLED;
  config.TRANSMISSION_MODE.enableRepeater = REPEATER_DISABLED;
  config.TRANSMISSION_MODE.enableLBT = LBT_DISABLED;
  config.TRANSMISSION_MODE.WORTransceiverControl = WOR_RECEIVER;
  config.TRANSMISSION_MODE.WORPeriod = WOR_2000_011;
  
  lora.setConfiguration(config, WRITE_CFG_PWR_DWN_SAVE);
}

void configureHighSpeed() {
  // Yüksek hız için konfigürasyon
  Serial.println("Yüksek hız konfigürasyonu uygulanıyor...");
  
  Configuration config;
  config.ADDH = 0x00;
  config.ADDL = 0x03;
  config.NETID = 0x00;
  config.CHAN = 23;
  config.SPED.airDataRate = AIR_DATA_RATE_111_625;  // En yüksek hız
  config.OPTION.subPacketSetting = SPS_032_11;      // Küçük paket = Hızlı
  config.TRANSMISSION_MODE.fixedTransmission = FT_TRANSPARENT_TRANSMISSION;
  
  // Diğer ayarlar...
  config.SPED.uartBaudRate = UART_BPS_115200;
  config.SPED.uartParity = MODE_00_8N1;
  config.OPTION.transmissionPower = POWER_22;
  config.OPTION.RSSIAmbientNoise = RSSI_AMBIENT_NOISE_DISABLED;
  config.TRANSMISSION_MODE.enableRSSI = RSSI_DISABLED;
  config.TRANSMISSION_MODE.enableRepeater = REPEATER_DISABLED;
  config.TRANSMISSION_MODE.enableLBT = LBT_DISABLED;
  config.TRANSMISSION_MODE.WORTransceiverControl = WOR_RECEIVER;
  config.TRANSMISSION_MODE.WORPeriod = WOR_500_000;
  
  lora.setConfiguration(config, WRITE_CFG_PWR_DWN_SAVE);
}

void configureFixedTransmission() {
  // Sabit iletim için konfigürasyon
  Serial.println("Sabit iletim konfigürasyonu uygulanıyor...");
  
  Configuration config;
  config.ADDH = 0x00;
  config.ADDL = 0x04;
  config.NETID = 0x00;
  config.CHAN = 23;
  config.SPED.airDataRate = AIR_DATA_RATE_010_24;
  config.OPTION.subPacketSetting = SPS_240_00;
  config.TRANSMISSION_MODE.fixedTransmission = FT_FIXED_TRANSMISSION;  // Sabit iletim aktif
  
  // Diğer ayarlar...
  config.SPED.uartBaudRate = UART_BPS_9600;
  config.SPED.uartParity = MODE_00_8N1;
  config.OPTION.transmissionPower = POWER_22;
  config.OPTION.RSSIAmbientNoise = RSSI_AMBIENT_NOISE_DISABLED;
  config.TRANSMISSION_MODE.enableRSSI = RSSI_DISABLED;
  config.TRANSMISSION_MODE.enableRepeater = REPEATER_DISABLED;
  config.TRANSMISSION_MODE.enableLBT = LBT_DISABLED;
  config.TRANSMISSION_MODE.WORTransceiverControl = WOR_RECEIVER;
  config.TRANSMISSION_MODE.WORPeriod = WOR_2000_011;
  
  lora.setConfiguration(config, WRITE_CFG_PWR_DWN_SAVE);
}
