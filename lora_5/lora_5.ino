#include <Arduino.h>
#include <LoRa_E22.h>
#include "../binary_protocol.h"

#define LORA_M0 19
#define LORA_M1 18
#define LORA_AUX 4
#define LORA_TX 16
#define LORA_RX 17

#define LORA_CHANNEL 50
#define LORA_ADDR_H 0x00
#define LORA_ADDR_L 0x0A               

LoRa_E22 E22(&Serial2, LORA_AUX, LORA_M0, LORA_M1);

uint16_t paket_sayisi = 1;
unsigned long son_yayin = 0;

void configureLoRa() {
  Serial.println("\n=== LoRa Konfigürasyonu ===");
  
  // Konfigürasyon yapısını oluştur
  Configuration configuration;
  
  // *** ADDRESS AYARLARI ***
  configuration.ADDH = 0x00;        // Yüksek adres byte'ı
  configuration.ADDL = 0x0A;        // Düşük adres byte'ı (10 decimal = 0x0A hex)
  configuration.NETID = 0x00;       // Network ID
  
  // *** CHANNEL AYARI ***
  configuration.CHAN = 50;          // Kanal 50
  
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
    Serial.println("Channel: 50");
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
  Serial.println("LORA 5 - Duzenli Yayin Modulu baslatildi");
  printModuleInfo();
  printConfig();
  delay(500);
}

void sendRegularData() {
  // Binary L5 data paketi oluştur
  LoraDataPacket l5data;
  
  l5data.packet_type = PACKET_TYPE_L5_DATA;
  l5data.paket_sayisi = paket_sayisi;
  l5data.temperature = 2830; // 28.30°C * 100
  l5data.checksum = calculateChecksum((uint8_t*)&l5data, sizeof(l5data));
  
  ResponseStatus rs = E22.sendFixedMessage(0x00, 0x0A, 10, (uint8_t*)&l5data, sizeof(l5data));
  
  Serial.print("Binary L5 data gonderildi (");
  Serial.print(sizeof(l5data));
  Serial.print(" bytes), Paket#: ");
  Serial.print(paket_sayisi);
  Serial.print(", Sicaklik: ");
  Serial.println(l5data.temperature / 100.0);
  Serial.print("Durum: ");
  Serial.println(rs.getResponseDescription());
  
  paket_sayisi++;
}

void loop() {
  unsigned long su_an = millis();
  
  // Her 4000ms (4 saniye) bir düzenli yayın yap (Lora4'ten farklı periyot)
  if (su_an - son_yayin >= 4000) {
    sendRegularData();
    son_yayin = su_an;
  }
  
  // Gelen mesajları dinle
  if (E22.available() > 1) {
    ResponseContainer rs = E22.receiveMessage();
    
    // Binary veri olarak işle
    uint8_t* data = (uint8_t*)rs.data.c_str();
    int length = rs.data.length();
    
    if (length > 0) {
      Serial.print("Binary paket alindi - Tip: 0x");
      Serial.print(data[0], HEX);
      Serial.print(", Boyut: ");
      Serial.print(length);
      Serial.println(" bytes");
    }
  }
  
  delay(10);
}
