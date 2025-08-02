#include <Arduino.h>
#include <LoRa_E22.h>
#include "../binary_protocol.h"

#define LORA_M0 19
#define LORA_M1 18
#define LORA_AUX 4
#define LORA_TX 16
#define LORA_RX 17

#define LORA_CHANNEL 10
#define LORA_ADDR_H 0x00
#define LORA_ADDR_L 0x0A             

LoRa_E22 E22(&Serial2, LORA_AUX, LORA_M0, LORA_M1);

uint16_t paket_sayisi_lora2 = 1;
uint16_t paket_sayisi_lora3 = 1;

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
  Serial.println("LORA 1 - Ana Kontrol Merkezi baslatildi");
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
  telemetry.hata_kodu = 0b01010; // 01010 binary
  telemetry.gonderme_saati = 1722426600; // Unix timestamp for 31/07/2025-14:30:00
  telemetry.basinc1 = 101325.0;
  telemetry.basinc2 = 101300.0;
  telemetry.yukseklik1 = 150.5;
  telemetry.yukseklik2 = 149.8;
  telemetry.irtifa_farki = 0.7;
  telemetry.inis_hizi = -2.3;
  telemetry.sicaklik = 254; // 25.4°C * 10
  telemetry.pil_gerilimi = 377; // 3.77V * 100
  telemetry.gps1_latitude = 39.123456;
  telemetry.gps1_longitude = 32.654321;
  telemetry.gps1_altitude = 155.2;
  telemetry.pitch = 152; // 15.2° * 10
  telemetry.roll = 87; // 8.7° * 10
  telemetry.yaw = 1805; // 180.5° * 10
  telemetry.rhrh = encodeRHRH('1', 'A', '2', 'B');
  telemetry.iot_s1_data = 225; // 22.5°C * 10
  telemetry.iot_s2_data = 231; // 23.1°C * 10
  telemetry.takim_no = TAKIM_NO;
  
  ResponseStatus rs = E22.sendFixedMessage(0x00, 0x0A, 20, (uint8_t*)&telemetry, sizeof(telemetry));
  
  Serial.print("Lora2'ye binary telemetry gonderildi (");
  Serial.print(sizeof(telemetry));
  Serial.print(" bytes), Paket#: ");
  Serial.println(paket_sayisi_lora2);
  Serial.print("Durum: ");
  Serial.println(rs.getResponseDescription());
  
  paket_sayisi_lora2++;
}

void sendRequestToLora3() {
  // Binary request paketi oluştur
  RequestPacket request;
  
  request.packet_type = PACKET_TYPE_REQUEST;
  request.paket_sayisi = paket_sayisi_lora3;
  
  ResponseStatus rs = E22.sendFixedMessage(0x00, 0x0A, 30, (uint8_t*)&request, sizeof(request));
  
  Serial.print("Lora3'e binary request gonderildi (");
  Serial.print(sizeof(request));
  Serial.print(" bytes), Paket#: ");
  Serial.println(paket_sayisi_lora3);
  Serial.print("Durum: ");
  Serial.println(rs.getResponseDescription());
  
  paket_sayisi_lora3++;
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
              char rhrh_str[5];
              decodeRHRH(btn->rhrh, rhrh_str);
              Serial.print(", RHRH: ");
              Serial.println(rhrh_str);
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
                Serial.println(l4data->temperature / 10.0);
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
                Serial.println(l5data->temperature / 10.0);
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
