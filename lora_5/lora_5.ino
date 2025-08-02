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
  l5data.temperature = 283; // 28.3°C * 10
  l5data.checksum = calculateChecksum((uint8_t*)&l5data, sizeof(l5data));
  
  ResponseStatus rs = E22.sendFixedMessage(0x00, 0x0A, 10, (uint8_t*)&l5data, sizeof(l5data));
  
  Serial.print("Binary L5 data gonderildi (");
  Serial.print(sizeof(l5data));
  Serial.print(" bytes), Paket#: ");
  Serial.print(paket_sayisi);
  Serial.print(", Sicaklik: ");
  Serial.println(l5data.temperature / 10.0);
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
