#include <Arduino.h>
#include <LoRa_E22.h>
#include "../binary_protocol.h"

#define LORA_M0 19
#define LORA_M1 18
#define LORA_AUX 4
#define LORA_TX 16
#define LORA_RX 17

#define LORA_CHANNEL 20
#define LORA_ADDR_H 0x00
#define LORA_ADDR_L 0x0A              

LoRa_E22 E22(&Serial2, LORA_AUX, LORA_M0, LORA_M1);

uint16_t paket_sayisi = 1;

// SEND komutu için zamanlayıcı değişkenleri
bool send_aktif = false;
unsigned long son_send_zamani = 0;
uint8_t send_sayaci = 0;
const uint8_t max_send_sayisi = 5;
const unsigned long send_araligi = 200; // 200ms

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
  Serial.println("LORA 2 - Button Control baslatildi");
  Serial.println("'SEND' komutunu yazarak Lora1'e button control gonderebilirsiniz.");
  printModuleInfo();
  printConfig();
  delay(500);
}

void sendButtonControlToLora1() {
  // Binary button control paketi oluştur
  ButtonControlPacket buttonControl;
  
  buttonControl.packet_type = PACKET_TYPE_BUTTON_CONTROL;
  buttonControl.paket_sayisi = paket_sayisi;
  buttonControl.manuel_ayrilma = 0; // false
  buttonControl.rhrh = encodeRHRH('5', 'C', '9', 'D');
  
  ResponseStatus rs = E22.sendFixedMessage(0x00, 0x0A, 10, (uint8_t*)&buttonControl, sizeof(buttonControl));
    
  Serial.print("Lora1'e binary button_control gonderildi (");
  Serial.print(sizeof(buttonControl));
  Serial.print(" bytes), Paket#: ");
  Serial.print(paket_sayisi);
  Serial.print(", Deneme: ");
  Serial.println(rs.getResponseDescription());
  
  paket_sayisi++;
}

void loop() {
  // Serial monitor'dan komut kontrol et
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim(); // Boşlukları temizle
    
    if (command.equalsIgnoreCase("SEND")) {
      Serial.println("SEND komutu alindi - 5 kere button control gonderilecek...");
      // SEND zamanlayıcısını başlat
      send_aktif = true;
      send_sayaci = 0;
      son_send_zamani = millis();
      sendButtonControlToLora1(); // İlk gönderiyi hemen yap
      send_sayaci++;
    }
  }
  
  // SEND zamanlayıcısı kontrol et
  if (send_aktif && send_sayaci < max_send_sayisi) {
    if (millis() - son_send_zamani >= send_araligi) {
      sendButtonControlToLora1();
      send_sayaci++;
      son_send_zamani = millis();
      
      if (send_sayaci >= max_send_sayisi) {
        send_aktif = false;
        Serial.println("5 kere button control gonderimi tamamlandi.");
      }
    }
  }
  
  // Lora 1'den binary telemetry bekle
  if (E22.available() > 1) {
    ResponseContainer rs = E22.receiveMessage();
    
    // Binary veri olarak işle
    uint8_t* data = (uint8_t*)rs.data.c_str();
    int length = rs.data.length();
    
    if (length > 0 && data[0] == PACKET_TYPE_TELEMETRY) {
      if (length >= sizeof(TelemetryPacket)) {
        TelemetryPacket* telemetry = (TelemetryPacket*)data;
        
        // Tüm telemetry verilerini detaylı şekilde göster
        Serial.println("=== LORA1'DEN BINARY TELEMETRY ALINDI ===");
        Serial.print("Paket#: "); Serial.println(telemetry->paket_sayisi);
        Serial.print("Uydu Status: "); Serial.println(telemetry->uydu_statusu);
        Serial.print("Hata Kodu: 0b"); Serial.println(telemetry->hata_kodu, BIN);
        Serial.print("Gonderme Saati: "); Serial.println(telemetry->gonderme_saati);
        
        Serial.print("Basinc1: "); Serial.print(telemetry->basinc1); Serial.println(" Pa");
        Serial.print("Basinc2: "); Serial.print(telemetry->basinc2); Serial.println(" Pa");
        Serial.print("Yukseklik1: "); Serial.print(telemetry->yukseklik1); Serial.println(" m");
        Serial.print("Yukseklik2: "); Serial.print(telemetry->yukseklik2); Serial.println(" m");
        Serial.print("Irtifa Farki: "); Serial.print(telemetry->irtifa_farki); Serial.println(" m");
        Serial.print("Inis Hizi: "); Serial.print(telemetry->inis_hizi); Serial.println(" m/s");
        
        Serial.print("Sicaklik: "); Serial.print(telemetry->sicaklik / 10.0); Serial.println("°C");
        Serial.print("Pil Gerilimi: "); Serial.print(telemetry->pil_gerilimi / 100.0); Serial.println("V");
        
        Serial.print("GPS Latitude: "); Serial.println(telemetry->gps1_latitude, 6);
        Serial.print("GPS Longitude: "); Serial.println(telemetry->gps1_longitude, 6);
        Serial.print("GPS Altitude: "); Serial.print(telemetry->gps1_altitude); Serial.println(" m");
        
        Serial.print("Pitch: "); Serial.print(telemetry->pitch / 10.0); Serial.println("°");
        Serial.print("Roll: "); Serial.print(telemetry->roll / 10.0); Serial.println("°");
        Serial.print("Yaw: "); Serial.print(telemetry->yaw / 10.0); Serial.println("°");
        
        // RHRH decode et
        char rhrh_str[5];
        decodeRHRH(telemetry->rhrh, rhrh_str);
        Serial.print("RHRH: "); Serial.println(rhrh_str);
        
        Serial.print("IoT Sensor 1: "); Serial.print(telemetry->iot_s1_data / 10.0); Serial.println("°C");
        Serial.print("IoT Sensor 2: "); Serial.print(telemetry->iot_s2_data / 10.0); Serial.println("°C");
        Serial.print("Takim No: "); Serial.println(telemetry->takim_no);
        Serial.println("==========================================");
        Serial.println("'SEND' yazarak button control gonderebilirsiniz.");
        
      } else {
        Serial.println("Hatali telemetry paketi boyutu!");
      }
    }
  }
  
  delay(10);
}
