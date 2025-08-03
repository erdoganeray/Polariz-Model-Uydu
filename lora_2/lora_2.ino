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

// Komut değişkenleri
uint32_t rhrh_degeri = encodeRHRH('0', '0', '0', '0'); // Başlangıç değeri 0000
uint8_t manuel_ayrilma_degeri = 0; // Başlangıç değeri false

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
  Serial.println("Komutlar:");
  Serial.println("  'SEND' - 5 kere button control gonder");
  Serial.println("  'RHRH:XXXX' - RHRH degerini guncelle (ornek: RHRH:5C9D)");
  Serial.println("  'MANUEL_AYRILMA' - Manuel ayrilma true yap");
  Serial.println("  'RESET_MANUEL' - Manuel ayrilma false yap");
  Serial.println();
  Serial.println("Baslangic degerleri:");
  Serial.println("  RHRH: 0000");
  Serial.println("  Manuel Ayrilma: false");
  Serial.println();
  printModuleInfo();
  printConfig();
  delay(500);
}

void sendButtonControlToLora1() {
  // Binary button control paketi oluştur
  ButtonControlPacket buttonControl;
  
  buttonControl.packet_type = PACKET_TYPE_BUTTON_CONTROL;
  buttonControl.paket_sayisi = paket_sayisi;
  buttonControl.manuel_ayrilma = manuel_ayrilma_degeri; // Güncellenmiş değer kullan
  buttonControl.rhrh = rhrh_degeri; // Güncellenmiş değer kullan
  
  ResponseStatus rs = E22.sendFixedMessage(0x00, 0x0A, 10, (uint8_t*)&buttonControl, sizeof(buttonControl));
    
  // RHRH ve Manuel Ayrılma durumunu da yazdır
  char rhrh_str[5];
  decodeRHRH(rhrh_degeri, rhrh_str);
  
  Serial.print("Lora1'e binary button_control gonderildi (");
  Serial.print(sizeof(buttonControl));
  Serial.print(" bytes), Paket#: ");
  Serial.print(paket_sayisi);
  Serial.print(", RHRH: ");
  Serial.print(rhrh_str);
  Serial.print(", Manuel Ayrilma: ");
  Serial.print(manuel_ayrilma_degeri ? "true" : "false");
  Serial.print(", Status: ");
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
    else if (command.startsWith("RHRH:")) {
      // RHRH komutunu işle (format: RHRH:5C9D)
      String rhrh_str = command.substring(5); // "RHRH:" kısmını atla
      if (rhrh_str.length() == 4) {
        char r1 = rhrh_str[0];
        char h1 = rhrh_str[1];
        char r2 = rhrh_str[2];
        char h2 = rhrh_str[3];
        
        rhrh_degeri = encodeRHRH(r1, h1, r2, h2);
        Serial.print("RHRH degeri guncellendi: ");
        Serial.println(rhrh_str);
      } else {
        Serial.println("HATA: RHRH formatı yanlış! Örnek: RHRH:5C9D");
      }
    }
    else if (command.equalsIgnoreCase("MANUEL_AYRILMA")) {
      // Manuel ayrılma komutunu işle
      manuel_ayrilma_degeri = 1; // true yap
      Serial.println("Manuel ayrilma degeri true olarak ayarlandi");
    }
    else if (command.equalsIgnoreCase("RESET_MANUEL")) {
      // Manuel ayrılma değerini sıfırla
      manuel_ayrilma_degeri = 0; // false yap
      Serial.println("Manuel ayrilma degeri false olarak sifirlandi");
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
        
        // Eğer manuel ayrılma aktifse, gönderim sonrası sıfırla
        if (manuel_ayrilma_degeri == 1) {
          manuel_ayrilma_degeri = 0;
          Serial.println("Manuel ayrilma degeri otomatik olarak false'a sifirlandi.");
        }
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
        
        // RHRH decode et
        char rhrh_str[5];
        decodeRHRH(telemetry->rhrh, rhrh_str);
        
        // Tarih formatını datetime string'e çevir
        unsigned long timestamp = telemetry->gonderme_saati;
        int day = (timestamp / 86400) % 30 + 1;  // Basit gün hesabı
        int month = ((timestamp / 86400) / 30) % 12 + 1;  // Basit ay hesabı
        int year = 2025;  // Sabit yıl
        int hour = (timestamp % 86400) / 3600;
        int minute = (timestamp % 3600) / 60;
        int second = timestamp % 60;
        
        // Hata kodunu binary string'e çevir
        String hata_kodu_str = "";
        for(int i = 4; i >= 0; i--) {
          hata_kodu_str += ((telemetry->hata_kodu >> i) & 1) ? "1" : "0";
        }
        
        // Virgülle ayrılmış format
        Serial.print(telemetry->paket_sayisi); Serial.print(",");
        Serial.print(telemetry->uydu_statusu); Serial.print(",");
        Serial.print(hata_kodu_str); Serial.print(",");
        Serial.printf("%02d.%02d.%d-%02d:%02d:%02d,", day, month, year, hour, minute, second);
        Serial.print(telemetry->basinc1, 1); Serial.print(",");
        Serial.print(telemetry->basinc2, 1); Serial.print(",");
        Serial.print(telemetry->yukseklik1, 1); Serial.print(",");
        Serial.print(telemetry->yukseklik2, 1); Serial.print(",");
        Serial.print(telemetry->irtifa_farki, 1); Serial.print(",");
        Serial.print(telemetry->inis_hizi, 1); Serial.print(",");
        Serial.print(telemetry->sicaklik / 10.0, 1); Serial.print(",");
        Serial.print(telemetry->pil_gerilimi / 100.0, 2); Serial.print(",");
        Serial.print(telemetry->gps1_latitude, 6); Serial.print(",");
        Serial.print(telemetry->gps1_longitude, 6); Serial.print(",");
        Serial.print(telemetry->gps1_altitude, 1); Serial.print(",");
        Serial.print(telemetry->pitch / 10.0, 1); Serial.print(",");
        Serial.print(telemetry->roll / 10.0, 1); Serial.print(",");
        Serial.print(telemetry->yaw / 10.0, 1); Serial.print(",");
        Serial.print(rhrh_str); Serial.print(",");
        Serial.print(telemetry->iot_s1_data / 10.0, 1); Serial.print(",");
        Serial.print(telemetry->iot_s2_data / 10.0, 1); Serial.print(",");
        Serial.println(telemetry->takim_no);
        
      } else {
        Serial.println("Hatali telemetry paketi boyutu!");
      }
    }
  }
  
  delay(10);
}
