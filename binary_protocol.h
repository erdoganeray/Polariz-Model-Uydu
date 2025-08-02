#ifndef BINARY_PROTOCOL_H
#define BINARY_PROTOCOL_H

#include <Arduino.h>

// Paket tipleri
#define PACKET_TYPE_TELEMETRY 0x01
#define PACKET_TYPE_BUTTON_CONTROL 0x02
#define PACKET_TYPE_REQUEST 0x03
#define PACKET_TYPE_PRESSURE_CONTAINER 0x04
#define PACKET_TYPE_L4_DATA 0x05
#define PACKET_TYPE_L5_DATA 0x06

// Sabit değerler
#define TAKIM_NO 656241

// Telemetry paketi yapısı (toplam ~70 byte)
struct __attribute__((packed)) TelemetryPacket {
    uint8_t packet_type;           // 1 byte - paket tipi
    uint16_t paket_sayisi;         // 2 byte - paket sayısı
    uint8_t uydu_statusu;          // 1 byte - uydu durumu
    uint8_t hata_kodu;             // 1 byte - 5 bitlik hata kodu (00000-11111)
    uint32_t gonderme_saati;       // 4 byte - Unix timestamp
    float basinc1;                 // 4 byte - Pascal
    float basinc2;                 // 4 byte - Pascal
    float yukseklik1;              // 4 byte - metre
    float yukseklik2;              // 4 byte - metre
    float irtifa_farki;            // 4 byte - metre
    float inis_hizi;               // 4 byte - m/s
    int16_t sicaklik;              // 2 byte - Celsius * 10 (örn: 254 = 25.4°C)
    uint16_t pil_gerilimi;         // 2 byte - Volt * 100 (örn: 377 = 3.77V)
    float gps1_latitude;           // 4 byte - derece
    float gps1_longitude;          // 4 byte - derece
    float gps1_altitude;           // 4 byte - metre
    int16_t pitch;                 // 2 byte - derece * 10
    int16_t roll;                  // 2 byte - derece * 10
    int16_t yaw;                   // 2 byte - derece * 10
    uint32_t rhrh;                 // 4 byte - encoded rakam/harf/rakam/harf
    int16_t iot_s1_data;           // 2 byte - Celsius * 10
    int16_t iot_s2_data;           // 2 byte - Celsius * 10
    uint32_t takim_no;             // 4 byte - sabit 656241
};

// Button Control paketi yapısı (~8 byte)
struct __attribute__((packed)) ButtonControlPacket {
    uint8_t packet_type;           // 1 byte - paket tipi
    uint16_t paket_sayisi;         // 2 byte - paket sayısı
    uint8_t manuel_ayrilma;        // 1 byte - true/false (1/0)
    uint32_t rhrh;                 // 4 byte - encoded rakam/harf/rakam/harf
};

// Request paketi yapısı (~3 byte)
struct __attribute__((packed)) RequestPacket {
    uint8_t packet_type;           // 1 byte - paket tipi
    uint16_t paket_sayisi;         // 2 byte - paket sayısı
};

// Pressure Container paketi yapısı (~7 byte)
struct __attribute__((packed)) PressureContainerPacket {
    uint8_t packet_type;           // 1 byte - paket tipi
    uint16_t paket_sayisi;         // 2 byte - paket sayısı
    float basinc1;                 // 4 byte - Pascal
};

// L4/L5 Data paketi yapısı (~7 byte)
struct __attribute__((packed)) LoraDataPacket {
    uint8_t packet_type;           // 1 byte - paket tipi (L4_DATA veya L5_DATA)
    uint16_t paket_sayisi;         // 2 byte - paket sayısı
    int16_t temperature;           // 2 byte - Celsius * 10
    uint8_t checksum;              // 1 byte - basit checksum
};

// RHRH encoding/decoding fonksiyonları
uint32_t encodeRHRH(char r1, char h1, char r2, char h2) {
    return ((uint32_t)r1 << 24) | ((uint32_t)h1 << 16) | ((uint32_t)r2 << 8) | (uint32_t)h2;
}

void decodeRHRH(uint32_t encoded, char* result) {
    result[0] = (encoded >> 24) & 0xFF;
    result[1] = (encoded >> 16) & 0xFF;
    result[2] = (encoded >> 8) & 0xFF;
    result[3] = encoded & 0xFF;
    result[4] = '\0';
}

// Checksum hesaplama
uint8_t calculateChecksum(uint8_t* data, size_t length) {
    uint8_t checksum = 0;
    for (size_t i = 0; i < length - 1; i++) { // Son byte checksum olduğu için dahil etme
        checksum ^= data[i];
    }
    return checksum;
}

// Paket doğrulama
bool validatePacket(uint8_t* data, size_t length, uint8_t expected_type) {
    if (length < 1) return false;
    if (data[0] != expected_type) return false;
    
    // Bazı paketlerde checksum varsa kontrol et
    if (expected_type == PACKET_TYPE_L4_DATA || expected_type == PACKET_TYPE_L5_DATA) {
        uint8_t calculated_checksum = calculateChecksum(data, length);
        return calculated_checksum == data[length - 1];
    }
    
    return true;
}

#endif
