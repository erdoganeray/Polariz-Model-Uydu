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
#define TAKIM_NO 626541

// Uydu status kodları
#define STATUS_UCUSA_HAZIR 0      // Uçuşa Hazır (Roket Ateşlenmeden Önce)
#define STATUS_YUKSELME 1         // Yükselme
#define STATUS_MODEL_INIS 2       // Model Uydu İniş
#define STATUS_AYRILMA 3          // Ayrılma
#define STATUS_GOREV_INIS 4       // Görev Yükü İniş
#define STATUS_KURTARMA 5         // Kurtarma (Görev Yükü'nün Yere Teması)

// Hata kodu bit pozisyonları (6 bit toplam)
#define ERROR_BIT_MODEL_INIS_HIZI      0  // Bit 0: Model uydu iniş hızı (12-14 m/s)
#define ERROR_BIT_GOREV_INIS_HIZI      1  // Bit 1: Görev yükü iniş hızı (6-8 m/s)
#define ERROR_BIT_TASIYICI_BASINC      2  // Bit 2: Taşıyıcı basınç verisi
#define ERROR_BIT_GOREV_KONUM          3  // Bit 3: Görev yükü konum verisi
#define ERROR_BIT_AYRILMA              4  // Bit 4: Ayrılma durumu
#define ERROR_BIT_SPEKTRAL_FILTRE      5  // Bit 5: Multi-spektral mekanik filtreleme sistemi

// Telemetry paketi yapısı (toplam ~70 byte)
struct __attribute__((packed)) TelemetryPacket {
    uint8_t packet_type;           // 1 byte - paket tipi
    uint16_t paket_sayisi;         // 2 byte - paket sayısı
    uint8_t uydu_statusu;          // 1 byte - uydu durumu
    uint8_t hata_kodu;             // 1 byte - 6 bitlik hata kodu (bit0-5: 6 farklı hata durumu)
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
    int16_t iot_s1_data;           // 2 byte - IoT sensör 1 verisi
    int16_t iot_s2_data;           // 2 byte - IoT sensör 2 verisi
    uint32_t takim_no;             // 4 byte - sabit 656241
};

// Button Control paketi yapısı (~8 byte)
struct __attribute__((packed)) ButtonControlPacket {
    uint8_t packet_type;           // 1 byte - paket tipi
    uint16_t paket_sayisi;         // 2 byte - paket sayısı
    uint8_t manuel_ayrilma;        // 1 byte - 0: default, 1: ayrılma, 2: birleşme
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

// L4/L5 Data paketi yapısı (~5 byte)
struct __attribute__((packed)) LoraDataPacket {
    uint8_t packet_type;           // 1 byte - paket tipi (L4_DATA veya L5_DATA)
    uint16_t paket_sayisi;         // 2 byte - paket sayısı
    int16_t temperature;           // 2 byte - temperature verisi
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

// Hata kodu yönetimi fonksiyonları
void setErrorBit(uint8_t* hata_kodu, uint8_t bit_position) {
    *hata_kodu |= (1 << bit_position);
}

void clearErrorBit(uint8_t* hata_kodu, uint8_t bit_position) {
    *hata_kodu &= ~(1 << bit_position);
}

bool getErrorBit(uint8_t hata_kodu, uint8_t bit_position) {
    return (hata_kodu & (1 << bit_position)) != 0;
}

void clearAllErrors(uint8_t* hata_kodu) {
    *hata_kodu = 0;
}

// Hata durumu kontrol fonksiyonları
void updateModelInisHiziError(uint8_t* hata_kodu, float inis_hizi) {
    if (inis_hizi < 12.0 || inis_hizi > 14.0) {
        setErrorBit(hata_kodu, ERROR_BIT_MODEL_INIS_HIZI);
    } else {
        clearErrorBit(hata_kodu, ERROR_BIT_MODEL_INIS_HIZI);
    }
}

void updateGorevInisHiziError(uint8_t* hata_kodu, float gorev_inis_hizi) {
    if (gorev_inis_hizi < 6.0 || gorev_inis_hizi > 8.0) {
        setErrorBit(hata_kodu, ERROR_BIT_GOREV_INIS_HIZI);
    } else {
        clearErrorBit(hata_kodu, ERROR_BIT_GOREV_INIS_HIZI);
    }
}

void updateTasiyiciBasincError(uint8_t* hata_kodu, bool basinc_veri_var) {
    if (!basinc_veri_var) {
        setErrorBit(hata_kodu, ERROR_BIT_TASIYICI_BASINC);
    } else {
        clearErrorBit(hata_kodu, ERROR_BIT_TASIYICI_BASINC);
    }
}

void updateGorevKonumError(uint8_t* hata_kodu, bool konum_veri_var) {
    if (!konum_veri_var) {
        setErrorBit(hata_kodu, ERROR_BIT_GOREV_KONUM);
    } else {
        clearErrorBit(hata_kodu, ERROR_BIT_GOREV_KONUM);
    }
}

void updateAyrilmaError(uint8_t* hata_kodu, bool ayrilma_gerceklesti) {
    if (!ayrilma_gerceklesti) {
        setErrorBit(hata_kodu, ERROR_BIT_AYRILMA);
    } else {
        clearErrorBit(hata_kodu, ERROR_BIT_AYRILMA);
    }
}

void updateSpektralFiltreError(uint8_t* hata_kodu, bool filtre_calisiyor) {
    if (!filtre_calisiyor) {
        setErrorBit(hata_kodu, ERROR_BIT_SPEKTRAL_FILTRE);
    } else {
        clearErrorBit(hata_kodu, ERROR_BIT_SPEKTRAL_FILTRE);
    }
}

// Uydu status belirleme fonksiyonu (algoritma tabanlı)
uint8_t determineUyduStatusAlgorithm(float yukseklik1, float inis_hizi, uint8_t current_uydu_statusu, 
                                    bool start_button, bool ayrilma_status, bool manual_ayrilma,
                                    float tolerans0, float tolerans1, float tolerans2, float tolerans3, 
                                    float tolerans4, float tolerans5, float ayrilma_toleransi) {
    
    uint8_t new_status = current_uydu_statusu;
    
    // Status 3: Ayrılma - İlk öncelik
    if (((inis_hizi > tolerans3) && (current_uydu_statusu > 1) && (ayrilma_status == 0) && (yukseklik1 >= (400 + ayrilma_toleransi))) || (manual_ayrilma == 1)) {
        new_status = STATUS_AYRILMA;  // 3
    }
    // Status 0: Uçuşa Hazır
    else if ((abs(inis_hizi) < tolerans0) && (current_uydu_statusu <= 0) && (start_button == 1) && (yukseklik1 < 5)) {
        new_status = STATUS_UCUSA_HAZIR;  // 0
    }
    // Status 1: Yükselme
    else if ((inis_hizi < (-1 * tolerans1)) && (current_uydu_statusu >= 0 && current_uydu_statusu < 2) && (yukseklik1 >= 5)) {
        new_status = STATUS_YUKSELME;  // 1
    }
    // Status 2: Model Uydu İniş
    else if ((inis_hizi > tolerans2) && (current_uydu_statusu > 0) && (ayrilma_status == 0)) {
        new_status = STATUS_MODEL_INIS;  // 2
    }
    // Status 4: Görev Yükü İniş
    else if ((inis_hizi > tolerans4) && (current_uydu_statusu > 2) && (ayrilma_status == 1)) {
        new_status = STATUS_GOREV_INIS;  // 4
    }
    // Status 5: Kurtarma (sadece önceki status 4 ise)
    else if ((abs(inis_hizi) < tolerans5) && (current_uydu_statusu >= 3)) {
        if (current_uydu_statusu == STATUS_GOREV_INIS) {
            new_status = STATUS_KURTARMA;  // 5
        } else {
            new_status = 255;  // Default (-1)
        }
    }
    
    return new_status;
}

// Eski fonksiyon compatibility için korundu
uint8_t determineUyduStatus(float current_altitude, float previous_altitude, 
                           bool bmp_status, bool mpu_status, bool rtc_status, 
                           bool gps_status, bool sd_status, bool adc_status, bool lora_status,
                           uint8_t previous_status = 255) {
    
    // Eğer bütün sensörler çalışır durumdaysa uçuşa hazır (0)
    if (bmp_status && mpu_status && rtc_status && gps_status && sd_status && adc_status && lora_status) {
        // Yükseklik kontrolü yapılmadıysa uçuşa hazır
        if (current_altitude <= 10.0) { // Yer seviyesinde
            return STATUS_UCUSA_HAZIR;
        }
    }
    
    // Yükseklik bazlı durum belirleme
    if (current_altitude <= 0.1) {
        // STATUS_KURTARMA (5) için şart: Bir önceki status STATUS_GOREV_INIS (4) olmalı
        if (previous_status == STATUS_GOREV_INIS) {
            return STATUS_KURTARMA; // 5: Kurtarma (Görev Yükü'nün Yere Teması)
        } else {
            // Eğer önceki status 4 değilse, default status -1 döndür
            return 255; // -1 (uint8_t olarak 255)
        }
    }
    else if (current_altitude < 400.0) {
        return STATUS_GOREV_INIS; // 4: Görev Yükü İniş
    }
    else if (current_altitude >= 400.0 && current_altitude <= 450.0) {
        return STATUS_AYRILMA; // 3: Ayrılma
    }
    else {
        // 450m üzerinde - yükseklik değişimine bak
        if (current_altitude > previous_altitude + 1.0) { // Yükseklik artıyorsa
            return STATUS_YUKSELME; // 1: Yükselme
        }
        else if (current_altitude < previous_altitude - 1.0) { // Yükseklik azalıyorsa
            return STATUS_MODEL_INIS; // 2: Model Uydu İniş
        }
        else {
            // Belirsizse default status -1 döndür
            return 255; // -1 (uint8_t olarak 255)
        }
    }
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
    if (length < 1) {
        return false;
    }
    if (data[0] != expected_type) {
        return false;
    }
    
    // Bazı paketlerde checksum varsa kontrol et
    if (expected_type == PACKET_TYPE_L4_DATA || expected_type == PACKET_TYPE_L5_DATA) {
        if (length < sizeof(LoraDataPacket)) {
            return false;
        }
        
        uint8_t calculated_checksum = calculateChecksum(data, length);
        uint8_t received_checksum = data[length - 1];
        
        return calculated_checksum == received_checksum;
    }
    
    return true;
}

#endif
