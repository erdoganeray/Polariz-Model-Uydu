# Senkranizasyon Yöntemleri
## Master-Slave Zaman Senkranizasyonu
// Lora 1 (Master) - Ana senkronizasyon merkezi
class TimeMaster {
private:
    unsigned long systemStartTime;
    uint16_t syncCounter = 0;
    
public:
    void broadcastSyncSignal() {
        struct SyncPacket {
            char header[4] = "SYN";
            uint16_t syncId;
            uint32_t masterTime;
            uint8_t checksum;
        } syncPkt;
        
        syncPkt.syncId = syncCounter++;
        syncPkt.masterTime = millis();
        syncPkt.checksum = calculateChecksum(&syncPkt);
        
        // Tüm kanallarda senkron sinyali yayınla
        for(int channel = 10; channel <= 50; channel += 10) {
            sendToChannel(channel, &syncPkt, sizeof(syncPkt));
        }
    }
};

// Diğer Loralar (Slaves)
class TimeSlave {
private:
    long timeOffset = 0;
    unsigned long lastSyncTime = 0;
    
public:
    void processSyncSignal(SyncPacket* sync) {
        unsigned long localTime = millis();
        timeOffset = sync->masterTime - localTime;
        lastSyncTime = localTime;
        
        Serial.print("Time sync: offset=");
        Serial.println(timeOffset);
    }
    
    unsigned long getSynchronizedTime() {
        return millis() + timeOffset;
    }
};

## Fase Tabanlo Döngü Kontrolü

// Ana döngü faz yönetimi
enum CommPhase {
    PHASE_SYNC = 0,        // 0-50ms: Senkronizasyon
    PHASE_LORA2 = 1,       // 50-300ms: Lora 2 ile iletişim  
    PHASE_LORA3 = 2,       // 300-550ms: Lora 3 ile iletişim
    PHASE_LISTEN_45 = 3,   // 550-950ms: Lora 4&5 dinleme
    PHASE_PROCESS = 4      // 950-1000ms: Veri işleme
};

class PhaseManager {
private:
    unsigned long cycleStartTime = 0;
    CommPhase currentPhase = PHASE_SYNC;
    
public:
    CommPhase getCurrentPhase() {
        unsigned long elapsed = millis() - cycleStartTime;
        
        if (elapsed >= 1000) {
            cycleStartTime = millis();
            currentPhase = PHASE_SYNC;
            return PHASE_SYNC;
        }
        
        if (elapsed < 50) return PHASE_SYNC;
        else if (elapsed < 300) return PHASE_LORA2;
        else if (elapsed < 550) return PHASE_LORA3;
        else if (elapsed < 950) return PHASE_LISTEN_45;
        else return PHASE_PROCESS;
    }
    
    unsigned long getPhaseTimeRemaining() {
        unsigned long elapsed = millis() - cycleStartTime;
        unsigned long phaseEnd[] = {50, 300, 550, 950, 1000};
        return phaseEnd[currentPhase] - elapsed;
    }
};

# Veri Tipi Optimizasyonu

// Optimize edilmiş veri yapıları
struct __attribute__((packed)) TelemetryData {
    uint8_t header = 0xAA;           // 1 byte - packet identifier
    uint16_t packet_count : 10;      // 10 bit - 0-1023 arası (1000 yeterli)
    uint8_t satellite_status : 4;    // 4 bit - 0-15 arası
    uint8_t error_code : 5;          // 5 bit - hata kodu bits
    uint32_t timestamp;              // 4 byte - Unix timestamp
    uint32_t pressure1;              // 4 byte - Pascal (float yerine int*100)
    uint32_t pressure2;              // 4 byte - Pascal
    uint16_t altitude1;              // 2 byte - metre (0-65535m yeterli)
    uint16_t altitude2;              // 2 byte - metre
    uint8_t altitude_diff;           // 1 byte - fark (0-255m)
    int16_t descent_speed_x10;       // 2 byte - m/s * 10 (hassasiyet için)
    int8_t temperature;              // 1 byte - Celsius (-128 +127)
    uint16_t battery_voltage_x100;   // 2 byte - Volt * 100
    int32_t gps_lat_x1000000;       // 4 byte - Latitude * 1000000
    int32_t gps_lon_x1000000;       // 4 byte - Longitude * 1000000
    uint16_t gps_alt;                // 2 byte - GPS altitude
    int16_t pitch_x10;               // 2 byte - Derece * 10
    int16_t roll_x10;                // 2 byte - Derece * 10  
    uint16_t yaw_x10;                // 2 byte - Derece * 10
    uint32_t rhrh;                   // 4 byte - encoded RHRH
    int8_t iot_s1_temp;              // 1 byte - IoT sensor 1
    int8_t iot_s2_temp;              // 1 byte - IoT sensor 2
    uint32_t team_id = 656241;       // 4 byte - sabit takım no
    uint8_t checksum;                // 1 byte - veri doğrulama
}; // Toplam: ~60 byte (String 200+ byte yerine)

struct __attribute__((packed)) ButtonControlData {
    uint8_t header = 0xBB;
    uint16_t packet_count : 10;
    bool manual_separation : 1;
    uint32_t rhrh;
    uint8_t checksum;
}; // Toplam: ~8 byte

struct __attribute__((packed)) RequestData {
    uint8_t header = 0xCC;
    uint16_t packet_count : 10;
    uint8_t checksum;
}; // Toplam: ~4 byte

# Sürekli Dinleme Modu Detayları
## Interrupt Tabanlı Veri Alımı
// Lora 1 - Ana kontrol merkezi sürekli dinleme
class ContinuousListener {
private:
    volatile bool dataReceived = false;
    uint8_t rxBuffer[256];
    size_t bufferIndex = 0;
    unsigned long lastDataTime = 0;
    
public:
    void setup() {
        // AUX pin interrupt'ı - veri geldiğinde tetiklenir
        pinMode(LORA_AUX, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(LORA_AUX), 
                       onLoRaDataReceived, FALLING);
    }
    
    static void IRAM_ATTR onLoRaDataReceived() {
        // Interrupt handler - minimal işlem
        dataReceived = true;
    }
    
    void processIncomingData() {
        if (dataReceived) {
            dataReceived = false;
            
            // Gelen veriyi hızlıca buffer'a al
            while (e22ttl.available()) {
                if (bufferIndex < sizeof(rxBuffer)-1) {
                    rxBuffer[bufferIndex++] = e22ttl.read();
                }
            }
            
            lastDataTime = millis();
            parseReceivedData();
        }
    }
    
    void parseReceivedData() {
        // Header'a göre veri tipini belirle
        uint8_t header = rxBuffer[0];
        
        switch(header) {
            case 0xBB: // Button Control
                processButtonControl((ButtonControlData*)rxBuffer);
                break;
            case 0xDD: // Pressure Container  
                processPressureData((PressureData*)rxBuffer);
                break;
            case 0xEE: // Lora 4 Data
                processLora4Data((Lora4Data*)rxBuffer);
                break;
            case 0xFF: // Lora 5 Data
                processLora5Data((Lora5Data*)rxBuffer);
                break;
        }
        
        // Buffer temizle
        bufferIndex = 0;
        memset(rxBuffer, 0, sizeof(rxBuffer));
    }
};

## Çoklu Kanal Dinleme Stratejisi

// Kanal hopping ile tüm kanalları dinleme
class MultiChannelListener {
private:
    uint8_t channels[5] = {10, 20, 30, 40, 50};
    uint8_t currentChannelIndex = 0;
    unsigned long lastChannelSwitch = 0;
    
public:
    void updateChannelScanning() {
        unsigned long now = millis();
        
        // Her 200ms'de bir kanal değiştir
        if (now - lastChannelSwitch >= 200) {
            switchToNextChannel();
            lastChannelSwitch = now;
        }
    }
    
    void switchToNextChannel() {
        currentChannelIndex = (currentChannelIndex + 1) % 5;
        uint8_t newChannel = channels[currentChannelIndex];
        
        // Kanal değiştirme
        ResponseStructContainer c = e22ttl.getConfiguration();
        Configuration config = *(Configuration*)c.data;
        config.CHAN = newChannel;
        e22ttl.setConfiguration(config, WRITE_CFG_PWR_DWN_LOSE);
        
        Serial.print("Switched to channel: ");
        Serial.println(newChannel);
    }
};

## Buffer Yönetimi ve Overflow Koruması

// Circular buffer implementation
class CircularBuffer {
private:
    static const size_t BUFFER_SIZE = 1024;
    uint8_t buffer[BUFFER_SIZE];
    volatile size_t head = 0;
    volatile size_t tail = 0;
    volatile size_t count = 0;
    
public:
    bool push(uint8_t data) {
        if (count >= BUFFER_SIZE) {
            // Buffer overflow - eski veriyi sil
            pop();
        }
        
        buffer[head] = data;
        head = (head + 1) % BUFFER_SIZE;
        count++;
        return true;
    }
    
    bool pop(uint8_t* data) {
        if (count == 0) return false;
        
        *data = buffer[tail];
        tail = (tail + 1) % BUFFER_SIZE;
        count--;
        return true;
    }
    
    void clear() {
        head = tail = count = 0;
        memset(buffer, 0, BUFFER_SIZE);
    }
    
    size_t available() { return count; }
    bool isFull() { return count >= BUFFER_SIZE; }
};

## Ana Loop Implementasyonu

// Lora 1 ana döngü - sürekli dinleme + fazlı iletişim
void loop() {
    static PhaseManager phaseManager;
    static ContinuousListener listener;
    static MultiChannelListener channelScanner;
    
    // Sürekli veri dinleme (interrupt tabanlı)
    listener.processIncomingData();
    
    // Fazlı iletişim kontrolü
    CommPhase currentPhase = phaseManager.getCurrentPhase();
    
    switch(currentPhase) {
        case PHASE_SYNC:
            broadcastSyncSignal();
            break;
            
        case PHASE_LORA2:
            sendTelemetryToLora2();
            // Bu faz sırasında Lora 2'den gelen veriyi bekle
            break;
            
        case PHASE_LORA3:
            sendRequestToLora3();
            // Bu faz sırasında Lora 3'ten gelen veriyi bekle
            break;
            
        case PHASE_LISTEN_45:
            // Bu faz sırasında sadece dinle
            channelScanner.updateChannelScanning();
            break;
            
        case PHASE_PROCESS:
            processCollectedData();
            printStatistics();
            break;
    }
    
    // CPU yükünü azalt
    delay(5);
}

# Buffer Temizliği ve Timeout Yönetimi

void clearLoRaBuffer() {
    while(e22ttl.available()) {
        e22ttl.read(); // Eski verileri temizle
    }
}

bool waitForDataWithTimeout(unsigned long timeout) {
    unsigned long startTime = millis();
    clearLoRaBuffer(); // Önce buffer'ı temizle
    
    while(millis() - startTime < timeout) {
        if(e22ttl.available() > 0) {
            return true;
        }
        delay(10);
    }
    
    Serial.println("Timeout - no data received");
    return false;
}