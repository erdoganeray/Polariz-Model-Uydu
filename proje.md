# 0.9
- 5li şekilde çalışan kodun temeli atıldı.
- 4lü loralarla iletişim sağlandı.
- Son loranın çalışmama sebebi muhtemelen donanımsal.

# 1.0
- 4lü lora için süre optimizasyonu üzerine çalışılacak.
- Alınan gönderilen veriler string den binary e geçildi.

# 1.1
- Zaman optimizasyonu yapıldı. Loopta 3ten ve 4den veri alınabiliyor. 2den alınan veri çoğunlukla yakalanamıyor. Muhtemelen bir kere gönderdiğinde Lora 1in müsait olmamasından kaynaklı olarak veriyi okuyamaması.

# 1.2
- Zaman optimizasyomu da oldu.
- Sırasıyla 0.8 ve 1.2 saniyede bir veri gönderimi yapabiliyor yükteki lora
- Yükteki lora 2 3 ve 4teki veriyi her loopta yaklayamıyor. Şimdilik kabul edilebilir düzeyde.

# 1.3
- Şimdi işleyişi biraz değiştireceğiz. Lora 1 bir kere veri yolladıktan sonra 1 saniyeyi tamamlayana kadar veri dinleme yapacak. Lora 2, sürekli veri dinleme yapacak serial monitore "SEND" yazıldıktan sonra veri yollayacak.
- Şu anda 1 saniye işlem sağlanıyor, her loopta 3 mutlaka yakalanıyor, 4 neredeyse her loopta yakalanıyor, Lora 2den sürekli veri yerine istenildiğinde veri gidiyor, bu veri mutlaka Lora 1e iletiliyor, bu veri zamanı Lora 2nin veri alması işi biraz sekteye uğruyor.

# 1.4
- Python kodu entegre edilecek.


# <copilot bu başlığı ignorelasın>
* Buffer temizliği ile ilgili herhangi bir kod eklenmedi, mutlaka kontrol edilsin
* Alternatif senaryo: Lora 1 bir kere veri gönderecek, sonra 1 saniyeyi doldurana kadar veri dinleyecek. Bufferda veri toplama ve işleme
* LBT özelliği eklenebilir.
* ACK bekleme süresi 50ms gibi küçük değerler öneriliyor, biz burada çok vakit kaybediyor olabiliriz.
* CPU Yükü Dengesi: Interrupt tabanlı veri alımı kullan, Main loop'ta delay(5) ile CPU rahatlatma ,Gereksiz Serial.print azalt, String işlemleri yerine binary data
* Bellek Yönetimi: Statik buffer boyutları belirle, Circular buffer ile overflow koruması, memset() ile buffer temizliği, Stack overflow riskine dikkat
* Bufferda aynı anda veri gelmesi durumunda öncelik özelliği ekleyebilirsin. Örneğin yisteki veri en önemlisi, sonra taşıyıcı gibi

# Bu Proje Nedir

5 adet ESP32 ve 5 adet Lora E22 900t22d modüllerinin birbirleriyle haberleşmesini sağlayan, bir model uydu sistemini simüle eden bir proje yapacağız.

*Bu projede şu an 5 adet .ino dosyası oluşturulacak. Henüz .py dosyasının içini doldurma. Bu dosyadaki tüm içerikler Claude adlı klasörde olacak*

# Klasör Yapısı
- lora 1
    - lora_1.ino
- lora 2
    - lora_2.ino
- lora 3
    - lora_3.ino
- lora 4
    - lora_4.ino
- lora 5
    - lora_5.ino
- lora.py                       // python koduna daha sonra geleceğiz, şimdi *BİR ŞEY YAPMA, SADECE DOSYAYI OLUŞTUR*

# Projenin İşleyişi

* Lora 1, ana kontrol merkezi olacak.
* Lora 4 ve Lora 5 birkaç saniyede bir birkaç yüz ms kadar düzenli yayın yapacak.
* Lora 2 ve Lora 3, başlangıçta ve genel olarak Lora 1i dinleyecek, Lora 1den veri aldıktan sonra Lora 1e veri gönderecek.
* <comm_loop>: Lora 1, başladıktan sonra <telemetry> verisini Lora 2ye iletecek -> Lora 1 dinleme moduna geçecek -> Lora 2, Lora 1den veriyi alacak, <button_control> verisini Lora 1e iletecek -> Lora 1, Lora 2den verisi aldıktan sonra Lora 3e <request> verisini iletecek -> Lora 1, dinleme moduna geçecek -> Lora 3, Lora 1den veriyi aldıktan sonra <pressure_container> verisini Lora 1e iletecek -> Lora 1 Lora 3ten veriyi alacak. -> Lora 1 sırasıyla Lora 4 ve Lora 5i dinleyecek -> Lora 4 ve 5i bir süre dinledikten sonra <comm_loop>un başına dönecek.
* lora_1.ino, <comm_loop> u sürekli olarak tekrarlayacak

# Verilerin İçerikleri
!! paket_sayisi dışındaki verileri sabit tut, herhangi bir simülasyon yöntemi ekleme. Amacımız tüm sistemin çalışıp çalışmadığını görmek.

## <telemetry>
!! Veri tiplerini en az yer kaplayacak şekilde, verileri ise sabit/değişmeyen uygun bir değer olarak seç

paket_sayisi        : her gönderildiğinde 1 artacak, tek değişken değer, bu projede bu değeri maksimum 1000 olarak göreceğiz
uydu_statusu        : tek haneli bir rakam
hata_kodu           : 0 ve 1lerden oluşan 5 haneli bir şey, örn: 00110, 11010
gonderme_saati      : tarih ve saat bilgisi
basinc1             : pascal cinsinden basınç
basinc2             : pascal cinsinden basınç
yukseklik1          : metre cinsinden irtifa
yukseklik2          : metre cinsinden irtifa
irtifa_farki        : metre cinsinden |yukseklik1 - yukseklik2|
inis_hizi           : m/s cinsinden hız
sicaklik            : celcius cinsinden sıcaklık
pil_gerilimi        : volt cinsinden pil gerilim
gps1_latitude       : gps latitude verisi
gps1_longitud       : gps longitude verisi
gps1_altitude       : gps altitude verisi
pitch               : derece cinsinden pitch değeri
roll                : derece cinsinden roll değeri
yaw                 : derece cinsinden yaw değeri
rhrh                : rakam/harf/rakam/harf şeklinde 4 haneli bir şey, örn: 1A4B, 5C7D
iot_s1_data         : celcius cinsinden sıcaklık
iot_s2_data         : celcius cinsinden sıcaklık
takim_no            : sabit 656241

## <button_control>
paket_sayisi        : her gönderildiğinde 1 artacak, tek değişken değer, bu projede bu değeri maksimum 1000 olarak göreceğiz
manuel_ayrilma      : True or False
rhrh                : rakam/harf/rakam/harf şeklinde 4 haneli bir şey, örn: 1A4B, 5C7D

## <request>
paket_sayisi        : her gönderildiğinde 1 artacak, tek değişken değer, bu projede bu değeri maksimum 1000 olarak göreceğiz

## <pressure_container>
paket_sayisi        : her gönderildiğinde 1 artacak, tek değişken değer, bu projede bu değeri maksimum 1000 olarak göreceğiz
basinc1             : pascal cinsinden basınç değeri

# LoRa Konfigürasyon Bilgileri

## Lora 1 (Ana Kontrol Merkezi)
```
#define LORA_CHANNEL 10
#define LORA_ADDR_H 0x00
#define LORA_ADDR_L 0x0A
```

## Lora 2 (Button Control)
```
#define LORA_CHANNEL 20
#define LORA_ADDR_H 0x00
#define LORA_ADDR_L 0x0A
```

## Lora 3 (Pressure Container)
```
#define LORA_CHANNEL 30
#define LORA_ADDR_H 0x00
#define LORA_ADDR_L 0x0A
```

## Lora 4 (Düzenli Yayın Modülü)
```
#define LORA_CHANNEL 40
#define LORA_ADDR_H 0x00
#define LORA_ADDR_L 0x0A
```

## Lora 5 (Düzenli Yayın Modülü)
```
#define LORA_CHANNEL 50
#define LORA_ADDR_H 0x00
#define LORA_ADDR_L 0x0A
```

# Gönderilen Mesaj Verileri

## Lora 1 Gönderdiği Mesajlar

### Lora 2'ye Telemetry Mesajı:
```
"TEL,1,5,01010,31/07/2025-14:30:00,101325.0,101300.0,150.5,149.8,0.7,-2.3,25.4,3.7,39.123456,32.654321,155.2,15.2,8.7,180.5,1A2B,22.5,23.1,656241"
```

### Lora 3'e Request Mesajı:
```
"REQ,1"
```

## Lora 2 Gönderdiği Mesajlar

### Lora 1'e Button Control Mesajı:
```
"BTN,1,false,5C9D"
```

## Lora 3 Gönderdiği Mesajlar

### Lora 1'e Pressure Container Mesajı:
```
"PRS,1,98765.4"
```

## Lora 4 Gönderdiği Mesajlar

### Düzenli Veri Mesajı (her 3 saniyede):
```
"L4DATA,1,25.8"
```

## Lora 5 Gönderdiği Mesajlar

### Düzenli Veri Mesajı (her 4 saniyede):
```
"L5DATA,1,28.3"
```

# Dikkat Etmen Gerekenler

* 5 Lora için farklı kanal ve adres tanımla
* #include <LoRa_E22.h> kütüphanesini kullan
* void setup()'ta printConfig() fonksiyonu dönsün, debug için
* Lora için kurulum ve loop döngülerini lora_receiver.ino ve lora_transmitter.ino daki gibi yaz. Bu çok önemli, farklı kodlar kütüphane ve modül uyumsuzluğuna sebep oluyor.
* her .ino kodunda her veri gönderildiğinde ve alındığında verilerle birlikte monitor e yazılsın. monitor ekranındaki yazıları basit tut.
* Veri dinleme aşamalarında veri gelmemesi durumunda tıkanma olmaması için timeout lar ekle. Bir süre dinlesin, veri yoksa bir sonraki aşamaya geçilsin.

## Sabit Pin Tanımlamaları

Tüm Loraların pinleri aşağıdaki şekilde tanımlanacak 

#define LORA_M0 19
#define LORA_M1 18
#define LORA_AUX 4
#define LORA_TX 16
#define LORA_RX 17

## .ino Kodlarında Asla Yapılmayacaklar

* Asla ve asla lora_receiver.ino ve lora_transmitter.ino kodları dışında farklı tanımlama yapma. Özellikle konfigürasyon için yeni kodlar ekleme. Kütüphane ve modül uyumsuzluğu oluyor.

# Python Kodu

## Widget Yerleşimleri

Main Screen (yatay)
1. Sol Panel (dikey)
    1.1 Grafikler (3x2)     // bazı telemetri verilerinin grafikleri
        1.1.(1,1) Basınç grafiği: pascal
        1.1.(1,2) Yükseklik grafiği: metre
        1.1.(2,1) İniş Hızı: m/s
        1.1.(2,2) Atmosfer Sıcaklığı: celcius
        1.1.(3,1) İrtifa Farkı: m
        1.1.(3,2) IoT Sıcaklık: celcius
    1.2 Log Ekranı          // debug için log ekranı, text alanı
2. Sağ Panel (dikey)
    2.1 Kamera ve Harita (yatay)
        2.1.1 Kamera        // kamera görüntüsü olacak
        2.1.2 Harita        // konum verisinden harita üzerinde yer gösterilecek
    2.2 Data ve 3d model (yatay)
        2.2.1 Data (dikey)
            2.2.1.1 Telekomut (dikey)
                2.2.1.1.1   // 4 adet yanyana liste seçeneği, şimdilik 1, 2 ve A seçilebilsin
                2.2.1.1.2   // RHRH: Telekomut Gönder butonu
            2.2.1.2 Hata Kodu   // 2 satır ve 6 sütunlu bir grafik. Üst satırlarda 1, 2, 3, 4, 5, 6 yazacak. Alt satırdaki hücrelerde bir şey yazmayacak, dolgu renkleri ya kırmızı ya da yeşil olacak
            2.2.1.3 Text    // Üst satır: Pil Gerilimi: 5V (%37), Alt satır: Statü: 0 (Uçuşa Hazır)
        2.2.2 3D Model      // pitch, roll, yaw bilgise göre model uydunun 3 boyutlu konumu
    2.3 Com Seçimi/Ayrılma ve Logo (yatay)
        2.3.1 Com Seçimi/Ayrılma (dikey)
            2.3.1.1         // Manuel Ayrılma butonu
            2.3.1.2         // Com Seçimi: texti
            2.3.1.3         // Liste, bu listede aktif comlar listelenecek
        2.3.2 Logo          // yis_logo.png gelecek