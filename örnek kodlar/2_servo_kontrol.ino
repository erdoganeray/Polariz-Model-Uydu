#include <Servo.h>

Servo servo1;
Servo servo2;

int currentAngle1 = 90; // Servo1 başlangıç açısı
int currentAngle2 = 90; // Servo2 başlangıç açısı

void setup() {
  Serial.begin(9600);

  servo1.attach(9);   // Servo1 sinyal pini
  servo2.attach(10);  // Servo2 sinyal pini

  servo1.write(currentAngle1);
  servo2.write(currentAngle2);

  Serial.println("Iki servo kontrolu basladi.");
  Serial.println("Ornek giris: 100 50 (Servo1=100°, Servo2=50°)");
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n'); // Satırı oku
    input.trim(); // Baştaki/sondaki boşlukları temizle

    // Boş veri kontrolü
    if (input.length() == 0) return;

    // Boşluğa göre ayır
    int spaceIndex = input.indexOf(' ');
    if (spaceIndex == -1) {
      Serial.println("Hata: Iki sayi girin (ornegin: 100 50)");
      return;
    }

    String angle1Str = input.substring(0, spaceIndex);
    String angle2Str = input.substring(spaceIndex + 1);

    int angle1 = angle1Str.toInt();
    int angle2 = angle2Str.toInt();

    // Geçerli aralık kontrolü
    if (angle1 >= 0 && angle1 <= 180 && angle2 >= 0 && angle2 <= 180) {
      currentAngle1 = angle1;
      currentAngle2 = angle2;
      Serial.print("Servo1 acisi: ");
      Serial.print(currentAngle1);
      Serial.print(" | Servo2 acisi: ");
      Serial.println(currentAngle2);
    } else {
      Serial.println("Hata: Aci 0-180 arasinda olmali!");
    }
  }

  // Servoları sabit tut
  servo1.write(currentAngle1);
  servo2.write(currentAngle2);
}
