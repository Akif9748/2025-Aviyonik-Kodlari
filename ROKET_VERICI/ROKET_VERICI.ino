// 9748,
// MAX3232 dahil edilmemiştir. Bu kod, uçuş için uygun bir koddur.

//#define TESTMODU 1

//#define ALICI_ADRES 97
//#define ALICI_KANAL 79
//#define NET_ID 46


#define LoraTX D9
#define GpsRX D13

#define BUZZER D4
#define PATLAMAK D12

#define MINIMUM_IRTIFA 50

#define MAIN_INTERVAL 100     // Ana Döngü: 10HZ
#define PATLAMA_SURESI 10000  // SGU EYLEM SÜRESİ
#define LORA_INTERVAL 900     // must= < GPS 1 hz kullanalım
#define UKBTEST_INTERVAL 100  // 10HZ
#define GPS_INTERVAL 900      // 1HZ

#define SEALEVELPRESSURE_HPA 1014  
#define GRAVITY 9.80665



#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <Adafruit_BMP280.h>
#include <lsm6dsm.h>
#include <deneyap.h>



//////////////////////////////////////////////////////
///////////////////////KALMAN/////////////////////////
//////////////////////////////////////////////////////

class Kalman {
private:
  const float Q;  // Süreç gürültüsü (Düşük. ESP32 veri kaybetmez. Çok az kaybedebilir Floatları sadece.)
  const float R;  // Ölçüm gürültüsü (Değişken)
  float P;        // hata kovaryansı (Q ama sadece başlangıç. Q'nun başlangıçta yumuşak olmasını sağlar)

public:
  float X;  // tahmin edilen değer (public)

  Kalman(float q, float r, float p, float initial)
    : Q(q), R(r), P(p), X(initial) {}

  float update(float olcum) {
    P += Q;
    float K = P / (P + R);
    X += K * (olcum - X);
    P *= (1 - K);
    return X;
  }
  void reset(float initialX = 0.0, float initialP = 1.0) {
    X = initialX;
    P = initialP;
  }
};


Kalman Basinc_Kalman(0.001, 1 * 1 /* tolerans ** 2 */, 1.0 /* arastir */, 0.0);
Kalman XIvme_Kalman(0.001, 0.1, 1.0, 0.0);
Kalman YIvme_Kalman(0.001, 0.1, 1.0, 0.0);
Kalman ZIvme_Kalman(0.001, 0.1, 1.0, 0.0);
Kalman Irtifa_Kalman(0.001, 1 * 1, 1.0, 0.0);
Kalman Aci_Kalman(0.001, 0.1, 1.0, 0.0);




//////////////////////////////////////////////////////
///////////////////////KALMAN/////////////////////////
//////////////////////////////////////////////////////






// Class Tanımlaması
Adafruit_BMP280 BMP;
LSM6DSM IMU;
TinyGPSPlus GPS;

HardwareSerial ComboSerial(2);
// Class Tanımlaması







// GLOBAL Değişken Tanımlamaları

// HYI/SUT RAW BIT SETI
bool ana_parasut = false;
bool irtifa_kosul = false;
bool aci_kosul = false;
bool min_irtifa_kosul = false;

// BMP
double KALKIS_BASINC;  // Başlangıç basıncı

// RAW EK-6 DATA
struct {
  float RAMPA_IRTIFA;
  float BASINC;
  float ACI;
  float AX, AY, AZ;
} RawSensorData;

// Millis seti
unsigned long patlama_millis, irtifa_millis, gps_millis;  //, MAX_3232_millis;

enum MOD { NORMAL_MOD,
           SUT_MOD,
           SIT_MOD };

MOD roket_mod = NORMAL_MOD;


TaskHandle_t Haberlesme_Handle;

// GLOBAL Değişken Tanımlamaları









void setup() {
  // BMP başlatma
  BMP.begin(0x76);
  KALKIS_BASINC = BMP.readPressure() / 100;

  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, HIGH);


  // PATLAMA RÖLE TERS ÇALIŞIYOR:
  digitalWrite(PATLAMAK, HIGH);
  pinMode(PATLAMAK, OUTPUT);


  // IMU başlatma
  IMU.begin();


  //LoRa ve GPS birlikte başlatma
  ComboSerial.begin(9600, SERIAL_8N1, GpsRX, LoraTX);

  delay(2500);  // Daha iyi basınc kaydı
  KALKIS_BASINC = BMP.readPressure() / 100;

#ifdef TESTMODU
  Serial.begin(115200);
  Serial.println();
  Serial.print("PUSULA ROKET TAKIMI TEST KIPI from Akif9748 ");
  Serial.print("KALKIS_BASINC:  ");
  Serial.println(KALKIS_BASINC);
  Serial.println();
#endif


  ParalelIslemYoneticisi();
  digitalWrite(BUZZER, LOW);
}


void ParalelIslemYoneticisi() {
  switch (roket_mod) {
    case NORMAL_MOD:
      xTaskCreatePinnedToCore(
        Haberlesme,
        "HABERLESME",
        10000,
        NULL,
        1,
        &Haberlesme_Handle,
        0);
      break;
  }
}





void loop() {
  if (roket_mod != SUT_MOD)
    SensorVeriOku();

  Basinc_Kalman.update(RawSensorData.BASINC);
  XIvme_Kalman.update(RawSensorData.AX);
  YIvme_Kalman.update(RawSensorData.AY);
  ZIvme_Kalman.update(RawSensorData.AZ);
  Irtifa_Kalman.update(RawSensorData.RAMPA_IRTIFA);
  Aci_Kalman.update(RawSensorData.ACI);

  KURTARMA(Aci_Kalman.X, Irtifa_Kalman.X);

  vTaskDelay(MAIN_INTERVAL / portTICK_PERIOD_MS);
};



/*!
 * @brief Calculates the approximate altitude using barometric pressure and the
 * supplied sea level hPa as a reference.
 * @param seaLevelhPa
 *        The current hPa at sea level.
 * @return The approximate altitude above sea level in meters.
 */
float readAltitude(float pressure, float seaLevelhPa) {
  return 44330 * (1.0 - pow(pressure / seaLevelhPa, 0.1903));
}



// RAW Veri Okuma. Kart pozisyonuna göre düzeltme yapar.
void SensorVeriOku() {
  RawSensorData.BASINC = BMP.readPressure() / 100;
  RawSensorData.RAMPA_IRTIFA = readAltitude(RawSensorData.BASINC, KALKIS_BASINC);

  // X ve Z'nin yerleri değiştirilmiştir
  RawSensorData.AX = IMU.readFloatAccelZ();
  RawSensorData.AY = IMU.readFloatAccelY();
  RawSensorData.AZ = IMU.readFloatAccelX();  // kart tersse ters yap bunu mesela
  // X ve Z'nin yerleri değiştirilmiştir
  // HYI EKINDE ANLATILIYOR


  RawSensorData.ACI = fabs(atan2(sqrt(RawSensorData.AX * RawSensorData.AX + RawSensorData.AY * RawSensorData.AY), -RawSensorData.AZ) * 180.0 / PI);
#ifdef TESTMODU

  Serial.println("=== SENSOR VERISI ===");
  Serial.print("BASINC (hPa): ");
  Serial.println(RawSensorData.BASINC, 2);
  Serial.print("RAMPA_IRTIFA (m): ");
  Serial.println(RawSensorData.RAMPA_IRTIFA, 2);

  Serial.print("Ivme X (AX): ");
  Serial.println(RawSensorData.AX, 4);
  Serial.print("Ivme Y (AY): ");
  Serial.println(RawSensorData.AY, 4);
  Serial.print("Ivme Z (AZ): ");
  Serial.println(RawSensorData.AZ, 4);

  Serial.print("ACI (°): ");
  Serial.println(RawSensorData.ACI, 2);
  Serial.println("=====================");
#endif
}


float eski_irtifa;


// TODO: algoritmadaki açı değişkeni net değil.
void KURTARMA(float aci, float irtifa) {
  unsigned long new_millis = millis();

  min_irtifa_kosul = (irtifa > MINIMUM_IRTIFA);

  irtifa_kosul = (irtifa < eski_irtifa);
  if (new_millis - irtifa_millis >= 100) {
    irtifa_millis = new_millis;
    eski_irtifa = irtifa;
  }

  aci_kosul = aci > 45;

#ifdef TESTMODU
  Serial.println(aci);
#endif

  bool kosul = (aci_kosul && irtifa_kosul && min_irtifa_kosul);

  if (kosul && !ana_parasut) {
    ana_parasut = true;
    digitalWrite(PATLAMAK, LOW);
    patlama_millis = new_millis;
  }

  if (!digitalRead(PATLAMAK))
    if (new_millis - patlama_millis >= PATLAMA_SURESI) {
      patlama_millis = new_millis;
      digitalWrite(PATLAMAK, HIGH);
    }
}






typedef struct __attribute__((packed)) {
  uint8_t header;  // 1 byte

  float enlem;          // 4 byte
  float boylam;         // 4 byte
  float gps_irtifa;     // 4 byte
  float basinc;         // 4 byte
  float basinc_irtifa;  // 4 byte
  float ivmex;          // 4 byte
  float ivmey;          // 4 byte
  float ivmez;          // 4 byte
  float gyrox;          // 4 byte
  float gyroy;          // 4 byte
  float gyroz;          // 4 byte
  float aci;            // 4 byte

  uint8_t parasut_durum;  // 1 byte
  uint8_t checksum;       // 1 byte
  uint8_t footer;         // 1 byte

} LoraPaket;


// GPS
float enlem, boylam, gps_irtifa;


void Haberlesme(void* pvParameters) {

#ifdef TESTMODU
  Serial.print("Haberlesme ONCORE:    ");
  Serial.println(xPortGetCoreID());
#endif

  while (true) {
    unsigned long new_millis = millis();

    if (new_millis - gps_millis >= GPS_INTERVAL) {
      gps_millis = new_millis;

      // GPS
      while (ComboSerial.available())
        GPS.encode(ComboSerial.read());

      if (GPS.location.isValid()) {
        enlem = GPS.location.lat();
        boylam = GPS.location.lng();
      }

      if (GPS.altitude.isValid()) {
        gps_irtifa = GPS.altitude.meters();
      }
    }

    float GyroX = IMU.readFloatGyroZ();  // DEGISTIRILDI
    float GyroY = IMU.readFloatGyroY();  // HYI
    float GyroZ = IMU.readFloatGyroX();  // DEGISTIRILDI



    LoraPaket paket;

    // Header
    paket.header = 0xAB;

    // GPS
    paket.enlem = enlem;
    paket.boylam = boylam;
    paket.gps_irtifa = gps_irtifa;

    // BMP
    paket.basinc = Basinc_Kalman.X;
    paket.basinc_irtifa = Irtifa_Kalman.X;

    // IMU //

    // ACC
    paket.ivmex = XIvme_Kalman.X;
    paket.ivmey = YIvme_Kalman.X;
    paket.ivmez = ZIvme_Kalman.X;

    // GYRO
    paket.gyrox = GyroX;
    paket.gyroy = GyroY;
    paket.gyroz = GyroZ;

    paket.aci = Aci_Kalman.X;

    paket.parasut_durum = ana_parasut ? 2 : 1;

    uint8_t* gonderilecekPaket = (uint8_t*)&paket;

    uint8_t sum = 0;
    for (size_t i = 0; i < sizeof(LoraPaket) - 2; ++i)
      sum += gonderilecekPaket[i];

    paket.checksum = sum % 256;

    paket.footer = 0x56;

    ComboSerial.write(gonderilecekPaket, sizeof(paket));
    vTaskDelay(LORA_INTERVAL / portTICK_PERIOD_MS);
  }
}
