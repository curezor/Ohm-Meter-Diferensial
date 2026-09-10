#include <Adafruit_ADS1X15.h>
#include <Preferences.h>
#include <Wire.h>
//opsional, jika pengin lebih stabil wifinya off biar noise pembacaan berkurang
#include "esp_wifi.h"
#include "esp_bt.h"

//Nilai itungan
Adafruit_ADS1115 ads;
Preferences prefs;

float R_REF;
int SAMPLES = 500;

float currentVoltsPerBit = 0.0001875F; 
adsGain_t currentGain = GAIN_TWOTHIRDS;

//pembacaan adc mentah
struct AdcReadings {
  double diff01;
  double singleA2;
};

//fitur gain adapt dengan filter moving
void updateAutoGain(double rawValue) {
  double absRaw = abs(rawValue);
  adsGain_t nextGain = GAIN_TWOTHIRDS;
  float vBit = 0.0001875F;
  if (absRaw < 12000) { 
    nextGain = GAIN_SIXTEEN; //0.256v
    vBit = 0.0000078125F;
  } else if (absRaw < 24000) {
    nextGain = GAIN_EIGHT;   //0.512v
    vBit = 0.000015625F;
  } else if (absRaw < 26000) {
    nextGain = GAIN_FOUR;    //1.024V
    vBit = 0.00003125F;
  } else if (absRaw < 27000) {
    nextGain = GAIN_TWO;     //2.048V
    vBit = 0.0000625F;
  } else if (absRaw < 28000) {
    nextGain = GAIN_ONE;     //4.096V
    vBit = 0.000125F;
  } else {
    nextGain = GAIN_TWOTHIRDS; //6.144V
    vBit = 0.0001875F;
  }

  if (nextGain != currentGain) {
    currentGain = nextGain;
    currentVoltsPerBit = vBit;
    ads.setGain(currentGain);
    delay(10);
  }
}

//hasil rata2 
AdcReadings getFilteredReadings() {
  double sumDiff = 0;
  double sumA2 = 0;
  
  double minDiff = 999999, maxDiff = -999999;
  double minA2 = 999999, maxA2 = -999999;

  for (int i = 0; i < SAMPLES; i++) {
    //ukur diferensial
    double rawDiff = ads.readADC_Differential_0_1();
    //single ended
    double rawA2 = ads.readADC_SingleEnded(2);

    sumDiff += rawDiff;
    sumA2 += rawA2;

    if (rawDiff < minDiff) minDiff = rawDiff;
    if (rawDiff > maxDiff) maxDiff = rawDiff;
    if (rawA2 < minA2) minA2 = rawA2;
    if (rawA2 > maxA2) maxA2 = rawA2;
  }
  sumDiff -= (minDiff + maxDiff);
  sumA2 -= (minA2 + maxA2);

  AdcReadings data;
  data.diff01 = sumDiff / (SAMPLES - 2);
  data.singleA2 = sumA2 / (SAMPLES - 2);
  
  return data;
}
void setup() {
  Serial.begin(115200);
  delay(500);
  esp_wifi_stop();
  esp_bt_controller_disable();
  esp_bt_controller_deinit();

  prefs.begin("OhmMeter", false);
  R_REF = prefs.getFloat("last_r_ref", 20000.0); 

  if (!ads.begin()) {
    Serial.println("ADS1115 tidak ditemukan!");
    while (1);
  }
  ads.setGain(GAIN_TWOTHIRDS);
  ads.setDataRate(RATE_ADS1115_860SPS);

  Serial.println();
  Serial.println("=== OHM METER DIFERENSIAL (AUTO-GAIN) ===");
  Serial.println("Wiring: R_REF di Atas (VCC), Rx di Bawah (GND)");
  Serial.print("R_REF: ");
  Serial.println(R_REF);
  Serial.print("Default Sampling: ");
  Serial.println(SAMPLES);
  Serial.println();
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input == "0") {
      R_REF = 20000.0;
      prefs.putFloat("last_r_ref", R_REF);
      Serial.println("R_REF = 20k");
    } 
    else if (input == "1") {
      R_REF = prefs.getFloat("cal_rref", 20000.0);
      Serial.print("R_REF (cal)= ");
      Serial.println(R_REF);
    } 
    else if (input.startsWith("cal")) {
      float newVal = input.substring(3).toFloat();
      if (newVal > 1000 && newVal < 1000000) {
        prefs.putFloat("cal_rref", newVal);
        Serial.print("Kalibrasi R_REF disimpan: ");
        Serial.println(newVal);
      } else {
        Serial.println("Nilai kalibrasi tidak valid!");
      }
    } 
    else if (input.startsWith("s") || input.startsWith("S")) {
      int newSamples = input.substring(1).toInt();
      if (newSamples >= 5 && newSamples <= 3000) {
        SAMPLES = newSamples;
        Serial.print("Jumlah sampling diubah menjadi: ");
        Serial.println(SAMPLES);
      } else {
        Serial.println("Nilai sampling tidak valid!");
      }
    }
  }

  //1
  AdcReadings rawData = getFilteredReadings();

  //2
  double maxRawSeen = max(abs(rawData.diff01), abs(rawData.singleA2));
  updateAutoGain(maxRawSeen);

  //3
  double vComponentAtas = rawData.diff01 * currentVoltsPerBit;
  double vComponentBawah = rawData.singleA2 * currentVoltsPerBit;
  
  //4
  double vSourceTerukur = vComponentAtas + vComponentBawah; 

  if (vComponentAtas < 0.0005) { //5
    Serial.println("Noise / open circuit");
    delay(500);
    return; 
  }

  //hasil ukur
  double Rx = R_REF * (vComponentBawah / vComponentAtas);
  
  Serial.print("VCC ESP32 (Ukur): ");
  Serial.print(vSourceTerukur, 4);
  Serial.print(" V | Gain Saat Ini: ");
  Serial.print(currentVoltsPerBit * 32768.0, 3);
  Serial.print("V | Hasil Hitung Rx: ");
  Serial.print(Rx, 2);
  Serial.println(" Ohm");

  delay(800);
}
