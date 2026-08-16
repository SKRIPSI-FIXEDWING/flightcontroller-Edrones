#include "Bandpass.h"

Bandpass::Bandpass()
{

  int sensorValue;
  int sensorValue2;
  int sensorValue3;
  float EMA_a_low = 0;        //initialization of EMA alpha
  float EMA_a_high = 0.2;
  float EMA_a_low2 = 0;
  float EMA_a_high2 = 0.2;
  float EMA_a_low3 = 0;
  float EMA_a_high3 = 0.2;

  int EMA_S_low = 0;        //initialization of EMA S
  int EMA_S_high = 0;
  int EMA_S_low2 = 0;
  int EMA_S_high2 = 0;
  int EMA_S_low3 = 0;
  int EMA_S_high3 = 0;

  int highpass = 0;
  int highpass2 = 0;
  int highpass3 = 0;
  int bandpass = 0;
  int bandpass2 = 0;
  int bandpass3 = 0;

  void hitung();

  int hitungX();
  int hitungY();
  int hitungZ();
}

void Bandpass::hitung() {
  EMA_S_low = (EMA_a_low * sensorValue) + ((1 - EMA_a_low) * EMA_S_low);
  EMA_S_high = ((EMA_a_high * sensorValue) + ((1 - EMA_a_high) * EMA_S_high));
  EMA_S_low2 = (EMA_a_low2 * sensorValue2) + ((1 - EMA_a_low2) * EMA_S_low2);
  EMA_S_high2 = (EMA_a_high2 * sensorValue2) + ((1 - EMA_a_high2) * EMA_S_high2);
  EMA_S_low3 = (EMA_a_low3 * sensorValue3) + ((1 - EMA_a_low3) * EMA_S_low3);
  EMA_S_high3 = (EMA_a_high3 * sensorValue3) + ((1 - EMA_a_high3) * EMA_S_high3);



  highpass = sensorValue - EMA_S_low;
  highpass2 = sensorValue2 - EMA_S_low2;
  highpass3 = sensorValue3 - EMA_S_low3;

  bandpass = EMA_S_high - EMA_S_low;
  bandpass2 = EMA_S_high2 - EMA_S_low2;
  bandpass3 = EMA_S_high3 - EMA_S_low3;


}

int Bandpass::hitungX(int sensorValueX) {
  sensorValue = sensorValueX;
  EMA_S_low = (EMA_a_low * sensorValueX) + ((1 - EMA_a_low) * EMA_S_low);
  EMA_S_high = ((EMA_a_high * sensorValueX) + ((1 - EMA_a_high) * EMA_S_high));
  return EMA_S_high - EMA_S_low;
}

int Bandpass::hitungY(int sensorValueY) {
  sensorValue2 = sensorValueY;
  EMA_S_low2 = (EMA_a_low2 * sensorValueY) + ((1 - EMA_a_low2) * EMA_S_low2);
  EMA_S_high2 = ((EMA_a_high2 * sensorValueY) + ((1 - EMA_a_high2) * EMA_S_high2));
  return EMA_S_high2 - EMA_S_low2;
}

int Bandpass::hitungZ(int sensorValueZ) {
  sensorValue3 = sensorValueZ;
  EMA_S_low3 = (EMA_a_low3 * sensorValueZ) + ((1 - EMA_a_low3) * EMA_S_low3);
  EMA_S_high3 = ((EMA_a_high3 * sensorValueZ) + ((1 - EMA_a_high3) * EMA_S_high3));
  return EMA_S_high3 - EMA_S_low3;
}


