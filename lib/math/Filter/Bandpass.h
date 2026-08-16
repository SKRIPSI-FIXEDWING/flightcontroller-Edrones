//#pragma once
#ifndef Bandpass_h
#define Bandpass_h



class Bandpass
{
  public:

    Bandpass();

    int sensorValue;
    int sensorValue2;
    int sensorValue3;

    int highpass;
    int highpass2;
    int highpass3;
    int bandpass;
    int bandpass2;
    int bandpass3;

    float EMA_a_low;      //initialization of EMA alpha
    float EMA_a_high;
    float EMA_a_low2;
    float EMA_a_high2;
    float EMA_a_low3;
    float EMA_a_high3;

    int EMA_S_low;        //initialization of EMA S
    int EMA_S_high;
    int EMA_S_low2;
    int EMA_S_high2;
    int EMA_S_low3;
    int EMA_S_high3;

    void hitung();

    int hitungX(int sensorValueX);
    int hitungY(int sensorValueY);
    int hitungZ(int sensorValueZ);
    
  private:

};



#endif