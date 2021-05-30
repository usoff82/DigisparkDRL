#include <Arduino.h>
#include <DigiUSB.h>
//#include <avr/eeprom.h>

#define DEBUG 1

#define LRLIGTH_PIN 0
#define SRLIGTH_PIN 1
#define SRLSTATE_PIN 3

#define SETPOINT 500.0
#define HYSTER 100
#define LRL2DRL 0.2

// Простой фильтр Калмана
float simpleKalman(float newVal) {
  float _err_measure = 12.8;  // примерный шум измерений
  float _q = 0.005;   // скорость изменения значений 0.001-1, варьировать самому
  float _kalman_gain, _current_estimate;
  static float _err_estimate = _err_measure;
  static float _last_estimate;
  _kalman_gain = (float)_err_estimate / (_err_estimate + _err_measure);
  _current_estimate = _last_estimate + (float)_kalman_gain * (newVal - _last_estimate);
  _err_estimate =  (1.0 - _kalman_gain) * _err_estimate + fabs(_last_estimate - _current_estimate) * _q;
  _last_estimate = _current_estimate;
  return _current_estimate;
}

// Включение/выключение ближнего света
bool regulSR(float light) {
  static bool relayState = false;
  if (light < (SETPOINT - HYSTER)) relayState = true;
  else if (light > (SETPOINT + HYSTER)) relayState = false;
  
  digitalWrite(SRLIGTH_PIN, relayState);
  return relayState;
}

// Управление яркостью дальнего света
float regulLR(bool SRlightState, float light) {
  float DRligth = /*(light) **/ 1024 * LRL2DRL;
  if (SRlightState == true ) { 
    DRligth = 0;
  }
  analogWrite(LRLIGTH_PIN, DRligth);
  return DRligth;
}

void setup() {                
  // initialize the digital pin as an output.
  DigiUSB.begin();
  pinMode(LRLIGTH_PIN, OUTPUT); 
  pinMode(SRLIGTH_PIN, OUTPUT); //LED on Model A P1
  pinMode(ADC1D, INPUT); // Light sensor
//  pinMode(SRLSTATE_PIN, INPUT); // SR light status
}

void loop() {
  float raw;
  float smooth;
  bool SRlightState = digitalRead(SRLSTATE_PIN);
  float DLRligt;

  raw = analogRead(1); // ADC1D ==> 1
  smooth = simpleKalman(raw);
  SRlightState = regulSR(smooth);
  DLRligt = regulLR(SRlightState, smooth);

#if (DEBUG == 1)
  DigiUSB.print(raw);
  DigiUSB.print(",");
  DigiUSB.print(smooth);
  DigiUSB.print(",");
  if (SRlightState == true ) {
    DigiUSB.print(SETPOINT);
  } else {
    DigiUSB.print(0);
  }
  DigiUSB.print(",");
  DigiUSB.println(DLRligt);
#endif

  //DigiUSB.refresh();
  DigiUSB.delay(1000);
/*
  analogWrite(0,255); //Turn the pin on full (100%)
    DigiUSB.delay(1000);
    analogWrite(0,128); //Turn the pin on half (50%)
    DigiUSB.delay(1000);
    analogWrite(0,0);   //Turn the pin off (0%)
    DigiUSB.delay(1000);
    */
}

