#ifndef HMIDISPLAY_H
#define HMIDISPLAY_H
#include <Arduino.h>

class HMI_Display
{
public:
  HMI_Display(HardwareSerial &serial);
  void begin(unsigned long baud = 9600);

private:
  HardwareSerial *_serial;
  unsigned long _tmr1;
};
#endif