#include "Frameduino.h"

#include <Arduino.h>

using namespace Frameduino;

hal_pin_info_t

void setup()
{
  Serial.begin(9600);
  hal_pin_attach(13, PIN_CONFIG_DIGITAL_OUTPUT);
  hal_pin_detach(13);
}

void loop()
{
  pin.Set(true);
  pin.Set(false);
}
