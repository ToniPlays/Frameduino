#include "Frameduino.h"

#include <Arduino.h>

using namespace Frameduino;

pin_info_t pin;
pin_info_t pin2;

void setup()
{
  Serial.begin(9600);
  hal_pin_attach(3, PIN_CONFIG_DIGITAL_OUTPUT, &pin);
  //hal_pin_attach(9, PIN_CONFIG_DIGITAL_OUTPUT, &pin2);

  hal_set_timer_interrupt(2, 10000);
  //hal_set_timer_interrupt(&pin2, 6000);

  Serial.println("Done");
}

void loop()
{

}


ISR(TIMER2_COMPA_vect)
{
  hal_pin_toggle(pin);
}

ISR(TIMER1_COMPA_vect)
{
  hal_pin_toggle(pin2);
}