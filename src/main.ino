#include "Frameduino.h"

using namespace Frameduino;

pin_info_t pin;
pin_info_t ledPin;

void on_interrupt_test(void* data)
{
  hal_pin_toggle(&ledPin);
}

void setup()
{
  Serial.begin(9600);

  hal_pin_attach(9, PIN_CONFIG_DIGITAL_OUTPUT, &pin);
  hal_pin_attach(13, PIN_CONFIG_DIGITAL_OUTPUT, &ledPin);

  hal_set_timer_interrupt(2, 60);
  
  hal_pin_attach_interrupt(&pin, PIN_INTERRUPT_RISING, on_interrupt_test);
}

void loop()
{

}


ISR(TIMER2_COMPA_vect)
{
  Serial.println("Oof");
  hal_system_tick();
  hal_pin_toggle(&ledPin);
}