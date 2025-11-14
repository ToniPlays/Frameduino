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

  hal_system_enable();

  hal_pin_attach(13, PIN_CONFIG_DIGITAL_OUTPUT, &ledPin);
  hal_pin_attach(16, PIN_CONFIG_DIGITAL_INPUT, &pin);

  hal_set_timer_interrupt(1, 1);
  hal_pin_attach_interrupt(&pin, PIN_INTERRUPT_RISING, on_interrupt_test);
}

void loop()
{

}


ISR(TIMER1_COMPA_vect)
{
  hal_system_tick();
  hal_pin_toggle(&ledPin);
}