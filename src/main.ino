#include "Frameduino.h"

using namespace Frameduino;

void setup()
{
  hal_logger_register(new hal_default_logger_t());
  hal_system_enable();

  hal_spi_register_device(new spi_device_t());
}

void loop()
{
  
}