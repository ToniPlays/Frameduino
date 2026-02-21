#include "Frameduino.h"
#include "mcp2515.h"

using namespace Frameduino;

pin_info_t can_stb;
pin_info_t sig_pwr;
MCP2515 mcp;

void setup()
{
  hal_system_enable();
  hal_logger_register(new hal_default_logger_t());
  hal_pin_attach(8, PIN_CONFIG_DIGITAL_OUTPUT, &can_stb);
  hal_pin_attach(3, PIN_CONFIG_DIGITAL_OUTPUT, &sig_pwr);

  hal_spi_register(10, &mcp);
  hal_pin_write(&can_stb, true);
  hal_pin_write(&sig_pwr, true);

  mcp.set_mode();

  hal_logger_log_i("Ready");
  delay(15000);
  hal_pin_write(&sig_pwr, false);
  hal_system_sleep(SLEEP_MODE_STANDBY);

  
  hal_eeprom_read(nullptr, 0, sizeof(pin_info_t));
  hal_eeprom_write(0, &can_stb, sizeof(pin_info_t));
}

void loop()
{
  hal_spi_device_update(10);
  // hal_set_clock_prescaler(0);
  // delay(5000);

  // delay(500);
  // hal_set_clock_prescaler(6);
  // delay(50);
}