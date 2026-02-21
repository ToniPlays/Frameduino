#ifndef MCP2515_H
#define MCP2515_H

#include "Frameduino.h"

using namespace Frameduino;

class MCP2515 : public spi_device_t
{
public:
    MCP2515() = default;
    ~MCP2515() = default;

    void init() {

    };
    void update() {

    };

    void set_mode()
    {
        uint8_t canctrl = 0x00;

        transaction([](spi_device_t *device, void *user_data)
                    {
                        uint8_t data[] = {0x03, 0x0F};
                        device->transfer(data, 2);
                        device->request((uint8_t*)user_data, 1);
                    },
                    &canctrl);

        canctrl &= ~0B11100000;
        canctrl |= 0x20;

        hal_logger_log_err(String(canctrl).c_str());
        transaction([](spi_device_t *device, void *user_data) {
            uint8_t req[] = {0x02, 0x0F, *(uint8_t*)user_data};
            device->transfer(req, 3); 
        }, &canctrl);

        hal_logger_log_i("Mode updated");
    }
};

#endif