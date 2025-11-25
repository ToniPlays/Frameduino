#ifndef FRAMEDUINO_SPI_DEVICE_H
#define FRAMEDUINO_SPI_DEVICE_H

#include "hardware/gpio/pin.h"
#include "system/core/hal_system.h"

#include <SPI.h>

namespace Frameduino
{
    class spi_device_t
    {
    public:
        spi_device_t();
        ~spi_device_t();

        bool transfer(uint8_t *bytes, uint8_t length);
        bool request(uint8_t *buffer, uint8_t length);

        void transaction(void (*cb)(spi_device_t *, void *), void* user_data);

        pin_info_t* get_cs_pin() { return &cs_pin; }
        void set_cs_pin(pin_info_t* pin) { cs_pin = *pin; }

        virtual void init() = 0;
        virtual void update() = 0;

    private:
        pin_info_t cs_pin;
    };


    static void hal_spi_register(uint8_t cs, spi_device_t* device)
    {
        #ifdef FRAMEDUINO_DEBUG
        if(!device)
        {
            hal_logger_log_err("Cannot register null SPI device");
            return;
        }
        #endif

        pin_info_t pin;
        hal_pin_attach(cs, PIN_CONFIG_DIGITAL_OUTPUT, &pin);
        hal_pin_write(&pin, true);
        device->set_cs_pin(&pin);
        device->init();

        hal_spi_device_t spi = {};
        spi.pin = cs;
        spi.device = device;

        SPI.begin();

        HAL::hal_get_system_info()->devices.add(spi);
        hal_logger_log_w(("Registered SPI device at pin " + String(cs)).c_str());
    }
    static void hal_spi_device_update(uint8_t cs)
    {
        spi_device_t* device = HAL::hal_get_system_info()->devices.find([](hal_spi_device_t* d, void* user_data) { return d->pin == *(uint8_t*)user_data; }, &cs);
        if(device) device->update();
    }
}

#endif