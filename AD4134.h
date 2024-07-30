#ifndef AD4134_H
#define AD4134_H

#include <Arduino.h>
#include <SPI.h>

class AD4134 {
public:
    AD4134(int csPin);

    void begin();
    void configure();
    uint32_t readADC(uint8_t channel);

private:
    int _csPin;

    void writeRegister(uint8_t reg, uint8_t value);
    uint8_t readRegister(uint8_t reg);
    void updateRegister(uint8_t reg, uint8_t mask, uint8_t value);
    uint32_t spiRead(uint8_t reg_addr);
    void spiWrite(uint8_t reg_addr, uint8_t value);
};

#endif
