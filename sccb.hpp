#ifdef __cplusplus

#include <cstdint>
#include "stm32f4xx_hal.h"

class Sccb {
public:
    Sccb(uint16_t sda, uint16_t scl, GPIO_TypeDef* gpiOx);
    void init();
    bool writeSlaveRegister(uint8_t slaveAddress, uint8_t registerAddress, uint8_t value);
    bool readSlaveRegister(uint8_t slaveAddress, uint8_t registerAddress, uint8_t& value);

private:
    static uint32_t halfFreq;

    const uint16_t SDA;
    const uint16_t SCL;
    GPIO_TypeDef *GPIOx;
    void sdaSetLow(uint32_t timeToWait) const;
    void sdaSetHigh(uint32_t timeToWait) const;
    void sclSetHigh(uint32_t timeToWait) const;
    void sclSetLow(uint32_t timeToWait) const;
    [[nodiscard]] GPIO_PinState getSda() const;
    void sendStart();
    void sendStop();
    void sendNack();
    void sdaAsInput();
    void sclAsInput();
    void sdaAsOutput() const;
    void sclAsOutput() const;
    uint8_t getByte();
    [[nodiscard]] GPIO_PinState getScl() const;
    bool sendBytes(uint8_t* data, uint8_t size);
    void sendBit(uint8_t i, uint8_t data);
    bool waitForSlaveAck();
};

#endif