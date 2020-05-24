#include "sccb.hpp"

namespace {

bool isTimeNotElapsed(uint32_t startTime, uint32_t wait) {
    return (DWT->CYCCNT - startTime) < wait;
}

void delay(uint32_t wait) {
    uint32_t clk_cycle_start = DWT->CYCCNT;
    while(isTimeNotElapsed(clk_cycle_start, wait));
}


void delay(uint32_t startTime, uint32_t wait) {
    while(isTimeNotElapsed(startTime, wait));
}


uint32_t DWT_Delay_Init() {
    /* Disable TRC */
    CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk; // ~0x01000000;

    /* Enable TRC */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // 0x01000000;

    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    /* Reset the clock cycle counter value */
    DWT->CYCCNT = 0;
/* 3 NO OPERATION instructions */
    __ASM volatile ("NOP");
    __ASM volatile ("NOP");
    __ASM volatile ("NOP");
    /* Check if clock cycle counter has started */
    if(DWT->CYCCNT) {
        return 0; /*clock cycle counter started*/
    } else {
        return 1; /*clock cycle counter not started*/
    }
}
}

void Sccb::sdaSetHigh(uint32_t timeToWait) const {
    const uint32_t start = DWT->CYCCNT;
    GPIOB->BSRR = (uint32_t) SDA;
    delay(start, timeToWait);
}

void Sccb::sdaSetLow(uint32_t timeToWait) const {
    const uint32_t start = DWT->CYCCNT;
    GPIOB->BSRR = (uint32_t) SDA << 16U;
    delay(start, timeToWait);
}

void Sccb::sclSetHigh(uint32_t timeToWait) const {
    const uint32_t start = DWT->CYCCNT;
    GPIOB->BSRR = (uint32_t) SCL;
    delay(start, timeToWait);
}

void Sccb::sclSetLow(uint32_t timeToWait) const {
    const uint32_t start = DWT->CYCCNT;
    GPIOB->BSRR = (uint32_t) SCL << 16U;
    delay(start, timeToWait);
}


GPIO_PinState Sccb::getSda() const {
    return HAL_GPIO_ReadPin(GPIOB, SDA);
}

GPIO_PinState Sccb::getScl() const {
    return HAL_GPIO_ReadPin(GPIOB, SCL);
}


void Sccb::sdaAsInput() {
    sdaSetHigh(0);
}

void Sccb::sclAsInput() {
    sclSetHigh(0);
}

void Sccb::sdaAsOutput() const {
    GPIOB->BSRR = (uint32_t) SDA;
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = SDA;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void Sccb::sclAsOutput() const {
    GPIOB->BSRR = (uint32_t) SCL;
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = SCL;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}
void Sccb::sendNack() {
    sdaSetHigh(0);
    sclSetLow(halfFreq);
    sclSetHigh(halfFreq);
    sclSetLow(0);
    sdaSetLow(halfFreq);
}

void Sccb::sendStart() {
    sdaSetHigh(0);
    sclSetHigh(0);
    sdaSetLow(halfFreq);
    sclSetLow(0);
}

void Sccb::sendStop() {
    sdaSetLow(0);
    sclSetLow(halfFreq);
    sclSetHigh(halfFreq);
    sdaSetHigh(2*halfFreq);
}

bool Sccb::waitForSlaveAck() {
    sdaAsInput();
    sclSetLow(halfFreq);
    auto success = getSda() == GPIO_PIN_RESET;
    sclSetHigh(halfFreq);
    return success || getSda() == GPIO_PIN_RESET;
}

bool Sccb::sendBytes(uint8_t* ptr, uint8_t size) {
    for(uint8_t start = 0; start < size; ++start) {
        const auto data = ptr[start];

        for(uint8_t i = 0; i < 8; i++) {
            sendBit(i, data);
            sclSetLow(halfFreq);
            sclSetHigh(halfFreq);
            sclSetLow(0);
        }
        if(!waitForSlaveAck()) {
            return false;
        }
        sclSetLow(0);
        const auto isNotLastIteration = (start + 1) < size;
        if(isNotLastIteration) {
            sendBit(0, ptr[start + 1]);
        }
        sclSetLow(0);
    }
    return true;
}

void Sccb::sendBit(uint8_t i, uint8_t data) {
    if(static_cast<uint8_t>(data << i) & 128u) {
        sdaSetHigh(0);
    } else {
        sdaSetLow(0);
    }
}

uint8_t Sccb::getByte() {
    sdaAsInput();
    uint8_t data = 0;
    for(uint8_t i = 8; i > 1; i--) {
        if(getSda() == GPIO_PIN_SET) {
            data++;
        }
        data <<= 1u;
        sclSetLow(halfFreq);
        sclSetHigh(halfFreq);
    }
    if(getSda() == GPIO_PIN_SET) {
        data++;
    }
    data <<= 1u;
    sclSetLow(0);

    return data;
}

void Sccb::init() {
    DWT_Delay_Init();
    sdaAsOutput();
    sclAsOutput();
}

bool Sccb::writeSlaveRegister(uint8_t const slaveAddress, uint8_t const registerAddress, uint8_t const value) {
    sendStart();
    uint8_t data[] = {static_cast<uint8_t>(slaveAddress << 1u), registerAddress, value};
    const auto success = sendBytes(data, 3);
    sendStop();
    return success;
}

bool Sccb::readSlaveRegister(uint8_t const slaveAddress, uint8_t const registerAddress, uint8_t& value) {
    uint8_t data[] = {static_cast<uint8_t>(slaveAddress << 1u), registerAddress};
    sendStart();

    if(!sendBytes(data, 2)) {
        sendStop();
        return false;
    }

    data[0] += 1;
    sendStop();
    sendStart();
    if(!sendBytes(data, 1)) {
        sendStop();
        return false;
    }
    value = getByte();
    sendNack();
    sendStop();
    return true;
}

Sccb::Sccb(const uint16_t sda, uint16_t scl) : SDA(sda), SCL(scl) {
    halfFreq = HAL_RCC_GetHCLKFreq() / 100'000 / 2;
}

uint32_t Sccb::halfFreq;
