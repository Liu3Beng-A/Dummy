#include "mt6816_stm32.h"
#include "spi.h"

void MT6816::SpiInit()
{
    MX_SPI1_Init();
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint16_t MT6816::SpiTransmitAndRead16Bits(uint16_t _dataTx)
{
    uint16_t dataRx = 0;
    transactionValid = false;
    GPIOA->BRR = GPIO_PIN_15; // Chip select
    // SysTick cannot necessarily preempt the 20kHz interrupt. A HAL tick
    // timeout could therefore wait forever. Bound the peripheral polling in
    // DWT cycles (SPI is 16-bit, 9MHz), including a failed peripheral.
    __HAL_SPI_ENABLE(&hspi1);
    const uint32_t began = DWT->CYCCNT;
    const uint32_t budget = SystemCoreClock / 200000; // 5us for the whole word
    while (!(hspi1.Instance->SR & SPI_SR_TXE) && DWT->CYCCNT - began < budget) {}
    if (hspi1.Instance->SR & SPI_SR_TXE)
    {
        *reinterpret_cast<volatile uint16_t*>(&hspi1.Instance->DR) = _dataTx;
        while (!(hspi1.Instance->SR & SPI_SR_RXNE) && DWT->CYCCNT - began < budget) {}
        if (hspi1.Instance->SR & SPI_SR_RXNE)
        {
            dataRx = *reinterpret_cast<volatile uint16_t*>(&hspi1.Instance->DR);
            transactionValid = true;
        }
    }
    GPIOA->BSRR = GPIO_PIN_15;
    if (!transactionValid) __HAL_SPI_CLEAR_OVRFLAG(&hspi1);

    return dataRx;
}
