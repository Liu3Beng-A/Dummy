#include "mt6816_base.h"

bool MT6816Base::Init()
{
    SpiInit();
    UpdateAngle();

    // Check if the stored calibration data are valid
    angleData.rectifyValid = true;
    for (uint32_t i = 0; i < RESOLUTION; i++)
    {
        if (quickCaliDataPtr[i] == 0xFFFF)
            angleData.rectifyValid = false;
    }

    return angleData.rectifyValid;
}


uint16_t MT6816Base::UpdateAngle(uint8_t maxAttempts)
{
    angleData.sampleValid = false;
    dataTx[0] = (0x80 | 0x03) << 8;
    dataTx[1] = (0x80 | 0x04) << 8;

    // Closed-loop ticks request one bounded attempt (2 x 5us SPI).
    // A failed sample remains invalid and is retried on the next 50us tick.
    // Initialization and electrical calibration retain the default three attempts.
    if (maxAttempts < 1) maxAttempts = 1;
    if (maxAttempts > 3) maxAttempts = 3;
    for (uint8_t i = 0; i < maxAttempts; i++)
    {
        dataRx[0] = SpiTransmitAndRead16Bits(dataTx[0]);
        const bool firstReadValid = SpiTransactionValid();
        if (!firstReadValid) { spiRawData.checksumFlag = false; continue; }
        dataRx[1] = SpiTransmitAndRead16Bits(dataTx[1]);

        if (!firstReadValid || !SpiTransactionValid())
        {
            spiRawData.checksumFlag = false;
            continue;
        }

        spiRawData.rawData = ((dataRx[0] & 0x00FF) << 8) | (dataRx[1] & 0x00FF);

        // Fixed-cost parity fold; equivalent for all 65536 raw words.
        uint16_t parity=spiRawData.rawData;
        parity ^= parity >> 8; parity ^= parity >> 4;
        parity ^= parity >> 2; parity ^= parity >> 1;
        if (parity & 0x01)
        {
            spiRawData.checksumFlag = false;
        } else
        {
            spiRawData.checksumFlag = true;
            break;
        }
    }

    if (spiRawData.checksumFlag)
    {
        spiRawData.rawAngle = spiRawData.rawData >> 2;
        spiRawData.noMagFlag = (bool) (spiRawData.rawData & (0x0001 << 1));
        if (!spiRawData.noMagFlag)
        {
            angleData.rawAngle = spiRawData.rawAngle;
            angleData.rectifiedAngle = quickCaliDataPtr[angleData.rawAngle];
            angleData.sampleValid = angleData.rectifiedAngle < 51200;
        }
    }

    return angleData.rectifiedAngle;
}


bool MT6816Base::IsCalibrated()
{
    return angleData.rectifyValid;
}

