#include "encoder_calibrator_stm32.h"
#include "Platform/Memory/stockpile_f103cb.h"

void EncoderCalibrator::BeginWriteFlash()
{
    Stockpile_Flash_Data_Begin(&stockpile_quick_cali);
}


void EncoderCalibrator::EndWriteFlash()
{
    Stockpile_Flash_Data_End(&stockpile_quick_cali);
}


void EncoderCalibrator::ClearFlash()
{
    Stockpile_Flash_Data_Empty(&stockpile_quick_cali);
}


void EncoderCalibrator::WriteFlash16bitsAppend(uint16_t _data)
{
    Stockpile_Flash_Data_Write_Data16(&stockpile_quick_cali, &_data, 1);
}

void EncoderCalibrator::ReadFlash16bits(uint16_t* data, uint32_t num)
{
    Stockpile_Flash_Data_Read_Data16(&stockpile_quick_cali, data, num);
}
