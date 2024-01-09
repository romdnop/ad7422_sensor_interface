#include "adt7422.h"

/*
Configuration Register (Register Address 0x03)

Bits [1:0] - These two bits set the number of undertemperature/overtemperature faults that can occur before
            setting the INT pin and CT pin. This helps to avoid false triggering due to temperature noise.
            00 = 1 fault (default).
            01 = 2 faults.
            10 = 3 faults.
            11 = 4 faults.
Bit 2 - This bit selects the output polarity of the CT pin.
        0 = active low.
        1 = active high.
Bit 3 - This bit selects the output polarity of the INT pin.
        0 = active low.
        1 = active high.
Bit 4 - This bit selects between comparator mode and interrupt mode.
        0 = interrupt mode.
        1 = comparator mode.
Bits [6:5] - These two bits set the operational mode for the ADT7422.
            00 = continuous conversion (default). When one conversion is finished, the ADT7422 starts another.
            01 = one shot. Conversion time is typically 240 ms.
            10 = 1 SPS mode. Conversion time is typically 60 ms. This operational mode reduces the average
            current consumption.
            11 = shutdown. All circuitry except interface circuitry is powered down. 
Bit 7 - This bit sets up the resolution of the ADC when converting.
        0 = 13-bit resolution. Sign bit + 12 bits gives a temperature resolution of 0.0625°C.
        1 = 16-bit resolution. Sign bit + 15 bits gives a temperature resolution of 0.0078°C.
*/
uint8_t adt7422_init()
{
    uint8_t result = 0;
    uint8_t conf = 0;
    uint8_t device_id = 0;

    HAL_I2C_Mem_Read(&hi2c1,ADT7422_I2CADDR_DEFAULT,ADT7422_REG__ADT7422_ID,1,(uint8_t *)&device_id,1,1000);

    if(device_id != ADT7422_MANUFACTURER_ID)
    {
        return 0;
    }


    //set 16-bit temperature mode
    conf |= (1<<7);
    //result = HAL_I2C_Mem_Read(&hi2c1,ADT7422_I2CADDR_DEFAULT,ADT7422_REG__ADT7422_CONFIG,1,(uint8_t *)&conf,1,100);
    result = HAL_I2C_Mem_Write(&hi2c1,ADT7422_I2CADDR_DEFAULT,ADT7422_REG__ADT7422_CONFIG,1,(uint8_t *)&conf,1,100);
    if(result == HAL_OK)
    {
        return 1;
    }

    return 0;
}


/*
1. Write to the ADT7422 using the appropriate address.
2. Read the acknowledge bit.
3. Set the register address to 0x2F.
4. Read the acknowledge bit.
5. Apply stop condition.
6. Wait 200 µs for the device to reset the registers to the
default power-up settings
*/
void adt7422_reset()
{
    //to be implmented...
    //HAL_I2C_Mem_Write(&hi2c1,ADT7422_I2CADDR_DEFAULT,ADT7422_REG__ADT7422_SWRST,1,0xFF,1,50);
    
}

uint8_t ad7422_is_present()
{
    uint8_t temp = 0;
    temp = HAL_I2C_IsDeviceReady (&hi2c1, ADT7422_I2CADDR_DEFAULT, 5, 5);
    if(temp != HAL_OK)
    {
        return 1;
    }
    return 0;
}


/*
Bit 7 - This bit goes low when the temperature conversion result is written into the temperature value register. It
is reset to 1 when the temperature value register is read. In one shot and 1 SPS modes, this bit is reset after
a write to the operation mode bits in the configuration register.
*/
uint8_t ad7422_is_measurement_ready()
{
    uint8_t result = 0;
    uint8_t reg;

    HAL_I2C_Mem_Read(&hi2c1,ADT7422_I2CADDR_DEFAULT,ADT7422_REG__ADT7422_STATUS,1,(uint8_t *)&reg,1,100);
    if(reg & (1<<7))
    {
        result = 1;
    }
    return result;
}

uint16_t ad7422_read_temp()
{
    uint16_t temperature = 0xFFFF;
    uint8_t buff[2] = {0};
    //HAL_I2C_Master_Receive(&hi2c1, ADT7422_I2CADDR_DEFAULT,ADT7422_REG__ADT7422_TEMPMSB,)
    HAL_I2C_Mem_Read(&hi2c1, ADT7422_I2CADDR_DEFAULT, ADT7422_REG__ADT7422_TEMPMSB, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&buff, 2, 100);
    //HAL_I2C_Mem_Read(&hi2c1, ADT7422_I2CADDR_DEFAULT,ADT7422_REG__ADT7422_TEMPMSB,2,(uint8_t *)&temp_msb,2,10);
    //HAL_I2C_Mem_Read(&hi2c1, ADT7422_I2CADDR_DEFAULT,ADT7422_REG__ADT7422_TEMPLSB,1,(uint8_t *)&temp_lsb,1,10);
    temperature = (uint16_t)buff[0]<<8;
    temperature |= buff[1];
    return temperature;
}


