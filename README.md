# About
USB interface for [ADT7422](https://www.analog.com/media/en/technical-documentation/data-sheets/ADT7422.pdf) temperature sensor based on STM32F4 [board](https://docs.zephyrproject.org/latest/boards/arm/blackpill_f411ce/doc/index.html) by WeAct.


# Sensor connection

* PB6 - I2C Data
* PB7 - I2C Clock

# Data format

The board send temperature readings over Virtual COM-port every second. The data is being send in the following format:

<table>
  <tr>
    <th>Byte 1</th>
    <th>Byte 2</th>
    <th>Byte 3</th>
    <th>Byte 4</th>
  </tr>
  <tr>
    <td>Temp MSB</td>
    <td>Temp LSB</td>
    <td>CRC8</td>
    <td>CR ('\r')</td>
  </tr>
</table>

Where CRC8 is calculated by:

    uint8_t CRC8(const uint8_t *data, size_t length) {
        uint8_t crc = 0;
        
        for (size_t i = 0; i < length; ++i) {
            crc ^= data[i];
            
            for (int j = 0; j < 8; ++j) {
                if (crc & 0x80) {
                    crc = (crc << 1) ^ 0x07;  // Polynomial 0x07
                } else {
                    crc <<= 1;
                }
            }
        }
        
        return crc;
    }

# Temperature conversion formula

Use the following formulas to calculate the measured temperatures in °C from the ADT7422 output codes in 16-bit format (where ADC Code uses all 16 bits of the data byte, including the sign bit): 

_Positive Temperature = ADC Code (Decimal)/128_\
_Negative Temperature = (ADC Code (Decimal) − 65,536)/128_ 

Where Bit 15 (sign bit) is removed from the ADC code:

_Negative Temperature = (ADC Code (Decimal) − 32,768)/128_

# Build

Project can be built in _[Visual Studio Code](https://code.visualstudio.com/)_ with _[C/C++](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools)_ and _[STM32](https://marketplace.visualstudio.com/items?itemName=bmd.stm32-for-vscode)_ plugins installed.

To build and debug [ARM Embedded Toolchain](https://developer.arm.com/downloads/-/gnu-rm) and [OpenOCD](https://openocd.org/) are both required.
