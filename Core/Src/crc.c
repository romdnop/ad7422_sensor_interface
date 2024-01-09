#include "crc8.h"
// CRC-8 calculation function
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