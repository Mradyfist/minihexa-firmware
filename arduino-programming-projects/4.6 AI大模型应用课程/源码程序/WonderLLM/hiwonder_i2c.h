#ifndef HIWONDER_I2C_H_
#define HIWONDER_I2C_H_

#include "global.h"
#include "Wire.h"

#ifdef __cplusplus
extern "C" {
#endif

/*transmit one byte to the slave*/
bool wire_write_byte(uint8_t addr, uint8_t val);

/*write several bytes to a slave register*/
bool wire_write_array(uint8_t addr, 
                      uint8_t reg,
                      uint8_t *val,
                      uint16_t len);

/*read several bytes from a slave register*/
int wire_read_array(uint8_t addr, 
                    uint8_t reg, 
                    uint8_t *val, 
                    uint8_t len);

/*write several bytes to the slave**/
bool wireWritemultiByte(uint8_t addr, 
                        uint8_t *val, 
                        unsigned int len);

/*write several bytes to the slave*/
int wireReadmultiByte(uint8_t addr, 
                      uint8_t *val, 
                      unsigned int len);

#ifdef __cplusplus
}
#endif

#endif