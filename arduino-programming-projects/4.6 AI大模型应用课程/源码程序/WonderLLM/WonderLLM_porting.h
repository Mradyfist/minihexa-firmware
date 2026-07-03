#ifndef _WONDERLLM_PORTING_H
#define _WONDERLLM_PORTING_H

#include <Arduino.h>
#include <stdbool.h>
#include "stdint.h"
#include <Wire.h>
// --- I2C & protocol definitions ---
#define WONDERLLM_SLAVE_ADDRESS 0x55//0x55
#define I2C_TIMEOUT              10 // ms

extern const char tool_finish[];
extern const char tool_buzzer[];
extern const char tool_led[];
extern const char tool_stop[];
extern const char tool_SpeedMode[];
extern const char tool_RunningMode[];
extern const char tool_status[];
extern const char tool_move[];
extern const char tool_BaryCenterMove[];
extern const char tool_InclinationAngleMove[];
extern const char tool_ActionGroup[];

extern const char vision_prompt[];

/**
 * @brief System delay function interface
 */
void delay_ms(int ms_num);

/**
 * @brief System real-time clock retrieval interface
 */
uint32_t Get_time_now(void);

/**
 * @brief IIC scan interface
 * @note  1. Not strictly required to run; only for the sake of program robustness
 */
bool Detect_WonderLLM();

/**
 * @brief IIC rate configuration interface
 * @note When transferring long strings (especially Chinese), this function must be called to set the IIC rate
 *       up to 400,000, otherwise WonderLLM will not be able to receive the data completely 
 */
void IIC_Config_MCP_Transmit(void);

/**
 * @brief IIC rate configuration interface
 * @note 1. Not strictly required to run; if all other IIC devices support the 400W rate, there is no need to switch back
 *         the lower 100W; calling this function is for compatibility with other low-speed IIC devices
 */
void IIC_Config_normal_Transmit(void);

/**
 * @brief Low-level I2C data receive interface
 */
// int WonderLLM_Receive_Data(uint8_t* buffer, uint16_t size);
int WonderLLM_Receive_Data(uint8_t* buffer, uint16_t size,bool stop_flag);

/**
 * @brief Low-level I2C data send interface
 */
int WonderLLM_Send_Data(uint8_t* buffer, uint16_t len);


#endif