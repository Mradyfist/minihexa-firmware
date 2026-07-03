#ifndef __HIWONDER_BOARD_H__
#define __HIWONDER_BOARD_H__

#include "global.h"
#include "SensorQMI8658.hpp"
#include "MadgwickAHRS.h"
#include "esp_adc_cal.h"
#include "driver/ledc.h"
#include "driver/adc.h"

#define MINIHEXA_V1_1

#define BUZZER_PIN      21
#define LEDC_CHANNEL_0  0
#define LEDC_TIMER_BIT  13                // set 13-bit resolution
#define LEDC_BASE_FREQ  3000              // set the 5kHz PWM base frequency

#define BAT_DEC_ADDR    0x46
#define BAT_READ_REG    0
#define DEFAULT_VREF    1100              // default reference voltage of 1.1V
#define NO_OF_SAMPLES   64                // number of ADC samples
#define ADC_WIDTH       ADC_WIDTH_12Bit   // ADC 12-bit width
#define ADC_ATTEN       ADC_ATTEN_DB_11   // 6dB attenuator
#define ADC_PIN         ADC1_CHANNEL_5    // ADC pin
#define WINDOW_SIZE     8  // filter window size
#define R21             100000
#define R22             10000


class HW_Board {
  public:
    uint16_t bat_voltage;
    
    void begin(void);
    /**
     * @brief Get button state
     * 
     * @return true  -not pressed
     * @return false -pressed
     */
    bool get_button_state(void);

    /**
     * @brief Get sound sensor value
     * 
     * @return int 
     */
    int get_sound_val(void);

    /**
     * @brief Update gyroscope data
     * 
     */
    void _imu_update(void);

    /**
     * @brief Update gyroscope data
     * @param  state    -true  on
     *                  -false off
     */
    void imu_update(bool state);

    /**
     * @brief Get gyroscope acceleration data
     * 
     * @param  val
     */
    void get_imu_acc(float *val);

    /**
     * @brief Get gyroscope angular velocity data
     * 
     * @param  val
     */
    void get_imu_gyro(float *val);

    /**
     * @brief Get gyroscope Euler angle data
     * 
     * @param  val
     */
    void get_imu_euler(float *val);  
    /**
     * @brief Update battery voltage data
     * 
     * @return  
     */
    void bat_voltage_update(void);

  private:

    IMUdata acc;
    IMUdata acc_offset;
    IMUdata acc_cali;
    IMUdata gyro;
    IMUdata gyro_offset;
    IMUdata gyro_cali;
    IMUdata euler;  /* x-roll y-pitch z-yaw */

    SensorQMI8658 qmi;
    Madgwick filter;

    TimerHandle_t imu_timer;
    TimerHandle_t buzzer_timer;
    TimerHandle_t bat_monitor_timer;
    esp_adc_cal_characteristics_t *adc_chars = NULL;
    
    bool buzzer_on_state = false;
    bool imu_on_state = false;
    uint8_t voltage_index = 0;  
    int last_sample_voltage = 0;
    uint32_t voltage_buffer[WINDOW_SIZE] = {0};

    const int led_pin = 2;
    const int button_pin = 0;
    const int sda_pin = 22;
    const int scl_pin = 23;
    const int sound_pin = 34;
    const uint16_t imu_update_interval = 10;
    const uint16_t bat_monitor_update_interval = 1000;

    /* static calibration */
    void imu_calibarate(void);

};

#endif
