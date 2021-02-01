/*
 * Library for Multicell Batter Gas Gauge with
 * Voltage and Current Measurement for the Husky Satellite 2
 * 
 * Husky Satellite Lab
 * Created by Katharine Lundblad on February 1st, 2021
 */

#ifndef COULOMBCOUNTER_H
#define COULOMBCOUNTER_H

#include <Wire.h>

/*
 * LTC2943 WIRE Address Assignment
 */
#define LTC2943_I2C_ADDRESS 0x64 /** 0xC8 **/
#define LTC2943_I2C_ALERT_RESPONSE 0x0C

/*
 * Define Register Addresses for charge and voltage
 * measurement
 */
#define LTC2943_STATUS_REG                          0x00
#define LTC2943_CONTROL_REG                         0x01
#define LTC2943_ACCUM_CHARGE_MSB_REG                0x02
#define LTC2943_ACCUM_CHARGE_LSB_REG                0x03
#define LTC2943_CHARGE_THRESH_HIGH_MSB_REG          0x04
#define LTC2943_CHARGE_THRESH_HIGH_LSB_REG          0x05
#define LTC2943_CHARGE_THRESH_LOW_MSB_REG           0x06
#define LTC2943_CHARGE_THRESH_LOW_LSB_REG           0x07
#define LTC2943_VOLTAGE_MSB_REG                     0x08
#define LTC2943_VOLTAGE_LSB_REG                     0x09
#define LTC2943_VOLTAGE_THRESH_HIGH_MSB_REG         0x0A
#define LTC2943_VOLTAGE_THRESH_HIGH_LSB_REG         0x0B
#define LTC2943_VOLTAGE_THRESH_LOW_MSB_REG          0x0C
#define LTC2943_VOLTAGE_THRESH_LOW_LSB_REG          0x0D
#define LTC2943_CURRENT_MSB_REG                     0x0E
#define LTC2943_CURRENT_LSB_REG                     0x0F
#define LTC2943_CURRENT_THRESH_HIGH_MSB_REG         0x10
#define LTC2943_CURRENT_THRESH_HIGH_LSB_REG         0x11
#define LTC2943_CURRENT_THRESH_LOW_MSB_REG          0x12
#define LTC2943_CURRENT_THRESH_LOW_LSB_REG          0x13

/*
 * Define Conversion Constants
 */
const float LTC2943_CHARGE_lsb = 0.34E-3; // 0.34 mAh
const float LTC2943_VOLTAGE_lsb = 1.44E-3; // 1.44 mV
const float LTC2943_CURRENT_lsb = 29.3E-6; // 29.3 uV
const float LTC2943_TEMPERATURE_lsb = 0.25; // 0.25 C
const float LTC2943_FULLSCALE_VOLTAGE = 23.6; // 23.6 V
const float LTC2943_FULLSCALE_CURRENT = 60E-3; // 60 mV
const float LTC2943_FULLSCALE_TEMPERATURE = 510; // 510 K

/** 
 * Write an 8-bit code to a specified register on the LTC2943
 * @param i2c_address register address for LTC2943
 * @param adc_command "command byte" for LTC2943
 * @param code value written to the register
 * @return The function returns the state of the acknowledge bit after the I2C address write. 
 * 0=acknowledge, 1=no acknowledge.
 */
int8_t LTC2943_write(uint8_t i2c_address, uint8_t adc_command, uint8_t code);

/*
 * Writing a 16-bit code to the specified register on the the LTC2943
 * @param i2c_address register address for LTC2943
 * @param adc_command "command byte" for LTC2943
 * @param code value written to the register
 * @return The function returns the state of the acknowledge bit after the I2C address write. 
 * 0=acknowledge, 1=no acknowledge. This is done via the ALCC pin on the LTC2943
 */
int8_t LTC2943_write_16_bits(uint8_t i2c_address, uint8_t adc_command, uint16_t code);

/*
 * Reading an 8 bit code from the specified register on the the LTC2943
 * @param i2c_address register address for LTC2943
 * @param adc_command "command byte" for LTC2943
 * @param adc_code a pointer to the code value read from the register
 * @return The function returns the state of the acknowledge bit after the I2C address read. 
 * 0=acknowledge, 1=no acknowledge. This is done via the ALCC pin on the LTC2943
 */
int8_t LTC2943_read(uint8_t i2c_address, uint8_t adc_command, uint8_t *adc_code);

/*
 * Reading an 16 bit code from the specified register on the the LTC2943
 * @param i2c_address register address for LTC2943
 * @param adc_command "command byte" for LTC2943
 * @param adc_code a pointer to the code value read from the register
 * @return The function returns the state of the acknowledge bit after the I2C address read. 
 * 0=acknowledge, 1=no acknowledge. This is done via the ALCC pin on the LTC2943
 */
int8_t LTC2943_read_16_bits(uint8_t i2c_address, uint8_t adc_command, uint16_t *adc_code);


/* 
 * Calculates charge in coulombs in the LTC2943
 * @param adc_code Raw adc value
 * @param resistor sense resistor value, in series with the LTC2943, current prototype 50mOhms
 * @param prescalar the prescalar value of the charge
 * @return the Coulombs of charge in the ACR register.
 */
float LTC2943_code_to_coulombs(uint16_t adc_code, float resistor, uint16_t prescalar);

/*
 * Calculates charge in mAh in the LTC2943
 * @param adc_code Raw adc value
 * @param resistor sense resistor value, in series with the LTC2943, current prototype 50mOhms
 * @param prescalar the prescalar value of the charge
 * @return the Coulombs of charge in the ACR register.
 */
float LTC2943_code_to_mAh(uint16_t adc_code, float resistor, uint16_t prescalar);

/*
 * Calculate the LTC2943 SENSE+ voltage, connected to negative
 * terminal side of resistor while SENSE- connected to positive 
 * enabling voltage measurement
 * @adc_code raw adc value
 * @return SENSE+ voltage in volts
 */
float LTC2943_code_to_voltage(uint16_t adc_code);

/*
 * Calculate the LTC2943 current with a sense resistor (in parallel with LTC2943)
 * @param adc_code the raw adc value
 * @param resistor the sense resistor value, keep within 50 mOhms for precision (prototype is 50mOhms)
 * @return current through sense resistor
 */
float LTC2943_code_to_current(uint16_t adc_code, float resistor);
