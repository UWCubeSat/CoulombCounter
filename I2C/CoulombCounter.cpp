/*
 * Library for the Coulomb Counter, measuring battery
 * charge state, voltage, and current. Implemented with 
 * solar panels as load, input range allows use with multicell 
 * batteries up to 20V, we will be using Lithiom Ion batteries (3.3V).
 * Precision is achieved via an integrated sense resistor between the positive
 * terminal of the battery and a load/charger. We will be storing measurements
 * via the internal registers accessed on the I2C/SMBus Interface, or WIRE in Arduino
 * Source: Source: https://github.com/ceech/Power-monitor-LTC2943/
 * 
 * Husky Satellite Lab
 * Created by Katharine Lundblad on February 1st, 2021
 */

#include <stdint.h>
#include "CoulombCounter.h"
#include <Wire.h>
#include "LT_I2C.h"


void LTC2943_reset() {
  /* Set pin modes for desired communication */
}

/*
 * Write 8 bit code to the LTC2943
 * @return state of acknowledge bit after the address is written 
 * 0 = acknowledge, 1 = no acknowledge
 */
int8_t LTC2943_write(uint8_t i2c_address, uint8_t adc_command, uint8_t code) {
  int32_t acknowledge_bit;

  acknowledge_bit = i2c_write_byte_data(i2c_address, adc_command, code);
  return(acknowledge_bit);
}

/*
 * Write 16 bit code to LTC2943
 * @return state of acknowledge bit after address written
 */
int8_t LTC2943_write_16_bits(uint8_t i2c_address, uint8_t adc_command, uint16_t code) {
  int8_t acknowledge_bit;

  ackowledge_bit = i2c_write_word_data(i2c_address, adc_command, code);
  return(acknowledge_bit); 
}

/*
 * Reads an 8-bit adc_code from LTC2943
 */
int8_t LTC2943_read(uint8_t i2c_address, uint8_t adc_command, uint8_t *adc_code) {
  int32_t acknowledge_bit;

  acknowledge_bit = i2c_read_byte_data(i2c_address, adc_command, adc_code);
  return(acknowledge_bit);
}

/*
 * Reads a 16-bit adc_code from LTC2943
 */
int8_t LTC2943_read_16_bits(uint8_t i2c_address, uint8_t adc_command, uint16_t *adc_code) {
  int32_t acknowledge_bit;

  acknowledge_bit = i2c_read_word_data(i2c_address, adc_command, adc_code);
  return(acknowledge_bit);
}

/*
 * Converts 16-bit raw adc_code to coulombs
 */
float LTC2943_code_to_coulombs(uint16_t adc_code, float resistor, uint16_t prescalar) {
  float coulomb_charge;
  coulomb_charge =  1000 * (float)(adc_code * LTC2943_CHARGE_LSB * prescalar * 50E-3)/(resistor * 4096);
  coulomb_charge = coulomb_charge*3.6f;
  return(coulomb_charge);
}

/*
 * Converts 16-bit raw adc code to mAh
 */
float LTC2943_code_to_mAh(uint16_t adc_code, float resistor, uint16_t prescalar ) {
  float mAh_charge;
  mAh_charge = 1000 * (float)(adc_code * LTC2943_CHARGE_LSB * prescalar * 50E-3)/(resistor * 4096);
  return(mAh_charge);
}

/*
 * Converts 16-bit raw adc code to volts
 */
float LTC2943_code_to_voltage(uint16_t adc_code) {
  float voltage;
  voltage = ((float)adc_code / (65535)) * LTC2943_FULLSCALE_VOLTAGE;
  return(voltage);
}

/*
 * Converts 16 bit raw adc code to amps
 */
float LTC2943_code_to_current(uint16_t adc_code, float resistor) {
  float current;
  current = (((float)adc_code - 32767)/(32767)) * ((float)(LTC2943_FULLSCALE_CURRENT) / resistor);
  return(current);
}

/*
 * Calculates average current from 16 bit raw adc code
 */
float LTC2943_code_to_avgcurrent(uint16_t adc_code, float resistor) {
  
}

/*
 * Calculates average voltage from 16 bit raw adc code
 */
float LTC2943_code_to_avgvoltage(uint16_t adc_code) {
  
}

/*
 * Set and clear bits in a control register, accessed via ALCC pin on the LTC2943
 */
int8_t LTC2943_register_set_clear_bits(uint8_t i2c_address, uint8_t register_address, uint8_t bits_to_set, uint8_t bits_to_clear) {
  uint8_t register_data;
  int8_t acknowledge_bit = 0;

  ack |= LTC2943_read(i2c_address, register_address, &register_data);
  // bits_to_clear will be inverted and bitwise AND'd with the register
  // every location with a 1 will result in a 0 in the register
  register_data = register_data & (~bits_to_clear);
  register_data = register_data | bits_to_set; // bits_to_set will be bitwise OR'd with the register
  acknowledge_bit |= LTC2943_write(i2c_address, register_address, register_data);
  return(acknowledge_bit);
}
