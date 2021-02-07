/*
 * Using the WIRE interface on the ESP32, reads
 * charge, current and voltage from the LTC2943.
 * Precision is maintained via two SENSE pins and a sense
 * resistor. Current prototype uses a 50mOhm sense resistor.
 * Source: Source: https://github.com/ceech/Power-monitor-LTC2943/
 * 
 * Husky Satellite Lab
 * Created by Katharine Lundblad on February 1st 2021
 */

#include "Wire.h"
#include "CoulombCounter.h"

void setup {
  Wire.begin(); 
  Serial.begin(115200);
  delay(1); // time for registers to be set to default state
}

void loop {
  while (!Serial.available()) { // waits for signal to be fed to serial
    
  }
  
  test();
  while (1) {
    
  }
}

void test() {
  int8_t adc_command
  float resistor = 0.05; // 50 mOhm sense resistor

  adc_command = LTC2943_SENSE_MONITOR | LTC2943_AUTOMATIC_MODE; // Builds commands to set LTC2943 to automatic mode
  ack |= LTC2943_write(LTC2943_I2C_ADDRESS, LTC2943_CONTROL_REG, adc_command);   // Sets the LTC2943 to automatic mode

  ack |= LTC2943_read_16_bits(LTC2943_I2C_ADDRESS, LTC2943_CHARGE_MSB_REG, &charge_code);  // Reads the ADC registers that contains charge value
  charge = LTC2943_code_to_coulombs(charge_code, resistor, prescalarValue); // Calculates charge from charge code, resistor and prescalar

  ack |= LTC2943_read_16_bits(LTC2943_I2C_ADDRESS, LTC2943_CURRENT_MSB_REG, &current_code); // Reads the voltage code across sense resistor
  current = LTC2943_code_to_current(current_code, resistor); // Calculates current from current code, resistor value.

  ack |= LTC2943_read_16_bits(LTC2943_I2C_ADDRESS, LTC2943_VOLTAGE_MSB_REG, &voltage_code);   // Reads voltage voltage code
  VIN = LTC2943_VIN_code_to_voltage(voltage_code);  // Calculates VIN voltage from VIN code and lsb

  Serial.println(charge);
  Serial.println(current);
  Serial.println(VIN); 
  Wire.endTransmission(); 
}
