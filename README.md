# coulomb-counter
Evaluation board to send the bus voltage and accumulated charge from a battery to the MSP430. 
It does this by monitoring voltage developed across a sense resistor. Differential voltage between
the SENS+ and SENSE- pins is applied to an auto-zeroed differential analog integrater to track charge.
For precision, the sensor should maintain below a threshold of 50mV
