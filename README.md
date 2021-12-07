# MAX86150-DevBoard-MSP430-Driver
The MSP430 program for extracting data from the MAX86150 sensor and outputting it via backchannel UART on the MSP430F5529 development board in the required format for the desktop program.

A beginner project for learning electronics.

### Wiring Connections

Wiring Diagram
MSP430   --------- MAX86150      Logic Analyser
P4.1     --------- SDA
P4.2     --------- SCL
P2.7     --------- INT
3.3V     --------- VCC
GND      --------- GND
P3.2 ----------------------------- Normal Pin (Interrupt Debug GPIO toggled)

### Brief process undertaken
1. Find a sensor on Digikey.
2. Draw the schematic (in KiCad) based on the datasheet for the MAX86150. (Remember the logic level for the sensor is 1.8v while the MSP430F5529 is 3.3v)
3. Create the PCB.
4. Get the PCB manufactured at a PCB house + stencil.
5. Order the parts from Digikey.
6. Add solder paste and using a hot air gun and heat the components.
7. Write a basic I2C driver (Only (single) master read and write is supported).
8. Write the desktop script in Python to open a Serial port, process high level data and output a heart rate.
