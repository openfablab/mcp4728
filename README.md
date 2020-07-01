# MCP4728 library for micropython

Helper library for the Microchip MCP4728 I2C 12-bit Quad DAC 

It is based on Adafruit_CircuitPython_MCP4728 library by Bryan Siepert
https://github.com/adafruit/Adafruit_CircuitPython_MCP4728/blob/master/adafruit_mcp4728.py

Ported to microPython by Alexander Olikevich (openfablab) with some changes:

* Channel properties read from device not only at init but at every call
* Added power down control function
* Added config() function for laconic channel initial setup
* Changed "raw_value" to "value" and channel names from "channel_a" to "a", etc.
* Removed confusing emulation of 16 bit interface
* Fixed incorrect register values types on initialisation 
* Gains values read and write as 1 or 2
* Rewrited Vref control for simplicity

# Usage example (on ESP32):

from machine import Pin, I2C
import mcp4728
i2c = I2C(0, scl=Pin(27), sda=Pin(4), freq=400000)

# Create the MCP4725 drivers specifying
# the I2C bus the MCP4728 is connected to.
# and the I2C slave address of the sensor
dac1=mcp4728.MCP4728(i2c,0x60)
dac2=mcp4728.MCP4728(i2c,0x61)

# Set initial value, vref, gain and power mode for channel a
dac1.a.config(0,1,1,0) 

# Get channel b 12-bit current value
dac1.b.value

# Set channel b 12-bit value
dac1.b.value=4095
dac1.b.value=0
dac1.b.value=123

# Get channel b value as a floating point number in the range 0.0 to 1.0
dac1.b.normalized_value

# Set channel b value as a floating point number in the range 0.0 to 1.0
dac1.b.normalized_value=0.51123

# Get channel d Vref
dac1.d.vref

# Set channel d Vref (voltage reference source). Set 0 for VDD or 1 for internal 2.048V reference.
dac1.d.vref=1

# Get channel c gain
dac1.c.gain

# Sets the gain of the channel c if the Vref for the channel is 1 (INTERNAL)
# Has no effect if the Vref for the channel is 0 (VDD)
# With gain set to 1, the output voltage goes from 0v to 2.048V. If a channel's gain is set
# to 2, the voltage goes from 0v to 4.096V.
dac1.c.gain=1

# Get channel b power state
dac1.b.pdm

# Set channel b power state. 0 for normal operation, or
# other to turn off most of the channel circuits and connect 
# VOUT to GND by resistor (1: 1 kΩ, 2: 100 kΩ, 3: 500 kΩ).
dac1.b.pdm=0
