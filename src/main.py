import os
import time
import busio
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn
import pwmio
import numpy

# IO-pin setup
ir_led0_pin = digitalio.DigitalInOut(board.D14)     # IR-led for sensor0
ir_led0_pin.direction = digitalio.Direction.OUTPUT

ir_led1_pin = digitalio.DigitalInOut(board.D15)     # IR-led for sensor1
ir_led1_pin.direction = digitalio.Direction.OUTPUT

# pwm signal for IR-led pins
ir_led0_pwm = pwmio.PWMOut(board.D14, frequency=38000, duty_cycle=50)
ir_led1_pwm = pwmio.PWMOut(board.D15, frequency=38000, duty_cycle=50)



# create the spi bus
spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)

# create the cs (chip select)
cs = digitalio.DigitalInOut(board.D8)

# create the mcp object
mcp = MCP.MCP3008(spi, cs)

# create an analog input channels
ir_sensor0 = AnalogIn(mcp, MCP.P0)
ir_sensor1 = AnalogIn(mcp, MCP.P1)

while(True):
    print("sensor0")
    print('Raw ADC Value: ', ir_sensor0.value)
    print('ADC Voltage: ' + str(ir_sensor0.voltage) + 'V')

    print("sensor1")
    print('Raw ADC Value: ', ir_sensor1.value)
    print('ADC Voltage: ' + str(ir_sensor1.voltage) + 'V')

    time.sleep(0.5)



"""
last_read = 0       # this keeps track of the last potentiometer value
tolerance = 250     # to keep from being jittery we'll only change
                    # volume when the pot has moved a significant amount
                    # on a 16-bit ADC

def remap_range(value, left_min, left_max, right_min, right_max):
    # this remaps a value from original (left) range to new (right) range
    # Figure out how 'wide' each range is
    left_span = left_max - left_min
    right_span = right_max - right_min

    # Convert the left range into a 0-1 range (int)
    valueScaled = int(value - left_min) / int(left_span)

    # Convert the 0-1 range into a value in the right range.
    return int(right_min + (valueScaled * right_span))

while True:
    # we'll assume that the pot didn't move
    trim_pot_changed = False

    # read the analog pin
    trim_pot = chan0.value

    # how much has it changed since the last read?
    pot_adjust = abs(trim_pot - last_read)

    if pot_adjust > tolerance:
        trim_pot_changed = True

    if trim_pot_changed:
        # convert 16bit adc0 (0-65535) trim pot read into 0-100 volume level
        set_volume = remap_range(trim_pot, 0, 65535, 0, 100)

        # set OS volume playback volume
        print('Volume = {volume}%' .format(volume = set_volume))
        set_vol_cmd = 'sudo amixer cset numid=1 -- {volume}% > /dev/null' \
        .format(volume = set_volume)
        os.system(set_vol_cmd)

        # save the potentiometer reading for the next loop
        last_read = trim_pot

    # hang out and do nothing for a half second
    time.sleep(0.5)
"""

"""
# led set up
led = digitalio.DigitalInOut(board.D18)
led.direction = digitalio.Direction.OUTPUT

list = [0.5, 3, 1, 2, 1, 0.5, 2, 1, 4]
toggle = False

for i in list:
    if toggle == True:
        led.value = True    # Turn LED ON
        print("ON for", i, "seconds")
        toggle = False
    elif toggle == False:
        led.value = False   # Turn LED OFF
        print("OFF for", i, "seconds")
        toggle = True
    time.sleep(i)       # Wait for i seconds
"""