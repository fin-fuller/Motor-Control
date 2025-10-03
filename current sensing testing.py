


#---this file is for a low level approach to reading the current from an Adafruit breakout board with INA228 power monitor. Using uPython for use with the Raspberry Pi Pico


from machine import Pin, I2C, PWM
from time import sleep
import sys
import struct #this is used to convert a float to byte array

#---CURRENT SENSING PINS---#
Isense = I2C(scl=Pin(17), sda=Pin(16), freq=400000) #I2C IS FOR DATA TRANSFER
alrt = Pin(18, Pin.IN, Pin.PULL_UP) #default state is active low


#---MOTOR2 PIN---#
p5 = Pin(5, Pin.OUT, Pin.PULL_DOWN) #defines MOTOR 2 output pin. Set as pull down for safety on startup (ensures the motor is not in an "on" state)
pwm5 = PWM(p5)#defines pin 5 as a pwm pin
pwm5.freq(1000)#sets the frequency of switching - a balance between transistor switching speed and smooth operation/low noise etc
pwm5.duty_u16(0)#set to 0 to continue making sure we are at a safe startup condition

p4 = Pin(4, Pin.OUT, Pin.PULL_DOWN) #sets the direction pin

#MOTOR1 PIN---#
p3 = Pin(3, Pin.OUT, Pin.PULL_DOWN)
pwm3 = PWM(p3)
pwm3.freq(1000)
pwm3.duty_u16(0)

p2 = Pin(2, Pin.OUT, Pin.PULL_DOWN)

def ihandler(pin):
    pwm3.duty_u16(0)
    sys.exit()
def current_sense():
    try:
    #----PROCESS FLOW FOR CURRENT MONITORING---#
    #reset the adc_config (automatically records the shunt voltage)

    #----PROCESS FLOW FOR CURRENT MONITORING---#

    #Set the current threshold
    #set the LSB for current monitoring
    #Internal resistance is known, so this can stay the same - could be a parameter to change if adjustments are needed
    #ADC config register to be set to continuous current monitoring - DO WE ACTUALLY NEED THIS?
    #DIAG alert set for triggering the alert and therefore the falling of the ALRT pin to 0, which the MCU will pick up

        def twos_comp(byte_array): #takes a hex string (whatever i input into the register)
            value = int.from_bytes(byte_array) #convert to a binary literal(0s, and 1s)
            print(f"Binary literal == {value}")
            if byte_array[0] == '1':  # MSB is 1 → negative
                value -= 1 << len(byte_array)
            return value
        
        def int_to_bytes(value, size_in_bytes): #int to bytes can be used for byte order aswell as twos complement
            return value.to_bytes(size_in_bytes, "big") #always in big endian format for the INA228. returns an 
        
        def float_to_bytes(value):
            print("float_to_byte function started")
            bytes = struct.pack('>h', value).hex() #converts it to a hex string
            #scaled_bytes = struct.pack('>f', value*1000).hex()
            print(f"the value from the floats_to_bytes function, in hex string, is {bytes}")
            print(f"the number of bytes in the result is {len(bytes)//2}")# "//" is divide and round to the nearest whole number
            #print(f"the value form the scaled_bytes function, in hex string is {scaled_bytes}")
            return bytes
        
        def read_current():
            raw = Isense.readfrom_mem(device, 0x7, 4)
            value = int.from_bytes(raw, 'big', signed=True)
            current = value * i_LSB  # Convert to amps
            return current

        #---SCAN FOR AVAILABLE DEVICES ONT HE I2C PROTOCOL---#
        device = Isense.scan()[0] #looks for devices that are connected, and returns the first in the list (i am only communicating with 1 device over the protocol)
        if device:
            print("device found:", device)
        #---SCAN FOR AVAILABLE DEVICES ONT HE I2C PROTOCOL---#

        #---SETTING THE VARIABLES REQUIRED---#
        pwm3.duty_u16(30000)
        r = 0.015#ohms #setting the resistance value for the internal shunt resistor (this is an inbuilt feature of the Adafruit breakout board)
        max_I = 0.65 #arbitrary figure roughly half the stall current
        i_LSB = max_I/2**19 #as given by page 31 of the INA228 datasheet
        calibration_const = 13107.2*(10**6)
        V_shunt = int(round(calibration_const*i_LSB*r)) #multiply by 4 if converting to using the ADC1 as per page 31 of the INA228 datasheet
        print(f"V_shunt = {V_shunt} and type {type(V_shunt)}")               
        V_LSB_step = 312.5*10**(-9)
        V_limit = max_I*r
        V_limit_LSB = int(round(V_limit/V_LSB_step*16)) #adjustment of 16 is used as a correction factor to scale up the lsb step reading, because its a 16 bit register as opposed to a 20 bit
        sovl_LSB = 0.00000125 #1.25microvolts                         
        sovl_input = V_limit_LSB*sovl_LSB
        sovl_input_test = 1
        print(f"sovl_input is: {sovl_input}")
        #---SETTING THE VARIABLES REQUIRED---#

        sleep(1)
        
        #----ADC_CONFIG REGISTER----#
        #---DATATYPE EXPECTED = 16-bit unsigned (not twos complement) integer, big endian format (MSB first)
        print("variables passed") 
        reset_hex = 0x0F00 #the hex representation of the reset for the ADC_CONFIG register
        reset_bin = int_to_bytes(reset_hex, 2) #sets the number of bytes returned to 2 (for the 16 bit register)
        Isense.writeto_mem(device,1, reset_bin)#the INA228 expects "big endian format where the most significant byte is read first"
        adc_reg_read = Isense.readfrom_mem(device, 1, 2)
        print(f"Return from the ADC register = {adc_reg_read}")
        if adc_reg_read == reset_bin:
            print("ADC set correctly - the register is returning what was input")
        #this is setting the adc_config at address 0x1 to continuous shunt voltage monitoring (bit 12-15 set as "9"). Set on the "MODE" bit
        #WHEN THE ADC CONFIG IS SET TO MONITORING THE CURRENT AND VOLTAGE, THE THRESHOLDS ON PAGE 17 ARE ARE CONSTANTLY MONITORED
        #----ADC_CONFIG REGISTER----#

        #Must set the Shunt over voltage limit in SHNTOL and that is accessed from the diag_ALRT register(RO) from "SHNTOL". the threshold is set in "SOVL"

        #---SOVL---#
        #---DATATYPE EXPECTED - 16-bit signed integer with a conversion factor of 1.25microV/LSB when in ADCRANGE 1
        #reset the SOVL
        reset_hex = 0x7FFF
        Isense.writeto_mem(device, 0xC, reset_hex.to_bytes(2, "big")) #reset the register ready for 
        Isense.writeto_mem(device, 0xC, float_to_bytes(sovl_input_test))#accesses the first result in the tuple (bytes)
        sovl_read = Isense.readfrom_mem(device, 0xC, 2).hex()
        print(f"Return from the sovl register: {sovl_read}")
        
        #---SOVL---#
        raise Exception("Program shortened for debugging - DELIBERATE")
        #---SHUNT_CAL IN ORDER TO MEASURE THE VOLTAGE ACROSS Vin+ and Vin----#
        Isense.writeto_mem(device, 0x2, int_to_bytes(V_shunt*sovl_LSB, 2))
        SHUNT_CAL_READ = Isense.readfrom_mem(device, 0x2, 2)
        print(f"LSB passed, and set to {int.from_bytes(SHUNT_CAL_READ)}")
        #----SHUNT_CAL---#


        #----CURRENT REGISTER - READ ONLY----#
        I = Isense.readfrom_mem(device, 0x7, 3)
        # Convert bytes to binary string (MicroPython-safe)
        def bytes_to_binary_string(data):
            bits = ''
            for byte in data:
                for i in range(7, -1, -1):
                    bits += str((byte >> i) & 1)
            return bits

        I_str = bytes_to_binary_string(I)  # Now a string of 0s and 1s
        I_str_twos_comp = twos_comp(I_str)

        while True:
            print(f"current read passed, the value of which is {read_current():.6f}A")
            sleep(2)
        #----CURRENT REGISTER - READ ONLY----#
        


        #----DIAG_ALRT REGISTER - WRITE TO----#
        motor1_I = Isense.readfrom_mem(device, 0xB, 2) #this is reading the diag alrt
        motor1_I = motor1_I.hex()
        print(f"motor 1 current = {motor1_I}")
        #----DIAG_ALRT REGISTER - WRITE TO----#
        
        stop = alrt.irq(trigger=alrt.IRQ_FALLING, handler=ihandler) #triggered when the alrt pin falls
        while True:
            sleep(0.5)

    except Exception as e:
        print(f"Exception = {e}")
        pwm3.duty_u16(0)
        sys.exit()
current_sense()
    

#---CURRENT SENSING TESTING---#
