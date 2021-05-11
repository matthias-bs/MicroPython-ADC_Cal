###############################################################################
# adc1_cal.py
#
# This module provides the ADC1Cal class
#
# MicroPython ESP32 ADC1 conversion using V_ref calibration value 
#
# The need for calibration is described in [1] and [4].
#
# Limitations of the current implementation ("works for me"):
# - only ADC1 is supported (as the name says)
# - only "V_ref"-calibration
# - only ADC width: 10 bits
# - only attenuator setting: 6 dB
#
# For a full discussion of the three different calibration options see [1]
#
# The V_ref calibration value can be read with the tool espefuse.py [1], e.g.:
# $ espefuse.py --port <port> adc_info
# Detecting chip type... ESP32
# espefuse.py v3.0
# ADC VRef calibration: 1065mV
#
# This is now done in the constructor if its argument <vref> is None.
#
# The ESP32 documentation is very fuzzy concerning the ADC input range,
# full scale value or LSB voltage, respectively.
# The MicroPython quick reference [3] is also (IMHO) quite misleading. 
# A good glimpse is provided in [4]. 
# 
# - "Per design the ADC reference voltage is 1100 mV, however the true
#   reference voltage can range from 1000 mV to 1200 mV amongst different
#   ESP32s." [1]
# - according to [1], ESP32 provides a "suggested input range" of 150 ~ 1750mV
#   with attenuation of 6dB.
#
# Please refer to the section "Minimizing Noise" in [1].
#
# The calibration algorithm and constants are based on [2].
#
# [1] https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/adc.html#adc-calibration
# [2] https://github.com/espressif/esp-idf/blob/master/components/esp_adc_cal/esp_adc_cal_esp32.c
# [3] https://docs.micropython.org/en/latest/esp32/quickref.html#adc-analog-to-digital-conversion
# [4] https://esp32.com/viewtopic.php?t=1045 ([Answered] What are the ADC input ranges?)
#
# created: 04/2021 updated: 05/2021
#
# This program is Copyright (C) 04/2021 Matthias Prinke
# <m.prinke@arcor.de> and covered by GNU's GPL.
# In particular, this program is free software and comes WITHOUT
# ANY WARRANTY.
#
# History:
#
# 20210418 Created
# 20210510 Ported calibration from esp_adc_cal_esp32.c [2]
# 20210511 Added using of efuse calibration value
#          Fixed usage example
#
# ToDo:
# - rewrite to inherit from machine.ADC class
# - add support of "Two Point Calibration"
# - add support of other bit widths
#   (What's the point in 12 bits with this noise level?)
# - add support of other attenuator settings
#   (What's the point in the 11dB setting with its crooked characteristics?) 
#
###############################################################################

import machine
from machine import Pin
from machine import ADC

# Constant from
# https://github.com/espressif/esp-idf/blob/master/components/soc/esp32/include/soc/soc.h
_DR_REG_EFUSE_BASE      = const(0x3ff5A000)

# Constants from 
# https://github.com/espressif/esp-idf/blob/master/components/soc/esp32/include/soc/efuse_reg.h
_EFUSE_ADC_VREF         = const(0x0000001F)
_EFUSE_BLK0_RDATA4_REG  = (_DR_REG_EFUSE_BASE + 0x010)

# Constants from
# esp_adc_cal_esp32.c
_ADC_12_BIT_RES         = const(4096)
_LIN_COEFF_A_SCALE      = const(65536)
_LIN_COEFF_A_ROUND      = const(32768) # LIN_COEFF_A_SCALE/2
_ADC1_VREF_ATTEN_SCALE  = const(105481)
_ADC1_VREF_ATTEN_OFFSET = const(107)
_VREF_REG               = _EFUSE_BLK0_RDATA4_REG
_VREF_OFFSET            = const(1100)
_VREF_STEP_SIZE         = const(7)
_VREF_FORMAT            = const(0)
_VREF_MASK              = const(0x1F)


#################################################################################
# ADC1Cal class - ADC voltage output using V_ref calibration value and averaging
#################################################################################
class ADC1Cal:
    """
    Get battery voltage.

    Attributes:
        name (string):      instance name (for debugging)
        _pin (int):         ADC input pin no.
        _div (float):       voltage divider (V_in = V_meas * div)
        _samples (int):     number of ADC samples for averaging
        vref (int):         ADC reference voltage in mV (from efuse calibration data or supplied by programmer)
        _coeff_a (float):   conversion function coefficient 'a'
        _coeff_b (float):   conversion function coefficient 'b'
        _adc (machine.ADC): ADC object
    """
    def __init__(self, pin_no, div, vref=None, samples=10, name=""):
        """
        The constructor for Battery class.

        Parameters:
            pin_no (int):           ADC input pin no.
            div (float):            voltage divider (V_in = V_meas * div)
            vref (int):             reference voltage (supplied by programmer)
            samples (int):          number of ADC samples for averaging
            name (string):          instance name
        """
        self.name     = name
        self._pin     = pin_no
        self._div     = div 
        self._samples = samples
        self.vref     = self.read_efuse_vref() if (vref is None) else vref
        self._coeff_a = self.vref * _ADC1_VREF_ATTEN_SCALE / _ADC_12_BIT_RES
        self._coeff_b = _ADC1_VREF_ATTEN_OFFSET
        
            
        # create ADC object on GPIO pin
        self._adc = ADC(Pin(self._pin, Pin.IN))
        
        # 6dB attenuation
        # -> "suggested input range: 150 ~ 1750mV [1]
        self._adc.atten(ADC.ATTN_6DB)
        
        # set 10 bit return values (returned range 0-1023)
        self._adc.width(ADC.WIDTH_10BIT)

    def read_efuse_vref(self):
        # eFuse stores deviation from ideal reference voltage
        ret = _VREF_OFFSET  # Ideal vref
        # GET_REG_FIELD():
        # https://github.com/espressif/esp-idf/blob/master/components/soc/esp32/include/soc/soc.h
        # Bit positions:
        # https://github.com/espressif/esp-idf/blob/master/components/soc/esp32/include/soc/efuse_reg.h
        # EFUSE_RD_ADC_VREF : R/W ;bitpos:[12:8] ;default: 5'b0
        bits = (machine.mem32[_VREF_REG] >> 8) & _VREF_MASK
        ret += self.decode_bits(bits, _VREF_MASK, _VREF_FORMAT) * _VREF_STEP_SIZE;
        return ret # ADC Vref in mV

    def decode_bits(self, bits, mask, is_twos_compl):
        if (bits & ~(mask >> 1) & mask): # Check sign bit (MSB of mask)
            # Negative
            if (is_twos_compl):
                ret = -(((~bits) + 1) & (mask >> 1))  # 2's complement
            else:
                ret = -(bits & (mask >> 1))     # Sign-magnitude
        else: 
            # Positive
            ret = bits & (mask >> 1)
    
        return ret

    
    @property
    def voltage(self):
        """
        Get voltage measurement [mV].

        Returns:
            int: voltage [mV]
        """
        raw_val = 0
        
        for i in range(self._samples):
            raw_val += self._adc.read()
            
        raw_val = int(round(raw_val / self._samples))
        raw_val <<= 2
        voltage = (((self._coeff_a * raw_val) + _LIN_COEFF_A_ROUND) / _LIN_COEFF_A_SCALE) + self._coeff_b
        voltage = round(voltage / self._div)
        
        return voltage

    
    def __str__(self):
        if (self.name != ""):
            name_str = "Name: {} ".format(self.name)
        else:
            name_str = ""
        
        raw_val = self._adc.read()
        
        return ("{}Pin# {:2}, raw value: {}, value: {}"
                .format(name_str, self._pin, raw_val, self.voltage))


from time import sleep

if __name__ == "__main__":
    ADC_PIN   = 35                # ADC input pin no.
    VREF      = 1065              # V_ref in mV (device specific value -> espefuse.py --port <port> adc_info)
    DIV       = 100 / (100 + 200) # (R1 / R1 + R2) -> V_meas = V(R1 + R2); V_adc = V(R1)  
    AVERAGING = 10                # no. of samples for averaging

    # Using programmer-supplied calibration value
    ubatt = ADC1Cal(ADC_PIN, DIV, VREF, AVERAGING, "ADC1 User Calibrated")

    # Using efuse calibration value
    ubatt = ADC1Cal(ADC_PIN, DIV, None, AVERAGING, "ADC1 eFuse Calibrated")
    
    print('ADC Vref: {:4}mV'.format(ubatt.vref))
    
    while 1:
        print('Voltage:  {:4}mV'.format(ubatt.voltage))
        sleep(5)
