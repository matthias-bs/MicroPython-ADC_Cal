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
#
# For a full discussion of the three different calibration options see [1]
#
# The V_ref calibration value can be read with the tool espefuse.py [1]
# Example:
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
#
# - Attenuation and "suggested input ranges" [1]
#   +----------+-------------+-----------------+
#   |          | attenuation | suggested range |
#   |    SoC   |     (dB)    |      (mV)       |
#   +==========+=============+=================+
#   |          |       0     |    100 ~  950   |
#   |          +-------------+-----------------+
#   |          |       2.5   |    100 ~ 1250   |
#   |   ESP32  +-------------+-----------------+
#   |          |       6     |    150 ~ 1750   |
#   |          +-------------+-----------------+
#   |          |      11     |    150 ~ 2450   |
#   +----------+-------------+-----------------+
#   |          |       0     |      0 ~  750   |
#   |          +-------------+-----------------+
#   |          |       2.5   |      0 ~ 1050   |
#   | ESP32-S2 +-------------+-----------------+
#   |          |       6     |      0 ~ 1300   |
#   |          +-------------+-----------------+
#   |          |      11     |      0 ~ 2500   |
#   +----------+-------------+-----------------+
#
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
# created: 04/2021 updated: 12/2021
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
# 20210511 Added internal reading of efuse calibration value
#          Fixed usage example
# 20210512 Modified class ADC1Cal to inherit from machine.ADC class
#          All bit widths are supported now
#          Removed rounding of the result in voltage()   
#          Added support of 0/2.5/6 dB attenuation
# 20211206 Merged pull request by codemee: added support for ATTN_11DB
#
# ToDo:
# - add support of "Two Point Calibration" 
#
###############################################################################

import machine
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
_ADC1_VREF_ATTEN_SCALE  = [57431, 76236, 105481, 196602]
_ADC1_VREF_ATTEN_OFFSET = [75, 78, 107, 142]
_VREF_REG               = _EFUSE_BLK0_RDATA4_REG
_VREF_OFFSET            = const(1100)
_VREF_STEP_SIZE         = const(7)
_VREF_FORMAT            = const(0)
_VREF_MASK              = const(0x1F)

_LUT_VREF_LOW           = const(1000)
_LUT_VREF_HIGH          = const(1200)
_LUT_ADC_STEP_SIZE      = const(64)
_LUT_POINTS             = const(20)
_LUT_LOW_THRESH         = const(2880)
_LUT_HIGH_THRESH        = (_LUT_LOW_THRESH + _LUT_ADC_STEP_SIZE)

# 20 Point lookup tables, covering ADC readings from 2880 to 4096, step size of 64
# LUT for VREF 1000mV
_lut_adc1_low = [2240, 2297, 2352, 2405, 2457, 2512, 2564, 2616, 2664, 2709,
                 2754, 2795, 2832, 2868, 2903, 2937, 2969, 3000, 3030, 3060]
# LUT for VREF 1200mV
_lut_adc1_high = [2667, 2706, 2745, 2780, 2813, 2844, 2873, 2901, 2928, 2956,
                  2982, 3006, 3032, 3059, 3084, 3110, 3135, 3160, 3184, 3209]

#################################################################################
# ADC1Cal class - ADC voltage output using V_ref calibration value and averaging
#################################################################################
class ADC1Cal(machine.ADC):
    """
    Extension of ADC class for using V_ref calibration value and averaging

    Attributes:
        name (string):      instance name (for debugging)
        _pin (int):         ADC input pin no.
        _div (float):       voltage divider (V_in = V_meas * div)
        _width (int):       encoded width of ADC result (0...3)
        _samples (int):     number of ADC samples for averaging
        vref (int):         ADC reference voltage in mV (from efuse calibration data or supplied by programmer)
        _coeff_a (float):   conversion function coefficient 'a'
        _coeff_b (float):   conversion function coefficient 'b'
    """
    def __init__(self, pin, div, vref=None, samples=10, name=""):
        """
        The constructor for Battery class.

        Parameters:
            pin (machine.Pin):      ADC input pin
            div (float):            voltage divider (V_in = V_meas * div)
            vref (int):             reference voltage (optionally supplied by programmer)
            samples (int):          number of ADC samples for averaging
            name (string):          instance name
        """
        super().__init__(pin)
        self.name     = name
        self._div     = div
        self._width   = 3
        self._samples = samples
        self.vref     = self.read_efuse_vref() if (vref is None) else vref
        self._atten   = None
        self.atten(ADC.ATTN_6DB)

    def atten(self, attenuation):
        """
        Select attenuation of input signal
        
        Parameter identical to ADC.atten()
        
        Currently ADC.ATTN_11DB is not supported!

        Parameters:
            attenuation (int): ADC.ATTN_0DB / ADC.ATTN_2_5DB / ADC.ATTN_6DB /  ADC.ATTN_11DB
        """        
        # assert (attenuation != ADC.ATTN_11DB), "Currently ADC.ATTN_11DB is not supported!"
        super().atten(attenuation)
        self._coeff_a = self.vref * _ADC1_VREF_ATTEN_SCALE[attenuation] / _ADC_12_BIT_RES
        self._coeff_b = _ADC1_VREF_ATTEN_OFFSET[attenuation]
        self._atten = attenuation
        
    def width(self, adc_width):
        """
        Select bit width of conversion result
        
        Parameter identical to ADC.width()

        Parameters:
            adc_width (int): ADC.WIDTH_9BIT / ADC.WIDTH_10BIT / BITADC.WIDTH_11BIT / ADC.WIDTH_12BIT
        """        
        assert (adc_width >= 0 and adc_width < 4), "Expecting ADC_WIDTH9 (0), ADC_WIDTH10 (1), ADC_WIDTH11 (2), or ADC_WIDTH12 (3)"
        super().width(adc_width)
        self._width = adc_width
            
    def read_efuse_vref(self):
        """
        Read V_ref calibration value from efuse (i.e. read SOC hardware register)

        Returns:
            int: calibrated ADC reference voltage (V_ref) in mV
        """        
        # eFuse stores deviation from ideal reference voltage
        ret = _VREF_OFFSET  # Ideal vref
        
        # GET_REG_FIELD():
        # https://github.com/espressif/esp-idf/blob/master/components/soc/esp32/include/soc/soc.h
        # Bit positions:
        # https://github.com/espressif/esp-idf/blob/master/components/soc/esp32/include/soc/efuse_reg.h
        # EFUSE_RD_ADC_VREF : R/W ;bitpos:[12:8] ;default: 5'b0
        bits = (machine.mem32[_VREF_REG] >> 8) & _VREF_MASK
        ret += self.decode_bits(bits, _VREF_MASK, _VREF_FORMAT) * _VREF_STEP_SIZE
        
        return ret # ADC Vref in mV

    def decode_bits(self, bits, mask, is_twos_compl):
        """
        Decode bit value from two's complement or sign-magnitude to integer

        Parameters:
            bits (int):                bit-field value
            mask (int):                bit mask
            is_twos_complement (bool): True - two's complement / False: sign-magnitude
            
        Returns:
            int: decoded value
        """      
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

    # Only call when ADC reading is above threshold
    def calculate_voltage_lut(self, adc):        
        # Get index of lower bound points of LUT
        i = int((adc - _LUT_LOW_THRESH) / _LUT_ADC_STEP_SIZE)

        # Let the X Axis be self.vref, Y axis be ADC reading, and Z be voltage
        x2dist = _LUT_VREF_HIGH - self.vref;                 # (x2 - x)
        x1dist = self.vref - _LUT_VREF_LOW;                  # (x - x1)
        y2dist = ((i + 1) * _LUT_ADC_STEP_SIZE) + _LUT_LOW_THRESH - adc;  # (y2 - y)
        y1dist = adc - ((i * _LUT_ADC_STEP_SIZE) + _LUT_LOW_THRESH);      # (y - y1)
        
        # For points for bilinear interpolation
        q11 = _lut_adc1_low[i];                # Lower bound point of _lut_adc1_low
        q12 = _lut_adc1_low[i + 1];            # Upper bound point of _lut_adc1_low
        q21 = _lut_adc1_high[i];               # Lower bound point of _lut_adc1_high
        q22 = _lut_adc1_high[i + 1];           # Upper bound point of _lut_adc1_high

        # Bilinear interpolation
        # Where z = 1/((x2-x1)*(y2-y1)) * ((q11*x2dist*y2dist) + (q21*x1dist*y2dist) + (q12*x2dist*y1dist) + (q22*x1dist*y1dist))
        voltage = (q11 * x2dist * y2dist) + (q21 * x1dist * y2dist) + (q12 * x2dist * y1dist) + (q22 * x1dist * y1dist);
        # voltage += ((_LUT_VREF_HIGH - _LUT_VREF_LOW) * _LUT_ADC_STEP_SIZE) / 2; # Integer division rounding
        voltage /= ((_LUT_VREF_HIGH - _LUT_VREF_LOW) * _LUT_ADC_STEP_SIZE);     # Divide by ((x2-x1)*(y2-y1))
        return voltage

    def interpolate_two_points(self, y1, y2, x_step, x):
        # Interpolate between two points (x1,y1) (x2,y2) between 'lower' and 'upper' separated by 'step'
        return ((y1 * x_step) + (y2 * x) - (y1 * x) + (x_step / 2)) / x_step

    def calculate_voltage_linear(self, raw_val):
        voltage = (((self._coeff_a * raw_val) + _LIN_COEFF_A_ROUND) / _LIN_COEFF_A_SCALE) + self._coeff_b
        return voltage

    @property
    def voltage(self):
        """
        Get voltage measurement [mV].

        Returns:
            float: voltage [mV]
        """
        assert (self._atten is not None), "Currently ADC.ATTN_11DB is not supported!"
        
        raw_val = 0
        
        # Read and accumulate ADC samples
        for i in range(self._samples):
            raw_val += self.read()
        
        # Calculate average
        raw_val = int(round(raw_val / self._samples))
        
        # Extend result to 12 bits (required by calibration function)
        raw_val <<= (3 - self._width)
        
        if self._atten == ADC.ATTN_11DB and raw_val >= _LUT_LOW_THRESH:  # Check if in non-linear region
        # Use lookup table to get voltage in non linear portion of ADC_ATTEN_DB_11
            lut_voltage = self.calculate_voltage_lut(raw_val)
            # If ADC is transitioning from linear region to non-linear region
            if raw_val <= _LUT_HIGH_THRESH:
                # Linearly interpolate between linear voltage and lut voltage
                linear_voltage = self.calculate_voltage_linear(raw_val)
                voltage = self.interpolate_two_points(linear_voltage, lut_voltage, _LUT_ADC_STEP_SIZE, (raw_val - _LUT_LOW_THRESH))
            else:
                voltage = lut_voltage
        else:
            # Apply calibration function
            voltage = self.calculate_voltage_linear(raw_val)
        
        # Apply external input voltage divider
        voltage = voltage / self._div
        
        return voltage

    
    def __str__(self):
        _atten = ["0dB", "2.5dB", "6dB", "11dB"]
        if (self.name != ""):
            name_str = "Name: {} ".format(self.name)
        else:
            name_str = ""
        
        raw_val = self.read()
        
        return ("{} width: {:2}, attenuation: {:>5}, raw value: {:4}, value: {}"
                .format(name_str, 9+self._width, _atten[self._atten], raw_val, self.voltage))


from time import sleep
from machine import Pin

if __name__ == "__main__":
    ADC_PIN   = 35                # ADC input pin no.
    VREF      = 1065              # V_ref in mV (device specific value -> espefuse.py --port <port> adc_info)
#    DIV       = 100 / (100 + 200) # (R1 / R1 + R2) -> V_meas = V(R1 + R2); V_adc = V(R1)  
    DIV       = 1
    AVERAGING = 10                # no. of samples for averaging
    
    adc_widths = [ADC.WIDTH_9BIT, ADC.WIDTH_10BIT, ADC.WIDTH_11BIT, ADC.WIDTH_12BIT]
    adc_atten  = [ADC.ATTN_0DB, ADC.ATTN_2_5DB, ADC.ATTN_6DB, ADC.ATTN_11DB]

    # Using programmer-supplied calibration value
    #ubatt = ADC1Cal(Pin(ADC_PIN, Pin.IN), DIV, VREF, AVERAGING, "ADC1 User Calibrated")

    # Using efuse calibration value
    ubatt = ADC1Cal(Pin(ADC_PIN, Pin.IN), DIV, None, AVERAGING, "ADC1 eFuse Calibrated")

    # Test all supported attenuation/width permutations
    for attenuation in adc_atten:
        # set attenuation
        ubatt.atten(attenuation)

        for width in adc_widths:
            # set ADC result width
            ubatt.width(width)
       
            # Print object info
            print(ubatt)
    
    # set ADC result width
    ubatt.width(ADC.WIDTH_10BIT)
    
    # set attenuation
    ubatt.atten(ADC.ATTN_6DB)
    
    print()
    print('ADC Vref: {:4}mV'.format(ubatt.vref))
    print()
    
    while 1:
        print('Voltage:  {:4.1f}mV'.format(ubatt.voltage))
        sleep(5)
