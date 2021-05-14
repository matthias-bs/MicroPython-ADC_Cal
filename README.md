# MicroPython-ADC_Cal
**MicroPython ESP32 library for calibrated on-chip ADC conversion**

The [Espressif IDF API Reference](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/adc.html) describes the limitations of the ESP32's built-in analog-to-digital converter - and provides a (partial) solution in terms of providing a method for improving it's accuracy by using a chip specific calibration value (or two calibration values, respectively) stored in the efuse.

This [ADC calibration scheme](https://github.com/espressif/esp-idf/blob/master/components/esp_adc_cal/esp_adc_cal_esp32.c) is (in parts) provided by the ADC1Cal class.

The following restrictions apply:
* Attenuation mode 11 dB is currently not supported<br>(In this mode, more effort in termes of programming is required and more memory would be needed.)
* Only V_ref calibration mode is supported<br>(This is due to the fact that the author does not have a proper ESP32 device to test two point calibration mode.)

`ADC1Cal` extends MicroPython's ESP32 `machine.ADC` class. Please refer to the [MicroPython ESP32 ADC documentation](https://docs.micropython.org/en/latest/esp32/quickref.html#adc-analog-to-digital-conversion) for methods and attributes inherited from the `ADC` class.

Example usage:

        from machine import Pin
        import adc1_cal
        
        ADC_PIN   = 35                # ADC input pin no.
        DIV       = 1                 # no input devider - voltage to be measured is directly connected to analog input
        AVERAGING = 10                # no. of samples for averaging (10 si default)
        
        # vref = None -> V_ref calibration value is read from efuse
        ubatt = ADC1Cal(Pin(ADC_PIN, Pin.IN), DIV, None, AVERAGING, "ADC1 Calibrated")
        
        # set ADC result width
        ubatt.width(ADC.WIDTH_10BIT)
    
        # set attenuation
        ubatt.atten(ADC.ATTN_6DB)
    
        print('ADC Vref: {:4}mV'.format(ubatt.vref))
    
        print('Voltage:  {:4.1f}mV'.format(ubatt.voltage))

        
