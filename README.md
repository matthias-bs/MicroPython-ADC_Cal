# MicroPython-ADC_Cal
**MicroPython ESP32 library for calibrated on-chip ADC conversion**

The [Espressif IDF API Reference](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/adc.html) describes the limitations of the ESP32's built-in analog-to-digital converter - and provides a (partial) solution in terms of providing a method for improving it's accuracy by using a chip specific calibration value (or two calibration values, respectively) stored in the efuse.

This [ADC calibration scheme](https://github.com/espressif/esp-idf/blob/master/components/esp_adc_cal/esp_adc_cal_esp32.c) is (in parts) provided by the ADC1Cal class.

The following restrictions apply:
* Attenuation mode 11 dB is currently not supported<br>(In this mode, more effort in termes of programming is required and more memory would be needed.)
* Only V_ref calibration mode is supported<br>(This is due to the fact that the author does not have a proper ESP32 device to test two point calibration mode.)

`ADC1Cal` extends MicroPython's ESP32 `machine.ADC` class.

Example usage:
