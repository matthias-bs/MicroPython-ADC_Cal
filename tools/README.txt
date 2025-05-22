"Black" is used for MicroPython code style checking:
https://github.com/psf/black

"uncrustify" is NOT used in this project, thus uncrustify.cfg wil not be used!

 MicroPython-ADC_Cal/.github/workflows/code-formatting.yml calls ci.sh,
 which in turn executes codeformat.py, which will execute black.py.
 
