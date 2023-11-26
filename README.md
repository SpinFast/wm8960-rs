# Wolfson WM8960 Codec Driver in Rust

A Rust driver for the common, though now obsolete Wolfson WM8960 commonly used on NXP development
boards.

The device can provides 48kHz 24bit audio ADC/DAC, a small builtin class d speaker amp, simple routing
and mixing, a I2S/TDM digital audio interface, along with an I2C control interface.

The I2C control interface has a total of 56 9bit registers with 7bit addresses (16bit total i2c transfers).

Only I2C writes are possible with the interface so the driver maintains an in memory mirror of the registers
requiring 112 bytes of memory.
