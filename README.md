# PiezoSerial

Currently only one family of piezo systems is supported, the 30DV300 and 30DV50 models manufactured by Piezosystem Jena.  This package exposes most of the serial commands described in the [manual](https://github.com/HolyLab/PiezoSerial.jl/blob/master/doc/30DVxxx_OEM-Controller-Manual_20150703.pdf) for those models. The interface is documented below, but in some cases it will be necessary to refer to the manual to understand the effects of these commands.

Make sure the piezo amplifier is connected and turned on.  Then begin serial communication by calling `find_piezo_serial`, which returns an open serial port object bound to the amplifier.

Then send/receive signals by passing the serial port and relevant arguments to the functions below:

### Info
`status`: Summary of various system settings

`runtime_minutes`: Total number of minutes the system has been active since manufacture

### PID
`kp`, `ki`, `kd`, `set_kp`, `set_ki`, `set_kd`: getters and setters for PID control parameters

`set_closed_loop`, `set_open_loop`, `is_open_loop`, `is_closed_loop`: turn PID control on/off, query status

### Analog I/O settings
`accepts_mod`, `mod_on`, `mod_off`: Allow/disallow control of piezo position via voltage command over the MOD connection

`monsrc`, `set_monsrc`:  Set the source of the voltage readout over the MON connection (see manual)

### Digital filter settings
`is_notch_on`, `is_notch_off`, `set_notch_on`, `set_notch_off`: turn notch filtering of the command signal on/off

`notchf`, `set_notchf`, `notchb`, `set_notchb`:  getters and setters for notch filter frequency and bandwidth (notch should be placed at the resonant frequency, which depends on system load.  See manual and read piezo theory.)

`is_lowpass_on`, `is_lowpass_off`: turn lowpass filtering of the command signal on/off

`lowpassf`, `set_lowpassf`: get/set lowpass filter frequency

`slewrate`, `set_slewrate`: get/set slew rate of the command signal

### Other system commands
`is_fan_on`, `is_fan_off`, `set_fan_on`, `set_fan_off`: get/set amplifier fan status

`position`, `set_position`: get/set current position of the piezoelectric actuator (in most cases analog control is preferable to using these commands)
