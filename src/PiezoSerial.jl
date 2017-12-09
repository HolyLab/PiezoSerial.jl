__precompile__()

module PiezoSerial

import Base.listen
import Base.position

using SerialPorts
using ImagineHardware

export find_piezo_serial,
        kp, ki, kd, set_kp, set_ki, set_kd,
        status,
        runtime_minutes,
        accepts_mod, mod_on, mod_off,
        position, set_position,
        is_open_loop, is_closed_loop, set_closed_loop, set_open_loop,
        is_notch_on, is_notch_off, notchf, set_notchf, notchb, set_notchb,
        is_lowpass_on, is_lowpass_off, lowpassf, set_lowpassf,
        slewrate, set_slewrate

ENTER = "\r\n" #on Windows

SUPPORTED_AMPLIFIERS = ["30DV300";]

has_serial_control(amp::SimpleAmplifier) = has_serial(name(amp))
has_serial_control(ampname::String) = in(ampname, SUPPORTED_AMPLIFIERS)

#SerialPorts.jl just uses PySerial. Here's the constructor from the PySerial docs:
#__init__(port=None, baudrate=9600, bytesize=EIGHTBITS, parity=PARITY_NONE, stopbits=STOPBITS_ONE, timeout=None, xonxoff=False, rtscts=False, write_timeout=None, dsrdtr=False, inter_byte_timeout=None)
#30DV300 amp serial port properties: 115200 baud, 8 bit, no parity, 1 stop bit, software handshake (XON / XOFF)
#...so the only extra parameters that we need to set different from the default are baudrate and xonxoff (set to true)

function piezo_serialport(portname::String)
    #modified from SerialPorts source code:
    py_ptr = SerialPorts.PySerial[:Serial](portname, 115200, xonxoff=true)
    sp = SerialPort(portname,
               115200,
               py_ptr[:bytesize],
               py_ptr[:parity],
               py_ptr[:stopbits],
               py_ptr[:timeout],
               py_ptr[:xonxoff],
               py_ptr[:rtscts],
               py_ptr[:dsrdtr], py_ptr)
    atexit(()->close(sp))
    return sp
end

#try communication
function has_piezo(p::SerialPort)
    try
        rmin = runtime_minutes(p)
        return true
    catch
        return false
    end
end

#Iterate through serial ports and return an open SerialPort to the first found with a piezo amp attached
function find_piezo_serial()
    allports = list_serialports()
    for p in allports
	prt = piezo_serialport(p)
        if has_piezo(prt)
            return prt
        else
            close(prt)
        end
    end
    error("No supported serial-capable piezo was found")
end

function wait_available(p::SerialPort, timeout = 0.8) #timeout in seconds
    t = time()
    while time() - t < timeout
        if nb_available(p) > 0
            return true
        end
        sleep(0.1)
    end
    error("Timed out")
end

function listen(p::SerialPort, timeout = 1.0; prefix = "") #timeout in seconds
	resp = listen_raw(p, timeout)
	temp =  split(resp, ENTER)
	if length(temp) !=2
		error("Expected only one press of the ENTER key in the response string.  Retry the command or try listen_raw.")
	end
	resp =  temp[1] #remove trailing newline
	if !isempty(prefix)
		if resp[1:length(prefix)] != prefix
			error("The expected prefix wasn't returned.  This was returned: $resp")
		end
		resp = resp[length(prefix)+2:end] #one extra index for the comma
	end
	return resp
end
function listen_raw(p::SerialPort, timeout= 1.0)
    wait_available(p, timeout)
    resp =  readavailable(p)
end

function status_register(piezoport::SerialPort)
	comstr = "stat"
    write(piezoport, comstr * ENTER)
    dig_str = listen(piezoport; prefix = comstr)
	return reverse(bits(parse(UInt16, dig_str))) #bstr elements 1 to 16 correspond with 0 to 15 in the 30DV300 manual
end

function status(piezoport::SerialPort)
	bstr = status_register(piezoport)
    bstr[1] == '0' ? print("Piezo is unplugged\n") : print("Piezo is plugged in\n")
    bstr[2] == '0' && bstr[3] == '0' ? print("Piezo has no measurement system\n") : begin
        bstr[2] == '1' && bstr[3] == '0' ? print("Piezo has strain gauge measurement system\n") : print("Piezo has capacitive measurement system\n") end
    bstr[4] == '0' ? print("Piezo system is closed-loop capable\n") : print("Piezo system is open-loop only\n")
    bstr[7] == '0' ? print("Piezo voltage is not enabled\n") : print("Piezo voltage is enabled\n")
    bstr[8] == '0' ? print("Piezo is set to open-loop mode\n") : print("Piezo is set to closed-loop mode\n")
    all(collect(bstr[10:12]).== '0') ? print("Built-in function generator is disabled\n") : print("Built-in function generator is enabled\n")
    bstr[13] == '0' ? print("Input notch filter is off\n") : print("Input notch filter is on\n")
    bstr[14] == '0' ? print("Input lowpass filter is off\n") : print("Input lowpass filter is on\n")
    bstr[16] == '0' ? print("Fan is off\n") : print("Fan is on\n")
    return bstr
end

function available_commands(piezoport::SerialPort)
    write(piezoport, "s" * ENTER)
	@show resp = listen_raw(piezoport)
    temp = split(resp, ['\n', '\r', ' '])
	return temp[find(x->length(x) > 0, temp)]
end

function runtime_minutes(piezoport::SerialPort)
    comstr = "rohm"
    write(piezoport, comstr * ENTER)
    return parse(Int, listen(piezoport; prefix = comstr))
end

function accepts_mod(piezoport::SerialPort)
    write(piezoport, "modon" * ENTER)
    return parse(Bool, first(listen(piezoport; prefix="modon")))
end
mod_on(piezoport::SerialPort) = write(piezoport, "modon,1" * ENTER)
mod_off(piezoport::SerialPort) = write(piezoport, "modon,0" * ENTER)

function position(piezoport::SerialPort)
    write(piezoport, "mess" * ENTER)
    return listen(piezoport; prefix="mess")
end
#Note that tp should be in microns when in closed-loop mode and in volts (-20.0 to 130.0) when in open-loop mode
function set_position(piezoport::SerialPort, p::Float64)
    tp = trunc(p, 3)
	@assert tp >= 0.0 && tp <= 800.0
    write(piezoport, "set,$tp " * ENTER)
end

#kp, ki, and kd much each lie in the range 0 to 999 according to the manual.  Yet our kd was set to -0.3 when shipped(?)
function set_kp(piezoport::SerialPort, val::Float64)
    tval = trunc(val, 3)
    write(piezoport, "kp,$(tval)" * ENTER)
end
#Getting isn't mentioned in the manual, but this works to get rather than set the value
function kp(piezoport::SerialPort)
    comstr = "kp"
    write(piezoport, comstr * ENTER)
    return parse(Float64, listen(piezoport; prefix=comstr))
end
function set_ki(piezoport::SerialPort, val::Float64)
    @assert val >=0.0 && val <= 999.0
    @show tval = trunc(val, 3)
    write(piezoport, "ki,$(tval)" * ENTER)
end
function ki(piezoport::SerialPort)
    comstr = "ki"
    write(piezoport, comstr * ENTER)
    return parse(Float64, listen(piezoport; prefix=comstr))
end
function set_kd(piezoport::SerialPort, val::Float64)
    @assert val >=0.0 && val <= 999.0
    @show tval = trunc(val, 3)
    write(piezoport, "kd,$(tval)" * ENTER)
end
function kd(piezoport::SerialPort)
    comstr = "kd"
    write(piezoport, comstr * ENTER)
    return parse(Float64, listen(piezoport; prefix=comstr))
end

is_open_loop(piezoport::SerialPort) = status_register(piezoport)[8] == '0'
is_closed_loop(piezoport::SerialPort) = !is_open_loop(piezoport)
function set_open_loop(piezoport::SerialPort)
    write(piezoport, "cl,0" * ENTER)
    sleep(0.1)
    if !is_open_loop(piezoport)
        error("Command did not succeed.")
    end
end
function set_closed_loop(piezoport::SerialPort)
    write(piezoport, "cl,1" * ENTER)
    sleep(0.1)
    if !is_closed_loop(piezoport)
        error("Command did not succeed.")
    end
end

#NOTE: was on when shipped
is_notch_off(piezoport::SerialPort) = status_register(piezoport)[13] == '0'
is_notch_on(piezoport::SerialPort) = !is_notch_off(piezoport)

#NOTE: the value when shipped was 200
function notchf(piezoport::SerialPort)
    comstr = "notchf"
    write(piezoport, comstr * ENTER)
    return parse(Int, listen(piezoport; prefix=comstr))
end
function set_notchf(piezoport::SerialPort, freq::Int) #Hz units
    @assert freq>=0 && freq <= 20000
    write(piezoport, "notchf,$(freq)" * ENTER)
    sleep(0.1)
    f = notchf(piezoport)
    if f != freq
        error("Notch frequency setting failed.  Current value is $f")
    end
end

#NOTE: the value when shipped was 100
function notchb(piezoport::SerialPort)
    comstr = "notchb"
    write(piezoport, comstr * ENTER)
    return parse(Int, listen(piezoport; prefix=comstr))
end
function set_notchb(piezoport::SerialPort, bw::Int) #Hz units
    @assert bw >=0 && bw <= 2 * notchf(piezoport)
    write(piezoport, "notchb,$(bw)" * ENTER)
    sleep(0.1)
    f = notchb(piezoport)
    if f != bw
        error("Notch bandwidth setting failed.  Current value is $f")
    end
end

#sr set/get slewrate 0.0000002 to 500.0 V/s (applied to MOD)
function slewrate(piezoport::SerialPort)
    comstr = "sr"
    write(piezoport, comstr * ENTER)
    return parse(Float64, listen(piezoport; prefix=comstr))
end
#Note: when shipped by the value was 40.0 V/s, seems a bit low?
function set_slewrate(piezoport::SerialPort, sr::Float64) #V/s units
    @assert sr >=0.0000002 && sr<=500.0
    write(piezoport, "sr,$(sr)" * ENTER)
    sleep(0.1)
    s = slewrate(piezoport)
    if s != sr
        error("Slewrate setting failed.  Current value is $s")
    end
end

#Note: this was off when shipped from the company
is_lowpass_off(piezoport::SerialPort) = status_register(piezoport)[14] == '0'
is_lowpass_on(piezoport::SerialPort) = !is_lowpass_off(piezoport)

#lpon set/get (1 on, 0 off)
function set_lowpass_on(piezoport::SerialPort)
    if is_lowpass_on(piezoport)
        warn("Lowpass filter is already on")
    end
    write(piezoport, "lpon,1" * ENTER)
    sleep(0.1)
    if !is_lowpass_on(piezoport)
        error("Failed to turn on lowpass filter")
    end
end

function set_lowpass_off(piezoport::SerialPort)
    if is_lowpass_off(piezoport)
        warn("Lowpass filter is already off")
    end
    write(piezoport, "lpon,0" * ENTER)
    sleep(0.1)
    if !is_lowpass_off(piezoport)
        error("Failed to turn off lowpass filter")
    end
end

#lpf set/get 0 to 20000
#NOTE: the value when shipped was 45 (but it was turned off)
function lowpassf(piezoport::SerialPort)
    comstr = "lpf"
    write(piezoport, comstr * ENTER)
    return parse(Int, listen(piezoport; prefix=comstr))
end
function set_lowpassf(piezoport::SerialPort, fnew::Int) #Hz units
    @assert fnew >=0 && fnew <= 20000
    write(piezoport, "lpf,$(fnew)" * ENTER)
    sleep(0.1)
    f = lowpassf(piezoport)
    if f != fnew
        error("Lowpass frequency setting failed.  Current value is $f")
    end
end

########################
#BELOW IS NOT YET TESTED
########################

#sets default values for kp, ki, kd stored in the device
set_pid_defaults(piezoport::SerialPort) = write(piezoport, "sstd" * ENTER)
#Set it back to the way it was shipped to us. (We asked for custom calibration)
#This may or may not be different from the defaults stored in the controller
function set_pid_factory(piezoport::SerialPort)
    set_kp(piezoport, -0.3)
    set_ki(piezoport, 50.0)
    set_kd(piezoport, 0.1)
end

#TODO: support the function generator commands

end #module
