#__precompile__()

#module PiezoSerial

using SerialPorts
using ImagineHardware


SUPPORTED_AMPLIFIERS = ["30DV300";]

has_serial_control(amp::SimpleAmplifier) = has_serial(name(amp))
has_serial_control(ampname::String) = in(ampname, SUPPORTED_AMPLIFIERS)

#SerialPorts.jl just uses PySerial. Here's the constructor from the PySerial docs:
#__init__(port=None, baudrate=9600, bytesize=EIGHTBITS, parity=PARITY_NONE, stopbits=STOPBITS_ONE, timeout=None, xonxoff=False, rtscts=False, write_timeout=None, dsrdtr=False, inter_byte_timeout=None)

#30DV300 amp serial port properties:
#115200 baud, 8 bit, no parity, 1 stop bit, software handshake (XON / XOFF)

#...so the only extra parameters that we need to set different from the default are budrate and xonxoff (set to true)

function piezo_serialport(portname::String)
    #modified from SerialPorts source code:
    py_ptr = PySerial[:Serial](portname, 115200, xonxoff=true)
    sp = SerialPort(portname,
               115200,
               py_ptr[:bytesize],
               py_ptr[:parity],
               py_ptr[:stopbits],
               py_ptr[:timeout],
               py_ptr[:xonxoff],
               py_ptr[:rtscts],
               py_ptr[:dsrdtr], py_ptr)
    atexit(close(sp))
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

function listen(p::SerialPort, timeout = 1.0) #timeout in seconds
    wait_available(p, timeout)
    return readavailable(p)
end

function status(piezo_port::SerialPort)
    write(piezoport, "stat\n")
    resp = listen(piezoport) #This should be a two-element Vector{UInt8}
    if length(resp) != 2
        error("Expected a 16-bit status register but received $(length(resp)) bytes")
    end
    bstr = reverse(bits(reinterpret(UInt16, resp)[1]))
    #References 30DV300 manual
    bstr[1] == '0' ? print("Piezo is unplugged\n") : print("Piezo is plugged in\n")
    bstr[2] == '0' && bstr[3] == '0' ? print("Piezo has no measurement system\n") : begin
        bstr[2] == '1' && bstr[3] == '0' ? print("Piezo has strain gauge measurement system\n") : print("Piezo has capacitive measurement system\n") end
    bstr[4] == '0' ? print("Piezo system is closed-loop capable\n") : print("Piezo system is open-loop only\n")
    bstr[6] == '0' ? print("Piezo voltage is not enabled\n") : print("Piezo voltage is enabled\n")
    bstr[7] == '0' ? print("Piezo is set to open-loop mode\n") : print("Piezo is set to closed-loop mode\n")
    all(bstr[9:11].== '0') ? print("Built-in function generator is disabled\n") : print("Built-in function generator is enabled\n")
    bstr[12] == '0' ? print("Input notch filter is off\n") : print("Input notch filter is on\n")
    bstr[13] == '0' ? print("Input lowpass filter is off\n") : print("Input lowpass filter is on\n")
    bstr[15] == '0' ? print("Fan is off\n") : print("Fan is on\n")
    return bstr
end

function available_commands(piezoport::SerialPort)
    write(piezoport, "s\n")
    return listen(piezoport)
end

function runtime_minutes(piezoport::SerialPort)
    write(piezoport, "rohm\n")
    return listen(piezoport)
end

function position(piezoport::SerialPort)
    write(piezoport, "mess\n")
    return listen(piezoport)
end
#Note that tp should be in microns when in closed-loop mode and in volts (-20.0 to 130.0) when in open-loop mode
function set_position(piezoport::SerialPort, p::Float64)
    tp = trunc(p, 3)
    write(piezoport, "set,$tp\n")
end

#kp, ki, and kd much each lie in the range 0 to 999
function set_kp(piezoport::SerialPort, val::Float64)
    @assert val >=0.0 && val <= 999.0
    tval = trunc(val, 3)
    write(piezoport, "kp,$(tval)\n")
end
#Getting isn't mentioned in the manual, but guessing this will get rather than set the value
function kp(piezoport::SerialPort)
    write(piezoport, "kp\n")
    return listen(piezoport)
end

#sets default values for kp, ki, kd stored in the device
set_pid_defaults(piezoport::SerialPort) = write(piezoport, "sstd\n")

function set_open_loop(piezoport::SerialPort)
    write(piezoport, "cl,0")
    #todo: verify write success
end
function set_closed_loop(piezoport::SerialPort)
    write(piezoport, "cl,1")
    #todo: verify write success
end

function notchf(piezoport::SerialPort)
    write(piezoport, "notchf\n")
    return listen(piezoport)
end
function set_notchf(piezoport::SerialPort, freq::Int) #Hz units
    @assert notchf>=0 && notchf <= 20000
    write(piezoport, "notchf,$(freq)\n")
    #todo: verify success
end

#others todo:
#sr set/get slewrate 0.0000002 to 500.0 V/s (applied to MOD)
#notchb set/get 0 to max of 2 * notch frequency
#lpon set/get (0 on, 1 off)
#lpf set/get 0 to 20000

#end #module
