#!/usr/bin/env python3

import os,sys,struct

import u3 as u3py

class ClockStuckException(Exception): pass
class NackException(Exception): pass

# error codes from
# https://labjack.com/support/datasheets/u3/low-level-function-reference/errorcodes
# example code uses magic numbers(!)

SCRATCH_WRT_FAIL = 1
SCRATCH_ERASE_FAIL = 2
DATA_BUFFER_OVERFLOW = 3
ADC0_BUFFER_OVERFLOW = 4
FUNCTION_INVALID = 5
SWDT_TIME_INVALID = 6
XBR_CONFIG_ERROR = 7

FLASH_WRITE_FAIL = 16
FLASH_ERASE_FAIL = 17
FLASH_JMP_FAIL = 18
FLASH_PSP_TIMEOUT = 19
FLASH_ABORT_RECIEVED = 20
FLASH_PAGE_MISMATCH = 21
FLASH_BLOCK_MISMATCH = 22
FLASH_PAGE_NOT_IN_CODE_AREA = 23
MEM_ILLEGAL_ADDRESS = 24
FLASH_LOCKED = 25
INVALID_BLOCK = 26
FLASH_ILLEGAL_PAGE = 27
FLASH_TOO_MANY_BYTES = 28
FLASH_INVALID_STRING_NUM = 29

SMBUS_INQ_OVERFLOW = 32
SMBUS_OUTQ_UNDERFLOW = 33
SMBUS_CRC_FAILED = 34

SHT1x_COMM_TIME_OUT = 40
SHT1x_NO_ACK = 41
SHT1x_CRC_FAILED = 42
SHT1X_TOO_MANY_W_BYTES = 43
SHT1X_TOO_MANY_R_BYTES = 44
SHT1X_INVALID_MODE = 45
SHT1X_INVALID_LINE = 46

STREAM_IS_ACTIVE = 48
STREAM_TABLE_INVALID = 49
STREAM_CONFIG_INVALID = 50
STREAM_BAD_TRIGGER_SOURCE = 51
STREAM_NOT_RUNNING = 52
STREAM_INVALID_TRIGGER = 53
STREAM_ADC0_BUFFER_OVERFLOW = 54
STREAM_SCAN_OVERLAP = 55
STREAM_SAMPLE_NUM_INVALID = 56
STREAM_BIPOLAR_GAIN_INVALID = 57
STREAM_SCAN_RATE_INVALID = 58
STREAM_AUTORECOVER_ACTIVE = 59
STREAM_AUTORECOVER_REPORT = 60
STREAM_SOFTPWM_ON = 61
STREAM_INVALID_RESOLUTION = 63

PCA_INVALID_MODE = 64
PCA_QUADRATURE_AB_ERROR = 65
PCA_QUAD_PULSE_SEQUENCE = 66
PCA_BAD_CLOCK_SOURCE = 67
PCA_STREAM_ACTIVE = 68
PCA_PWMSTOP_MODULE_ERROR = 69
PCA_SEQUENCE_ERROR = 70
PCA_LINE_SEQUENCE_ERROR = 71
TMR_SHARING_ERROR = 72

EXT_OSC_NOT_STABLE = 80
INVALID_POWER_SETTING = 81
PLL_NOT_LOCKED = 82

INVALID_PIN = 96
PIN_CONFIGURED_FOR_ANALOG = 97
PIN_CONFIGURED_FOR_DIGITAL = 98
IOTYPE_SYNCH_ERROR = 99
INVALID_OFFSET = 100
IOTYPE_NOT_VALID = 101
INVALID_CODE = 102

UART_TIMEOUT = 112
UART_NOTCONNECTED = 113
UART_NOTENALBED = 114
I2C_BUS_BUSY = 116
TOO_MANY_BYTES = 118
TOO_FEW_BYTES = 119
DSP_PERIOD_DETECTION_ERROR = 128
DSP_SIGNAL_OUT_OF_RANGE = 129
MODBUS_RSP_OVERFLOW = 144
MODBUS_CMD_OVERFLOW = 145

class FIOPin(object):
    "Digital pin interface using the interface of nrf5x.py and drivers.py"
    def __init__(self, parent, number):
        self.parent = parent
        self.number = number

        io = parent.configIO()
        analogmask = io['FIOAnalog'] | (io['EIOAnalog'] << 8)
        # e.g., {'NumberOfTimersEnabled': 0, 'TimerCounterPinOffset': 4,
        #        'DAC1Enable': 0, 'FIOAnalog': 31, 'EIOAnalog': 0,
        #        'TimerCounterConfig': 64, 'EnableCounter1': False,
        #        'EnableCounter0': False}
        self.analog = bool(analogmask & (1<<number))

    def set(self):
        self._digital()
        self.parent.setFIOState(self.number, state=True)
    def clear(self):
        self._digital()
        self.parent.setFIOState(self.number, state=False)
    def read(self):
        self._digital()
        return bool(self.parent.getDIState(self.number))

    def _digital(self):
        if self.analog:
            self.parent.configDigital(self.number)
        self.analog = False

    @property
    def v(self):
        if self.number >= 16:
            raise NotImplemented("higher IO are digital-only")
        if not self.analog:
            self.analog = True
            self.parent.configAnalog(self.number)
        return self.parent.getAIN(self.number)

    @property
    def hiz(self):
        raise NotImplemented()

class DACPin(object):
    def __init__(self, parent, number):
        self.parent = parent
        self.number = number
        self._counts = None

    @property
    def v(self):
        return self.parent.dacBitsToVoltage(self.counts,
                                            dacNumber = self.number,
                                            is16Bits = True)

    @v.setter
    def v(self, volts):
        self.counts = self.parent.voltageToDACBits(volts,
                                                   dacNumber = self.number,
                                                   is16Bits = True)
    @property
    def counts(self):
        return self._counts

    @counts.setter
    def counts(self, c):
        c = int(round(c))

        c = max(0, c)
        c = min(0xFFFF, c)

        self._counts = c

        self.parent.getFeedback(u3py.DAC16(Dac = self.number,
                                           Value = self._counts))
class PWM(object):
    """LabJack PWM.
    Assumes it is the only timer / counter in use."""

    def __init__(self, pin):
        self.pin = pin
        self.parent = pin.parent
        self.parent.configTimerClock(TimerClockBase = 2)
        self.duty = 0.

    @property
    def duty(self):
        return (65536 - self._duty)/65536.

    @duty.setter
    def duty(self, v):
        self._duty = int(65536*(1.0-float(v)))
        if self._duty < 0:
            self._duty = 0
        if self._duty > 65536:
            self._duty = 65536
        if self._duty < 65536:
            if not self._pwming:
                self.parent.configIO(NumberOfTimersEnabled = 1,
                                     TimerCounterPinOffset = self.pin.number)
                self._pwming = True
            self.parent.getFeedback(u3py.Timer0Config(TimerMode = 0, Value = self._duty))
        else:
            self.parent.configIO(NumberOfTimersEnabled = 0)
            self.pin.clear()
            self._pwming = False

class UART(object):
    """Asynchronous receiver / transmitter using (a subset of) the interface
    of pySerial"""
    def __init__(self, txd_pin, rxd_pin=None, baudrate=None, timeout=None):

        if rxd_pin:
            assert txd_pin.parent == rxd_pin.parent # both pins must be same labjack
            assert txd_pin.number+1 == rxd_pin.number # must be order TXD, RXD.

        self.parent = txd_pin.parent

        self.parent.configIO(EnableUART = True,
                             TimerCounterPinOffset = txd_pin.number)

        self.baudrate = baudrate
        self.timeout = timeout
        self.readbuffer = b''

    @property
    def baudrate(self):
        ret = self.parent.asynchConfig(Update = False)
        def baud_from_factor(factor):
            # BaudFactor = (2**16) - 48000000//(2 * DesiredBaud)
            # BaudFactor - (2**16) = -48000000//(2 * DesiredBaud)
            # DesiredBaud * (BaudFactor - (2**16)) = -48000000//2
            # DesiredBaud = -48000000//(2 * BaudFactor - (2**16))
            return 48000000//(2 * (2**16 - factor))

        return baud_from_factor(ret['BaudFactor'])

    @baudrate.setter
    def baudrate(self, baud):
        ret = self.parent.asynchConfig(Update = True,
                                       UARTEnable = True,
                                       DesiredBaud = baud)

    def write(self, b):
        b = list(bytes(b))
        while b:
            this_b = b[:32]
            b = b[len(this_b):]
            ret = self.parent.asynchTX(this_b)
            assert ret['NumAsynchBytesSent'] == len(this_b)

            if ret['NumAsynchBytesInRXBuffer']:
                self._fill_rx_buffer()

    def read(self, size=1):
        import time
        t0 = time.time()
        while 1:
            if len(self.readbuffer) >= size:
                ret = self.readbuffer[:size]
                self.readbuffer = self.readbuffer[size:]
                return ret
            elif self.timeout is not None and time.time() > t0 + self.timeout:
                ret = self.readbuffer
                self.readbuffer = b''
                return ret

            self._fill_rx_buffer()

    def _fill_rx_buffer(self):
        rb = self.parent.asynchRX()
        ct = rb['NumAsynchBytesInRXBuffer']
        self.readbuffer += bytes(rb['AsynchBytes'][:ct])

class I2CAddressed(object):
    def __init__(self, parent, address):
        self.parent = parent
        self.address = address

    def write(self, command, data, stop=True):
        return self(I2CBytes = [command]+data,
                    NumI2CBytesToReceive = 0)

    def read(self, count, stop=True):
        return self(I2CBytes = [],
                    NumI2CBytesToReceive = count)

    def __call__(self, *args, **kwargs):
        kwargs.setdefault('Address', self.address)
        return self.parent(*args, **kwargs)

    def compat(self):
        return I2CAddressedCompat(self)

class I2CAddressedCompat(object):
    def __init__(self, parent):
        self.parent = parent

    def write(self, data):
        print('data',data)
        return self.exchange(data=data, count=0)

    def read(self, count):
        return self.exchange(data=[], count=count)

    def exchange(self, data, count):
        ret = self.parent(I2CBytes = list(data),
                          NumI2CBytesToReceive = count)
        print(ret)
        return bytes(ret['I2CBytes'])

class I2C(object):
    "LabJack I2C bus using the interface of nrf5x.py and drivers.py"
    def __init__(self, scl_pin, sda_pin):
        assert scl_pin.parent == sda_pin.parent # both pins must be same labjack
        self.parent = scl_pin.parent
        self.sda = sda_pin.number
        self.scl = scl_pin.number

    def address(self, address):
        return I2CAddressed(self, address)

    def __call__(self, *args, **kwargs):
        """ Here's a giant back door.

        Calling the object is the same as calling u3 objects's "i2c" method.

        So this is not at all the same interface as the nrf5x.py i2c objects.

        But *this* interface could be grafted on to nrf5x.py fairly easily.
        """

        kwargs.setdefault('I2CBytes',[])
        kwargs.setdefault('EnableClockStretching',True)
        kwargs.setdefault('NumI2CBytesToReceive', 0)
        #kwargs.setdefault('SpeedAdjust',200)
        #kwargs['ResetAtStart'] = True

        kwargs['SDAPinNum']=self.sda
        kwargs['SCLPinNum']=self.scl

        try:
            ret = self.parent.i2c(*args, **kwargs)
        except u3py.LowlevelErrorException as e:
            if e.errorCode==I2C_BUS_BUSY:
                raise ClockStuckException()
            else:
                raise e

        ackarray = bytes(ret['AckArray']+[0]*4)[:4]

        acked, = struct.unpack('<I', ackarray)
        bytelen = len(kwargs['I2CBytes'])

        if bytelen > 0 and bytelen <= 31:
            data = [(acked >> (b+1)) & 1 for b in range(bytelen)]
            message = {'Address':bytelen & 1,
                       'Data':[(bytelen >> (b+1)) & 1 for b in range(bytelen)]}

            # At least the address should be acknowledged.
            if 0==(acked & 1):
                raise NackException(message)

        return ret

class ManagedU3(u3py.U3):
    def __init__(self, *args, **kwargs):
        super(ManagedU3, self).__init__(*args, **kwargs)
        self.getCalibrationData()

        # assert self.deviceName in ['U3-HV']

        self.FIO = [FIOPin(self, pin) for pin in range(8)]
        self.EIO = [FIOPin(self, pin+8) for pin in range(8)]
        self.CIO = [FIOPin(self, pin+16) for pin in range(8)]

        for pin in range(8):
            setattr(self, 'FIO%d'%pin, self.FIO[pin])
        for pin in range(8):
            setattr(self, 'EIO%d'%pin, self.EIO[pin])
        for pin in range(8):
            setattr(self, 'CIO%d'%pin, self.CIO[pin])

        if self.deviceName == 'U3-HV':
            self.AIN = [self.FIO[pin] for pin in range(4)]
            for pin in range(4):
                setattr(self, 'AIN%d'%pin, self.AIN[pin])

        elif self.deviceName == 'U3-LV':
            self.AIN = [(self.FIO + self.EIO)[pin] for pin in range(8 + 8)]
            for pin in range(8 + 8):
                setattr(self, 'AIN%d'%pin, self.AIN[pin])

        else:
            raise Exception('Unknown LabJack type "%s"'%self.deviceName)

        self.DAC = [DACPin(self, pin) for pin in range(2)]
        for pin in range(2):
            setattr(self, 'DAC%d'%pin, self.DAC[pin])

    def dacBitsToVoltage(self, bits, dacNumber=0, is16Bits = False):
        """ Filling in a missing function in u3 API. This is just algrebra on voltageToDACBits() """

        if is16Bits:
            bits /= 256.

        if self.calData is not None:
            volts = (bits - self.calData['dac%sOffset' % dacNumber]) / self.calData['dac%sSlope' % dacNumber]
        else:
            volts = bits / 51.717

        return volts

    def i2c_bus(self, sda, scl):
        return I2C(sda_pin=sda, scl_pin=scl)

u3_devices = {}

def u3(*args, **kwargs):
    global u3_devices

    if ('serial' not in kwargs) or (kwargs['serial'] not in u3_devices):
        u3 = ManagedU3(*args, **kwargs)
        u3_devices[u3.serialNumber]=u3
        return u3

    return u3_devices[serial_number]

def unit_tests(lj):
    "connect FIO4 and FIO5 for these tests"

    lj.FIO5.set()
    assert lj.FIO4.read()
    lj.FIO5.clear()
    assert not lj.FIO4.read()

    lj.FIO5.set()
    assert lj.FIO4.read()
    lj.FIO5.clear()
    assert not lj.FIO4.read()

    lj.FIO5.set()
    assert lj.FIO4.v > 2.3
    lj.FIO5.clear()
    assert lj.FIO4.v < 2.5

    "connect DAC0 and AIN1 for these tests"
    def settle():
        import time
        time.sleep(0.1)

    lj.DAC0.v = 0.2
    settle()
    assert abs(lj.DAC0.v - lj.AIN1.v) < 0.1

    lj.DAC0.v = 4.2
    settle()
    assert abs(lj.DAC0.v - lj.AIN1.v) < 0.1

    lj.DAC0.v = 2.2
    settle()
    assert abs(lj.DAC0.v - lj.AIN1.v) < 0.1

    print("ok.")

if __name__=="__main__":
    #lj = u3(LJSocket="localhost:6000", serial=320042174)
    lj = u3()
    print(lj)
    print(lj.serialNumber)
    unit_tests(lj)
