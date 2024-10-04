#!/usr/bin/env python3

import intelhex
import time
from .u3_manager import u3_manager
import itertools
from u3 import BitStateWrite, BitStateRead, WaitShort

"""

Program PIC microprocessors.

I hooked up like so:

| signal  | pin | color  | labjack |
|---------+-----+--------+---------|
| !MCLR   |   1 | brown  | FIO4    |
| +3V3    |   2 | red    | VS      |
| GND     |   3 | orange | GND     |
| ICSPDAT |   4 | yellow | FIO5    |
| ICSPCLK |   5 | green  | FIO6    |
| PGM     |   6 | blue   | --      |

"""

# this spaces out the timing so transactions are recognizable on the
# logic analyzer.  Speed penalty about ~2x 
dbg_flush = False

def dummy_print(*args, **kwargs): pass

# debug printing function, set to "print" for debug.
dbg_print = dummy_print

def reverse_bits(bit_count, val):
    ret = 0
    for x in range(bit_count):
        ret <<= 1
        ret |= val & 1
        val >>= 1
    return ret

class Pic:
    USERID_ADDR = 0x8000
    REVID_ADDR = 0x8005
    DEVID_ADDR = 0x8006

    def __init__(self,
                 mclr_pin=None,
                 icspdat_pin=None,
                 icspclk_pin=None,
                 skip_mclr = False,
                 invert_mclr = False):
        self.mclr = mclr_pin
        self.dat = icspdat_pin
        self.clk = icspclk_pin
        self.invert_mclr = invert_mclr

        # "set" means asserted
        if invert_mclr:
            self.mclr_set = self.mclr_high
            self.mclr_clr = self.mclr_low
        else:
            self.mclr_set = self.mclr_low
            self.mclr_clr = self.mclr_high

        if not skip_mclr:
            self.mclr_clr()

        self.dat.clear()
        self.clk.clear()

        self.tx_bits = 0
        self.tx_bits_ct = 0


    def mclr_high(self):
        self.mclr.set()
    def mclr_low(self):
        self.mclr.clear()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, exc_traceback):
        if not exc_type:
            self.flush_tx()

    @property
    def name(self):
        return self.__class__.__name__

    def send_value_msb(self, width, val):
        """ prepares to clock out value msb-first 

        val is value to send (integer)
        width is number of bits to send
        """        
        return self.send_value_lsb(width, reverse_bits(width, val))
        
    def send_value_lsb(self, width, val):
        """ prepares to clock out value lsb-first 

        val is value to send (integer)
        width is number of bits to send
        """
        dbg_print(f"sending {width} bits {val:x}")
        
        assert val >= 0
        mask = (1 << width) - 1
        assert 0 == val & (~mask)
        
        self.tx_bits |= val << self.tx_bits_ct
        self.tx_bits_ct += width
                
    def _flush_tx_bits(self):

        lj = self.clk.parent
        fb_commands = []

        # I measure 9 us between command executions
        # setup and hold times are 100 ns, so no need to add delays
        
        dbg_print(f"bit flushing {self.tx_bits_ct} bits {self.tx_bits:x}")
        for bit_num in range(self.tx_bits_ct):
            bit = self.tx_bits & 1
            fb_commands += [                
                BitStateWrite(self.dat.number, bit),
                BitStateWrite(self.clk.number, 1),
                # WaitShort(Time = setup_time/128e-6)
                BitStateWrite(self.clk.number, 0)
                # WaitShort(Time = hold_time/128e-6)
            ]
            self.tx_bits >>= 1
        self.tx_bits_ct = 0
            
        lj.getFeedback(fb_commands)

    def _flush_tx_spi(self):

        dbg_print(f"spi flushing {self.tx_bits_ct} bits {self.tx_bits:x}")
        
        spi_bytes = []
        while self.tx_bits_ct >= 8:
            spi_bytes.append(reverse_bits(8, self.tx_bits & 0xff))
            self.tx_bits >>= 8
            self.tx_bits_ct -= 8
    
        unused = 0xff
        lj = self.clk.parent

        chunklen = 50

        for pos in range(0, len(spi_bytes), chunklen):
            lj.spi(SPIBytes = spi_bytes[pos:pos+chunklen],
                   CLKPinNum = self.clk.number,
                   MOSIPinNum = self.dat.number,
                   MISOPinNum = unused,
                   CSPinNum = unused,
                   SPIMode='B',
                   AutoCS=False)

    def flush_tx(self):
        self._flush_tx_spi()
        self._flush_tx_bits()
            
    def read_value_lsb(self, width):
        """ width is number of bytes to read (lsb-first)
        return value is integer
        """        
        ret = self.read_value_msb(width)
        return reverse_bits(8*width, ret)

    def read_value_msb(self, width):
        """ width is number of bytes to read (msb-first)
        return value is integer
        """
        self.flush_tx()

        unused = 0xff
        lj = self.clk.parent
        spi = lj.spi(SPIBytes = [0]*width,
                     CLKPinNum = self.clk.number,
                     MOSIPinNum = unused,
                     MISOPinNum = self.dat.number,
                     CSPinNum = unused,
                     SPIMode='B',
                     AutoCS=False)
        ret = 0
        for i in range(width):
            ret <<= 8
            ret += spi['SPIBytes'][i]

        dbg_print("read %x"%ret)
        return ret

    def sleep(self, timeout):
        if timeout > 4e-6: # timeouts less than this are taken-care of
                           # by the slow SPI clock
            self.flush_tx()
            time.sleep(timeout)

class Pic16F_Enhanced_Midrange(Pic):

    USER_ADDR = 0x8000

    CONFIG_ADDR = 0x8007
    CONFIG_LEN = 0xC
    DIA_ADDR = 0x8100
    DIA_LEN = 0x40
    EEPROM_ADDR = 0xF000
    EEPROM_LEN = 0x100

    CONFIG_LEN = 5
    
    KEY = 0x4d434850 # 'MCHP'

    Tenth = 250e-6
    Tents = 100e-9
    Tckh = 100e-9
    Tckl = 100e-9
    Tdly = 1e-6
    Tpint = 0.002 # program memory
    Tpint_config = 0.0056 # config memory

    def send8_24(self, d1, d2):
        self.send8(d1)
        self.sleep(self.Tdly)
        self.send24(d2<<1)
        self.sleep(self.Tdly)

    def send32(self, d): self.send_value_msb(32, d)
    def send16(self, d): self.send_value_msb(16, d)
    def send24(self, d): self.send_value_msb(24, d)
    def send8(self, d): self.send_value_msb(8, d)

    def read24(self):
        start_bit = (1<<31)
        stop_bit = (1<<0)
        bits_mask = ~(start_bit | stop_bit)

        return (self.read_value_msb(24//8) & bits_mask) >> 1

    def enter_lvp(self):
        self.mclr_set()
        self.sleep(self.Tenth)
        self.send32(self.KEY)
        self.sleep(self.Tenth)

    def set_PC(self, pc):
        self.pc = pc
        self.send8_24(0x80, pc)

    def increment_pc(self):
        self.pc += 1
        self.send8(0xf8)

    def load_mem(self, d, inc=False):
        "loads one word of data from PC into buffer on target"
        self.send8_24([0x00, 0x02][inc], d)
        self.pc += inc

    def write_row(self):
        self.send8(0xe0) # internally timed
        if self.pc < self.USER_ADDR:
            self.sleep(self.Tpint)
        else:
            self.sleep(self.Tpint_config)

    def read_mem(self, inc=True):
        if inc:
            self.send8(0xfe)
        else:
            self.send8(0xfc)

        self.sleep(self.Tdly)
        ret = self.read24()
        self.pc += inc
        return ret

    def read_word(self, address):
        self.set_PC(address)
        return self.read_mem()

    def read_device_id(self):
        return self.read_word(self.DEVID_ADDR)

    def read_config_stuff(self):
        mmap = (("user id 0", 0x8000),
                ("user id 1", 0x8001),
                ("user id 2", 0x8002),
                ("user id 3", 0x8003),
                ("reserved", 0x8004),
                ("revision id", 0x8005),
                ("device id", 0x8006),
                ("config 0", 0x8007),
                ("config 1", 0x8008),
                ("config 2", 0x8009),
                ("config 3", 0x800a),
                ("config 4", 0x800b))
        for name,addr in mmap:
            dbg_print(name, "@%04x:"%addr, "%04x"%self.read_word(addr))

    def exit_lvp(self):
        self.flush_tx()
        self.mclr_clr()

    def bulk_erase(self,
                   erase_eeprom = False,
                   erase_flash = False,
                   erase_user_id = False,
                   erase_config = False):
        cmd = sum([erase_eeprom << 0,
                   erase_flash << 1,
                   erase_user_id << 2,
                   erase_config << 3])

        #dbg_print("Erase cmd:",cmd)

        self.send8_24(0x18, cmd)
        self.sleep(self.Terab)

    @property
    def Terab(self):
        if self.FLASH_LEN <= 8192:
            return 0.010
        else:
            return 0.013

    def read_flash(self, start_addr = None, length = None):
        if start_addr is None:
            start_addr = 0
        if length is None:
            length = self.FLASH_LEN

        self.set_PC(start_addr)
        return [self.read_mem() for _ in range(length)]

    def write_flash(self, data, start_addr = None):
        if start_addr is None:
            start_addr = 0

        # This is the best way to write rows.
        # Any partial-row algo risks writing garbage left over in row latch

	# See note on p19 of spec about write sizes for different algorithms.
        if start_addr < self.FLASH_LEN:
            row_size = 32
        else:
            row_size = 1

        front_pad = start_addr % row_size
        start_addr -= front_pad
        back_pad = row_size - 1 - ((len(data)+row_size-1) % row_size)

        # dbg_print("data",data)
        # dbg_print("pads",front_pad, back_pad)

        data = [0x3fff] * front_pad + data + [0x3fff] * back_pad

        self.set_PC(start_addr)

        while data:
            row = data[:row_size]
            data = data[row_size:]

            for d in row[:(row_size-1)]:
                self.load_mem(d, inc=True)

            self.load_mem(row[row_size-1], inc=False)
            self.write_row()

            #dbg_print("write row @pc=%04x"%self.pc)
            self.increment_pc()

    @property
    def memory_map(self):
        """ Returns list of tuples:
        (start, stop, name, erase block)

        name is a string, one of "program", "config", "user", "eeprom"

        """

        return [
            (0, self.FLASH_LEN, "program", 32),
            (0x8000, 0x8004, "user", 4),
            (0x8007, 0x8007 + self.CONFIG_LEN, "config", 4),
            (0xf000, 0xf100, "eeprom", 1),
        ]

class Pic16F_40002317(Pic16F_Enhanced_Midrange):
    """
    Programming spec: 40002317

    https://ww1.microchip.com/downloads/aemDocuments/documents/MCU08/ProductDocuments/ProgrammingSpecifications/PIC16F180XX-Family-Programming-Specification-40002317.pdf
    """

class Pic16F18013(Pic16F_40002317):
    FLASH_LEN = 2*1024 # words
class Pic16F18014(Pic16F_40002317):
    FLASH_LEN = 4*1024
class Pic16F18015(Pic16F_40002317):
    FLASH_LEN = 8*1024
class Pic16F18023(Pic16F_40002317):
    FLASH_LEN = 2*1024
class Pic16F18024(Pic16F_40002317):
    FLASH_LEN = 4*1024
class Pic16F18025(Pic16F_40002317):
    FLASH_LEN = 8*1024
class Pic16F18026(Pic16F_40002317):
    FLASH_LEN = 16*1024
class Pic16F18044(Pic16F_40002317):
    FLASH_LEN = 4*1024
class Pic16F18045(Pic16F_40002317):
    FLASH_LEN = 8*1024
class Pic16F18046(Pic16F_40002317):
    FLASH_LEN = 16*1024
class Pic16F18054(Pic16F_40002317):
    FLASH_LEN = 4*1024
class Pic16F18055(Pic16F_40002317):
    FLASH_LEN = 8*1024
class Pic16F18056(Pic16F_40002317):
    FLASH_LEN = 16*1024
class Pic16F18074(Pic16F_40002317):
    FLASH_LEN = 4*1024
class Pic16F18075(Pic16F_40002317):
    FLASH_LEN = 8*1024
class Pic16F18076(Pic16F_40002317):
    FLASH_LEN = 16*1024

class Pic16F_40002266(Pic16F_Enhanced_Midrange):
    """
    Programming spec: 40002266

    https://ww1.microchip.com/downloads/aemtest/MCU08/ProductDocuments/ProgrammingSpecifications/PIC16F171XX-Family-Programming-Specification-40002266.pdf
    """
    Tpint = 0.0028 # program memory

class Pic16F17114(Pic16F_40002266):
    FLASH_LEN = 4*1024
class Pic16F17115(Pic16F_40002266):
    FLASH_LEN = 8*1024
class Pic16F17124(Pic16F_40002266):
    FLASH_LEN = 4*1024
class Pic16F17125(Pic16F_40002266):
    FLASH_LEN = 8*1024
class Pic16F17126(Pic16F_40002266):
    FLASH_LEN = 16*1024
class Pic16F17144(Pic16F_40002266):
    FLASH_LEN = 4*1024
class Pic16F17145(Pic16F_40002266):
    FLASH_LEN = 8*1024
class Pic16F17146(Pic16F_40002266):
    FLASH_LEN = 16*1024
class Pic16F17154(Pic16F_40002266):
    FLASH_LEN = 4*1024
class Pic16F17155(Pic16F_40002266):
    FLASH_LEN = 8*1024
class Pic16F17156(Pic16F_40002266):
    FLASH_LEN = 16*1024
class Pic16F17174(Pic16F_40002266):
    FLASH_LEN = 4*1024
class Pic16F17175(Pic16F_40002266):
    FLASH_LEN = 8*1024
class Pic16F17176(Pic16F_40002266):
    FLASH_LEN = 16*1024

class Pic16F_XLP(Pic16F_Enhanced_Midrange):
    # these from Table 4-1
    COMMAND_LOAD_CONFIG = 0x00
    COMMAND_LOAD_DATA = 0x02
    COMMAND_READ_DATA = 0x04
    COMMAND_INC_ADDRESS = 0x06
    COMMAND_RESET_ADDRESS = 0x16
    COMMAND_BEGIN_PROG_INT = 0x08
    COMMAND_BEGIN_PROG_EXT = 0x18
    COMMAND_END_PROG_EXT = 0x0a
    COMMAND_BULK_ERASE = 0x09
    COMMAND_ROW_ERASE = 0x11

    CONFIG_LEN = 2
    
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.pc = 99999999

    def increment_pc(self):
        self.send6_le(self.COMMAND_INC_ADDRESS) 
        self.pc += 1

    def set_PC(self, address):
        
        dbg_print("setting PC=%04x to %04x"%(self.pc, address))
        if address >= 0x8000: # config data
            if (self.pc < 0x8000) or (self.pc > address):
                dbg_print("  loading config")
                self.send6_le(self.COMMAND_LOAD_CONFIG)
                self.send16_le(0)
                self.pc = 0x8000
        else: # not config data
            if self.pc > address:
                dbg_print("  resetting address")
                self.send6_le(self.COMMAND_RESET_ADDRESS)
                self.pc = 0

        while self.pc < address:
            self.increment_pc()

    def read_mem(self, inc=True):
        self.send6_le(self.COMMAND_READ_DATA)
        self.sleep(self.Tdly)
        ret = self.read16_le()

        if inc:
            self.increment_pc()

        return ret

    def read16_le(self):
        "fourteen bits packed into sixteen clocks"
        start_bit = (1<<15)
        stop_bit = (1<<0)
        bits_mask = ~(start_bit | stop_bit)

        ret = (self.read_value_lsb(16//8) & bits_mask) >> 1
        return ret

    def send32_le(self, d):
        self.send_value_lsb(32, d)

    def send16_le(self, d):
        "fourteen bits packed into sixteen clocks"
        return self.send_value_lsb(16, (d & 0x3fff) << 1)

    def send6_le(self, cmd):
        ret = self.send_value_lsb(6, cmd)
        if dbg_flush: self.flush_tx()
        return ret
    
    def enter_lvp(self):
        self.mclr_set()
        self.sleep(self.Tenth)
        self.send32_le(self.KEY)
        self.send_value_lsb(1, 0)  # 33 bits!
        self.sleep(self.Tenth)

    def load_prog_mem(self, d, inc):
        dbg_print("load_prog %04x %03x"%(self.pc, d))
        self.send6_le(self.COMMAND_LOAD_DATA)        
        self.send16_le(d)
        if inc:
            self.increment_pc()

    def load_config_mem(self, d, inc):
        addr = self.pc
        
        dbg_print("load_config %04x %03x"%(self.pc, d))
        self.send6_le(self.COMMAND_LOAD_CONFIG)
        self.send16_le(d)
        self.pc = 0x8000
        while self.pc < addr:
            self.increment_pc()
            
        if inc:
            self.increment_pc()
            
    def load_mem(self, d, inc=False):
        "loads one word of data from PC into buffer on target"
        if self.pc >= 0x8000:
            return self.load_config_mem(d, inc)
        else:
            return self.load_prog_mem(d, inc)

    def write_row_prog_int(self):
        self.send6_le(self.COMMAND_BEGIN_PROG_INT)
        self.sleep(self.Tpint)

    def write_row_prog_ext(self):
        """not measurably faster than internal
        (due to USB timing no doubt.)"""
        self.send6_le(self.COMMAND_BEGIN_PROG_EXT)
        self.sleep(self.Tpext_min)
        self.send6_le(self.COMMAND_END_PROG_EXT)
        self.sleep(self.Tdis)      
                
    def write_row_config(self):
        self.send6_le(self.COMMAND_BEGIN_PROG_INT)
        self.sleep(self.Tpint_config)
            
    def write_row(self):
        " Write a row of data in one shot "
        if self.pc >= 0x8000:
            return self.write_row_config()
        else:
            return self.write_row_prog_int()
        
    def bulk_erase(self,
                   erase_eeprom = False,
                   erase_flash = False,
                   erase_user_id = False,
                   erase_config = False):

        if (erase_flash and
            (not erase_user_id) and
            erase_config):
            self.set_PC(0)
            self.send6_le(self.COMMAND_BULK_ERASE)

        elif (erase_flash and
              erase_user_id and
              erase_config):
            self.set_PC(0x8000)
            self.send6_le(self.COMMAND_BULK_ERASE)

        else:
            raise Exception("This bulk erase is not possible")
                    
        self.sleep(self.Terab)


class Pic16F_40001683(Pic16F_XLP):
    """
    Programming spec: 40001683

    https://ww1.microchip.com/downloads/en/DeviceDoc/40001683B.pdf
    """

    Tpint = 0.0025 # program memory
    Tpint_config = 0.005 # config memory
    Tpext_min = 0.001
    Tdis = 0.0003
    
class Pic16F1703(Pic16F_40001683):
    FLASH_LEN = 0x0800
class Pic16F1704(Pic16F_40001683):
    FLASH_LEN = 0x1000
class Pic16F1705(Pic16F_40001683):
    FLASH_LEN = 0x2000
class Pic16F1707(Pic16F_40001683):
    FLASH_LEN = 0x0800
class Pic16F1708(Pic16F_40001683):
    FLASH_LEN = 0x1000
class Pic16F1709(Pic16F_40001683):
    FLASH_LEN = 0x2000

class Pic16LF1703(Pic16F1703): pass
class Pic16LF1704(Pic16F1704): pass
class Pic16LF1705(Pic16F1705): pass
class Pic16LF1707(Pic16F1707): pass
class Pic16LF1708(Pic16F1708): pass
class Pic16LF1709(Pic16F1709): pass

# DS40001683B-page 9

device_ids = {
    0x30DB: Pic16F17114,
    0x30E2: Pic16F17115,
    0x30DC: Pic16F17124,
    0x30DE: Pic16F17125,
    0x30E0: Pic16F17126,
    0x30DD: Pic16F17144,
    0x30DF: Pic16F17145,
    0x30E1: Pic16F17146,
    0x3101: Pic16F17154,
    0x3103: Pic16F17155,
    0x3105: Pic16F17156,
    0x3102: Pic16F17174,
    0x3104: Pic16F17175,
    0x3106: Pic16F17176,

    0x30F1: Pic16F18013,
    0x30F2: Pic16F18014,
    0x30F5: Pic16F18015,
    0x30F3: Pic16F18023,
    0x30F4: Pic16F18024,
    0x30F6: Pic16F18025,
    0x30F9: Pic16F18026,
    0x30F7: Pic16F18044,
    0x30F8: Pic16F18045,
    0x30FA: Pic16F18046,
    0x30FB: Pic16F18054,
    0x30FC: Pic16F18055,
    0x30FF: Pic16F18056,
    0x30FD: Pic16F18074,
    0x30FE: Pic16F18075,
    0x3100: Pic16F18076,

    0x3061: Pic16F1703,
    0x3063: Pic16LF1703,
    0x3043: Pic16F1704,
    0x3045: Pic16LF1704,
    0x3055: Pic16F1705,
    0x3057: Pic16LF1705,
    0x3060: Pic16F1707,
    0x3062: Pic16LF1707,
    0x3042: Pic16F1708,
    0x3044: Pic16LF1708,
    0x3054: Pic16F1709,
    0x3056: Pic16LF1709,
}

def specialize(general_pic, cls):
    return cls(mclr_pin = general_pic.mclr,
               icspdat_pin = general_pic.dat,
               icspclk_pin = general_pic.clk,
               invert_mclr = general_pic.invert_mclr,
               skip_mclr = True)

def program(mclr_pin,
            icspdat_pin,
            icspclk_pin,
            hex_filename,
            require_pic=None,
            invert_mclr=False):

    found = False
    found_ids = []

    # This dance is required because different classes of PIC have
    # different readout mechanisms
    for cls in [Pic16F_Enhanced_Midrange,
                Pic16F_XLP]:
        general_pic = cls(mclr_pin = mclr_pin,
                          icspdat_pin = icspdat_pin,
                          icspclk_pin = icspclk_pin,
                          invert_mclr = invert_mclr)
        
        general_pic.enter_lvp()
        device_id = general_pic.read_device_id()

        found_ids.append(device_id)
        
        if device_id in device_ids:
            found = True
            break
        general_pic.exit_lvp()
                    
    if not found:
        raise Exception("No known PIC found:",
              ", ".join('0x%04x'%did for did in found_ids))
    
    with specialize(general_pic, device_ids[device_id]) as pic:

        print("Found", pic.name)
        if require_pic:
            assert require_pic.upper() == pic.name.upper()

        ih = intelhex.IntelHex16bit()
        ih.loadfile(hex_filename, format='hex')

        used_regions = []
        for seg_start, seg_stop in ih.segments():
            #dbg_print("seg", seg_start, seg_stop)
            # The spec is to have intel-hex addresses at twice the PIC address.
            assert 0 == seg_start % 2
            assert 0 == seg_stop % 2

            seg_start //= 2
            seg_stop //= 2

            assigned_regions = []

            for mem_start, mem_stop, name, erase_size in pic.memory_map:
                if mem_start >= seg_stop:
                    continue
                if seg_start >= mem_stop:
                    continue

                # round down to erase row
                seg_start = (seg_start//erase_size)*erase_size
                # round up to erase row size
                seg_stop  = ((seg_stop + erase_size - 1)//erase_size)*erase_size

                assigned_regions.append([name, seg_start, seg_stop])

            assert len(assigned_regions) <= 1
            used_regions.extend(assigned_regions)

            # Coalesce contiguous
            cont_regions = []
            last_region = None
            for region in used_regions:
                if last_region:
                    if (region[0] == last_region[0] and # compare names
                        last_region[2]>=region[1]): # contiguous
                        last_region[2]=region[2]
                    else:
                        cont_regions.append(last_region)
                        last_region = region
                else:
                    last_region = region
            if last_region:
                cont_regions.append(last_region)

        #dbg_print("used regions", used_regions)
        #dbg_print("contiguous regions", cont_regions)

        region_by_type = {}
        for c in cont_regions:
            region_by_type.setdefault(c[0], []).append(c[1:])

        print(region_by_type)
        pic.bulk_erase(#erase_eeprom = 'eeprom' in region_by_type,
                       erase_flash = 'program' in region_by_type,
                       erase_user_id = 'user' in region_by_type,
                       erase_config = 'config' in region_by_type)

        verify_errors = 0

        for r, reglist in region_by_type.items():
            for reg in reglist:
                words = [ih[x] for x in range(*reg)]
                #dbg_print(r, words, ["%04x"%x for x in reg])
                #dbg_print(">  ",pic.read_flash(start_addr = reg[0], length=reg[1]-reg[0]))
                pic.write_flash(words, start_addr = reg[0])
                #dbg_print(">> ",pic.read_flash(start_addr = reg[0], length=reg[1]-reg[0]))
                readback = pic.read_flash(start_addr = reg[0], length=reg[1]-reg[0])
                for word_wrote,word_read,addr in zip(words, readback, range(*reg)):
                    if (word_wrote ^ word_read) & 16383:
                        for start,end,name,_ in pic.memory_map:
                            if addr >= start and addr < end:
                                print(f"Verify Error at {addr:04x} (wrote {word_wrote:04x}, read {word_read:04x})")
                                verify_errors += 1

            #pic.read_config_stuff()

        pic.exit_lvp()
        return verify_errors
    
def main():
    import argparse
    parser = argparse.ArgumentParser(description="Program a PIC using LabJack.")
    parser.add_argument("--pic", "-p", help='Require this device before programming')
    parser.add_argument("hexfile", metavar="HEXFILE", nargs=1, help="Hex file to program")

    parser.add_argument(
        "--mclr", default='FIO4', help="Labjack pin for !MCLR. (Default %(default)s)"
    )

    parser.add_argument(
        "--invert_mclr", action="store_true", help="Positive-logic MCLR (for special hardware)."
    )

    parser.add_argument(
        "--icspdat", default='FIO5', help="Labjack pin for ICSPDAT. (Default %(default)s)",
    )

    parser.add_argument(
        "--icspclk", default='FIO6', help="Labjack pin for ICSPCLK. (Default %(default)s)"
    )

    args = parser.parse_args()

    lj = u3_manager.u3()
    return program(mclr_pin = getattr(lj, args.mclr),
                   icspdat_pin = getattr(lj, args.icspdat),
                   icspclk_pin = getattr(lj, args.icspclk),
                   require_pic = args.pic,
                   invert_mclr = args.invert_mclr,
                   hex_filename = args.hexfile[0])

if __name__=="__main__":
    main()
