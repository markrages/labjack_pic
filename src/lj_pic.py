#!/usr/bin/env python3

import intelhex
import time
import u3_manager
import u3
import itertools

"""

Read / write PIC16F18025 type part.

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

class Pic:
    USERID_ADDR = 0x8000
    REVID_ADDR = 0x8005
    DEVID_ADDR = 0x8006

    def __init__(self,
                 mclr_pin=None,
                 icspdat_pin=None,
                 icspclk_pin=None,
                 skip_mclr = False):
        self.mclr = mclr_pin
        self.dat = icspdat_pin
        self.clk = icspclk_pin

        if not skip_mclr:
            self.mclr.set()
        self.dat.clear()
        self.clk.clear()
        self.tx_spi = []

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, exc_traceback):
        if not exc_type:
            self.flush_tx_spi()
        
    def send_value(self, width, val):
        """ val is value to send (integer)
        width is number of bytes to send (big-endian)
        """        
        bytelist = list(reversed(
            [0xff & (val >> (8*b))
             for b in range(width)]
        ))
        self.tx_spi.append(bytelist)

    def flush_tx_spi(self):
        alldata = list(itertools.chain(*self.tx_spi))
        self.tx_spi = []

        unused = 0xff
        lj = self.clk.parent

        chunklen = 50
        
        for pos in range(0, len(alldata), chunklen):
            lj.spi(SPIBytes = alldata[pos:pos+chunklen],
                   CLKPinNum = self.clk.number,
                   MOSIPinNum = self.dat.number,
                   MISOPinNum = unused,
                   CSPinNum = unused,
                   SPIMode='B',
                   AutoCS=False)

    def read_value(self, width):
        """ width is number of bytes to read (big-endian)
        return value is integer
        """        
        self.flush_tx_spi()
        
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

        return ret    
               
    def sleep(self, timeout):
        if timeout > 4e-6: # timeouts less than this are taken-care of
                           # by the slow SPI clock
            self.flush_tx_spi()
            time.sleep(timeout)
            
        
class Pic16F_400237(Pic):
    """
    Programming spec: 40002317

    https://ww1.microchip.com/downloads/aemDocuments/documents/MCU08/ProductDocuments/ProgrammingSpecifications/PIC16F180XX-Family-Programming-Specification-40002317.pdf
    """

    CONFIG_ADDR = 0x8007
    CONFIG_LEN = 0xC
    DIA_ADDR = 0x8100
    DIA_LEN = 0x40
    EEPROM_ADDR = 0xF000
    EEPROM_LEN = 0x100

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
        
    def send32(self, d): self.send_value(32//8, d)
    def send24(self, d): self.send_value(24//8, d)
    def send8(self, d): self.send_value(8//8, d)
    
    def read24(self):
        start_bit = (1<<31)
        stop_bit = (1<<0)
        bits_mask = ~(start_bit | stop_bit)
        
        return (self.read_value(24//8) & bits_mask) >> 1
    
    def enter_lvp(self):
        self.mclr.clear()
        self.sleep(self.Tenth)
        self.send32(self.KEY)        
        self.sleep(self.Tenth)

    def set_PC(self, pc):
        self.send8_24(0x80, pc)

    def increment_pc(self):
        self.send8(0xf8)
        
    def load_mem(self, d, inc=False):
        self.send8_24([0x00, 0x02][inc], d)

    def write_row(self):
        self.send8(0xe0) # internally timed
        self.sleep(self.Tpint)
        
    def read_mem(self):
        self.send8(0xfe) # auto increment!
        self.sleep(self.Tdly)
        return self.read24()
    
    def read_device_id(self):
        # set PC address
        self.set_PC(self.DEVID_ADDR)
        return self.read_mem()

    def exit_lvp(self):
        self.flush_tx_spi()
        self.mclr.set()

    def lookup_id(self):
        self.enter_lvp()
        device_id = self.read_device_id()
        
        print("Device ID: %x"%device_id)
        cls = device_ids[device_id]
        
        return cls(mclr_pin = self.mclr,
                   icspdat_pin = self.dat,
                   icspclk_pin = self.clk,
                   skip_mclr = True)

    def bulk_erase(self,
                   erase_eeprom = False,
                   erase_flash = False,
                   erase_user_id = False,
                   erase_config = False):
        cmd = sum([erase_eeprom << 0,
                   erase_flash << 1,
                   erase_user_id << 2,
                   erase_config << 3])

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
            
        front_pad = start_addr % 32
        start_addr -= front_pad
        back_pad = 31 - ((31+len(data)) % 32)

        data = [0x3fff] * front_pad + data + [0x3fff] * back_pad
                
        pc = start_addr
        self.set_PC(pc)

        while data:
            row = data[:32]
            data = data[32:]
            for d in row[:31]:
                self.load_mem(d, inc=True)
            
            self.load_mem(row[31], inc=False)
            self.write_row()
            self.increment_pc()
    
class Pic16F18025(Pic16F_400237):
    FLASH_LEN = 8*1024 # words
    pass

device_ids = {
    # 0x30F1: Pic16F18013
    # 0x30F2: Pic16F18014
    # 0x30F5: Pic16F18015
    # 0x30F3: Pic16F18023
    # 0x30F4: Pic16F18024
    0x30F6: Pic16F18025
    # 0x30F9: Pic16F18026
    # 0x30F7: Pic16F18044
    # 0x30F8: Pic16F18045
    # 0x30FA: Pic16F18046
    # 0x30FB: Pic16F18054
    # 0x30FC: Pic16F18055
    # 0x30FF: Pic16F18056
    # 0x30FD: Pic16F18074
    # 0x30FE: Pic16F18075
    # 0x3100: Pic16F18076
}

if __name__=="__main__":
    lj = u3_manager.u3()
    #lj.reset()

    pic = Pic16F_400237(mclr_pin = lj.FIO4,
                        icspdat_pin = lj.FIO5,
                        icspclk_pin = lj.FIO6)

    with pic.lookup_id() as pic:
            
        N = 5

        print("Before erase:")
        print(' '.join("%08x"%c for c in pic.read_flash(length=N*2)))
        pic.bulk_erase(erase_flash=True)
        print("After erase:")    
        print(' '.join("%08x"%c for c in pic.read_flash(length=N*2)))
        pic.write_flash(list(range(5)))
        print("After write:")
        print(' '.join("%08x"%c for c in pic.read_flash(length=N*2)))
        print("After write:")
        print(' '.join("%08x"%c for c in pic.read_flash(length=N*2)))    
        readback = pic.read_flash(length=pic.FLASH_LEN)
        for r in readback:
            print(r)
        pic.exit_lvp()
    
