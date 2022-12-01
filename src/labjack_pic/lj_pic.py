#!/usr/bin/env python3

import intelhex
import time
from .u3_manager import u3_manager
import itertools
import sys

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

    @property
    def name(self):
        return self.__class__.__name__

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


class Pic16F_4002317(Pic):
    """
    Programming spec: 40002317

    https://ww1.microchip.com/downloads/aemDocuments/documents/MCU08/ProductDocuments/ProgrammingSpecifications/PIC16F180XX-Family-Programming-Specification-40002317.pdf
    """

    USER_ADDR = 0x8000

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
            print(name, "@%04x:"%addr, "%04x"%self.read_word(addr))

    def exit_lvp(self):
        self.flush_tx_spi()
        self.mclr.set()

    def lookup_id(self):
        self.enter_lvp()
        device_id = self.read_device_id()

        #print("Device ID: %x"%device_id)
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

        #print("Erase cmd:",cmd)

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

        # print("data",data)
        # print("pads",front_pad, back_pad)

        data = [0x3fff] * front_pad + data + [0x3fff] * back_pad

        self.set_PC(start_addr)

        while data:
            row = data[:row_size]
            data = data[row_size:]

            for d in row[:(row_size-1)]:
                self.load_mem(d, inc=True)

            self.load_mem(row[row_size-1], inc=False)
            self.write_row()

            #print("write row @pc=%04x"%self.pc)
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
            (0x8007, 0x800c, "config", 4),
            (0xf000, 0xf100, "eeprom", 1),
        ]

class Pic16F18013(Pic16F_4002317):
    FLASH_LEN = 2*1024 # words
class Pic16F18014(Pic16F_4002317):
    FLASH_LEN = 4*1024
class Pic16F18015(Pic16F_4002317):
    FLASH_LEN = 8*1024
class Pic16F18023(Pic16F_4002317):
    FLASH_LEN = 2*1024
class Pic16F18024(Pic16F_4002317):
    FLASH_LEN = 4*1024
class Pic16F18025(Pic16F_4002317):
    FLASH_LEN = 8*1024
class Pic16F18026(Pic16F_4002317):
    FLASH_LEN = 16*1024
class Pic16F18044(Pic16F_4002317):
    FLASH_LEN = 4*1024
class Pic16F18045(Pic16F_4002317):
    FLASH_LEN = 8*1024
class Pic16F18046(Pic16F_4002317):
    FLASH_LEN = 16*1024
class Pic16F18054(Pic16F_4002317):
    FLASH_LEN = 4*1024
class Pic16F18055(Pic16F_4002317):
    FLASH_LEN = 8*1024
class Pic16F18056(Pic16F_4002317):
    FLASH_LEN = 16*1024
class Pic16F18074(Pic16F_4002317):
    FLASH_LEN = 4*1024
class Pic16F18075(Pic16F_4002317):
    FLASH_LEN = 8*1024
class Pic16F18076(Pic16F_4002317):
    FLASH_LEN = 16*1024

device_ids = {
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
}

def program(mclr_pin,
            icspdat_pin,
            icspclk_pin,
            hex_filename):

    pic = Pic16F_4002317(mclr_pin = mclr_pin,
                         icspdat_pin = icspdat_pin,
                         icspclk_pin = icspclk_pin)

    with pic.lookup_id() as pic:

        print("Found", pic.name)

        ih = intelhex.IntelHex16bit()
        ih.loadfile(hex_filename, format='hex')

        used_regions = []
        for seg_start, seg_stop in ih.segments():
            #print("seg", seg_start, seg_stop)
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

        #print("used regions", used_regions)
        #print("contiguous regions", cont_regions)

        region_by_type = {}
        for c in cont_regions:
            region_by_type.setdefault(c[0], []).append(c[1:])

        print(region_by_type)
        pic.bulk_erase(#erase_eeprom = 'eeprom' in region_by_type,
                       erase_flash = 'program' in region_by_type,
                       erase_user_id = 'user' in region_by_type,
                       erase_config = 'config' in region_by_type)

        for r, reglist in region_by_type.items():
            for reg in reglist:
                words = [ih[x] for x in range(*reg)]
                #print(r, words, ["%04x"%x for x in reg])
                #print(">  ",pic.read_flash(start_addr = reg[0], length=reg[1]-reg[0]))
                pic.write_flash(words, start_addr = reg[0])
                #print(">> ",pic.read_flash(start_addr = reg[0], length=reg[1]-reg[0]))
                readback = pic.read_flash(start_addr = reg[0], length=reg[1]-reg[0])
                for word_wrote,word_read,addr in zip(words, readback, range(*reg)):
                    if (word_wrote ^ word_read) & 16383:
                        for start,end,name,_ in pic.memory_map:
                            if addr >= start and addr < end:
                                print(f"Verify Error at {addr:04x} (wrote {word_wrote:04x}, read {word_read:04x})")

            #pic.read_config_stuff()

        pic.exit_lvp()

def main():
    lj = u3_manager.u3()
    program(mclr_pin = lj.FIO4,
            icspdat_pin = lj.FIO5,
            icspclk_pin = lj.FIO6,
            hex_filename = sys.argv[1])
    return 0

if __name__=="__main__":
    main()
