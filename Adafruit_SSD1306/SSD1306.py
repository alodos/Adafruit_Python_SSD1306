# Copyright (c) 2014 Adafruit Industries
# Author: Tony DiCola
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
from __future__ import division
import logging
import time

import Adafruit_GPIO as GPIO
import Adafruit_GPIO.SPI as SPI


# Constants
SSD1306_I2C_ADDRESS = 0x3C    # 011110+SA0+RW - 0x3C or 0x3D
SSD1306_SETCONTRAST = 0x81
SSD1306_DISPLAYALLON_RESUME = 0xA4
SSD1306_DISPLAYALLON = 0xA5
SSD1306_NORMALDISPLAY = 0xA6
SSD1306_INVERTDISPLAY = 0xA7
SSD1306_DISPLAYOFF = 0xAE
SSD1306_DISPLAYON = 0xAF
SSD1306_SETDISPLAYOFFSET = 0xD3
SSD1306_SETCOMPINS = 0xDA
SSD1306_SETVCOMDETECT = 0xDB
SSD1306_SETDISPLAYCLOCKDIV = 0xD5
SSD1306_SETPRECHARGE = 0xD9
SSD1306_SETMULTIPLEX = 0xA8
SSD1306_SETLOWCOLUMN = 0x00
SSD1306_SETHIGHCOLUMN = 0x10
SSD1306_SETSTARTLINE = 0x40
SSD1306_MEMORYMODE = 0x20
SSD1306_COLUMNADDR = 0x21
SSD1306_PAGEADDR = 0x22
SSD1306_COMSCANINC = 0xC0
SSD1306_COMSCANDEC = 0xC8
SSD1306_SEGREMAP = 0xA0
SSD1306_CHARGEPUMP = 0x8D
SSD1306_EXTERNALVCC = 0x1
SSD1306_SWITCHCAPVCC = 0x2

# Scrolling constants
SSD1306_ACTIVATE_SCROLL = 0x2F
SSD1306_DEACTIVATE_SCROLL = 0x2E
SSD1306_SET_VERTICAL_SCROLL_AREA = 0xA3
SSD1306_RIGHT_HORIZONTAL_SCROLL = 0x26
SSD1306_LEFT_HORIZONTAL_SCROLL = 0x27
SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL = 0x29
SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL = 0x2A


class SSD1306Base(object):
    """Base class for SSD1306-based OLED displays.  Implementors should subclass
    and provide an implementation for the _initialize function.
    """

    def __init__(self, width, height, rst, dc=None, sclk=None, din=None, cs=None,
                 gpio=None, spi=None, i2c_bus=None, i2c_address=SSD1306_I2C_ADDRESS,
                 i2c=None, i2c_interface=None):
        self._log = logging.getLogger('Adafruit_SSD1306.SSD1306Base')
        self._spi = None
        self._i2c = None
        self.width = width
        self.height = height
        self._pages = height//8
        self._buffer = [0]*(width*self._pages)
        self._vertical_offset = None
        self._mux_ratio = None
        self._start_line = 0
        # Default to platform GPIO if not provided.
        self._gpio = gpio
        if self._gpio is None:
            self._gpio = GPIO.get_platform_gpio()
        # Setup reset pin.
        self._rst = rst
        if not self._rst is None:
            self._gpio.setup(self._rst, GPIO.OUT)
        # Handle hardware SPI
        if spi is not None:
            self._log.debug('Using hardware SPI')
            self._spi = spi
            self._spi.set_clock_hz(8000000)
        # Handle software SPI
        elif sclk is not None and din is not None and cs is not None:
            self._log.debug('Using software SPI')
            self._spi = SPI.BitBang(self._gpio, sclk, din, None, cs)
        # Handle hardware I2C
        elif i2c is not None:
            self._log.debug('Using hardware I2C with custom I2C provider.')
            self._i2c = i2c.get_i2c_device(i2c_address)
        else:
            self._log.debug('Using hardware I2C with platform I2C provider.')
            import Adafruit_GPIO.I2C as I2C
            if i2c_bus is None:
                self._i2c = I2C.get_i2c_device(i2c_address, i2c_interface=i2c_interface)
            else:
                self._i2c = I2C.get_i2c_device(i2c_address, busnum=i2c_bus, i2c_interface=i2c_interface)
        # Initialize DC pin if using SPI.
        if self._spi is not None:
            if dc is None:
                raise ValueError('DC pin must be provided when using SPI.')
            self._dc = dc
            self._gpio.setup(self._dc, GPIO.OUT)

    def _initialize(self):
        raise NotImplementedError

    def command(self, c):
        """Send command byte to display."""
        if self._spi is not None:
            # SPI write.
            self._gpio.set_low(self._dc)
            self._spi.write([c])
        else:
            # I2C write.
            control = 0x00   # Co = 0, DC = 0
            self._i2c.write8(control, c)

    def data(self, c):
        """Send byte of data to display."""
        if self._spi is not None:
            # SPI write.
            self._gpio.set_high(self._dc)
            self._spi.write([c])
        else:
            # I2C write.
            control = 0x40   # Co = 0, DC = 0
            self._i2c.write8(control, c)

    def begin(self, init=True, vccstate=SSD1306_SWITCHCAPVCC):
        """Initialize display."""
        # Save vcc state.
        self._vccstate = vccstate
        if init:
            # Reset and initialize display.
            self.reset()
            self._initialize()
            # Turn on the display.
            self.command(SSD1306_DISPLAYON)

    def reset(self):
        """Reset the display."""
        if self._rst is None:
            return
        # Set reset high for a millisecond.
        self._gpio.set_high(self._rst)
        time.sleep(0.001)
        # Set reset low for 10 milliseconds.
        self._gpio.set_low(self._rst)
        time.sleep(0.010)
        # Set reset high again.
        self._gpio.set_high(self._rst)

    def display(self):
        """Write display buffer to physical display."""
        self.command(SSD1306_COLUMNADDR)
        self.command(0)              # Column start address. (0 = reset)
        self.command(self.width-1)   # Column end address.
        self.command(SSD1306_PAGEADDR)
        self.command(0)              # Page start address. (0 = reset)
        self.command(self._pages-1)  # Page end address.
        # Write buffer data.
        if self._spi is not None:
            # Set DC high for data.
            self._gpio.set_high(self._dc)
            # Write buffer.
            self._spi.write(self._buffer)
        else:
            for i in range(0, len(self._buffer), 16):
                control = 0x40   # Co = 0, DC = 0
                self._i2c.writeList(control, self._buffer[i:i+16])

    def displaypart(self, col_start, col_end, page_start, page_end):
        """Write part of display buffer to physical display."""

        if (col_start == 0) and (col_end == self.width-1) and (page_start == 0) and (page_end == self._pages-1):
            self.display();

        # Check for correct values
        if col_start < 0 or col_start >= self.width:
            raise ValueError('Start column must be greater then zero and less than width.')
        if col_end < col_start or col_end >= self.width:
            raise ValueError('End column must be greater or equal then start column and less than width.')
        if page_start < 0 or page_start >= self._pages:
            raise ValueError('Start page must be greater then zero and less than pages count.')
        if page_end < page_start or page_end >= self._pages:
            raise ValueError('End page must be greater or equal then start page and less than pages count.')

        # Prepare part of buffer
        video_buffer_part = [0]*((page_end - page_start + 1) * (col_end - col_start + 1))
        j = 0
        lenb = len(self._buffer)
        for i in range(lenb):
            if (col_start <= (i % self.width) <= col_end) and (page_start <= (i//self.width) <= page_end):
                video_buffer_part[j] = self._buffer[i]
                j = j + 1

        self.command(SSD1306_COLUMNADDR)
        self.command(col_start)      # Column start address. (0 = reset)
        self.command(col_end)        # Column end address.
        self.command(SSD1306_PAGEADDR)
        self.command(page_start)     # Page start address. (0 = reset)
        self.command(page_end)       # Page end address.

        # Write buffer data.
        if self._spi is not None:
            # Set DC high for data.
            self._gpio.set_high(self._dc)
            # Write buffer.
            self._spi.write(video_buffer_part)
        else:
            for i in range(0, len(video_buffer_part), 16):
                control = 0x40   # Co = 0, DC = 0
                self._i2c.writeList(control, video_buffer_part[i:i+16])

    def image(self, image):
        """Set buffer to value of Python Imaging Library image.  The image should
        be in 1 bit mode and a size equal to the display size.
        """
        if image.mode != '1':
            raise ValueError('Image must be in mode 1.')
        imwidth, imheight = image.size
        if imwidth != self.width or imheight != self.height:
            raise ValueError('Image must be same dimensions as display ({0}x{1}).' \
                .format(self.width, self.height))
        # Grab all the pixels from the image, faster than getpixel.
        pix = image.load()
        # Iterate through the memory pages
        index = 0
        for page in range(self._pages):
            # Iterate through all x axis columns.
            for x in range(self.width):
                # Set the bits for the column of pixels at the current position.
                bits = 0
                # Don't use range here as it's a bit slow
                for bit in [0, 1, 2, 3, 4, 5, 6, 7]:
                    bits = bits << 1
                    bits |= 0 if pix[(x, page*8+7-bit)] == 0 else 1
                # Update buffer byte and increment to next byte.
                self._buffer[index] = bits
                index += 1

    def clear(self):
        """Clear contents of image buffer."""
        self._buffer = [0]*(self.width*self._pages)

    def set_contrast(self, contrast):
        """Sets the contrast of the display.  Contrast should be a value between
        0 and 255."""
        if contrast < 0 or contrast > 255:
            raise ValueError('Contrast must be a value from 0 to 255 (inclusive).')
        self.command(SSD1306_SETCONTRAST)
        self.command(contrast)

    def dim(self, dim):
        """Adjusts contrast to dim the display if dim is True, otherwise sets the
        contrast to normal brightness if dim is False.
        """
        # Assume dim display.
        contrast = 0
        # Adjust contrast based on VCC if not dimming.
        if not dim:
            if self._vccstate == SSD1306_EXTERNALVCC:
                contrast = 0x9F
            else:
                contrast = 0xCF

    def deactivate_scroll(self):
        """
        Deactivate scroll (2Eh)

        This command stops the motion of scrolling. After sending 0x2E command to deactivate the scrolling action,the ram
        data needs to be rewritten.
        """
        self.command(SSD1306_DEACTIVATE_SCROLL)                    # 0x2E
        self.display()

    def activate_scroll(self):
        """
        Activate Scroll (2Fh)

        This command starts the motion of scrolling and should only be issued after the scroll setup parameters have
        been defined by the scrolling setup commands :26h/27h/29h/2Ah . The setting in the last scrolling setup command
        overwrites the setting in the previous scrolling setup commands.

        The following actions are prohibited after the scrolling is activated
            1.  RAM access (Data write or read)
            2.  Changing the horizontal scroll setup parameters
        """
        self.command(SSD1306_ACTIVATE_SCROLL)                    # 0x2F

    def horizontal_scroll_setup(self, direction, start_page, end_page, speed):
        """
        Horizontal Scroll Setup (26h/27h)

        This command consists of consecutive bytes to set up the horizontal scroll parameters and determines the
        scrolling start page, end page and scrolling speed. Before issuing this command the horizontal scroll must be
        deactivated (2Eh). Otherwise, RAM content may be corrupted.

        :param direction:   0 - Right Horizontal Scroll
                            1 - Left Horizontal Scroll
        :param start_page:  Define start page address - PAGE0 ~ PAGE{PAGES-1}
        :param end_page:    Define end page address - PAGE0 ~ PAGE{PAGES-1}
        :param speed:       Set time interval between each roll step in terms of frame frequency:
                                0 - 5 frames
                                1 - 64 frames
                                2 - 128 frames
                                3 - 256 frames
                                4 - 3 frames
                                5 - 4 frames
                                6 - 25 frames
                                7 - 2 frames
        :raise ValueError: Start page cannot be larger than end page
        """
        self.deactivate_scroll()

        # Check for correct values
        self.check_int(direction, 0, 1)
        self.check_int(start_page, 0, self._pages-1)
        self.check_int(end_page, 0, self._pages-1)
        self.check_int(speed, 0, 7)

        # Check if start_page is bigger than end_page
        if start_page > end_page:
            raise ValueError("Start page address cannot be bigger than end page address")

        self.command(SSD1306_LEFT_HORIZONTAL_SCROLL if direction else SSD1306_RIGHT_HORIZONTAL_SCROLL)  # 0x26/0x27
        self.command(0x00)           # Dummy byte (Set as 00h)
        self.command(start_page)     # Column start address. (0 = reset)
        self.command(speed)          # Set time interval between each scroll step in terms of frame frequency.
        self.command(end_page)       # Column end address.
        self.command(0x00)           # Dummy byte (Set as 00h)
        self.command(0xFF)           # Dummy byte (Set as FFh)

    def vertical_and_horizontal_scroll_setup(self, direction, start_page, end_page, speed, vertical_offset):
        """
         Continuous Vertical and Horizontal Scroll Setup (29h/2Ah)

        This command consists of 6 consecutive bytes to set up the continuous vertical scroll parameters and determines
        the scrolling start page, end page, scrolling speed and vertical scrolling offset.

        The bytes B[2:0], C[2:0] and D[2:0] of command 29h/2Ah are for the setting of the continuous horizontal
        scrolling. The byte E[5:0] is for the setting of the continuous vertical scrolling offset. All these bytes
        together are for the setting of continuous diagonal (horizontal + vertical) scrolling. If the vertical
        scrolling offset byte E[5:0] is set to zero, then only horizontal scrolling is performed (like command 26/27h).

        Before issuing this command the scroll must be deactivated (2Eh). Otherwise, RAM content may be corrupted.

        :param direction:       0 - Vertical and Right Horizontal Scroll
                                1 - Vertical and Left Horizontal Scroll
        :param start_page:      Define start page address - PAGE0 ~ PAGE{PAGES-1}
        :param end_page:        Define end page address -   PAGE0 ~ PAGE{PAGES-1}
        :param speed:           Set time interval between each roll step in terms of frame frequency:
                                0 - 5 frames
                                1 - 64 frames
                                2 - 128 frames
                                3 - 256 frames
                                4 - 3 frames
                                5 - 4 frames
                                6 - 25 frames
                                7 - 2 frames
        :param vertical_offset: Vertical scrolling offset e.g.
                                    01h refer to offset = 1 row
                                    3Fh refer to offset = 63 rows
        :raise ValueError:      Start page cannot be larger than end page
        """
        self.deactivate_scroll()

        # Check for correct values
        self.check_int(direction, 0, 1)
        self.check_int(start_page, 0, self._pages-1)
        self.check_int(end_page, 0, self._pages-1)
        self.check_int(speed, 0, 7)
        self.check_int(vertical_offset, 0, self.height-1)

        self._vertical_offset = vertical_offset

        # Check if start_page is bigger than end_page
        if start_page > end_page:
            raise ValueError("Start page address cannot be bigger than end page address")

        self.command(SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL if direction else SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL)  # 0x29/0x2A
        self.command(0x00)             # Dummy byte (Set as 00h)
        self.command(start_page)       # Page start address. (0 = reset)
        self.command(speed)            # Set time interval between each scroll step in terms of frame frequency.
        self.command(end_page)         # Page end address.
        self.command(vertical_offset)  # Vertical scrolling offset

    def set_vertical_scroll_area(self, start, count):

        """
        Set Vertical Scroll Area(A3h)

        This command consists of 3 consecutive bytes to set up the vertical scroll area. For the continuous vertical
        scroll function (command 29/2Ah), the number of rows that in vertical scrolling can be set smaller or equal to
        the MUX ratio.

        :param start:       Set No. of rows in top fixed area. The No. of rows in top fixed area is referenced to the
                            top of the GDDRAM (i.e. row 0).[RESET =0]

        :param count:       Set No. of rows in scroll area. This is the number of rows to be used for vertical
                            scrolling. The scroll area starts in the first row below the top fixed area. [RESET = 64]

        :raise ValueError:
        """
        self.check_int(start, 0, self.height-1)
        self.check_int(count, 0, self.height-1)

        if start + count > self._mux_ratio:
            raise ValueError("Start + Count cannot be larger than MUX ratio")

        if count > self._mux_ratio:
            raise ValueError("Count cannot be larger than MUX ratio")

        if not self._vertical_offset:
            raise ValueError("Vertical and horizontal scroll must be setup first")

        if not (self._vertical_offset < count):
            raise ValueError("Count cannot be smaller than vertical offset")

        if not (self._start_line < count):
            raise ValueError("Display start line must be smaller than Count")

        self.command(SSD1306_SET_VERTICAL_SCROLL_AREA)                    # 0xA3
        self.command(start)            # No. of rows in top fixed area. (0 = reset)
        self.command(count)            # No. of rows in scroll area. (64 = reset)

    def set_multiplex_ratio(self, ratio):
        """
        Set Multiplex Ratio (A8h)

        This command switches the default 63 multiplex mode to any multiplex ratio, ranging from 16 to 63 (for 128x64 displays). The output
        pads COM0~COM63 will be switched to the corresponding COM signal.

        :param ratio:   Set MUX ratio to N+1 MUX
                        N=A[5:0] : from 16MUX to 64MUX,
                        RESET= 111111b (i.e. 63d, 64MUX)
                        A[5:0] from 0 to 14 are invalid entry.
        """
        self.check_int(ratio, 15, self.height-1)
        self._mux_ratio = ratio

        self.command(SSD1306_SETMULTIPLEX)                    # 0xA8
        self.command(ratio)       # Multiplex Ratio. (111111b = reset for 128x64 displays)

    def set_display_start_line(self, start_line):
        """
        Set Display Start Line (40h~7Fh)

        This command sets the Display Start Line register to determine starting address of display RAM, by selecting a
        value from 0 to 63 (for 128x64 displays). With value equal to 0, RAM row 0 is mapped to COM0. With value equal to 1, RAM row 1 is
        mapped to COM0 and so on.

        :param start_line:  Set display RAM display start line register from 0-(height-1).
                            Display start line register is reset to 000000b during RESET.
        """
        self.check_int(start_line, 0, self.height-1)
        self._start_line = start_line

        self.command(SSD1306_SETSTARTLINE | start_line)                    # 0x40~7Fh

    @staticmethod
    def check_int(data, lower_range, upper_range):
        if lower_range >= upper_range:
            raise ValueError("Lower range cannot be greater that the upper")
        if not isinstance(data, int):
            raise TypeError("Value must be integer")
        if data > upper_range or data < lower_range:
            raise ValueError("Invalid value")

    def getHeight(self):
        return self.height

    def getWidth(self):
        return self.width

class SSD1306_128_64(SSD1306Base):
    def __init__(self, rst, dc=None, sclk=None, din=None, cs=None, gpio=None,
                 spi=None, i2c_bus=None, i2c_address=SSD1306_I2C_ADDRESS,
                 i2c=None, i2c_interface=None):
        # Call base class constructor.
        super(SSD1306_128_64, self).__init__(128, 64, rst, dc, sclk, din, cs,
                                             gpio, spi, i2c_bus, i2c_address, i2c, i2c_interface)
        self._mux_ratio = 0x3F

    def _initialize(self):
        # 128x64 pixel specific initialization.
        self.command(SSD1306_DISPLAYOFF)                    # 0xAE
        self.command(SSD1306_SETDISPLAYCLOCKDIV)            # 0xD5
        self.command(0x80)                                  # the suggested ratio 0x80
        self.command(SSD1306_SETMULTIPLEX)                  # 0xA8
        self.command(0x3F)
        self._mux_ratio = 0x3F
        self.command(SSD1306_SETDISPLAYOFFSET)              # 0xD3
        self.command(0x0)                                   # no offset
        self.command(SSD1306_SETSTARTLINE | 0x0)            # line #0
        self._start_line = 0
        self.command(SSD1306_CHARGEPUMP)                    # 0x8D
        if self._vccstate == SSD1306_EXTERNALVCC:
            self.command(0x10)
        else:
            self.command(0x14)
        self.command(SSD1306_MEMORYMODE)                    # 0x20
        self.command(0x00)                                  # 0x0 act like ks0108
        self.command(SSD1306_SEGREMAP | 0x1)
        self.command(SSD1306_COMSCANDEC)
        self.command(SSD1306_SETCOMPINS)                    # 0xDA
        self.command(0x12)
        self.command(SSD1306_SETCONTRAST)                   # 0x81
        if self._vccstate == SSD1306_EXTERNALVCC:
            self.command(0x9F)
        else:
            self.command(0xCF)
        self.command(SSD1306_SETPRECHARGE)                  # 0xd9
        if self._vccstate == SSD1306_EXTERNALVCC:
            self.command(0x22)
        else:
            self.command(0xF1)
        self.command(SSD1306_SETVCOMDETECT)                 # 0xDB
        self.command(0x40)
        self.command(SSD1306_DISPLAYALLON_RESUME)           # 0xA4
        self.command(SSD1306_NORMALDISPLAY)                 # 0xA6


class SSD1306_128_32(SSD1306Base):
    def __init__(self, rst, dc=None, sclk=None, din=None, cs=None, gpio=None,
                 spi=None, i2c_bus=None, i2c_address=SSD1306_I2C_ADDRESS,
                 i2c=None, i2c_interface=None):
        # Call base class constructor.
        super(SSD1306_128_32, self).__init__(128, 32, rst, dc, sclk, din, cs,
                                             gpio, spi, i2c_bus, i2c_address, i2c, i2c_interface)
        self._mux_ratio = 0x1F

    def _initialize(self):
        # 128x32 pixel specific initialization.
        self.command(SSD1306_DISPLAYOFF)                    # 0xAE
        self.command(SSD1306_SETDISPLAYCLOCKDIV)            # 0xD5
        self.command(0x80)                                  # the suggested ratio 0x80
        self.command(SSD1306_SETMULTIPLEX)                  # 0xA8
        self.command(0x1F)
        self._mux_ratio = 0x1F
        self.command(SSD1306_SETDISPLAYOFFSET)              # 0xD3
        self.command(0x0)                                   # no offset
        self.command(SSD1306_SETSTARTLINE | 0x0)            # line #0
        self._start_line = 0
        self.command(SSD1306_CHARGEPUMP)                    # 0x8D
        if self._vccstate == SSD1306_EXTERNALVCC:
            self.command(0x10)
        else:
            self.command(0x14)
        self.command(SSD1306_MEMORYMODE)                    # 0x20
        self.command(0x00)                                  # 0x0 act like ks0108
        self.command(SSD1306_SEGREMAP | 0x1)
        self.command(SSD1306_COMSCANDEC)
        self.command(SSD1306_SETCOMPINS)                    # 0xDA
        self.command(0x02)
        self.command(SSD1306_SETCONTRAST)                   # 0x81
        self.command(0x8F)
        self.command(SSD1306_SETPRECHARGE)                  # 0xd9
        if self._vccstate == SSD1306_EXTERNALVCC:
            self.command(0x22)
        else:
            self.command(0xF1)
        self.command(SSD1306_SETVCOMDETECT)                 # 0xDB
        self.command(0x40)
        self.command(SSD1306_DISPLAYALLON_RESUME)           # 0xA4
        self.command(SSD1306_NORMALDISPLAY)                 # 0xA6


class SSD1306_96_16(SSD1306Base):
    def __init__(self, rst, dc=None, sclk=None, din=None, cs=None, gpio=None,
                 spi=None, i2c_bus=None, i2c_address=SSD1306_I2C_ADDRESS,
                 i2c=None, i2c_interface=None):
        # Call base class constructor.
        super(SSD1306_96_16, self).__init__(96, 16, rst, dc, sclk, din, cs,
                                            gpio, spi, i2c_bus, i2c_address, i2c, i2c_interface)
        self._mux_ratio = 0x0F

    def _initialize(self):
        # 128x32 pixel specific initialization.
        self.command(SSD1306_DISPLAYOFF)                    # 0xAE
        self.command(SSD1306_SETDISPLAYCLOCKDIV)            # 0xD5
        self.command(0x60)                                  # the suggested ratio 0x60
        self.command(SSD1306_SETMULTIPLEX)                  # 0xA8
        self.command(0x0F)
        self._mux_ratio = 0x0F
        self.command(SSD1306_SETDISPLAYOFFSET)              # 0xD3
        self.command(0x0)                                   # no offset
        self.command(SSD1306_SETSTARTLINE | 0x0)            # line #0
        self._start_line = 0
        self.command(SSD1306_CHARGEPUMP)                    # 0x8D
        if self._vccstate == SSD1306_EXTERNALVCC:
            self.command(0x10)
        else:
            self.command(0x14)
        self.command(SSD1306_MEMORYMODE)                    # 0x20
        self.command(0x00)                                  # 0x0 act like ks0108
        self.command(SSD1306_SEGREMAP | 0x1)
        self.command(SSD1306_COMSCANDEC)
        self.command(SSD1306_SETCOMPINS)                    # 0xDA
        self.command(0x02)
        self.command(SSD1306_SETCONTRAST)                   # 0x81
        self.command(0x8F)
        self.command(SSD1306_SETPRECHARGE)                  # 0xd9
        if self._vccstate == SSD1306_EXTERNALVCC:
            self.command(0x22)
        else:
            self.command(0xF1)
        self.command(SSD1306_SETVCOMDETECT)                 # 0xDB
        self.command(0x40)
        self.command(SSD1306_DISPLAYALLON_RESUME)           # 0xA4
        self.command(SSD1306_NORMALDISPLAY)                 # 0xA6
