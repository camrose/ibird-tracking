# Copyright (c) 2012, Regents of the University of California
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
# - Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
# - Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# - Neither the name of the University of California, Berkeley nor the names
#   of its contributors may be used to endorse or promote products derived
#   from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
# 
# 
# ImageProc SPI interface
# 
# by Ryan C. Julian
#
# v.1.0
#
# Revisions:
#  Ryan C. Julian      2012-03-04    Initial release
#                       
# Notes:
#  - This module implements the ImageProc SPI interface, which emulates the XBee radio 
#    interface over an SPI channel.

__author__ = 'ryanjulian@berkeley.edu (Ryan Julian)'

import struct
import spidev

MAGIC_NUMBER = 0xAA

class ImageProcSPI(object):
    """
    Implements the ImageProc SPI interface, which emulates the XBee radio interface over an SPI channel.
    """
    
    def __init__(self, device):
        """
        Open the SPI channel.
        """
        self._spidev = spidev.Spidev(device)
        
    def send(self, status, type, data):
        """
        Send a command payload. Data must be at least 1 and no more than 
253 bytes in 
        length
        """
        # Check data length
        if len(data) >= 253:
            raise ImageProcSPIException('Data may not exceed 253 bytes')
        if len(data) < 1:
            raise ImageProcSPIException('Data must be at least 1 byte')
        # Build payload
        payload = chr(MAGIC_NUMBER) + chr(len(data)+2) + chr(status) \
                  + chr(type) + ''.join(data)
        # Attempt to send over SPI channel
        rv = self._spidev.write(payload)
        # Check for successful transmission
        if rv != len(payload):
            raise ImageProcSPIException('Transmission unsuccessful')
        return rv
        
    def receive(self, n):
        """
        Attempt to receive n bytes from the SPI line.
        """
        # Attempt to receive n bytes
        rv = self._spidev.read(n)
        # Check for successful transmission
        if len(rv) != n:
            raise ImageProcSPIException('Receive unsuccessful')
        return rv
    
class ImageProcSPIException(Exception):
    """
    ImageProcSPI exception class
    """
    
    pass
       
if __name__ == '__main__':
    """
    Test function
    """
    SPI_PORT = '/dev/spidev2.0'
    TEST_STRING = struct.pack('ff',3.14,2.71)
    
    spi = ImageProcSPI(SPI_PORT)
    spi.send(0x11, 0x22, TEST_STRING)
    raw_input("Press any key to receive...")
    rv = spi.receive(len(TEST_STRING)+2)
    
    print 'ImageProcSPI echo test (ImageProc configured to echo):'
    print 'Sent ' + repr(chr(0x11) + chr(0x22) + TEST_STRING) + ' to ' + SPI_PORT
    print 'Received response ' + repr(rv)
    if rv == (chr(0x11) + chr(0x22) + TEST_STRING):
        print 'TEST SUCCESSFUL!'
    else:
        print 'TEST FAILED!'
