#!/usr/bin/python

__AUTHOR__ = 'Ryan C. Julian'

import sys
import os
from time import sleep, time
from lib.imageproc_spi import *

SPI_PORT = '/dev/spidev2.0'

# Make sure kernel module is inserted
os.popen('insmod mt9v113.ko')

# Make sure AGC is off
os.popen('v4l2-ctl --set-ctrl=gain_automatic=0')

# Open the SPI interface
TEST_STRING = struct.pack('d',-1)
recv_len = len(TEST_STRING) + 2
spi = ImageProcSPI(SPI_PORT)

# Start tracking
sleep(0.5)
tracking = os.popen('./tennis_ballbot 2> /dev/null')
startTime = time()
while True:
  try:
    line = tracking.readline()
    if line != '' and line[0] == '#':
      xpos = int(line[1:-1])
      spi.send(0x13,0x22,struct.pack('<h',xpos))
      #recv = spi.receive(recv_len)
      if xpos >= 0:
        print str(xpos) +', ' + str(1/(time() - startTime)) + ' frame/sec'
        startTime = time()
  except KeyboardInterrupt:
    break
    
  sleep(0.020)

# Don't forget to stop tracking
#tracking_out.write('q')
#tracking_out.flush()
#tracking_out.close()
sleep(0.500)
tracking.close()
