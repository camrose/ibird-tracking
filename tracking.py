#!/usr/bin/python

import sys, subprocess, time, tempfile, threading, tty
from pythonlib.lib.command_interface import CommandInterface
from pythonlib.keyboardWireLinux import KeyboardInterface
from pythonlib.lib.xbee_manager import XBeeManager
from pythonlib.lib.keyboard_monitor import KeyboardMonitor
from xbee import XBee
from serial import *
from struct import *
from Queue import *
from pythonlib.lib.quaternion import *


# In future, add "from pythonlib.lib.network_coordinator import NetworkCoordinator"
# and discover/fetch client addresses
com = '/dev/ttyUSB0'
baud = 57600
DEFAULT_ADDR = 0x1020
DEFAULT_PAN = 0x1005
addr = '\x10\x21'
DEFAULT_CHAN = 0x12
xb = None

if __name__ == '__main__':

    print "Beginning initialization..."

    # Initialize xbee radio 
    print "Initializing XBee.."
    xb = XBeeManager(com = '/dev/ttyUSB0', baud = 57600)  
    if not xb.isInitialized():
        print "XBee initialization failed."
        sys.exit(-1)
    xb.setAddress(DEFAULT_ADDR, DEFAULT_PAN, DEFAULT_CHAN)
    xb.start()

    # Initialize keyboard getter
    print "Initializing keyboard monitor..."
    kbmon = KeyboardMonitor()
    if not kbmon.isInitialized():
        print "Keyboard monitor initialization failed."
        sys.exit(-1)
    kbmon.start()

    # Initialize communication modules
    print "Initializing communication modules..."
    comm = CommandInterface(addr, xb.putTxPacket)
    comm.enableDebug()
    comm.setSlewLimit(0.0)
    kbint = KeyboardInterface(comm)
    
    # Pipe the output from the process so we can read out the bird location
    tracking = subprocess.Popen(args = './tennis_ballbot', shell = False, stdout = subprocess.PIPE)

    # I-Bird preflight initialization
    try:
        yaw_offset = 0.0
        while True:

            if kbmon.hasKey():
                c = kbmon.getCh()
                if c == 'q':
                    break               
                kbint.process(c)

            line = tracking.stdout.readline()
            if line != '' and line[0] == '#':
                components = line[1:-1].split(',')
            #    print components
                y = int(components[1])
                x = int(components[0])
                if y < 120:
            #        #kbint.process('[')
                    comm.setRegulatorOffsets((yaw_offset, 0.0, 0.8))
                elif y > 120:
            #        #kbint.process(']')
                    comm.setRegulatorOffsets((yaw_offset, 0.0, 1.0))
                yaw_error_pixel = 160.0 - x;
                yaw_error_rad = (60.0/320.0)*0.0174533*yaw_error_pixel;
                yaw_offset = 3.0*yaw_error_rad;
                #print str(yaw_offset)
                #comm.rotateRefGlobal(quatGenerate(yaw_offset, (0,0,1)))
            time.sleep(0.02)

    except Exception as e:
        print e

    finally:

        # Clean up
        kbint.process('1') # Stop flight
        tracking.terminate()
        comm.close()
        #kbint.close()
        kbmon.close()
        xb.close()

