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

    pixel_pos = []
    row = 0;
    
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
        start_time = datetime.now()
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
                #wx = int(components[2])
                wx = 160;
                wy = 120;
                
                #wy = int(components[3])
                #if (abs(yaw_offset) > 1.7):
                #        yaw_offset = 0
                if y < wy:
            #        #kbint.process('[')
                    comm.setRegulatorOffsets((yaw_offset, 0.0, 1.0))
                    #comm.setGlobalRateSlew((yaw_offset, 0.0, 0.0))
                elif y > wy:
            #        #kbint.process(']')
                    comm.setRegulatorOffsets((yaw_offset, 0.0, 1.0))
                    #comm.setGlobalRateSlew((yaw_offset, 0.0, 0.0))
                yaw_error_pixel = wx - x;
                #if (yaw_error_pixel < 20):
                #    comm.zeroEstimate()
                #    comm.setRegulatorRef((1.0,0.0,0.0,0.0))
                yaw_error_rad = (60.0/320.0)*0.0174533*yaw_error_pixel;
                yaw_offset = 4.0*yaw_error_rad;
                print str(x) + "," + str(y) + "," + str(wx) + "," + str(wy)
                #print str(yaw_offset) + "," + str(yaw_error_rad) + "," + str(yaw_error_pixel)
                #comm.rotateRefGlobal(quatGenerate(yaw_offset, (0,0,1)))
                end_time = datetime.now()
                round_time = end_time - start_time
		dt = round_time.microseconds/1000.0
                pixel_pos.append([dt,x,y,wx,wy,yaw_error_pixel,yaw_error_rad,yaw_offset])
            time.sleep(0.02)

    except Exception as e:
        print e

    finally:
        
        today = datetime.today()
	d = str(today.year) + "_" + str(today.month) + "_" + str(today.day)
	t = str(today.hour) + "_" + str(today.minute) + "_" + str(today.second)
	fname = 'TrackingOutput-' + d + '-' + t + '.txt'
	record_log = open(fname, 'w')
        record_log.write("Time\tBird X\tBird Y\tWindow X\tWindow Y\tPixel" +
                         "Error\tYaw Rad Error\tYaw Offset\n")
        for i in pixel_pos.shape[0]
            record_log.write(str(pixel_pos[i][0]) + "\t" + str(pixel_pos[i][1]) +
                             "\t" + str(pixel_pos[i][2]) + "\t" + str(pixel_pos[i][3]) +
                             "\t" + str(pixel_pos[i][4]) + "\t" + str(pixel_pos[i][5]) +
                             "\t" + str(pixel_pos[i][6]) + "\t" + str(pixel_pos[i][7]) + "\n")
	
	
        # Clean up
        kbint.process('1') # Stop flight
        tracking.terminate()
        comm.close()
        #kbint.close()
        kbmon.close()
        xb.close()

