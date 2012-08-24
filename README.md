ibird-tracking
==============

Authors: Humphrey Hu, Ryan Julian, Cameron Rose

Code for tracking robots using OpenCV on a OMAP.

Use the commands in "keyboardWireLinux.py" to send commands to the iBird.

Press the 'm' key in the console to start the video feed. Drag the mouse in the
camera frame to draw a rectangle in the frame. Unfortunately, this rectangle
appears in the recorded video and not in the real-time feed (will fix this
eventually). Right click in the camera frame to grab a new frame, as the only
one displayed, initially, is the first frame to save memory. To send further
commands to the iBird click on the console to return focus to the console and
use keyboard commands as before. When you want to halt video recording, click on
the video window and press any key. It will take a moment to write the video to
disk, then the window will close. Finally, press the 'q' key in the console to
stop the keyboard manager for the iBird and 'Ctrl-C' to stop the program. The
output will be an 'mjpg' video and a text file of the position and rotation
inputs sent to the ibird to steer it towards the window.

IMPORTANT NOTE: If you do not wish to halt video capture, make sure that you
click away from the camera capture window before pressing any keys. This "issue"
will be fixed later so that only one key will halt capture to prevent this
annoyance from occurring.
