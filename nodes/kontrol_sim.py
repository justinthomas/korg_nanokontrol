#!/usr/bin/env python
#
# joystick input driver simulator for Korg NanoKontrol2 input device
#
# Author: Justin Thomas

import roslib; roslib.load_manifest('korg_nanokontrol2')
import rospy

import pygame
import pygame.midi

import sys

from sensor_msgs.msg import *

#control_axes = [{
#  # Syntax is channel: slide or knob number
#  # sliders
#  # Indicies 0 - 7 
#  0:  0,  1:  1,  2:  2,  3:  3,  4:  4,  5:  5,  6:  6, 7:  7, 23:  15,
#  # knobs
#  # Indicies 8 - 15
#  17:  9, 18: 10, 19: 11, 20: 12, 21: 13, 22: 14, 16: 8}]

#control_buttons = [[
#  # Row of Solo, Row of Mute, row of Rec
#  # Indicies 0 - 7, 8 - 15, and 16 - 23
#  32, 33, 34, 35, 36, 37, 38, 39, 48, 49, 50, 51, 52, 53, 54, 55, 64, 65, 66, 67, 68, 69, 70, 71,
#  # rew, ff, stop, play, rec
#  # Indicies 24 - 28
#  43, 44, 42, 41, 45,
#  # Track L, Track R, Marker Set, Marker Left, Marker Right, Cycle
#  # Indicies 29 - 34
#  58, 59, 60, 61, 62, 46]]

def main():
 
   rospy.init_node('kontrol')
   pub = rospy.Publisher('nanokontrol2', Joy, latch=True, queue_size=1)

   m = Joy()
   m.header.stamp = rospy.Time.now()
   m.axes = [ 0 ] * 16
   m.buttons = [ 0 ] * 35

   # Note: the first argument is the script name
   if len(sys.argv) > 4:
      m.axes[0] = float(sys.argv[2])
      m.axes[1] = float(sys.argv[3])
      m.axes[2] = float(sys.argv[4])
      m.buttons[int(sys.argv[1])] = 1

   elif len(sys.argv) > 2:
      m.axes[0] = float(sys.argv[1])
      m.axes[1] = float(sys.argv[2])
      m.axes[2] = float(sys.argv[3])
   
   else:
      m.buttons[int(sys.argv[1])] = 1

   pub.publish(m)
   rospy.sleep(1)

if __name__ == '__main__':
   try:
      main()
   except rospy.ROSInterruptException: pass
