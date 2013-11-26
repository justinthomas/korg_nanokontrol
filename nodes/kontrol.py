#!/usr/bin/env python
#
# joystick input driver for Korg NanoKontrol2 input device
#
# Author: Austin Hendrix
# Modified for NanoKontrol2 by: Justin Thomas

import roslib; roslib.load_manifest('korg_nanokontrol')
import rospy

import pygame
import pygame.midi

import sys

from sensor_msgs.msg import *

control_axes = [{
  # Syntax is channel: slide or knob number
  # sliders
  # Indicies 0 - 7 
  2:  0,  3:  1,  4:  2,  5:  3,  6:  4,  7:  5,  8:  6, 9:  7, 1:  8,
  # knobs
  # Indicies 8 - 15
  10:  9, 11: 10, 12: 11, 13: 12, 14: 13, 15: 14, 16: 15}]

control_buttons = [[
  # Row of Solo, Row of Mute, row of Rec
  # Indicies 0 - 7, 8 - 15, and 16 - 23
  32, 33, 34, 35, 36, 37, 38, 39, 48, 49, 50, 51, 52, 53, 54, 55, 64, 65, 66, 67, 68, 69, 70, 71,
  # rew, ff, stop, play, rec
  # Indicies 24 - 28
  43, 44, 42, 41, 45,
  # Track L, Track R, Marker Set, Marker Left, Marker Right, Cycle
  # Indicies 29 - 34
  58, 59, 60, 61, 62, 46]]

def main():
   pygame.midi.init()
   devices = pygame.midi.get_count()
   if devices < 1:
      print "No MIDI devices detected"
      exit(-1)
   print "Found %d MIDI devices" % devices

   if len(sys.argv) > 1:
      input_dev = int(sys.argv[1])
   else:
      input_dev = pygame.midi.get_default_input_id()
      if input_dev == -1:
         print "No default MIDI input device"
         exit(-1)
   print "Using input device %d" % input_dev

   controller = pygame.midi.Input(input_dev)

   rospy.init_node('kontrol')
   pub = rospy.Publisher('joy', Joy, latch=True)

   m = Joy()
   m.axes = [ 0 ] * 16
   m.buttons = [ 0 ] * 35
   mode = None

   p = False

   while not rospy.is_shutdown():
      m.header.stamp = rospy.Time.now()
      # count the number of events that are coalesced together
      c = 0
      while controller.poll():
         c += 1
         data = controller.read(1)
         #print data
         # loop through events received
         for event in data:
            control = event[0]
            timestamp = event[1]

            # look for continuous controller commands
            if (control[0] & 0xF0) == 176:
               control_id = control[1] | ((control[0] & 0x0F) << 8)

               # guess initial mode based on command
               if mode is None:
                  candidate = None
                  for index, control_axis in enumerate(control_axes):
                     if control_id in control_axis:
                        if candidate is not None:
                           candidate = None
                           break
                        candidate = index
                  for index, control_button in enumerate(control_buttons):
                     if control_id in control_button:
                        if candidate is not None:
                           candidate = None
                           break
                        candidate = index
                  mode = candidate
                  if mode is None:
                     print 'skipped because mode is yet unknown'
                     continue

               if control_id in control_axes[mode]:
                  control_val = float(control[2] - 63) / 63.0
                  if control_val < -1.0:
                     control_val = -1.0
                  if control_val > 1.0:
                     control_val = 1.0

                  axis = control_axes[mode][control_id]
                  m.axes[axis] = control_val
                  p = True

               if control_id in control_buttons[mode]:
                  button = control_buttons[mode].index(control_id)

#                  print "Button index: %d" % button

                  if control[2] != 0:
                     m.buttons[button] = 1
                  else:
                     m.buttons[button] = 0
                  p = True
            # look for mode commands
            elif control[0] == 79:
               mode = control[1]
               m.buttons[24] = mode
               p = True

      if p:
         pub.publish(m)
         p = False

      rospy.sleep(0.1) # 10Hz maximum input
                  


if __name__ == '__main__':
   try:
      main()
   except rospy.ROSInterruptException: pass
