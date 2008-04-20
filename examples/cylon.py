#!/usr/bin/env python
#
# Example UBW program
# Displays a sweeping lit LED across an array of LEDs.
# This example should be called cylon if you are old or young
# if you are somewhere in the middle, then probably it should
# be called Knight Rider.
#
# Copyright (C) 2008
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA

# The circuit for this example is simply 8 LEDs with current limiting resistors
# in series and using the pins on the UBW as sinks on port B pins.

import ubw
import time

def Run(ubw_device, pin_count):
  up = 1
  pin = 0
  while True:
    u.PinOutput('B', pin, 1)
    if pin == pin_count-1:
      up = -1
    if pin == 0:
      up = 1
    pin += up
    u.PinOutput('B', pin, 0)
    time.sleep(0.1)

if __name__ == '__main__':
  u = ubw.Ubw('/dev/ttyACM0')

  pins = range(8)

  for pin in pins:
    u.PinDirection('B', pin, ubw.OUTPUT)
    u.PinOutput('B', pin, 1)
        
  try:
    Run(u, len(pins))
  except KeyboardInterrupt:
    u.Stop()
