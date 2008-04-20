#!/usr/bin/env python
#
# Example UBW program
# Displays current count on pins in binary format
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
  clock = 0
  while True:
    map = ubw.ValueToMap(clock)[:pin_count]
    for pin_num, value in enumerate(map):
      ubw_device.PinOutput('B', pin_num, not value)
    time.sleep(1)
    clock += 1

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
