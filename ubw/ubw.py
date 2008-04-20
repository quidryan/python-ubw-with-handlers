#!/usr/bin/env python
#
# ubw - Python interface to the USB Bit Wacker
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

import Queue
import re
import select
import threading
import traceback

import serial

# Constants

BITMASK = [1<<i for i in range(8)]

OUTPUT = False
INPUT = True

A, B, C = 'ABC'

ANALOG_PINS = ['AN%d' % (i,) for i in range(12)]
ANALOG_MAP = {'AN0': ('A', 0),
              'AN1': ('A', 1),
              'AN2': ('A', 2),
              'AN3': ('A', 3),
              'AN4': ('A', 5),
              'AN5': None,
              'AN6': None,
              'AN7': None,
              'AN8': ('B', 2),
              'AN9': ('B', 3),
              'AN10': ('B', 1),
              'AN11': ('B', 4)}

# Responses

OK = re.compile(r'(?P<type>OK)')
INPUT_STATE = re.compile(r'I,(?P<porta>\d{3}),(?P<portb>\d{3}),(?P<portc>\d{3})')
VERSION = re.compile(r'UBW FW (?P<letter>[A-Z]) Version '
                     '(?P<major>\d+).(?P<minor>\d+).(?P<sub>\d+)')
ANALOG_STATE = re.compile(r'A'
                           '(,(?P<AN0>\d{4}))?'
                           '(,(?P<AN1>\d{4}))?'
                           '(,(?P<AN2>\d{4}))?'
                           '(,(?P<AN3>\d{4}))?'
                           '(,(?P<AN4>\d{4}))?'
                           '(,(?P<AN5>\d{4}))?'
                           '(,(?P<AN6>\d{4}))?'
                           '(,(?P<AN7>\d{4}))?'
                           '(,(?P<AN8>\d{4}))?'
                           '(,(?P<AN9>\d{4}))?'
                           '(,(?P<AN10>\d{4}))?'
                           '(,(?P<AN11>\d{4}))?')
MEMORY_READ = re.compile(r'MR,(?P<value>\d{1,3})')
PIN_INPUT = re.compile(r'PI,(?P<value>\d)')
ERROR = re.compile(r'!(?P<error_code>\d) Err: (?P<error_string>.*)')


# Helper functions

def ValueToMap(value):
  """Converts a single integer (0-255) representing a byte into a
     list of 8 booleans.

  Args:
    value: Integer value to convert
  Retruns:
    list of boolean values for each bit
  """
  return [bool(m&value) for m in BITMASK]

def MapToValue(map):
  """Converts a list of 8 booleans representing a byte into a 
     single integer value (0-255)

  Args:
    map: list of boolean values, one for each bit
  Returns:
    integer (0-255) representing the byte
  """
  return sum(v*m for v,m in zip(map, BITMASK))

# Exceptions

class Error(Exception):
  """Base exception for this module."""

class BadPinState(Error):
  """Invalid Pin State."""

class TooManyAsciiBytes(Error):
  """Too many ASCII bytes were attempted to be sent via 'BO'"""

class TooManyBytes(Error):
  """Too many bytes were attempted to be sent via 'BS'"""

class VersionMismatch(Error):
  """UBW Version may be too old (or too new!)"""

# UBW Exceptions
# Exceptions that can be raised by the UBW sending error messages.

class UbwError(Error):
  """Base Error"""

class Unused(UbwError):
  """Unused Error"""

class TxBufferOverrun(UbwError):
  """TX Buffer overun"""

class RxBufferOverrun(UbwError):
  """RX Buffer overrun"""

class MissingParameter(UbwError):
  """Missing Parameter"""

class NeedCommaNext(UbwError):
  """Need comma next"""

  CHAR_MATCH = re.compile(r'found: \'(?P<char>.)\'')

  def __init__(self, error_string):
    self.char = self.CHAR_MATCH.search(error_string).groupdict()['char']
    UbwError.__init__(self, error_string)

class InvalidParameterValue(UbwError):
  """Invalid parameter value"""

class ExtraParameter(UbwError):
  """Extra parameter"""

class UnknownCommand(UbwError):
  """Unknown command"""

  COMMAND_MATCH = re.compile(r'command \'(?P<command>.+)\'')

  def __init__(self, error_string):
    self.command = self.COMMAND_MATCH.search(
        error_string).groupdict()['command']
    UbwError.__init__(self, error_string)  

# Tuple of UBW based errors that can be used to reference the Exception by the
# error number sent by the UBW
ERRORS = (Unused, Unused, TxBufferOverrun, RxBufferOverrun, MissingParameter,
          NeedCommaNext, InvalidParameterValue, ExtraParameter, UnknownCommand)

## Main Classes

class Pin(object):
  """Class for tracking the current state of a pin on the UBW."""

  def __init__(self):
    self.direction = None
    self.is_analog = False
    self.analog = None
    self.input = None
    self.output = None
    self.rc = 0
    self.SetDirection(OUTPUT)

  def SetOutputValue(self, value):
    """Set the output value of the pin.

    Args:
      value: boolean value representing the state of the pin
    """
    if not self.direction == OUTPUT:
      self.SetDirection(OUTPUT)
    self.output = bool(value)

  def SetInputValue(self, value):
    """Set the current digital input value of the pin.

    Args:
      value: boolean logic value of the pin
    """
    self.input = bool(value)

  def SetAnalogValue(self, value):
    """Set the current analog input value of the pin.
    
    Args:
      value: integer analog input value in millivolts.
    """
    self.analog = value

  def SetDirection(self, direction):
    """Set the direction of the pin.
    
    Args:
      direction: INPUT or OUTPUT
    """
    self.direction = direction
    self.analog = None
    if direction == OUTPUT:
      self.input = None
      self.output = False
    else:
      self.input = None
      self.output = None
    
class Port(object):
  """Class for representing a port on the UBW."""

  def __init__(self, name):
    """Initialize the port.
    
    Args:
      name: string name of the port.
    """
    self.name = name
    self._pins = [Pin() for _ in range(8)]

  def __getitem__(self, item):
    """Allow for key lookups using the pin number."""
    return self._pins[item]

  def __iter__(self):
    """Iterate through the pins in the port."""
    for pin in self._pins:
      yield pin

  def SetPinsFromValue(self, port_value):
    """Set the pins' input vlaue in the port according to a byte value.

    Args:
      port_value: integer (0-255) representing the value of the port byte
    """
    for pin_num, pin_value in enumerate(ValueToMap(int(port_value))):
      self[pin_num].SetInputValue(pin_value)

  def SetOutputFromValue(self, port_value):
    """Set the pins' output value in the port according to a byte value.

    Args:
      port_value: integer (0-255) representing the value of the port byte
    """
    for pin_num, pin_value in enumerate(ValueToMap(int(port_value))):
      self[pin_num].SetOutputValue(pin_value)

  def SetDirectionFromValue(self, port_value):
    """Set the pins' direction in the port according to a byte value.

    Args:
      port_value: integer (0-255) representing the value of the port byte
    """
    for pin_num, pin_value in enumerate(ValueToMap(int(port_value))):
      self[pin_num].SetDirection(pin_value)

class Ubw(object):
  """Class representing the UBW."""

  def __init__(self, interface=None):
    """Initialize the UBW.

    Args:
      interface: string name of the device to open for comm with the UBW.
    """
    self.version_letter = None
    self.version_major = None
    self.version_minor = None
    self.version_sub = None
    self.memory = {}

    self.exception_callbacks = {}
    self.analog_callbacks = {}

    # Input Watch Queues
    # We use these to communicate with the input handling thread

    self.pin_input_queue = Queue.Queue()
    self.pin_ready_queue = Queue.Queue()
    self.memory_read_queue = Queue.Queue()
    self.memory_ready_queue = Queue.Queue()

    self.responses = ((INPUT_STATE, self.HandleInputState),
                 (ANALOG_STATE, self.HandleAnalogState),
                 (MEMORY_READ, self.HandleMemoryRead),
                 (PIN_INPUT, self.HandlePinInput),
                 (ERROR, self.HandleError),
                 (VERSION, self.HandleVersion),
                 (OK, self.HandleOk))
    if interface is not None:
      self.interface = serial.Serial(interface)
    else:
      self.interface = None
    
    self.ports = {}
    for port_name in 'ABC':
      port = Port(port_name)
      self.ports[port_name] = port
      setattr(self, port_name, port)
      for pin_num, pin in enumerate(port):
        setattr(self, '%s%d' % (port_name, pin_num), pin)

    for analog_name, pin_info in ANALOG_MAP.items():
      if pin_info is None:
        setattr(self, analog_name, None)
      else:
        port_name, pin_num = pin_info
        setattr(self, analog_name, self[port_name][pin_num])

    # Set the UBW to a known state

    self.Reset()
    self.ConfigureUbw(send_ok=False)
    self.Version()
    if ((self.version_letter, self.version_major, self.version_minor)
        != (1, 4)):
      raise VersionMismatch(self.version_letter, self.version_major,
                            self.version_minor, self.version_sub)

    # Start Input Handling
    self.running = True
    self.thread = threading.Thread(target=self.InputLoop)
    self.thread.start()

  def __del__(self):
    self.running = False

  def __getitem__(self, item):
    return getattr(self, item)

  def __iter__(self):
    """Allow for interation over self as interation over ports."""
    for port_name in sorted(self.ports):
      yield self.ports[port_name]

  def Stop(self):
    """Stop input handling thread, stop timers, stop PWM generators."""
    self.running = False
    self.Timer(0)
    self.Timer(0, analog_mode=False)
    for port_name in self.ports:
      for pin_num, pin in enumerate(self[port_name]):
        if pin.rc != 0:
          self.ServoOutput(port_name, pin_num)

  def RegisterExceptionCallback(self, exception, callback):
    """Register a callback to be called when a particular exception is handled.

    Args:
      exception: exception class to watch for
      callback: callback function to call with exception as an argument when
                the exception is raised
    """
    self.exception_callbacks[exception] = callback

  def RegisterAnalogCallback(self, analog_pin, callback):
    """Register a callback to be called when a particular pin has analog data.

    Args:
      analog_pin: string name of the analog pin
      callback: function to be called with analog pin and current value as args
    """
    self.analog_callbacks[analog_pin] = callback

  def GetVersionString(self):
    """Retrieve the version of the UBW firmware as a string.

    Returns:
      string of the UBW firmware version name
    """
    version_elements =  (self.version_letter, self.version_major,
                         self.version_minor, self.version_sub)
    if None in version_elements:
      return None
    return 'UBW FW %s Version %d.%d.%d' % version_elements

  def ParseResponse(self, response):
    """Handle a line of response from the UBW.

    Args:
      response: string response line sent by the UBW
    """
    for matcher, method in self.responses:
      match = matcher.search(response)
      if match is not None:
        method(match.groupdict())
        return
    # TODO: raise exception
    print 'Unhandled Response', response

  def InputLoop(self):
    """Main loop for checking for pending input and dispatching it."""
    poller = select.poll()
    poller.register(self.interface.fd, select.POLLIN|select.POLLPRI)
    while self.running:
      if poller.poll(100):
        line = self.interface.readline()
        try:
          self.ParseResponse(line)
        except Exception, e:
          if isinstance(e, tuple(self.exception_callbacks.keys())):
            self.exception_callbacks[e.__class__](e)
          else:
            # TODO: add a strict mode that actually raises the exception here
            print e.__class__, e
            traceback.print_exc()

  def HandleOk(self, ok):
    """Handle the 'OK' response from the UBW.
    
    Args:
      ok: dictionary of response values
    """
    pass

  def HandleInputState(self, input_state):
    """Handle the current pin input state response from the UBW.

    Args:
      input_state: dictionary of response values
    """
    for port_name, port_data in (('A', input_state['porta']),
                                 ('B', input_state['portb']),
                                 ('C', input_state['portc'])):
      self[port_name].SetPinsFromValue(int(port_data))

  def HandleVersion(self, version):
    """Handle the version response from the UBW.

    Args:
      version: dictionary of response values.
    """
    self.version_letter = version['letter']
    self.version_major = int(version['major'])
    self.version_minor = int(version['minor'])
    self.version_sub = int(version['sub'])

  def HandleAnalogState(self, analog_state):
    """Handle the current analog values of analog pins.

    Args:
      analog_state: dictionary of response values
    """
    for analog_pin, value in analog_state.items():
      millivolts = 0
      if value is not None:
        for pos, char in enumerate(value):
          if char != '0':
            millivolts = (int(value[pos:])*5000)/1024
            break
      if analog_pin in self.analog_callbacks:
        self.analog_callbacks[analog_pin](analog_pin, millivolts)
      if value is not None and ANALOG_MAP[analog_pin] is not None:
        port_name, pin_num = ANALOG_MAP[analog_pin]
        self[port_name][pin_num].SetAnalogValue(millivolts)        

  def HandleMemoryRead(self, value):
    """Handle a memory read response from the UBW.

    Args:
      value: dictionary of response values
    """
    address = self.memory_read_queue.get()
    self.memory[address] = int(value['value'])
    self.memory_ready_queue.put(address)

  def HandlePinInput(self, value):
    """Handle a pin input response from the UBW.

    Args:
      value: dictionary of response values
    """
    port, pin = self.pin_input_queue.get()
    self[port][pin].SetInputValue(value['value'])
    self.pin_ready_queue.put((port, pin))

  def HandleError(self, error):
    """Handle error responses from the UBW.

    Args:
      error: dictionary of respone values
    """
    raise ERRORS[int(error['error_code'])](error['error_string'])

  def SendCommand(self, command):
    """Send a command to the UBW.

    Args:
      command: string command to send (does not include line termination)
    """
    if self.interface is not None:
      self.interface.write('%s\n' % command)

  # UBW Base Commands
  # These methods match pretty much 1:1 with the basic UBW command set

  def Configure(self, direction_a, direction_b, direction_c, analog_count):
    """Configure pin directions for ports and set which ports are analog.
    
    Args:
      direction_a: integer value of PORTA byte (0-255)
      direction_b: integer value of PORTB byte (0-255)
      direction_c: integer value of PORTC byte (0-255)
      analog_count: number of pins to set as analog pins (0-12)
    """
    for port_name, direction in (('A', direction_a),
                                 ('B', direction_b),
                                 ('C', direction_c)):
      self[port_name].SetDirectionFromValue(direction)
      for pin in self[port_name]:
        pin.is_analog = False
      for pin_name in ANALOG_PINS[:analog_count]:
        pin = getattr(self, pin_name)
        if pin is not None:
          pin.is_analog = True
    self.SendCommand('C,%d,%d,%d,%d' % (direction_a, direction_b, direction_c,
                                        analog_count))

  def OutputState(self, output_a, output_b, output_c):
    """Set output values for ports.

    Args:
      output_a: integer value of PORTA byte (0-255)
      output_b: integer value of PORTB byte (0-255)
      output_c: integer value of PORTC byte (0-255)
    """
    for port_name, output in (('A', output_a),
                              ('B', output_b),
                              ('C', output_c)):
      self[port_name].SetOutputFromValue(output)
    self.SendCommand('O,%d,%d,%d' % (output_a, output_b, output_c))

  def InputState(self):
    """Retrieve the current input state of ports."""
    self.SendCommand('I')

  def Version(self):
    """Retrieve the version of the UBW."""
    self.SendCommand('V')

  def Reset(self):
    """Reset the UBW."""
    self.SendCommand('R')

  def Timer(self, delay, analog_mode=False):
    """Set the duration of the input or analog timer.
    
    Args:
      delay: integer delay between responses in milliseconds (0-30000)
      analog_mode: False for input timer, True for analog timer
    """
    if delay < 0 or delay > 30000:
      raise InvalidDelay
    self.SendCommand('T,%d,%d' % (delay, analog_mode))

  def SampleAnalog(self):
    """Sample the analog inputs."""
    self.SendCommand('A')

  def MemoryRead(self, address):
    """Read a memory address.
    
    Args:
      address:  integer address to read (0-4095)
    Returns:
      current integer value at that address
    """
    self.memory_read_queue.put(address)
    self.SendCommand('MR,%d' % (address,))
    while True:
      ready_address = self.memory_ready_queue.get()
      if ready_address != address:
        self.memory_ready_queue.put(ready_address)
      else:
        return self.memory[address]

  def MemoryWrite(self, address, value):
    """Write a value to a memory address.
    
    Args:
      address: integer address to write to (0-4095)
      value: integer value to write to that address (0-255)
    """
    if address < 0 or address >= 4096:
      raise InvalidMemoryAddress
    if value < 0 or value >= 256:
      raise InvalidMemoryValue
    self.SendCommand('MW,%d,%d' % (address, value))

  def PinDirection(self, port, pin, direction):
    """Set the direction of a pin.
    
    Args:
      port: string name of the port ('A', 'B', 'C')
      pin: integer number of the pin (0-7)
      direction: direction of the pin INPUT or OUTPUT
    """
    self.SendCommand('PD,%s,%d,%d' % (port, pin, direction))

  def PinInput(self, port, pin):
    """Read the value from a pin.
    
    Args:
      port: string name of the port ('A', 'B', 'C')
      pin: integer number of the pin (0-7)
    Returns:
      boolean value of the current pin input value
    """
    self.pin_input_queue.put((port, pin))
    self.SendCommand('PI,%s,%d' % (port, pin))
    while True:
      ready_pin = self.pin_ready_queue.get()
      if ready_pin != (port, pin):
        self.pin_ready_queue.put(ready_pin)
      else:
        return self[port][pin].input

  def PinOutput(self, port, pin, value):
    """Set the value of an output pin.
    
    Args:
      port: string name of the port ('A', 'B', 'C')
      pin: integer number of the pin (0-7)
      value: boolean value to set as the output logic value
    """
    self.SendCommand('PO,%s,%d,%d' % (port, pin, value))

  def ConfigureUbw(self, send_ok=None):
    """Configure operation parameters, only parameters with non-None values
       will be configured.
    
    Args:
      send_ok: boolean, True if the UBW should send 'OK' responses to
               successful commands
    """
    for param, value in ((1, send_ok),
                         ):
      if value is not None:
        self.SendCommand('CU,%d,%d' % (param, value))

  def ServoOutput(self, port, pin, value):
    """Set a pin to a PWM output value.
    
    Setting the value to 0 turns off the PWM.

    Args:
      port: string name of the port ('A', 'B', 'C')
      pin: integer number of the pin (0-7)
      value: integer duty cycle time (0-11890)
    """
    if value < 0 or value > 11890:
      raise InvalidPwmValue
    self[port][pin].rc = value
    self.SendCommand('RC,%s,%d,%d' % (port, pin, value))

  def BulkConfigure(self, initial, wait_mask, wait_delay, strobe_mask,
                    strobe_delay):
    """Configure the port for binary output.
    
    See UBW documentation for more details:
      http://greta.dhs.org/UBW/Doc/FirmwareDDocumentation_v143.html

    Args:
      initial: integer initial value written to PORTA
      wait_mask: integer busy bits to watch for
      wait_delay: integer max time to wait for wait delay busy bit to be set
      stobe_mask: strobe bits to be inverted after byte is written to PORTB
      strobe_delay: time in 830ns units that strobe bits are inverted
    """
    self.SendCommand('BC,%d,%d,%d,%d,%d' % (initial, wait_mask, wait_delay,
                                             strobe_mask, strobe_delay))

  def BulkOutput(self, ascii_bytes):
    """Output a stream of ascii_bytes to PORTB.
    
    See UBW documentation for more details:
      http://greta.dhs.org/UBW/Doc/FirmwareDDocumentation_v143.html

    Args:
      ascii_bytes: string of hex values represented as
                   2 ascii characters per byte (maximum of 30 bytes)
    """
    if len(ascii_bytes) > 30:
      raise TooManyAsciiBytes
    self.SendCommand('BO,%s' % (ascii_bytes,))

  def BulkStream(self, binary_bytes):
    """Output a stream of binary bytes to PORTB.
    
    See UBW documentation for more details:
      http://greta.dhs.org/UBW/Doc/FirmwareDDocumentation_v143.html

    Args:
      binary_bytes: string of bytes
    """
    if len(binary_bytes) > 56:
      raise TooManyBytes
    self.SendCommand('BS,%d,%s' % (len(binary_bytes), binary_bytes,))
