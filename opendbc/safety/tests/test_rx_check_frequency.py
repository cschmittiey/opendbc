#!/usr/bin/env python3
"""Every declared RxCheck frequency must be >= 10Hz.

safety_tick() in opendbc/safety/safety.h marks any rx check declared below
10Hz as frequency_invalid, which permanently sets safetyRxChecksInvalid and
forces controls_allowed = false at every 1Hz tick (commaai/opendbc#3359).
A sub-10Hz declaration compiles fine and passes the per-mode safety tests,
then breaks engagement in the car with a "Controls Mismatch" alert — this
happened with the Volvo SPA 0x349 CRUISE_CONTROL check declared at 5U.
For messages that genuinely arrive slower than 10Hz, declare 10U: the
lagging threshold is max(10 * period, 1s), so it never drops below 1s.
"""
import glob
import os
import re
import unittest

MODES_GLOB = os.path.join(os.path.dirname(__file__), "..", "modes", "*.h")

# matches one RxCheck msg entry: {ADDR, BUS, LEN, FREQU, ...}
# (TX CanMsg entries don't match: their 4th field is .check_relay, not a frequency)
RX_MSG = re.compile(r"\{\s*([A-Za-z0-9_]+)\s*,\s*([A-Za-z0-9_]+)\s*,\s*(\d+)\s*,\s*(\d+)U")

MIN_FREQ_HZ = 10


class TestRxCheckFrequency(unittest.TestCase):
  def test_min_declared_frequency(self):
    violations = []
    for path in sorted(glob.glob(MODES_GLOB)):
      with open(path) as f:
        for lineno, line in enumerate(f, 1):
          for m in RX_MSG.finditer(line):
            addr, _, _, freq = m.groups()
            if int(freq) < MIN_FREQ_HZ:
              violations.append(f"{os.path.basename(path)}:{lineno}: {addr} declared at {freq}U (< {MIN_FREQ_HZ}Hz)")
    self.assertEqual(violations, [], "rx checks declared below the 10Hz minimum enforced by safety_tick():\n" + "\n".join(violations))


if __name__ == "__main__":
  unittest.main()
