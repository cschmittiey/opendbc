#!/usr/bin/env python3
"""
Safety tests for Volvo CMA/SPA.

The safety mode lives at ``opendbc/safety/modes/volvo.h`` and is parameterised
by ``safetyParam``:

  - ``safetyParam == 0``       → CMA platform (Volvo XC40 Recharge)
  - ``safetyParam == VOLVO_FLAG_SPA`` → SPA platform (Volvo S60 Recharge,
    Polestar 2)

The two platforms share LCA/PSCM/etc. addresses on the main and party buses
but use *different* PT-bus addresses and signal scales for ECM_1, BUS1_SPEED,
and BUS1_CRUISE_CONTROL. This test file exercises both platforms through the
same generic ``CarSafetyTest`` harness so that any future divergence between
``carstate.py`` and ``volvo.h`` — e.g. a threshold drifting out of sync — is
caught on a laptop instead of in the car.

Companion to: ``opendbc/car/volvo/carstate.py`` (must agree on thresholds).
"""

import unittest

from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety


# Must match VOLVO_FLAG_SPA in opendbc/safety/modes/volvo.h
VOLVO_FLAG_SPA = 1

# Bus layout (must match volvo.h)
VOLVO_MAIN_BUS = 0
VOLVO_PT_BUS = 1
VOLVO_PARTY_BUS = 2

# Shared platform-independent addresses
VOLVO_LCA_STEER     = 0x58
VOLVO_LCA_2         = 0x69
VOLVO_LCA_3         = 0x57
VOLVO_LCA_4         = 0x90
VOLVO_LCA_5         = 0x67
VOLVO_LCA_6         = 0x97
VOLVO_LCA_7         = 0x92
VOLVO_PSCM          = 0x16
VOLVO_PSCM_RELATED  = 0x17

# Addresses panda TX'd with .check_relay = true, grouped by bus
_TX_PARTY_BUS_ADDRS = (
  VOLVO_LCA_STEER, VOLVO_LCA_2, VOLVO_LCA_3, VOLVO_LCA_4,
  VOLVO_LCA_5, VOLVO_LCA_6, VOLVO_LCA_7,
)
_TX_MAIN_BUS_ADDRS = (VOLVO_PSCM, VOLVO_PSCM_RELATED)


class TestVolvoSafetyBase(common.CarSafetyTest):
  """Shared RX/TX/relay tests for both CMA and SPA.

  Subclasses pin down the PT-bus DBC and per-platform gas/cruise encoding.
  """

  TX_MSGS = (
    [[a, VOLVO_PARTY_BUS] for a in _TX_PARTY_BUS_ADDRS]
    + [[a, VOLVO_MAIN_BUS] for a in _TX_MAIN_BUS_ADDRS]
  )
  RELAY_MALFUNCTION_ADDRS = {
    VOLVO_PARTY_BUS: _TX_PARTY_BUS_ADDRS,
    VOLVO_MAIN_BUS: _TX_MAIN_BUS_ADDRS,
  }
  # volvo.h installs no custom fwd hook, so the panda uses the default main<->party
  # forwarding (bus 0 <-> bus 2). For each TX addr with .check_relay=true, the panda
  # automatically refuses to forward from the bus where stock traffic lives TO the
  # bus where openpilot is TX'ing — that's what keeps the stock ECU from fighting us.
  FWD_BLACKLISTED_ADDRS = {
    VOLVO_MAIN_BUS: list(_TX_PARTY_BUS_ADDRS),   # stock on 0 must not forward to 2
    VOLVO_PARTY_BUS: list(_TX_MAIN_BUS_ADDRS),   # stock on 2 must not forward to 0
  }
  STANDSTILL_THRESHOLD = 0.1  # m/s, matches volvo.h

  # Subclasses set these
  SAFETY_PARAM: int = 0
  PT_DBC: str = ""

  MID_DBC = "volvo_mid_1"

  @classmethod
  def setUpClass(cls):
    if cls.__name__ == "TestVolvoSafetyBase":
      cls.safety = None
      raise unittest.SkipTest
    super().setUpClass()

  def setUp(self):
    self.pt_packer = CANPackerSafety(self.PT_DBC)
    self.mid_packer = CANPackerSafety(self.MID_DBC)
    # CarSafetyTest.packer is referenced by a few generic tests; point it at
    # the mid-bus DBC since most shared messages live there.
    self.packer = self.mid_packer
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.volvo, self.SAFETY_PARAM)
    self.safety.init_tests()

  # ---- Abstract methods from CarSafetyTest ----

  def _user_brake_msg(self, brake):
    # volvo.h reads BRAKE_PEDAL_PRESSED_B (bit 46, factor 1, active high)
    values = {"BRAKE_PEDAL_PRESSED_B": 1 if brake else 0}
    return self.mid_packer.make_can_msg_safety("LCA_2", VOLVO_MAIN_BUS, values)

  def _speed_msg(self, speed):
    # PT-bus BUS1_SPEED in m/s; DBC factor converts to/from raw bits per platform.
    values = {"BUS1_SPEED": speed}
    return self.pt_packer.make_can_msg_safety("BUS1_SPEED", VOLVO_PT_BUS, values)

  def _speed_msg_2(self, speed):
    # Volvo's safety mode only consumes a single vehicle-speed source, so the
    # generic rx_hook_speed_mismatch test doesn't apply. Returning None asks
    # the harness to skip it (see common.CarSafetyTest.test_rx_hook_speed_mismatch).
    return None

  # ---- Volvo-specific consistency tests ----

  def test_gas_threshold_self_consistent(self):
    """
    Sanity: the panda's gas_pressed edge is exactly where GAS_PRESSED_THRESHOLD
    is defined. Regression guard for the CMA-vs-SPA scale mismatch bug where
    carstate.py applied CMA thresholds (raw 0-255) to SPA's DBC-scaled percent.
    If you change the threshold in volvo.h, update carstate.py AND this test.
    """
    # Just below threshold → not pressed. Use a value strictly less than threshold.
    just_below = max(self.GAS_PRESSED_THRESHOLD - 1, 0)
    self._rx(self._user_gas_msg(just_below))
    self.assertFalse(self.safety.get_gas_pressed_prev(),
                     f"gas flagged at {just_below} (threshold={self.GAS_PRESSED_THRESHOLD})")

    # Just above threshold → pressed.
    self._rx(self._user_gas_msg(self.GAS_PRESSED_THRESHOLD + 1))
    self.assertTrue(self.safety.get_gas_pressed_prev(),
                    f"gas not flagged above {self.GAS_PRESSED_THRESHOLD}")


class TestVolvoCMA(TestVolvoSafetyBase):
  """
  Volvo CMA: XC40 Recharge.

  ECM_1.ACCELERATOR_PEDAL_POS is raw 0-255 (DBC factor 1). Idle is ~20; panda
  fires gas_pressed above 21. carstate.py uses the same raw threshold.
  """
  SAFETY_PARAM = 0
  PT_DBC = "volvo_front_1_cma"
  GAS_PRESSED_THRESHOLD = 21  # raw counts

  def _user_gas_msg(self, gas):
    values = {"ACCELERATOR_PEDAL_POS": gas}
    return self.pt_packer.make_can_msg_safety("ECM_1", VOLVO_PT_BUS, values)

  def _pcm_status_msg(self, enable):
    # CMA cruise is a pair of bools; either one being high enables cruise.
    values = {"CRUISE_CONTROL_ENABLED": 1 if enable else 0,
              "CRUISE_CONTROL_ENABLED_IDLE_TRAFFIC": 0}
    return self.pt_packer.make_can_msg_safety(
      "BUS1_CRUISE_CONTROL", VOLVO_PT_BUS, values)


class TestVolvoSPA(TestVolvoSafetyBase):
  """
  Volvo SPA: S60 Recharge, Polestar 2.

  ECM_1.ACCELERATOR_PEDAL_POS is DBC-scaled to percent (factor 0.00390625,
  range 0-100%). Idle is ~0; panda fires gas_pressed above 1.0%.
  carstate.py MUST use the same percent-based threshold — see
  opendbc/car/volvo/carstate.py. A mismatch here manifests in test_models.py
  as persistent ``panda safety doesn't agree with openpilot: {'gasPressed': N}``.
  """
  SAFETY_PARAM = VOLVO_FLAG_SPA
  PT_DBC = "volvo_front_1_spa"
  GAS_PRESSED_THRESHOLD = 1.0  # percent

  def _user_gas_msg(self, gas):
    values = {"ACCELERATOR_PEDAL_POS": gas}
    return self.pt_packer.make_can_msg_safety("ECM_1", VOLVO_PT_BUS, values)

  def _pcm_status_msg(self, enable):
    # SPA cruise signal has DBC factor -1 offset 1, so the "enabled" DBC value
    # is 1 (which encodes as raw bit 0 — panda inverts it).
    values = {"CRUISE_CONTROL_SPA_ENABLED": 1 if enable else 0}
    return self.pt_packer.make_can_msg_safety(
      "BUS1_CRUISE_CONTROL", VOLVO_PT_BUS, values)


if __name__ == "__main__":
  unittest.main()
