"""
The MoreTorque Torque Interceptor (TI) on a gen1 Mazda: the declared flag and what it changes in
CarParams, carstate through the real CarInterface and its bus 1 parser, the controller's second
command, and the controller run closed loop through the compiled safety model.
"""
from collections import deque
from itertools import groupby

import pytest

from opendbc.can import CANPacker
from opendbc.car import Bus, DT_CTRL
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.mazda.carcontroller import DASH_STEER_WARNING_QUIET_FRAMES, HUD_ALERT_FRAMES, CarController
from opendbc.car.mazda.carstate import TI_BOOT_FRAMES, TI_BYPASS_FRAMES, TI_DRIVER_OVER_RELEASE_FRAMES, TI_DROPOUT_ALERT_FRAMES, \
  TI_DROPOUT_FRAMES, TI_FAULT_FRAMES, TI_FEEDBACK_FRESH_FRAMES, TI_NOT_READY_ALERT_FRAMES, TI_REFIT_FRAMES, TI_RESTART_FRAMES
from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.mazdacan import TI_BUS, TI_KEY, create_ti_steering_control
from opendbc.car.mazda.tests.conftest import CAM_LANEINFO, CAM_LKAS, DBC_NAME, VisualAlert, car_params, car_params_sp, eps_fw, \
  frame, frames, hands_code, mazda_car_state, packer, step
from opendbc.car.mazda.values import CAR, LKAS_LIMITS, CarControllerParams, MazdaFlags, MazdaSafetyFlags, \
  TorqueInterceptorCause, TorqueInterceptorParams as TI, TorqueInterceptorState
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.sunnypilot.car.interfaces import setup_interfaces
from opendbc.sunnypilot.car.lateral_tune import get_speed_dep_config, get_speed_dep_config_for_car, get_steer_rail_schedule, \
  get_steer_slew_schedule, get_tune_scale

TI_LKAS = 0x249
TI_FEEDBACK = 0x24A
TI_REJECTED_SRC = 193  # bus 1 + 0xC0

STEER_TO_ZERO_FW = eps_fw(b'KSD5-3210X-C-00\x00\x00\x00\x00\x00\x00\x00\x00\x00')

LAT = dict(long_active=False, enabled=True, lat_active=True)


def ti_car_params(candidate=CAR.MAZDA_CX5, enabled="1", car_fw=None):
  """CarParams as card builds them: get_params, then the driver's toggles."""
  CP = car_params(candidate, alpha_long=False, car_fw=car_fw)
  CP_SP = car_params_sp(CP, candidate, alpha_long=False, car_fw=car_fw)
  setup_interfaces(CarInterface, CP, CP_SP, [{"TorqueInterceptorEnabled": enabled}])
  return CP, CP_SP


def ti_interface(candidate=CAR.MAZDA_CX5, car_fw=None) -> CarInterface:
  return CarInterface(*ti_car_params(candidate, car_fw=car_fw))


def ti_controller(candidate=CAR.MAZDA_CX5, car_fw=None):
  CP, CP_SP = ti_car_params(candidate, car_fw=car_fw)
  cc = CarController({Bus.pt: DBC_NAME}, CP, CP_SP)
  cs = mazda_car_state(CP, CP_SP)
  cs.ti_ready = True
  return cc, cs


def feedback(torque=0, version=0x10, state=TorqueInterceptorState.RUN, viol=0, error=0, ramp=0, copy=None, bus=TI_BUS):
  """TI_FEEDBACK as the interceptor sends it; the healthy idle frame is 7F 7F 10 03 00 00 00 30."""
  byte0 = (int(torque) + 127) & 0xFF
  return TI_FEEDBACK, bytes([byte0, byte0 if copy is None else copy, version, int(state), viol, error, ramp, 0x30]), bus


def ti_torque(sends) -> int | None:
  dat = frame(sends, TI_LKAS, TI_BUS)
  return None if dat is None else (((dat[0] & 0x0F) << 8) | dat[1]) - 2048


def at_rest_on(value: int, target: int) -> bool:
  """A steady request sits on its target or, for the odd frame, one count nearer zero
  (TorqueInterceptorParams.STEER_HELD_FRAMES)."""
  return value in (target, target - (1 if target > 0 else -1))


class Car:
  """A real CarInterface fed frame by frame: the interceptor's feedback at its 50 Hz plus
  whatever vehicle frames a test names."""

  def __init__(self, candidate=CAR.MAZDA_CX5, car_fw=None):
    self.CI = ti_interface(candidate, car_fw)
    self.packer = packer()
    self.i = 0
    self.speed_kph = 0.

  def vehicle(self):
    wheel = dict.fromkeys(("FL", "FR", "RL", "RR"), self.speed_kph)
    return [self.packer.make_can_msg("WHEEL_SPEEDS", 0, wheel),
            self.packer.make_can_msg("ENGINE_DATA", 0, {"SPEED": self.speed_kph})]

  def run(self, n, fb=None, extra=(), late=False):
    """n control frames. fb is the feedback frame (sent every other frame, as on the bus; on
    the frames between with late), a callable of the frame index returning one, or None for a
    silent interceptor."""
    ret = None
    for _ in range(n):
      msgs = [*self.vehicle(), *extra]
      this = fb(self.i) if callable(fb) else fb
      if this is not None and self.i % 2 == (1 if late else 0):
        msgs.append(this)
      ret, _ = self.CI.update([(int(self.i * DT_CTRL * 1e9), msgs)])
      self.i += 1
    return ret

  @property
  def CS(self):
    return self.CI.CS


class TestInterface:
  def test_the_toggle_sets_the_flag_the_safety_bit_and_lifts_the_floor(self):
    CP, _ = ti_car_params(CAR.MAZDA_CX5)
    assert CP.flags & MazdaFlags.TORQUE_INTERCEPTOR
    assert CP.flags & MazdaFlags.LEGACY_FW_EPS
    assert CP.safetyConfigs[0].safetyParam == MazdaSafetyFlags.EPS_HW | MazdaSafetyFlags.TORQUE_INTERCEPTOR
    # the 45 kph floor is gone; lateral stops where the interceptor is sent nothing
    assert CP.minSteerSpeed == TI.MIN_SPEED == 1.0
    assert not CP.steerAtStandstill

  def test_steer_to_zero_firmware_keeps_its_zero_floor(self):
    CP, _ = ti_car_params(CAR.MAZDA_CX5, car_fw=STEER_TO_ZERO_FW)
    assert CP.flags & MazdaFlags.STEER_TO_ZERO_EPS and CP.flags & MazdaFlags.TORQUE_INTERCEPTOR
    assert CP.minSteerSpeed == 0

  @pytest.mark.parametrize("enabled", ["0", 0, False])
  def test_off_changes_nothing(self, enabled):
    CP, CP_SP = ti_car_params(CAR.MAZDA_CX5, enabled=enabled)
    stock = car_params(CAR.MAZDA_CX5)
    assert CP.flags == stock.flags
    assert CP.safetyConfigs[0].safetyParam == stock.safetyConfigs[0].safetyParam == MazdaSafetyFlags.EPS_HW
    assert CP.minSteerSpeed == pytest.approx(LKAS_LIMITS.DISABLE_SPEED * CV.KPH_TO_MS)
    assert Bus.body not in CarInterface(CP, CP_SP).can_parsers

  def test_no_toggle_at_all_changes_nothing(self):
    CP = car_params(CAR.MAZDA_CX5)
    CP_SP = car_params_sp(CP, CAR.MAZDA_CX5)
    setup_interfaces(CarInterface, CP, CP_SP, None)
    assert not CP.flags & MazdaFlags.TORQUE_INTERCEPTOR
    assert CP.minSteerSpeed > 0

  @pytest.mark.parametrize("platform", list(CAR))
  def test_every_platform_takes_it_and_keeps_plain_ints(self, platform):
    CP, _ = ti_car_params(platform)
    assert CP.flags & MazdaFlags.TORQUE_INTERCEPTOR
    assert type(CP.flags) is int
    assert type(CP.safetyConfigs[0].safetyParam) is int
    assert CP.minSteerSpeed <= TI.MIN_SPEED

  def test_alpha_long_stays_with_the_steer_to_zero_firmware(self):
    # the interceptor lifts the lateral floor; it does not make the radar takeover available
    assert not ti_car_params(CAR.MAZDA_CX5)[0].alphaLongitudinalAvailable

  def test_the_flag_bits_are_free(self):
    assert MazdaFlags.TORQUE_INTERCEPTOR == 64
    assert not MazdaFlags.TORQUE_INTERCEPTOR & (MazdaFlags.GEN1 | MazdaFlags.EPS_HW | MazdaFlags.G46L_RADAR | MazdaFlags.HAS_BSM | 16)
    assert MazdaSafetyFlags.TORQUE_INTERCEPTOR == 8
    assert not MazdaSafetyFlags.TORQUE_INTERCEPTOR & (MazdaSafetyFlags.LONG | MazdaSafetyFlags.EPS_HW | 4)


class TestTune:
  """The interceptor is the reported actuator: the lateral stack reads its rail and slew."""

  def test_scale_is_three_quarters_of_the_eps_commands(self):
    # pdscomp's pairing on legacy firmware, an 800 count EPS command beside u x 600; the EPS
    # command is on 1200 here, so the same share is 900. (The interceptor author's own fork
    # sends it the EPS command's count, clipped to 600.)
    assert TI.STEER_SCALE == 900
    assert TI.STEER_SCALE / CarControllerParams.EPS_STEER_MAX == TI.STEER_MAX / CarControllerParams.TUNE_STEER_MAX

  def test_the_rail_is_the_interceptors_clip_exactly(self):
    CP, _ = ti_car_params()
    bp, rail = get_steer_rail_schedule(CP)
    assert bp == [0.] and rail == [TI.STEER_MAX / TI.STEER_SCALE]
    # at the rail the wire carries 600: the one nonzero value the interceptor lets sit still
    assert round(rail[0] * TI.STEER_SCALE) == TI.STEER_MAX

  def test_the_slew_is_the_interceptors(self):
    CP, _ = ti_car_params()
    assert get_steer_slew_schedule(CP) == ([0.], [TI.STEER_DELTA_UP / TI.STEER_SCALE], [TI.STEER_DELTA_DOWN / TI.STEER_SCALE])

  def test_the_tune_scale_is_unchanged(self):
    CP, _ = ti_car_params()
    stock = car_params(CAR.MAZDA_CX5)
    assert get_tune_scale(CP) == get_tune_scale(stock) == CarControllerParams.TUNE_SCALE
    assert CP.lateralTuning.torque.latAccelFactor == stock.lateralTuning.torque.latAccelFactor
    assert CP.lateralTuning.torque.friction == stock.lateralTuning.torque.friction

  @pytest.mark.parametrize("platform", list(CAR))
  def test_no_speed_binned_seeds_and_a_cache_of_its_own(self, platform):
    # the tables were learned on the EPS command alone, most behind steer-to-zero firmware;
    # lifting the floor must not hand a legacy EPS those. The default bins it learns on carry
    # a seed version no table uses, so neither plant restores what the other learned
    cfg = get_speed_dep_config_for_car(ti_car_params(platform)[0])
    assert cfg == {'seed_version': TI.SPEED_DEP_SEED_VERSION}
    assert all(entry.get('seed_version', 0) != TI.SPEED_DEP_SEED_VERSION for entry in get_speed_dep_config().values())
    assert get_speed_dep_config_for_car(car_params(platform)).get('seed_version', 0) != TI.SPEED_DEP_SEED_VERSION

  @pytest.mark.parametrize("platform", list(CAR))
  def test_without_it_the_helpers_are_untouched(self, platform):
    stock = car_params(platform)
    ccp = CarControllerParams(stock)
    assert not hasattr(ccp, "STEER_RAIL_SCHEDULE") and not hasattr(ccp, "STEER_SLEW_SCHEDULE") and not hasattr(ccp, "SPEED_DEP_CONFIG")
    assert ccp.STEER_OVERRIDE_BP == CarControllerParams.STEER_OVERRIDE_BP
    bp, rail = get_steer_rail_schedule(stock)
    assert bp == [float(x) for x in ccp.EPS_CEILING_LOOKUP[0]]
    assert rail == [x / ccp.STEER_MAX for x in ccp.EPS_CEILING_LOOKUP[1]]
    assert get_steer_slew_schedule(stock) == ([0.], [ccp.STEER_DELTA_UP / ccp.STEER_MAX], [ccp.STEER_DELTA_DOWN / ccp.STEER_MAX])

  def test_the_timing_constants_keep_their_reasons(self):
    # the interceptor's own documentation and the drives on record, not each other
    assert 0.06 < TI.FEEDBACK_TIMEOUT <= 0.07, "two lost 50 Hz frames and a late one, and 30 ms inside the panda's 100 ms"
    assert TI.FAULT_T - TI.FEEDBACK_TIMEOUT > 0.242 - 0.02, "the longest feedback pause on record is no fault"
    assert 1.09 < TI.BYPASS_T <= 1.5, "its slowest watchdog has it in bypass, and past that the EPS's reading is the only one"
    assert TI.REFIT_T >= TI.STEER_MAX / TI.STEER_DELTA_UP * DT_CTRL, "no count forgotten before the command is back"
    assert TI.REFIT_T < TI.DROPOUT_T < TI.DROPOUT_ALERT_T
    assert TI.NOT_READY_ALERT_T > TI.DRIVER_OVER_RELEASE_T + TI.FAULT_T, "a let-go wheel is not named"

  def test_the_eps_command_yields_over_the_interceptors_span(self):
    ccp = CarControllerParams(ti_car_params()[0])
    assert ccp.STEER_OVERRIDE_BP == [15, 30]


class TestCarState:
  def test_runs_with_real_parsers(self):
    car = Car()
    assert car.CI.can_parsers[Bus.body].bus == TI_BUS
    car.run(10)
    car.run(10, feedback())

  def test_a_healthy_interceptor_is_ready(self):
    car = Car()
    car.run(4, feedback())
    assert car.CS.ti_ready
    assert (car.CS.ti_version, car.CS.ti_state, car.CS.ti_violation, car.CS.ti_error) == (0x10, TorqueInterceptorState.RUN, 0, 0)

  @pytest.mark.parametrize("version", [1, 2, 0x10, 0x11, 0xFF])
  def test_any_firmware_that_reports_a_version_is_accepted(self, version):
    car = Car()
    car.run(4, feedback(version=version))
    assert car.CS.ti_ready

  @pytest.mark.parametrize("unfit", [
    dict(state=TorqueInterceptorState.DISCOVER),
    dict(state=TorqueInterceptorState.OFF),
    dict(state=TorqueInterceptorState.OFF, viol=0x12),  # the boot frame: CAN timeout, clearing
    dict(state=TorqueInterceptorState.DRIVER_OVER),
    dict(viol=0x11),
    dict(error=1),
    dict(ramp=1),
    dict(version=0),
    dict(torque=5, copy=0x7F),   # the copy disagrees
  ])
  def test_anything_short_of_run_and_clean_is_not_ready(self, unfit):
    car = Car()
    car.run(4, feedback())
    assert car.CS.ti_ready
    car.run(2, feedback(**unfit))
    assert not car.CS.ti_ready

  def test_firmware_version_1_has_no_ramp_byte(self):
    car = Car()
    car.run(4, feedback(version=1, ramp=1))
    assert car.CS.ti_ready

  def test_silence_ends_readiness_inside_the_pandas_timeout(self):
    car = Car()
    car.run(4, feedback())  # last heard on the third of these
    # two missed feedback frames are tolerated, the third is not
    car.run(TI_FEEDBACK_FRESH_FRAMES - 2)
    assert car.CS.ti_ready
    car.run(1)
    assert not car.CS.ti_ready
    assert TI.FEEDBACK_TIMEOUT < 0.1, "the controller must stop before the panda's 100 ms refuses it"

  def test_two_lost_frames_and_a_late_one_keep_it_ready(self):
    # 50 Hz frames on a 100 Hz loop: after two lost ones the next may land a cycle late
    car = Car()
    car.run(4, feedback())  # last heard on the third of these
    car.run(5)              # two frames lost, the third due on the last of these and not there yet
    assert car.CS.ti_ready
    car.run(1, feedback(), late=True)
    assert car.CS.ti_ready and car.CS.ti_feedback_silent_frames == 0
    # a third lost frame is the interceptor gone
    car.run(TI_FEEDBACK_FRESH_FRAMES)
    assert not car.CS.ti_ready

  def test_feedback_on_the_wrong_bus_is_not_the_interceptor(self):
    car = Car()
    car.run(10, feedback(bus=0))
    assert not car.CS.ti_ready

  def test_driver_torque_is_the_interceptors_reading(self):
    car = Car()
    eps = car.packer.make_can_msg("STEER_TORQUE", 0, {"STEER_TORQUE_SENSOR": 40})
    ret = car.run(4, feedback(torque=-9), extra=[eps])
    assert ret.steeringTorque == -9

  def test_a_silent_interceptor_is_in_bypass_and_the_eps_reads_the_driver(self):
    car = Car()
    eps = car.packer.make_can_msg("STEER_TORQUE", 0, {"STEER_TORQUE_SENSOR": 40})
    ret = car.run(20, None, extra=[eps])
    assert ret.steeringTorque == 40
    assert ret.steeringPressed

  def test_gone_silent_its_last_reading_stands_until_it_is_in_bypass(self):
    # hands off, the interceptor steering, and its link drops: what the EPS reports next is the
    # interceptor's bias, not the driver, until the interceptor's watchdogs have it in bypass.
    # Read as a press it would turn the take-control alarm for this very fault into a banner
    car = Car()
    biased = car.packer.make_can_msg("STEER_TORQUE", 0, {"STEER_TORQUE_SENSOR": 40})
    car.run(20, feedback(torque=0), extra=[biased])
    for _ in range(TI_BYPASS_FRAMES - 2):
      ret = car.run(1, None, extra=[biased])
      assert ret.steeringTorque == 0 and not ret.steeringPressed
    assert not car.CS.ti_fresh
    # past that the EPS's reading is the driver's again
    ret = car.run(20, None, extra=[biased])
    assert ret.steeringTorque == 40 and ret.steeringPressed
    # and its own the moment it is back
    ret = car.run(4, feedback(torque=-3), extra=[biased])
    assert ret.steeringTorque == -3

  def test_a_press_it_last_reported_is_held_through_the_silence_too(self):
    car = Car()
    car.run(20, feedback(torque=12))
    assert car.run(TI_BYPASS_FRAMES // 2).steeringPressed

  @pytest.mark.parametrize("peak", [7, 12, 16, -12, TI.STEER_OVERRIDE_NOW])
  def test_one_noise_peak_before_a_silence_is_not_a_press(self, peak):
    # what stands through the silence is whether the driver was steering, not the last sample:
    # counted on every frame of the hold, one peak of the reading's noise (10 to 16 on record)
    # would be a press five frames in, and would soften the alarm for the fault that follows
    car = Car()
    car.run(20, feedback())
    car.run(2, feedback(torque=peak))
    for _ in range(TI_BYPASS_FRAMES - 2):
      assert not car.run(1).steeringPressed
    assert car.CS.steering_pressed_cnt == 0

  def test_a_feedback_pause_does_not_make_a_press_of_what_was_not_one(self):
    car = Car()
    car.run(20, feedback())
    car.run(4, feedback(torque=12))       # two samples above the threshold: not yet a press
    assert not car.run(1).steeringPressed
    for _ in range(24):                   # a pause as long as the longest on record
      assert not car.run(1).steeringPressed
    # and the count that was part way there has run down: a third sample starts over
    assert not car.run(2, feedback(torque=12)).steeringPressed

  def test_a_hard_push_it_reported_last_is_held(self):
    car = Car()
    car.run(20, feedback())
    assert car.run(1, feedback(torque=TI.STEER_OVERRIDE_NOW + 1)).steeringPressed
    assert car.run(TI_BYPASS_FRAMES // 2).steeringPressed

  def test_steering_pressed_threshold_and_filter(self):
    car = Car()
    assert not car.run(20, feedback(torque=TI.STEER_THRESHOLD)).steeringPressed
    assert not car.run(TI.STEER_PRESSED_FRAMES, feedback(torque=TI.STEER_THRESHOLD + 1)).steeringPressed
    assert car.run(2, feedback(torque=TI.STEER_THRESHOLD + 1)).steeringPressed

  def test_a_hard_push_is_pressed_at_once(self):
    car = Car()
    car.run(20, feedback())
    assert car.run(1, feedback(torque=-(TI.STEER_OVERRIDE_NOW + 1))).steeringPressed

  def test_no_fault_while_fit_through_the_eps_floor_and_lockout(self):
    # 20 kph on legacy firmware: the EPS blocks its own LKAS, the interceptor steers
    car = Car()
    car.speed_kph = 20.
    blocked = car.packer.make_can_msg("STEER_RATE", 0, {"LKAS_BLOCK": 1})
    cruising = car.packer.make_can_msg("CRZ_CTRL", 0, {"CRZ_ACTIVE": 1, "CRZ_AVAILABLE": 1})
    ret = car.run(50, feedback(), extra=[blocked, cruising])
    assert ret.cruiseState.enabled
    assert not ret.steerFaultTemporary
    assert not ret.lowSpeedAlert
    # the EPS's own floor is still tracked: the controller sends that EPS nothing below it
    assert not car.CS.lkas_allowed_speed and car.CS.lkas_blocked
    # above it, once the EPS lifts its block
    car.speed_kph = 60.
    car.run(10, feedback(), extra=[blocked, cruising])
    assert not car.CS.lkas_allowed_speed
    car.run(10, feedback(), extra=[car.packer.make_can_msg("STEER_RATE", 0, {"LKAS_BLOCK": 0}), cruising])
    assert car.CS.lkas_allowed_speed
    # its hands-off lockout at speed is no fault either
    assert not car.run(50, feedback(), extra=[blocked, cruising]).steerFaultTemporary

  @pytest.mark.parametrize("unfit", [dict(state=TorqueInterceptorState.OFF, viol=0x11), dict(ramp=1), dict(error=2), None])
  def test_unfit_while_moving_becomes_a_temporary_steer_fault(self, unfit):
    car = Car()
    car.speed_kph = 60.
    assert not car.run(20, feedback()).steerFaultTemporary
    fb = None if unfit is None else feedback(**unfit)
    # a dropout shorter than FAULT_T costs the interceptor's torque, not the driver's lateral
    assert not car.run(TI_FAULT_FRAMES, fb).steerFaultTemporary
    assert not car.CS.ti_ready
    assert car.run(TI_FEEDBACK_FRESH_FRAMES + 2, fb).steerFaultTemporary
    # back in RUN, it has to stay there for REFIT_T before the fault is dropped
    assert car.run(TI_REFIT_FRAMES - 2, feedback()).steerFaultTemporary
    assert car.CS.ti_ready
    assert not car.run(4, feedback()).steerFaultTemporary

  @pytest.mark.parametrize("pause", [24, TI_FAULT_FRAMES - TI_FEEDBACK_FRESH_FRAMES])
  def test_the_longest_feedback_pause_on_record_is_no_fault(self, pause):
    # 242 ms (pdscomp's logs): it costs the interceptor's torque for the pause, nothing else
    car = Car()
    car.CS.ti_stream_frames = TI_BOOT_FRAMES
    car.speed_kph = 60.
    car.run(20, feedback())
    for _ in range(pause):
      assert not car.run(1).steerFaultTemporary
    assert not car.CS.ti_ready
    assert not car.run(300, feedback()).steerFaultTemporary
    assert not car.CS.ti_not_ready

  def test_one_short_dropout_is_no_fault(self):
    car = Car()
    car.speed_kph = 60.
    car.run(20, feedback())
    assert not car.run(TI_FAULT_FRAMES - 2, feedback(ramp=1)).steerFaultTemporary
    assert not car.run(200, feedback()).steerFaultTemporary
    assert car.CS.ti_unfit_frames == 0

  @pytest.mark.parametrize("unfit, fit, fb", [
    (18, 4, feedback(ramp=1)), (24, 6, feedback(state=TorqueInterceptorState.OFF, viol=0x12)), (10, 40, feedback(ramp=1)),
    (18, 4, None), (24, 6, None), (12, 38, None),   # 4 or more feedback frames lost at a time
  ])
  def test_dropouts_that_keep_coming_are_a_fault(self, unfit, fit, fb):
    # each unfit frame restarts the interceptor's command from zero, so unfitness in bursts
    # steers no better than unfitness that stays: the bursts add up until it has been fit for
    # REFIT_T, and the fault holds through the fit frames between them
    assert fit < TI_REFIT_FRAMES
    car = Car()
    car.CS.ti_stream_frames = TI_BOOT_FRAMES
    car.speed_kph = 60.
    car.run(20, feedback())
    faulted = []
    for _ in range(80):
      faulted.append(car.run(unfit, fb).steerFaultTemporary)
      faulted.append(car.run(fit, feedback()).steerFaultTemporary)
    assert True in faulted[:40]
    assert all(faulted[faulted.index(True):])
    # and long enough, it is named
    assert car.CS.ti_not_ready
    # fit for REFIT_T, the fault is dropped
    assert not car.run(TI_REFIT_FRAMES + 2, feedback()).steerFaultTemporary
    # the name is not: it kept dropping out, and stays named until every dropout is paid off
    assert car.CS.ti_not_ready and car.CS.ti_dropping_out
    car.run(TI_DROPOUT_ALERT_FRAMES + TI_DROPOUT_FRAMES, feedback())
    assert not car.CS.ti_not_ready and car.CS.ti_unfit_cause == TorqueInterceptorCause.NONE

  @pytest.mark.parametrize("unfit, fit, fb, fault_on", [
    (2, 50, feedback(ramp=1), 4), (2, TI_REFIT_FRAMES - 3, feedback(ramp=1), 4),   # 20 ms glitches: 2, 14, 26, 38 frames counted
    (8, 48, None, 4),                                                              # four feedback frames lost at a time
    (30, 50, feedback(ramp=1), 2),                                                 # 0.3 s twice
  ])
  def test_short_dropouts_inside_the_refit_window_are_a_fault(self, unfit, fit, fb, fault_on):
    # a dropout that comes before the last one's count was cleared restarts a command that had
    # not climbed back: it is charged RESTART_T (0.1 s) on top of its own frames
    assert TI_RESTART_FRAMES == 10 and TI_FAULT_FRAMES == 30
    car = Car()
    car.CS.ti_stream_frames = TI_BOOT_FRAMES
    car.speed_kph = 60.
    car.run(20, feedback())
    faulted = []
    for _ in range(20):
      faulted.append(car.run(unfit, fb).steerFaultTemporary)
      faulted.append(car.run(fit, feedback()).steerFaultTemporary)
    # never the first: one dropout up to FAULT_T is no fault
    assert faulted.index(True) == 2 * (fault_on - 1)
    assert all(faulted[faulted.index(True):])
    assert car.CS.ti_not_ready

  def test_dropouts_further_apart_than_the_refit_window_are_no_fault(self):
    car = Car()
    car.CS.ti_stream_frames = TI_BOOT_FRAMES
    car.speed_kph = 60.
    car.run(20, feedback())
    for _ in range(20):
      assert not car.run(TI_FAULT_FRAMES - 2, feedback(ramp=1)).steerFaultTemporary
      assert not car.run(TI_REFIT_FRAMES + 2, feedback()).steerFaultTemporary
      assert car.CS.ti_unfit_frames == 0
    # each still cost a ramp from zero: kept up like this, it is named (TestNotReadyAlert)
    assert car.CS.ti_not_ready

  @pytest.mark.parametrize("malformed", [dict(torque=5, copy=0x7F), dict(version=0), dict(state=4)])
  def test_one_malformed_frame_is_a_fault_at_once(self, malformed):
    # not a two-cycle dropout: on that frame the panda drops cruise and MADS lateral alike
    # (mazda.h's quality flag) and refuses both commands until the driver engages again
    car = Car()
    car.speed_kph = 60.
    car.run(20, feedback())
    assert car.run(1, feedback(**malformed), late=car.i % 2 == 1).steerFaultTemporary
    assert car.CS.ti_malformed
    assert car.run(1).steerFaultTemporary
    # and for REFIT_T after, however healthy the frames that follow
    assert car.run(TI_REFIT_FRAMES - 4, feedback()).steerFaultTemporary
    assert not car.CS.ti_malformed and car.CS.ti_ready
    assert not car.run(8, feedback()).steerFaultTemporary

  @pytest.mark.parametrize("malformed", [dict(torque=5, copy=0x7F), dict(version=0), dict(state=4)])
  @pytest.mark.parametrize("healthy_last", [True, False])
  def test_a_malformed_frame_counts_with_a_healthy_one_in_the_same_cycle(self, malformed, healthy_last):
    # card hands carstate every frame that arrived since its last cycle, and after a late one
    # that is two: the panda has judged both, so the last one being healthy changes nothing
    car = Car()
    car.speed_kph = 60.
    car.run(TI_REFIT_FRAMES + 20, feedback())
    first, last = (feedback(**malformed), feedback()) if healthy_last else (feedback(), feedback(**malformed))
    ret = car.run(1, last, extra=[first], late=car.i % 2 == 1)
    assert car.CS.ti_malformed and ret.steerFaultTemporary
    assert car.CS.ti_ready == healthy_last
    assert car.CS.ti_unfit_cause == TorqueInterceptorCause.MALFORMED
    assert car.run(TI_REFIT_FRAMES - 4, feedback()).steerFaultTemporary
    assert car.CS.ti_unfit_cause == TorqueInterceptorCause.MALFORMED
    assert not car.run(8, feedback()).steerFaultTemporary
    assert car.CS.ti_unfit_cause == TorqueInterceptorCause.NONE

  def test_a_malformed_frame_at_a_stop_is_a_fault_when_the_car_moves(self):
    car = Car()
    car.run(20, feedback())
    assert not car.run(2, feedback(version=0)).steerFaultTemporary
    car.speed_kph = 30.
    assert car.run(1, feedback(version=0)).steerFaultTemporary

  @pytest.mark.parametrize("fb, cause, code", [
    (None, TorqueInterceptorCause.SILENT, 0),
    (feedback(torque=5, copy=0x7F), TorqueInterceptorCause.MALFORMED, 0),
    (feedback(state=TorqueInterceptorState.OFF, viol=0x11, error=3), TorqueInterceptorCause.VIOLATION, 0x11),
    (feedback(error=0x2A), TorqueInterceptorCause.ERROR, 0x2A),
    (feedback(state=TorqueInterceptorState.OFF), TorqueInterceptorCause.NOT_RUNNING, TorqueInterceptorState.OFF),
    (feedback(state=TorqueInterceptorState.DRIVER_OVER), TorqueInterceptorCause.NOT_RUNNING, TorqueInterceptorState.DRIVER_OVER),
    (feedback(ramp=1), TorqueInterceptorCause.RAMP_DOWN, 0),
  ])
  def test_why_it_was_not_fit_is_kept_until_it_has_been_fit_for_good(self, fb, cause, code):
    # what the alert says: the reason, not whatever the frame of the moment reads. Under
    # dropouts that keep coming, and for REFIT_T after the last, that frame is a healthy one
    car = Car()
    car.speed_kph = 60.
    car.run(20, feedback())
    assert car.CS.ti_unfit_cause == TorqueInterceptorCause.NONE
    car.run(20, fb)
    assert (car.CS.ti_unfit_cause, car.CS.ti_unfit_code) == (cause, code)
    car.run(TI_REFIT_FRAMES - 4, feedback())
    assert car.CS.ti_ready and (car.CS.ti_unfit_cause, car.CS.ti_unfit_code) == (cause, code)
    car.run(8, feedback())
    assert (car.CS.ti_unfit_cause, car.CS.ti_unfit_code) == (TorqueInterceptorCause.NONE, 0)

  def test_a_latch_is_not_forgotten_at_a_stop(self):
    # latched off while moving: the fault waits for the car to move again, what was counted
    # against the interceptor is kept, and it is faulted again on the first frame that does
    car = Car()
    car.speed_kph = 30.
    car.run(20, feedback())
    latched = feedback(state=TorqueInterceptorState.OFF, viol=0x11)
    assert car.run(TI_FAULT_FRAMES + 10, latched).steerFaultTemporary
    car.speed_kph = 0.
    assert not car.run(300, latched).steerFaultTemporary
    assert car.CS.ti_unfit_frames > TI_FAULT_FRAMES
    car.speed_kph = 30.
    assert car.run(1, latched).steerFaultTemporary

  def test_a_crawl_around_min_speed_still_reaches_the_fault(self):
    car = Car()
    car.run(20, feedback())
    latched = feedback(state=TorqueInterceptorState.OFF, viol=0x11)
    just_over, just_under = TI.MIN_SPEED * CV.MS_TO_KPH + 0.3, TI.MIN_SPEED * CV.MS_TO_KPH - 0.3
    faulted = False
    for i in range(400):
      car.speed_kph = just_over if (i // 5) % 2 == 0 else just_under
      faulted |= car.run(1, latched).steerFaultTemporary
    assert faulted

  def test_the_driver_steering_through_it_is_no_fault(self):
    # DRIVER_OVER: the interceptor steps aside for a hard push. Not ready, and not a fault
    # while the driver holds the wheel or has just let go
    car = Car()
    car.speed_kph = 60.
    car.run(20, feedback())
    over = TorqueInterceptorState.DRIVER_OVER
    ret = car.run(300, feedback(torque=60, state=over))
    assert ret.steeringPressed and not car.CS.ti_ready and not ret.steerFaultTemporary
    # let go: it has DRIVER_OVER_RELEASE_T to come back
    ret = car.run(TI_DRIVER_OVER_RELEASE_FRAMES + TI_FAULT_FRAMES - 10, feedback(state=over))
    assert not ret.steeringPressed and not ret.steerFaultTemporary
    assert not car.run(20, feedback()).steerFaultTemporary
    # stuck there with nobody on the wheel, it is a fault like any other
    assert car.run(TI_DRIVER_OVER_RELEASE_FRAMES + TI_FAULT_FRAMES + 20, feedback(state=over)).steerFaultTemporary

  def test_stepping_aside_for_the_driver_counts_as_fit_toward_a_fault_clearing(self):
    # a fault counted against it, then the driver steers through it: those frames are the
    # interceptor working, and REFIT_T of them clears the count like RUN does
    car = Car()
    car.speed_kph = 60.
    car.run(20, feedback())
    assert car.run(TI_FAULT_FRAMES + 10, feedback(ramp=1)).steerFaultTemporary
    over = feedback(torque=60, state=TorqueInterceptorState.DRIVER_OVER)
    assert car.run(TI_REFIT_FRAMES - 10, over).steerFaultTemporary
    assert not car.run(20, over).steerFaultTemporary

  def test_the_excuse_needs_the_interceptor_to_be_saying_so(self):
    # DRIVER_OVER was the last thing it said, the hand is still on the wheel (the held
    # reading), and it has gone silent: that is silence, not stepping aside
    car = Car()
    car.CS.ti_stream_frames = TI_BOOT_FRAMES
    car.speed_kph = 60.
    car.run(40, feedback(torque=60, state=TorqueInterceptorState.DRIVER_OVER))
    ret = car.run(TI_FEEDBACK_FRESH_FRAMES + TI_FAULT_FRAMES + 2)
    assert ret.steeringPressed and ret.steerFaultTemporary

  def test_unfit_at_a_stop_is_no_fault(self):
    # it is sent nothing below MIN_SPEED and may sit out a stop in its own time
    car = Car()
    assert not car.run(20, feedback(state=TorqueInterceptorState.OFF, viol=0x12)).steerFaultTemporary
    car.speed_kph = TI.MIN_SPEED * CV.MS_TO_KPH - 0.5
    assert not car.run(20, feedback(state=TorqueInterceptorState.OFF, viol=0x12)).steerFaultTemporary

  def test_the_cars_lane_keep_setting_and_camera_fault_do_not_stop_it(self):
    car = Car()
    off = car.packer.make_can_msg("CAM_SETTINGS", 2, {"LKAS_INERVENTION_ON1": 0, "ILKAS_NTERVENTION_ON2": 0})
    faulted = car.packer.make_can_msg("CAM_LKAS", 2, {"ERR_BIT_1": 1})
    ret = car.run(20, feedback(), extra=[off, faulted])
    assert not ret.invalidLkasSetting
    assert not ret.steerFaultPermanent

  def test_the_panda_refusing_the_command_is_reported_for_one_cycle(self):
    car = Car()
    car.run(4, feedback())
    assert not car.CS.ti_rejected
    refused = (TI_LKAS, bytes.fromhex("0806 0806 c461ce60"), TI_REJECTED_SRC)
    car.run(1, feedback(), extra=[refused])
    assert car.CS.ti_rejected
    car.run(1, feedback())
    assert not car.CS.ti_rejected


class TestNotReadyAlert:
  """CarState.ti_not_ready: the interceptor named to the driver, not every pause of it."""

  @staticmethod
  def streaming(car, frames=TI_BOOT_FRAMES):
    car.CS.ti_stream_frames = frames
    return car

  def test_never_before_the_command_stream_is_up(self):
    # card runs from ignition; the stream, and with it the interceptor, starts later
    car = Car()
    car.speed_kph = 50.
    car.run(TI_BOOT_FRAMES + TI_NOT_READY_ALERT_FRAMES * 2)
    assert not car.CS.ti_not_ready

  def test_not_while_it_boots(self):
    # OFF, then RUN some 5 s after the first command frame
    car = self.streaming(Car(), frames=1)
    car.speed_kph = 50.
    for _ in range(TI_BOOT_FRAMES - 10):
      car.CS.ti_stream_frames += 1
      car.run(1, feedback(state=TorqueInterceptorState.OFF, viol=0x12))
      assert not car.CS.ti_not_ready

  def test_silent_under_a_running_stream_at_any_speed(self):
    # unplugged, unpowered, or bus 1 not on the OBD-II pins: it answers within one frame otherwise
    car = self.streaming(Car())
    car.run(TI_NOT_READY_ALERT_FRAMES - 1)
    assert not car.CS.ti_not_ready
    car.run(1)
    assert car.CS.ti_not_ready
    # heard and fit for REFIT_T, it is forgotten
    car.run(TI_REFIT_FRAMES - 2, feedback())
    assert car.CS.ti_not_ready
    car.run(4, feedback())
    assert not car.CS.ti_not_ready

  def test_unfit_while_moving(self):
    car = self.streaming(Car())
    car.speed_kph = 50.
    latched = feedback(state=TorqueInterceptorState.OFF, viol=0x11)
    car.run(TI_NOT_READY_ALERT_FRAMES - 1, latched)
    assert not car.CS.ti_not_ready
    car.run(2, latched)
    assert car.CS.ti_not_ready
    car.run(TI_REFIT_FRAMES + 2, feedback())
    assert not car.CS.ti_not_ready

  def test_a_latch_named_while_moving_stays_named_at_a_stop(self):
    # the alert (and MADS's no-entry on it) going away at a red light would read as recovery
    car = self.streaming(Car())
    car.speed_kph = 50.
    latched = feedback(state=TorqueInterceptorState.OFF, viol=0x11)
    car.run(TI_NOT_READY_ALERT_FRAMES + 5, latched)
    assert car.CS.ti_not_ready
    car.speed_kph = 0.
    car.run(500, latched)
    assert car.CS.ti_not_ready
    car.run(TI_REFIT_FRAMES + 2, feedback())
    assert not car.CS.ti_not_ready

  def test_named_it_takes_no_new_engagement_moving_or_stopped(self):
    # stopped there is no steer fault to refuse cruise, and SET under the standing alert would
    # be the take-control alarm on pulling away: carNotReady is upstream's no-entry and nothing more
    car = self.streaming(Car())
    car.speed_kph = 50.
    assert not car.run(20, feedback()).carNotReady
    latched = feedback(state=TorqueInterceptorState.OFF, viol=0x11)
    ret = car.run(TI_NOT_READY_ALERT_FRAMES + 5, latched)
    assert ret.carNotReady and ret.steerFaultTemporary
    car.speed_kph = 0.
    ret = car.run(100, latched)
    assert ret.carNotReady and not ret.steerFaultTemporary
    assert not car.run(TI_REFIT_FRAMES + 2, feedback()).carNotReady

  def test_a_car_without_one_is_never_not_ready(self):
    CP = car_params(CAR.MAZDA_CX5)
    CI = CarInterface(CP, car_params_sp(CP, CAR.MAZDA_CX5))
    ret, _ = CI.update([(0, [])])
    assert not ret.carNotReady

  def test_not_at_a_stop_and_not_for_the_driver_steering_through_it(self):
    car = self.streaming(Car())
    car.run(TI_NOT_READY_ALERT_FRAMES * 2, feedback(state=TorqueInterceptorState.OFF, viol=0x12))
    assert not car.CS.ti_not_ready
    car.speed_kph = 50.
    car.run(TI_NOT_READY_ALERT_FRAMES * 2, feedback(torque=60, state=TorqueInterceptorState.DRIVER_OVER))
    assert not car.CS.ti_not_ready

  def test_a_dropout_that_ends_does_not_alert(self):
    car = self.streaming(Car())
    car.speed_kph = 50.
    car.run(20, feedback())
    for _ in range(3):
      car.run(TI_NOT_READY_ALERT_FRAMES // 2, feedback(state=TorqueInterceptorState.OFF, viol=0x12))
      car.run(TI_REFIT_FRAMES + 2, feedback())
    assert not car.CS.ti_not_ready

  def test_dropping_out_again_and_again_is_named_without_a_fault(self):
    # a 20 ms ramp-down a little over once a second: never a fault, the count is cleared
    # between them, and every one a restart of the command from zero. More often than once in
    # DROPOUT_T, kept up, it is named
    car = self.streaming(Car())
    car.speed_kph = 50.
    car.run(20, feedback())
    named_after = None
    for n in range(1, 13):
      assert not car.run(2, feedback(ramp=1)).steerFaultTemporary
      assert not car.run(TI_REFIT_FRAMES + 10, feedback()).steerFaultTemporary
      if named_after is None and car.CS.ti_not_ready:
        named_after = n
    # each owes DROPOUT_T and pays off its own 1.12 s: the fifth takes it past DROPOUT_ALERT_T
    assert named_after == 5
    # the frame of the moment is a healthy one; what the driver is told is why it drops out
    assert car.CS.ti_ready and car.CS.ti_unfit_cause == TorqueInterceptorCause.RAMP_DOWN
    # it stays named, coming and going with every dropout would be an alert of its own each
    # time, until all of it is paid off
    car.run(TI_DROPOUT_ALERT_FRAMES - 2 * TI_REFIT_FRAMES, feedback())
    assert car.CS.ti_not_ready
    car.run(TI_DROPOUT_FRAMES + 2 * TI_REFIT_FRAMES, feedback())
    assert not car.CS.ti_not_ready and not car.CS.ti_dropping_out
    assert car.CS.ti_unfit_cause == TorqueInterceptorCause.NONE

  def test_a_dropout_now_and_then_is_not_named(self):
    car = self.streaming(Car())
    car.speed_kph = 50.
    car.run(20, feedback())
    for _ in range(12):
      car.run(24)
      car.run(TI_DROPOUT_FRAMES + 20, feedback())
      assert not car.CS.ti_not_ready
    assert car.CS.ti_dropout_frames == 0

  def test_dropouts_at_a_stop_are_not_counted(self):
    # stopped it is sent nothing: what it does there costs no command a restart
    car = self.streaming(Car())
    car.run(20, feedback())
    for _ in range(12):
      car.run(2, feedback(ramp=1))
      car.run(60, feedback())
    assert car.CS.ti_dropout_frames == 0 and not car.CS.ti_not_ready


class TestWire:
  def test_the_idle_frame(self):
    addr, dat, bus = create_ti_steering_control(CANPacker(DBC_NAME), 0)
    assert (addr, bus) == (TI_LKAS, TI_BUS)
    assert dat == bytes.fromhex("0800 0800 c461ce60")

  @pytest.mark.parametrize("torque", [-600, -1, 1, 6, 600])
  def test_the_request_is_sent_twice_under_the_run_option(self, torque):
    _, dat, _ = create_ti_steering_control(CANPacker(DBC_NAME), torque)
    request = ((dat[0] << 8) | dat[1]) - 2048
    copy = ((dat[2] << 8) | dat[3]) - 2048
    assert request == copy == torque
    assert dat[4:] == TI_KEY.to_bytes(4, "big") == bytes.fromhex("c461ce60")


class TestController:
  def test_streams_every_frame_engaged_or_not(self):
    cc, cs = ti_controller()
    for lat_active in (False, True, False):
      for _ in range(20):
        _, sends = step(cc, cs, long_active=False, enabled=lat_active, lat_active=lat_active, torque=0.2, v_ego=20.)
        assert len(frames(sends, TI_LKAS, TI_BUS)) == 1
    assert cs.ti_stream_frames == 60

  def test_a_car_without_it_sends_nothing_on_bus_1(self, stock_cc, stock_cs):
    for _ in range(20):
      _, sends = step(stock_cc, stock_cs, torque=0.5, v_ego=20., **LAT)
      assert all(bus != TI_BUS for _, _, bus in sends)

  def test_idle_is_zero(self):
    cc, cs = ti_controller()
    _, sends = step(cc, cs, long_active=False, enabled=False, lat_active=False, torque=0.5, v_ego=20.)
    assert ti_torque(sends) == 0

  def test_ramps_at_its_own_rate_to_the_scaled_command(self):
    cc, cs = ti_controller()
    target = round(0.3 * TI.STEER_SCALE)
    seen = []
    for _ in range(target // TI.STEER_DELTA_UP + 5):
      _, sends = step(cc, cs, torque=0.3, v_ego=20., **LAT)
      seen.append(ti_torque(sends))
    assert seen[:3] == [TI.STEER_DELTA_UP, 2 * TI.STEER_DELTA_UP, 3 * TI.STEER_DELTA_UP]
    assert seen[-1] == target == 270

  @pytest.mark.parametrize("sign", [1, -1])
  def test_the_rail_is_600_on_the_wire_and_nothing_higher(self, sign):
    cc, cs = ti_controller()
    for torque in (TI.STEER_MAX / TI.STEER_SCALE, 1.0):
      for _ in range(200):
        actuators, sends = step(cc, cs, torque=sign * torque, v_ego=20., **LAT)
      assert ti_torque(sends) == sign * TI.STEER_MAX
      assert actuators.torque == pytest.approx(sign * TI.STEER_MAX / TI.STEER_SCALE)

  def test_reports_the_interceptor_as_the_actuator(self):
    cc, cs = ti_controller()
    for _ in range(100):
      actuators, sends = step(cc, cs, torque=0.3, v_ego=20., **LAT)
      assert actuators.torqueOutputCan == ti_torque(sends)
      assert actuators.torque == pytest.approx(ti_torque(sends) / TI.STEER_SCALE)
    assert at_rest_on(ti_torque(sends), 270)
    assert actuators.torque == pytest.approx(0.3, abs=1 / TI.STEER_SCALE)

  def test_the_eps_command_is_the_same_frame_for_the_same_torque(self):
    # hands off and where the EPS takes a request, 0x243 is byte for byte what the car without
    # the interceptor sends for that controller torque
    cc, cs = ti_controller()
    CP = car_params(CAR.MAZDA_CX5)
    stock_cc = CarController({Bus.pt: DBC_NAME}, CP, car_params_sp(CP, CAR.MAZDA_CX5))
    stock_cs = mazda_car_state(stock_cc.CP, stock_cc.CP_SP)
    for i in range(150):
      torque = 0.5 if i < 100 else -0.2
      _, sends = step(cc, cs, torque=torque, v_ego=20., **LAT)
      _, stock_sends = step(stock_cc, stock_cs, torque=torque, v_ego=20., **LAT)
      assert frame(sends, CAM_LKAS) == frame(stock_sends, CAM_LKAS)

  def test_the_eps_command_stops_where_the_interceptors_rail_puts_it(self):
    # what differs is the torque it is given: the lateral stack's rail is the interceptor's
    # 600/900, so the EPS command tops out at 800 counts wherever its own ceiling is higher,
    # and it yields to the driver over the interceptor's span, not the EPS's 15 to 50
    cc, cs = ti_controller(CAR.MAZDA_CX5, car_fw=STEER_TO_ZERO_FW)
    assert cc.params.STEER_OVERRIDE_BP == [15, 30]
    rail = get_steer_rail_schedule(cc.CP)[1][0]
    for _ in range(200):
      _, sends = step(cc, cs, torque=rail, v_ego=6., **LAT)
    dat = frame(sends, CAM_LKAS)
    assert (((dat[0] & 0x0F) << 8) | dat[1]) - 2048 == 800
    assert ti_torque(sends) == TI.STEER_MAX
    for _ in range(100):
      _, sends = step(cc, cs, torque=0.5, v_ego=6., driver_torque=-30, steering_pressed=True, **LAT)
    dat = frame(sends, CAM_LKAS)
    assert (((dat[0] & 0x0F) << 8) | dat[1]) - 2048 == 0

  @pytest.mark.parametrize("state", [dict(lkas_allowed_speed=False), dict(lkas_blocked=True), dict(lkas_allowed_speed=False, lkas_blocked=True)])
  def test_a_legacy_eps_that_takes_no_request_is_sent_none(self, state):
    # below its floor and through its lockout the interceptor steers alone: the EPS gets the
    # zero command it always got there, and the interceptor its own
    cc, cs = ti_controller()
    for _ in range(50):
      actuators, sends = step(cc, cs, torque=0.3, v_ego=8., **state, **LAT)
      dat = frame(sends, CAM_LKAS)
      assert (((dat[0] & 0x0F) << 8) | dat[1]) - 2048 == 0
    assert at_rest_on(ti_torque(sends), 270)
    assert actuators.torque == pytest.approx(0.3, abs=1 / TI.STEER_SCALE)
    # once it takes one again the EPS command ramps from zero
    _, sends = step(cc, cs, torque=0.3, v_ego=20., **LAT)
    dat = frame(sends, CAM_LKAS)
    assert (((dat[0] & 0x0F) << 8) | dat[1]) - 2048 == cc.params.STEER_DELTA_UP

  def test_steer_to_zero_firmware_is_commanded_through_its_own_blocks(self):
    # that firmware's blocks are the standby its own latch handles, interceptor or not
    cc, cs = ti_controller(CAR.MAZDA_CX5, car_fw=STEER_TO_ZERO_FW)
    for _ in range(50):
      _, sends = step(cc, cs, torque=0.3, v_ego=8., lkas_blocked=True, **LAT)
    dat = frame(sends, CAM_LKAS)
    assert (((dat[0] & 0x0F) << 8) | dat[1]) - 2048 == 360

  def test_unfit_is_zero_at_once_and_the_ramp_restarts(self):
    cc, cs = ti_controller()
    for _ in range(50):
      step(cc, cs, torque=0.5, v_ego=20., **LAT)
    cs.ti_ready = False
    _, sends = step(cc, cs, torque=0.5, v_ego=20., **LAT)
    assert ti_torque(sends) == 0
    cs.ti_ready = True
    _, sends = step(cc, cs, torque=0.5, v_ego=20., **LAT)
    assert ti_torque(sends) == TI.STEER_DELTA_UP

  def test_a_refusal_restarts_the_ramp_from_zero(self):
    # the frame that learns of it is a zero, which the panda admits whatever its state
    cc, cs = ti_controller()
    for _ in range(50):
      step(cc, cs, torque=0.5, v_ego=20., **LAT)
    cs.ti_rejected = True
    _, sends = step(cc, cs, torque=0.5, v_ego=20., **LAT)
    assert ti_torque(sends) == 0
    cs.ti_rejected = False
    seen = []
    for _ in range(3):
      _, sends = step(cc, cs, torque=0.5, v_ego=20., **LAT)
      seen.append(ti_torque(sends))
    assert seen == [TI.STEER_DELTA_UP, 2 * TI.STEER_DELTA_UP, 3 * TI.STEER_DELTA_UP]

  def test_release_slews_down_instead_of_cutting(self):
    cc, cs = ti_controller()
    for _ in range(90):
      _, sends = step(cc, cs, torque=0.5, v_ego=20., **LAT)
    top = ti_torque(sends)
    assert top == 450
    seen = []
    for _ in range(top // TI.STEER_DELTA_DOWN + 2):
      _, sends = step(cc, cs, long_active=False, enabled=True, lat_active=False, torque=0., v_ego=20.)
      seen.append(ti_torque(sends))
    assert seen[:2] == [top - TI.STEER_DELTA_DOWN, top - 2 * TI.STEER_DELTA_DOWN]
    assert seen[-2:] == [0, 0]
    assert all(a > b or a == b == 0 for a, b in zip(seen, seen[1:], strict=False))

  def test_nothing_is_requested_below_min_speed(self):
    cc, cs = ti_controller()
    for _ in range(50):
      _, sends = step(cc, cs, torque=0.5, v_ego=TI.MIN_SPEED - 0.1, **LAT)
      assert ti_torque(sends) == 0
    _, sends = step(cc, cs, torque=0.5, v_ego=TI.MIN_SPEED, **LAT)
    assert ti_torque(sends) == TI.STEER_DELTA_UP

  def test_steer_to_zero_firmware_below_min_speed_is_the_eps_alone(self):
    # that firmware steers to a stop without the interceptor and still does: minSteerSpeed stays
    # zero, the EPS is commanded, the interceptor is sent nothing and reports nothing
    cc, cs = ti_controller(CAR.MAZDA_CX5, car_fw=STEER_TO_ZERO_FW)
    assert cc.CP.minSteerSpeed == 0
    for _ in range(60):
      actuators, sends = step(cc, cs, torque=0.3, v_ego=0.8, **LAT)
    dat = frame(sends, CAM_LKAS)
    assert (((dat[0] & 0x0F) << 8) | dat[1]) - 2048 == 360
    assert ti_torque(sends) == 0
    assert actuators.torque == 0 and actuators.torqueOutputCan == 0

  def test_the_dash_warning_follows_the_interceptor_below_the_eps_floor(self, stock_cc, stock_cs):
    # the cluster's hands-on-wheel frame mirrors the steerRequired alert where something
    # steers: above the EPS's floor on a car without the interceptor, anywhere with it
    alert = dict(visual_alert=VisualAlert.steerRequired, lkas_allowed_speed=False, v_ego=8., torque=0.3, **LAT)
    quiet = dict(lkas_allowed_speed=False, v_ego=8., torque=0.3, **LAT)

    def drive(cc, cs, cycles, **kwargs):
      hud = []
      for _ in range(cycles):
        _, out = step(cc, cs, **kwargs)
        hud.extend(hands_code(d) for d in frames(out, CAM_LANEINFO))
      return hud

    cc, cs = ti_controller()
    drive(cc, cs, DASH_STEER_WARNING_QUIET_FRAMES + 1, **quiet)
    assert drive(cc, cs, 2 * HUD_ALERT_FRAMES, **alert) == [(0b111, 1, 1)] * 2
    drive(stock_cc, stock_cs, DASH_STEER_WARNING_QUIET_FRAMES + 1, **quiet)
    assert drive(stock_cc, stock_cs, 2 * HUD_ALERT_FRAMES, **alert) == [(0, 0, 0)] * 2

  def test_slowing_to_a_stop_takes_the_request_down_to_zero(self):
    cc, cs = ti_controller()
    for _ in range(100):
      _, sends = step(cc, cs, torque=0.5, v_ego=5., **LAT)
    top = ti_torque(sends)
    assert at_rest_on(top, 450)
    # a walk at its own down rate, as when openpilot lets go, not a cut
    seen = []
    for _ in range(40):
      _, sends = step(cc, cs, torque=0.5, v_ego=0.5, **LAT)
      seen.append(ti_torque(sends))
    assert seen[:2] == [top - TI.STEER_DELTA_DOWN, top - 2 * TI.STEER_DELTA_DOWN]
    assert ti_torque(sends) == 0

  def test_with_the_interceptor_quiet_the_eps_command_yields_on_the_eps_reading(self):
    # carstate holds the interceptor's last reading through its silence (BYPASS_T), and so
    # does the panda. A driver who starts pushing then shows only on the EPS's own reading
    cc, cs = ti_controller(CAR.MAZDA_CX5, car_fw=STEER_TO_ZERO_FW)

    def eps_torque(sends):
      dat = frame(sends, CAM_LKAS)
      return (((dat[0] & 0x0F) << 8) | dat[1]) - 2048

    cs.ti_fresh = True
    for _ in range(100):
      _, sends = step(cc, cs, torque=0.5, v_ego=20., **LAT)
    assert eps_torque(sends) == 600
    # talking, the interceptor's reading is the driver's and the EPS's is its bias: ignored
    cs.eps_driver_torque = -80.
    for _ in range(20):
      _, sends = step(cc, cs, torque=0.5, v_ego=20., **LAT)
    assert eps_torque(sends) == 600
    # quiet, the held reading says nothing of a push that began since
    cs.ti_fresh = cs.ti_ready = False
    seen = []
    for _ in range(70):
      _, sends = step(cc, cs, torque=0.5, v_ego=20., **LAT)
      seen.append(eps_torque(sends))
    assert seen[9] <= 600 - 10 * cc.params.STEER_DELTA_DOWN + 1, "walking down inside 100 ms"
    assert seen[-1] == 0
    # a reading with the command's sign, the interceptor's bias if any is left, takes nothing
    cs.eps_driver_torque = 80.
    for _ in range(100):
      _, sends = step(cc, cs, torque=0.5, v_ego=20., **LAT)
    assert eps_torque(sends) == 600

  @pytest.mark.parametrize("sign", [1, -1])
  def test_the_driver_takes_it_all(self, sign):
    # 40 counts of command for each count of opposing torque past the allowance, read with the
    # controller's margin: all of it is gone two counts inside the panda's 30
    cc, cs = ti_controller()
    for _ in range(150):
      _, sends = step(cc, cs, torque=sign * 0.6, v_ego=20., **LAT)
    full = sign * 540
    assert at_rest_on(ti_torque(sends), full)
    margin = cc.params.STEER_DRIVER_MARGIN
    gone = TI.STEER_DRIVER_ALLOWANCE + TI.STEER_MAX // TI.STEER_DRIVER_MULTIPLIER - margin
    assert gone == 28
    for driver, share in ((10, full), (20, sign * (TI.STEER_MAX - (20 + margin - TI.STEER_DRIVER_ALLOWANCE) * TI.STEER_DRIVER_MULTIPLIER)),
                          (gone, 0), (60, 0)):
      for _ in range(150):
        _, sends = step(cc, cs, torque=sign * 0.6, v_ego=20., driver_torque=-sign * driver, steering_pressed=True, **LAT)
      assert at_rest_on(ti_torque(sends), share), driver

  @pytest.mark.parametrize("sign", [1, -1])
  @pytest.mark.parametrize("torque, driver", [(0.6, 20), (0.3, 0), (0.3, 10), (0.002, 0), (0.66, 0)])
  def test_a_request_at_rest_is_kept_moving(self, sign, torque, driver):
    # the interceptor reads a request that sits still off 0 and off its clip as a stuck sender
    # and latches off for 30 s. A steady hand against the command pins the driver limit there;
    # a quiet road pins the controller's own output
    cc, cs = ti_controller()
    seen = []
    for _ in range(600):
      _, sends = step(cc, cs, torque=sign * torque, v_ego=20., driver_torque=-sign * driver, steering_pressed=driver > TI.STEER_THRESHOLD, **LAT)
      seen.append(ti_torque(sends))
    limit = TI.STEER_MAX - max(driver + cc.params.STEER_DRIVER_MARGIN - TI.STEER_DRIVER_ALLOWANCE, 0) * TI.STEER_DRIVER_MULTIPLIER
    rest = sign * min(round(torque * TI.STEER_SCALE), limit)
    assert 0 < abs(rest) < TI.STEER_MAX
    settled = seen[200:]
    assert set(settled) == {rest, rest - sign}
    longest = max(len(list(run)) for _, run in groupby(settled))
    assert longest <= TI.STEER_HELD_FRAMES
    # one frame off it in every STEER_HELD_FRAMES + 1, and straight back
    assert settled.count(rest - sign) in (len(settled) // (TI.STEER_HELD_FRAMES + 1), len(settled) // (TI.STEER_HELD_FRAMES + 1) + 1)
    assert all(n == 1 for value, n in ((v, len(list(run))) for v, run in groupby(settled)) if value == rest - sign)

  @pytest.mark.parametrize("sign", [1, -1])
  @pytest.mark.parametrize("torque", [TI.STEER_MAX / TI.STEER_SCALE, 1.0, 0.])
  def test_a_request_at_rest_on_zero_or_the_clip_is_left_alone(self, sign, torque):
    # the two values the interceptor lets a request sit still on
    cc, cs = ti_controller()
    seen = []
    for _ in range(400):
      _, sends = step(cc, cs, torque=sign * torque, v_ego=20., **LAT)
      seen.append(ti_torque(sends))
    assert set(seen[200:]) == {sign * min(round(torque * TI.STEER_SCALE), TI.STEER_MAX)}

  def test_a_request_never_sits_still_however_it_is_driven(self):
    import random
    rng = random.Random(3)
    cc, cs = ti_controller()
    torque, driver, seen = 0., 0., []
    for i in range(6000):
      if i % 300 == 0:
        torque, driver = rng.choice([0., 0.01, 0.2, 0.5, 0.66, 0.67, 1.0]) * rng.choice([1, -1]), rng.choice([0, 0, 12, 18, 25, 40]) * rng.choice([1, -1])
      _, sends = step(cc, cs, torque=torque, v_ego=20., driver_torque=driver, steering_pressed=abs(driver) > TI.STEER_THRESHOLD, **LAT)
      seen.append(ti_torque(sends))
    runs = [(value, len(list(run))) for value, run in groupby(seen)]
    assert max(n for value, n in runs if value not in (0, TI.STEER_MAX, -TI.STEER_MAX)) <= TI.STEER_HELD_FRAMES
    assert any(n > TI.STEER_HELD_FRAMES for value, n in runs if abs(value) == TI.STEER_MAX)

  def test_a_push_with_the_command_costs_nothing(self):
    cc, cs = ti_controller()
    for _ in range(150):
      _, sends = step(cc, cs, torque=0.6, v_ego=20., driver_torque=40, steering_pressed=True, **LAT)
    assert at_rest_on(ti_torque(sends), 540)

  def test_steer_to_zero_firmware_keeps_its_own_eps_path_beside_it(self):
    cc, cs = ti_controller(CAR.MAZDA_CX5, car_fw=STEER_TO_ZERO_FW)
    assert cc.steer_to_zero and cc.torque_interceptor
    for _ in range(50):
      _, sends = step(cc, cs, torque=0.3, v_ego=20., **LAT)
    assert at_rest_on(ti_torque(sends), 270)


class Panda:
  """The compiled safety model with the interceptor admitted and openpilot steering: each frame
  it hears the interceptor, then judges the two commands the real controller sends. It also
  keeps what the host has read of the interceptor so far, HOST_LAG cycles behind the panda."""

  HOST_LAG = 1  # card cycles between the panda hearing a feedback frame and carstate reading it

  def __init__(self, safety_param=MazdaSafetyFlags.EPS_HW | MazdaSafetyFlags.TORQUE_INTERCEPTOR):
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.mazda, safety_param)
    self.safety.init_tests()
    self.safety.set_controls_allowed(True)
    self.in_flight: deque = deque([None] * self.HOST_LAG)
    self.read = None            # the last feedback frame carstate has read: (kwargs, driver torque)
    self.silent_frames = TI_FEEDBACK_FRESH_FRAMES

  def hear(self, frame_idx: int, fb_kwargs, driver) -> None:
    """One cycle: the panda hears the frame now, the host a cycle later."""
    self.safety.set_timer(frame_idx * 10_000)
    sent = fb_kwargs is not None and frame_idx % 2 == 0
    if sent:
      addr, dat, bus = feedback(torque=driver, **fb_kwargs)
      self.safety.safety_rx_hook(libsafety_py.make_CANPacket(addr, bus, dat))
    self.in_flight.append((fb_kwargs, driver) if sent else None)
    arrived = self.in_flight.popleft()
    if arrived is not None:
      self.read, self.silent_frames = arrived, 0
    else:
      self.silent_frames = min(self.silent_frames + 1, TI_FEEDBACK_FRESH_FRAMES)

  @property
  def fit(self) -> bool:
    """The interceptor as carstate rules on it (ti_ready), from what the host has read."""
    if self.read is None or self.silent_frames >= TI_FEEDBACK_FRESH_FRAMES:
      return False
    fb = self.read[0]
    return all(fb.get(k, 0) == 0 for k in ("viol", "error", "ramp")) and fb.get("state", TorqueInterceptorState.RUN) == TorqueInterceptorState.RUN

  @property
  def driver_torque(self):
    return 0 if self.read is None else self.read[1]

  def tx(self, addr, dat, bus) -> bool:
    return self.safety.safety_tx_hook(libsafety_py.make_CANPacket(addr, bus, dat))


class TestClosedLoopThroughThePanda:
  """What the controller sends for the interceptor, the panda admits, with two exceptions no
  controller can avoid: the frame already on its way when the panda's own permission falls (a
  brake, a cancel), and the one on its way when the interceptor reports itself unfit, which
  the panda hears a cycle before carstate does. Either costs the frames in flight and then a
  zero; a refusal costs the interceptor frames of its stream and the driver a restart of the
  ramp."""

  REPORT_DELAY = 2  # card cycles for the panda's refusal to come back on the can stream

  def loop(self, profile, cc=None, cs=None, panda=None, frame0=0, lat_active=lambda i: True):
    """profile yields (commanded torque, driver torque, v_ego, feedback kwargs or None for a
    silent interceptor). The controller reads the interceptor, and the driver torque it
    carries, a cycle after the panda and through the rules carstate applies. Returns (frames
    the panda refused, the interceptor torques it admitted)."""
    if cc is None or cs is None:
      cc, cs = ti_controller()
    panda = panda or Panda()
    refused_reports = deque([False] * self.REPORT_DELAY, maxlen=self.REPORT_DELAY)
    refused, admitted = [], []
    for i, (torque, driver, v_ego, fb_kwargs) in enumerate(profile, start=frame0):
      panda.hear(i, fb_kwargs, driver)
      cs.ti_ready, cs.ti_rejected = panda.fit, refused_reports[0]
      _, sends = step(cc, cs, long_active=False, enabled=True, lat_active=lat_active(i), torque=torque, v_ego=v_ego,
                      driver_torque=panda.driver_torque, steering_pressed=abs(panda.driver_torque) > TI.STEER_THRESHOLD)
      dat = frame(sends, TI_LKAS, TI_BUS)
      ok = panda.tx(TI_LKAS, dat, TI_BUS)
      refused_reports.append(not ok)
      if ok:
        admitted.append(ti_torque(sends))
      else:
        refused.append((i, ti_torque(sends)))
    return refused, admitted, cc, cs, panda

  def test_full_scale_reversals(self):
    profile = [(1.0 if (i // 150) % 2 == 0 else -1.0, 0, 20., {}) for i in range(900)]
    refused, admitted, *_ = self.loop(profile)
    assert refused == []
    assert max(admitted) == TI.STEER_MAX and min(admitted) == -TI.STEER_MAX

  @pytest.mark.parametrize("slope", [1, 2, 5, 10, 30])
  @pytest.mark.parametrize("sign", [1, -1])
  def test_the_driver_ramping_against_it(self, sign, slope):
    ramp = [(sign * 1.0, 0, 20., {}) for _ in range(120)]
    push = [(sign * 1.0, -sign * min(slope * f, 85), 20., {}) for f in range(200)]
    release = [(sign * 1.0, 0, 20., {}) for _ in range(200)]
    refused, admitted, *_ = self.loop(ramp + push + release)
    assert refused == []
    assert admitted[119] == sign * TI.STEER_MAX
    assert admitted[319] == 0
    assert admitted[-1] == sign * TI.STEER_MAX

  def test_a_noisy_driver_and_a_wandering_command(self):
    import random
    rng = random.Random(7)
    profile, torque, driver = [], 0., 0.
    for _ in range(4000):
      torque = max(-1., min(1., torque + rng.uniform(-0.05, 0.05)))
      driver = max(-85., min(85., driver + rng.uniform(-6, 6)))
      profile.append((torque, round(driver), 20., {}))
    refused, admitted, *_ = self.loop(profile)
    assert refused == []
    assert any(admitted)

  def test_lateral_dropping_in_and_out(self):
    profile = [(0.7, 0, 20., {}) for _ in range(600)]
    refused, admitted, *_ = self.loop(profile, lat_active=lambda i: (i // 80) % 2 == 0)
    assert refused == []
    assert max(admitted) > 400 and 0 in admitted[100:]

  def test_stop_and_go(self):
    profile = [(0.7, 0, 0.2 if 200 <= i < 400 else 10., {}) for i in range(700)]
    refused, admitted, *_ = self.loop(profile)
    assert refused == []
    assert set(admitted[280:399]) == {0}
    assert admitted[-1] == TI.STEER_MAX

  @pytest.mark.parametrize("unfit", [dict(state=TorqueInterceptorState.OFF, viol=0x11), dict(ramp=1),
                                     dict(state=TorqueInterceptorState.DRIVER_OVER), dict(error=3)])
  def test_the_interceptor_dropping_out_and_recovering(self, unfit):
    profile = [(0.6, 0, 20., {}) for _ in range(150)] + [(0.6, 0, 20., unfit) for _ in range(100)] + \
              [(0.6, 0, 20., {}) for _ in range(150)]
    refused, admitted, *_ = self.loop(profile)
    # the panda hears the unfit frame a cycle before carstate: the command on its way is
    # refused, and the next is the zero the controller sends an unfit interceptor anyway
    assert len(refused) <= Panda.HOST_LAG and all(i == 150 and at_rest_on(torque, 540) for i, torque in refused)
    assert at_rest_on(admitted[148], 540)
    assert set(admitted[150:245]) == {0}
    assert at_rest_on(admitted[-1], 540)

  def test_the_interceptor_going_silent_and_returning(self):
    profile = [(0.6, 0, 20., {}) for _ in range(150)] + [(0.6, 0, 20., None) for _ in range(100)] + \
              [(0.6, 0, 20., {}) for _ in range(150)]
    refused, admitted, *_ = self.loop(profile)
    # the controller's 70 ms is inside the panda's 100 ms: it stops before it is refused
    assert refused == []
    assert set(admitted[160:250]) == {0}
    assert at_rest_on(admitted[-1], 540)

  def test_openpilot_letting_go_walks_the_request_down(self):
    # lateral dropped by openpilot with the panda's permission still up (a software pause,
    # the MADS button, slowing through MIN_SPEED): 15 a frame, nothing refused
    refused, admitted, *_ = self.loop([(0.6, 0, 20., {}) for _ in range(190)], lat_active=lambda i: i < 150)
    assert refused == []
    assert at_rest_on(admitted[149], 540)
    down = admitted[150:153]
    assert [a - b for a, b in zip(down, down[1:], strict=False)] == [TI.STEER_DELTA_DOWN] * 2
    assert admitted[-1] == 0

  @pytest.mark.parametrize("lateral_follows", [0, 2, 6])
  def test_the_panda_letting_go_first_is_a_cut(self, lateral_follows):
    # a brake, a cancel or main off reaches the panda on CAN before openpilot acts on it: the
    # frames on their way are refused, the controller hears of it and sends zeros. No walk-down
    # reaches the interceptor, and its stream is never silent for 100 ms
    refused, admitted, cc, cs, panda = self.loop([(0.6, 0, 20., {}) for _ in range(150)])
    assert refused == [] and at_rest_on(admitted[-1], 540)
    panda.safety.set_controls_allowed(False)
    refused, admitted, *_ = self.loop([(0.6, 0, 20., {}) for _ in range(60)], cc=cc, cs=cs, panda=panda, frame0=150,
                                      lat_active=lambda i: i < 150 + lateral_follows)
    assert set(admitted) == {0}
    assert 1 <= len(refused) <= self.REPORT_DELAY + lateral_follows
    assert refused[-1][0] - 150 < 10, "the interceptor is back on a stream inside its 100 ms"

  def test_a_refusal_it_did_not_cause_costs_only_the_report_delay(self):
    # the panda's reference zeroed from outside after a clean ramp (controls dropped and came
    # back between two frames): the next command is far above one step from zero
    refused, admitted, cc, cs, panda = self.loop([(0.6, 0, 20., {}) for _ in range(150)])
    assert refused == [] and at_rest_on(admitted[-1], 540)
    panda.safety.set_controls_allowed(False)
    assert panda.tx(*create_ti_steering_control(CANPacker(DBC_NAME), 0))
    panda.safety.set_controls_allowed(True)
    refused, admitted, *_ = self.loop([(0.6, 0, 20., {}) for _ in range(200)], cc=cc, cs=cs, panda=panda, frame0=150)
    assert len(refused) == self.REPORT_DELAY
    assert at_rest_on(admitted[-1], 540)

  @pytest.mark.parametrize("driver", [-16, -20, -27, 22])
  def test_a_steady_hand_against_it(self, driver):
    # the request held on the driver limit and nudged a count now and then: all admitted
    profile = [(0.66 if driver < 0 else -0.66, driver, 20., {}) for _ in range(800)]
    refused, admitted, *_ = self.loop(profile)
    assert refused == []
    assert len(set(admitted[300:])) == 2

  def test_no_torque_reaches_it_while_openpilot_is_not_steering(self):
    panda = Panda()
    panda.safety.set_controls_allowed(False)
    refused, admitted, *_ = self.loop([(0.6, 0, 20., {}) for _ in range(100)], panda=panda)
    # the controller keeps asking and the panda keeps refusing: no torque gets through, and the
    # refusals it is told about keep the asking to a sawtooth off zero, never a blind ramp
    assert set(admitted) == {0}
    assert len(refused) >= 40 and all(torque != 0 for _, torque in refused)
    assert max(abs(torque) for _, torque in refused) <= self.REPORT_DELAY * TI.STEER_DELTA_UP
    # each refusal it learns of is answered with a zero, so the interceptor keeps the stream
    # it faults without: never 100 ms between two frames it is let to hear
    heard = sorted(set(range(100)) - {i for i, _ in refused})
    assert max(b - a for a, b in zip(heard, heard[1:], strict=False)) <= self.REPORT_DELAY + 1


def test_structs_carry_the_interceptor_state_for_card():
  # card reads these off CarState to publish and to alert on
  CP, CP_SP = ti_car_params()
  cs = mazda_car_state(CP, CP_SP)
  for name in ("ti_enabled", "ti_ready", "ti_not_ready", "ti_state", "ti_violation", "ti_error", "ti_version"):
    assert hasattr(cs, name)
  assert cs.ti_enabled
  stock = car_params(CAR.MAZDA_CX5)
  assert not mazda_car_state(stock, car_params_sp(stock, CAR.MAZDA_CX5)).ti_enabled
