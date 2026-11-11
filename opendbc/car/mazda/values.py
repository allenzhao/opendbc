from dataclasses import dataclass, field
from enum import Enum, IntEnum, IntFlag, StrEnum

from opendbc.car import Bus, CarSpecs, DbcDict, DT_CTRL, PlatformConfig, Platforms
from opendbc.car.carlog import carlog
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.structs import CarParams
from opendbc.car.docs_definitions import CarFootnote, CarHarness, CarDocs, CarParts, Column
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries
from opendbc.car.vin import Vin, is_valid_vin

Ecu = CarParams.Ecu


# Steer torque limits

class CarControllerParams:
  # The measured EPS envelope, mazda.h's MAZDA_EPS_HW_STEERING_LIMITS: the panda's safety tests take
  # their numbers from here. Evidence for every constant below: the Constants tables in
  # docs/zoompilot/mazda-lateral.md and mazda-longitudinal.md.
  EPS_STEER_MAX = 1200            # theoretical max_steer 2047
  STEER_DELTA_UP = 10             # torque increase per refresh: the EPS's own ~1000 counts/s slew
  STEER_DELTA_DOWN = 10           # torque decrease per refresh
  STEER_DRIVER_ALLOWANCE = 15     # allowed driver torque before start limiting
  STEER_DRIVER_MULTIPLIER = 15    # weight driver torque, tuned for the 2022 EPS
  STEER_DRIVER_FACTOR = 1         # from dbc
  # The share of the command given up to a driver pushing against it, the same at every speed.
  # The headroom above is on the 1200 rail, so it never reached the 620 highway ceiling.
  STEER_OVERRIDE_BP = [STEER_DRIVER_ALLOWANCE, 50]  # opposing driver torque, counts
  STEER_OVERRIDE_V = [1., 0.]                       # fraction of the command kept
  STEER_STEP = 1  # 100 Hz
  # Upstream's STEER_MAX, the scale params.toml's tunes, NNLC models and the manual torque override
  # use: latAccelFactor x TUNE_SCALE, friction / TUNE_SCALE.
  TUNE_STEER_MAX = 800
  TUNE_SCALE = EPS_STEER_MAX / TUNE_STEER_MAX

  ACCEL_MAX = 2.0   # m/s2
  ACCEL_MIN = -3.5  # m/s2
  # CRZ_INFO.ACCEL_CMD resolution: stock's holds sit at raw -1024 and -410, i.e. -1.0 and -0.4 m/s2.
  ACCEL_CMD_LSB = 1 / 1024  # m/s2 per count

  # Longitudinal message periods in 100 Hz control frames.
  LONG_STEP = 2        # CRZ_INFO/CRZ_CTRL at 50 Hz, matching stock
  RADAR_STEP = 10      # radar static + track frames at 10 Hz

  FSC_SETTLE_T = 1.0           # two CAM_LANEINFO frames past the camera's boot marker before the teardown may start
  STOCK_RADAR_ALIVE_T = 0.05   # a normal CRZ_INFO gap; it does not establish ownership
  STOCK_RADAR_GUARD_T = 1.27   # silence before ownership is trusted, ~12x the longest stock gap
  STOCK_RADAR_ACK_GUARD_T = 0.25  # the same once the radar acknowledged our programming request
  CAM_LANEINFO_FRESH_T = 1.5   # longer than one ~2 Hz CAM_LANEINFO period

  # The car's lane keep back on, the EPS holds LKAS_BLOCK for ~3 s whatever it is sent.
  LKAS_REARM_T = 3.0           # no lift of the block counts before this
  LKAS_REARM_FAULT_T = 4.0     # the block it raises is not a fault for this long

  # Both PEDALS cruise bits low this many samples (100 Hz) is a main-off; mazda.h's MAZDA_MAIN_OFF_DEBOUNCE.
  MAIN_OFF_DEBOUNCE_SAMPLES = 10
  CANCEL_SETTLE_T = 0.2       # s a cancel request must hold before the first press; the car answers its own inside it

  # Relax the command after the body ECU takes ownership of the brake hold.
  ACCEL_HOLD_LATCHED = -ACCEL_CMD_LSB  # m/s2, raw -1

  # Match stock's ACCEL_CMD ceiling during a latched release pulse.
  ACCEL_RESUME_PULSE_MAX = 250 * ACCEL_CMD_LSB  # m/s2, raw 250, latched releases only

  # Match stock's one-frame relaxation and subsequent release ramp.
  ACCEL_RELEASE_BAND = -260 * ACCEL_CMD_LSB  # m/s2, raw -260, the one-frame relax target at a never-latched release
  ACCEL_RELEASE_RAMP = 25 * ACCEL_CMD_LSB * 50  # m/s3, stock's release ramp (+25 raw per 50 Hz frame)

  # Permit a bounded breakaway ramp because Mazda longitudinal control has no integrator.
  ACCEL_BREAKAWAY_MAX = 1.45  # m/s2, ceiling for the still-stopped release ramp
  ACCEL_BREAKAWAY_OVERSHOOT = 0.75  # m/s2 above the plan the still-stopped ramp may climb
  ACCEL_BREAKAWAY_RAMP = 2.5  # m/s3, the still-stopped climb once past stock's release frames

  # Shape positive commands like stock MRCC: its accelerating command's ceiling by speed, and its
  # build rate (taken a third quicker once rolling: the plan sees a lead pull away before the radar
  # would). Positive commands only, so braking is never held longer than the plan asks.
  ACCEL_CEILING_BP = [0., 4., 9., 14., 18., 25.]  # m/s
  ACCEL_CEILING_V = [1.5, 1.75, 1.45, 0.95, 0.85, 0.65]  # m/s2, 14 m/s under stock's 1.05 to hold fourth
  ACCEL_BUILD_BP = [3., 6., 12., 20.]   # m/s
  ACCEL_BUILD_V = [ACCEL_RELEASE_RAMP, 0.8, 0.8, 0.6]  # m/s3, stock's own 0.6 at highway speed
  # Stock's throttle lift rate, while the plan is still >= 0; a brake request skips it.
  ACCEL_LIFT_LIMIT = -2.0  # m/s3
  # Limit upward plan-command slew in the brake region without delaying braking response.
  ACCEL_WINDUP_LIMIT = 4.0 * DT_CTRL     # m/s2 per frame
  ACCEL_WINDDOWN_LIMIT = -10.0 * DT_CTRL  # m/s2 per frame, clips only the p99.9+ steps

  def __init__(self, CP):
    # Use a sample window and margin to stay inside panda's fresher driver-torque envelope.
    self.STEER_DRIVER_SAMPLES = 10
    self.STEER_DRIVER_MARGIN = 2

    # One scale at every speed keeps the learned torque parameters in one unit (docs/zoompilot/lateral-tune.md).
    self.STEER_MAX = self.EPS_STEER_MAX
    # Clamp to the measured applied-torque ceiling so controlsd can detect saturation.
    self.EPS_CEILING_LOOKUP = ([8.0, 8.5, 9.4, 10.3, 11.2, 12.1, 13.0, 13.9, 14.5],
                               [1148, 1132, 1092, 1048, 1012,  920,  808,  676,  620])

    if CP.flags & MazdaFlags.STEER_TO_ZERO_EPS:
      # Stop commanding after sustained zero delivery to avoid a camera steering fault. Use
      # LKAS_EFFECTIVE because LKAS_BLOCK may still permit partial delivery.
      self.STEER_UNDELIVERED_MIN = 200      # counts; below this the EPS rounds to zero anyway
      self.STEER_UNDELIVERED_FRAMES = 20    # 200 ms at 100 Hz

      # Alert only after sustained non-delivery above maneuvering speed. Suppress normal
      # low-speed standby blocks identified by LKAS_TRACK_STATE.
      self.STEER_UNDELIVERED_ALERT_FRAMES = 80    # 0.8 s at 100 Hz, on top of the latch's 0.2
      self.STEER_UNDELIVERED_ALERT_MIN_SPEED = 12. * CV.MPH_TO_MS
      # A block that began below this speed is the EPS's standby from a stop; only one that began
      # rolling can be a dropout. carstate's first-engagement hold uses the same boundary.
      self.STEER_UNDELIVERED_ALERT_ORIGIN_SPEED = 1.0  # m/s

    if CP.flags & MazdaFlags.TORQUE_INTERCEPTOR:
      ti = TorqueInterceptorParams
      # steeringTorque is the Torque Interceptor's own sensor reading, and its command is gone
      # by allowance + STEER_MAX / multiplier counts of opposing driver torque. The EPS command
      # gives way over the same span, so a driver who has taken the interceptor's share is not
      # left fighting the EPS's.
      self.STEER_OVERRIDE_BP = [ti.STEER_DRIVER_ALLOWANCE, ti.STEER_DRIVER_ALLOWANCE + ti.STEER_MAX // ti.STEER_DRIVER_MULTIPLIER]

      # The interceptor steers at every speed and is the actuator the controller reports
      # (actuatorsOutput.torque = its command / STEER_SCALE), so the lateral stack reads its
      # rail and slew instead of the EPS command's (opendbc/sunnypilot/car/lateral_tune.py).
      # The rail is the interceptor's clip exactly: a command held there is 600 on the wire,
      # the one nonzero value it may sit still on.
      self.STEER_RAIL_SCHEDULE = ([0.], [ti.STEER_MAX / ti.STEER_SCALE])
      self.STEER_SLEW_SCHEDULE = ([0.], [ti.STEER_DELTA_UP / ti.STEER_SCALE], [ti.STEER_DELTA_DOWN / ti.STEER_SCALE])
      # The speed-binned seeds were learned on the EPS command alone, most behind steer-to-zero
      # firmware. With the interceptor adding its share the car learns its own from the default
      # bins, under a seed version of its own so neither plant restores the other's cache.
      self.SPEED_DEP_CONFIG = {'seed_version': ti.SPEED_DEP_SEED_VERSION}


class TorqueInterceptorParams:
  """The MoreTorque Torque Interceptor (TI): limits for its own command, CAM_LKAS2 on bus 1.

  The TI sits between the column torque sensor and the EPS and biases the signal the EPS
  reads, so the EPS's own power assist steers at any speed, the legacy firmware's 45 kph floor
  included. mazda.h's MAZDA_TI_STEERING_LIMITS enforces the same numbers in the panda and the
  safety tests take theirs from here. Protocol and violation codes: the TI's published safety
  documentation (docs/zoompilot/mazda-torque-interceptor.md).
  """
  # The TI clips the request at 600 itself. It also latches VIOL_LKAS_STUK for 30 s on a request
  # that sits still anywhere other than 0 or +-600, so nothing may plateau below this: the
  # controller's rail (lateral_tune.get_steer_rail_schedule) lands exactly on it.
  STEER_MAX = 600
  # The TI author's rates; its firmware ramps the bias down by itself on a faster rise.
  STEER_DELTA_UP = 6              # torque increase per refresh
  STEER_DELTA_DOWN = 15           # torque decrease per refresh
  STEER_DRIVER_ALLOWANCE = 15     # allowed driver torque before start limiting
  STEER_DRIVER_MULTIPLIER = 40    # weight driver torque
  STEER_DRIVER_FACTOR = 1
  # 100 Hz, engaged or not: a gap in the stream over 100 ms is the TI's VIOL_CAN_TMO.
  STEER_STEP = 1
  # Frames a request may sit on one value, other than 0 and the clip, before it is moved a count.
  STEER_HELD_FRAMES = 25

  # Counts per unit of normalized torque. The EPS command is u x EPS_STEER_MAX on a tune
  # TUNE_SCALE x upstream's, so this is upstream's u x 600: three quarters of the EPS command's
  # count, the pairing pdscomp's port ran on legacy firmware (an 800 count EPS command beside
  # u x 600). The TI author's own fork sends the TI the same count as the EPS command, clipped
  # to 600, which on this scale would be 1200.
  STEER_SCALE = round(STEER_MAX * CarControllerParams.TUNE_SCALE)

  # steeringPressed on the TI's reading of the real sensor (the EPS's own is the biased signal).
  STEER_THRESHOLD = 6
  STEER_PRESSED_FRAMES = 5
  STEER_OVERRIDE_NOW = 20         # above this, pressed at once: no need to out-wait the filter

  # No request below this speed. Held still, the command stops moving and the TI reads a stuck
  # sender; a stopped car needs no lateral authority either. On legacy firmware lateral is off
  # below it too (minSteerSpeed); on steer-to-zero firmware the EPS command carries on alone,
  # as it does without the TI, and the reported actuator reads zero there.
  MIN_SPEED = 1.0                 # m/s

  # TI_FEEDBACK is 50 Hz. Two frames may go missing, with a cycle of jitter to spare; past
  # that the TI no longer vouches for itself and its command returns to zero. Inside the
  # panda's 100 ms (mazda.h's MAZDA_TI_FEEDBACK_TIMEOUT_US), so a controller that has stopped
  # hearing it stops before it is refused.
  FEEDBACK_TIMEOUT = 0.07         # s
  # Unfit this long while the car moves is a steer fault. Anything shorter costs its command
  # for those frames and a ramp back from zero, not the driver's lateral. The longest feedback
  # pause on record is 242 ms (pdscomp's logs).
  FAULT_T = 0.3                   # s
  # Unfit frames add up until it has been fit this long: every one restarts its command from
  # zero, so unfitness in bursts steers no better than unfitness that stays.
  REFIT_T = 0.5                   # s
  # Gone silent, it may still be biasing the sensor until its own watchdogs put it in bypass
  # (its documentation: 0.6 s for a lost CAN link, 1.09 s for a hung CPU). Until then the EPS's
  # reading is not the driver's and the TI's last one stands.
  BYPASS_T = 1.2                  # s
  # Unfit, or silent under a running command stream, this long: the driver is told by name
  # (CarState.ti_not_ready).
  NOT_READY_ALERT_T = 2.0         # s
  # From the first command frame it answers within one and is in RUN some 5 s later. Nothing
  # is said about it before this.
  BOOT_T = 8.0                    # s
  # DRIVER_OVER is the TI stepping aside for a driver steering through it, not a fault. It may
  # take this long to return to RUN once the wheel is let go.
  DRIVER_OVER_RELEASE_T = 1.0     # s

  # The speed-binned learner's cache version for a car steered through the interceptor.
  SPEED_DEP_SEED_VERSION = 64


class TorqueInterceptorState(IntEnum):
  # TI_FEEDBACK.STATE. Only RUN applies the bias; the others are bypass (sensor straight to EPS).
  DISCOVER = 0      # booted, waiting for the keyed command stream
  OFF = 1           # start-up or recovery delay, or a latched violation
  DRIVER_OVER = 2   # the driver is steering harder than the TI can pass: it steps aside
  RUN = 3


@dataclass
class MazdaCarDocs(CarDocs):
  package: str = "All"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.mazda]))
  footnotes: list[Enum] = field(default_factory=lambda: [Footnote.COMMA_POWER_REQUIRED])
  requires_comma_power: bool = True


class Footnote(Enum):
  COMMA_POWER_REQUIRED = CarFootnote(
    "Mazda vehicles <b>require</b> comma power installation.",
    Column.MAKE, setup_note=True)


@dataclass(frozen=True, kw_only=True)
class MazdaCarSpecs(CarSpecs):
  tireStiffnessFactor: float = 0.7  # not optimized yet


@dataclass(frozen=True, kw_only=True)
class MazdaCX5_2022CarSpecs(CarSpecs):
  tireStiffnessFactor: float = 1.0


class MazdaFlags(IntFlag):
  # GEN1 platforms share CAN messages and camera hardware.
  GEN1 = 1

  # EPS firmware that steers to zero: no speed floor, the 1200-count low-speed scale, and
  # LKAS_TRACK_STATE standby semantics.
  STEER_TO_ZERO_EPS = 2
  # Every other gen1 Mazda EPS: the same hardware on firmware that keeps the 45 kph floor. Same
  # envelope and tune; the floor, the non-delivery latch and alpha long stay with steer-to-zero.
  LEGACY_FW_EPS = 4
  # Everything keyed on the measured hardware rather than on what the firmware permits.
  EPS_HW = STEER_TO_ZERO_EPS | LEGACY_FW_EPS

  # The G46L radar's dialect bit; see G46L_RADAR_FW below.
  G46L_RADAR = 8

  # Blind spot frame 0x477 present. 16 was the moving radar takeover until 2026-10: do not reuse
  # it, older logs carry it.
  HAS_BSM = 32

  # A Torque Interceptor is fitted and switched on (the TorqueInterceptorEnabled param, read in
  # opendbc/sunnypilot/car/interfaces.py): it steers through the torque sensor at any speed.
  TORQUE_INTERCEPTOR = 64


class MazdaSafetyFlags(IntFlag):
  LONG = 1
  # Selects the measured EPS envelope in panda safety; the interface sets it on every gen1 EPS.
  # 4 was the legacy-firmware copy of this bit until 2026-10: do not reuse it, an older car side
  # may still send it.
  EPS_HW = 2
  # Admits the Torque Interceptor's command and feedback on bus 1 (mazda.h's MAZDA_PARAM_TI),
  # and has the panda put that bus on the OBD-II pins, where the interceptor sits.
  TORQUE_INTERCEPTOR = 8


class WMI(StrEnum):
  JAPAN_PASSENGER = "JM1"   # Japan-built passenger cars
  JAPAN_CROSSOVER = "JM3"   # Japan-built crossovers, North America
  EXPORT_CROSSOVER = "JM7"  # Japan-built crossovers, export markets; same chassis and year fields
  MEXICO_PASSENGER = "3MZ"  # Mazda de Mexico (Mazda 3)
  # Export VINs without a model-year field use the EPS-swap fallback.
  OCEANIA_EXPORT = "JM0"


@dataclass
class MazdaPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {Bus.pt: 'mazda_2017', Bus.radar: 'mazda_2017'})
  flags: int = MazdaFlags.GEN1
  wmis: set[WMI] = field(default_factory=set)
  chassis_codes: set[str] = field(default_factory=set)
  years: set[str] = field(default_factory=set)


class CAR(Platforms):
  MAZDA_CX5_KE = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda CX-5 2012-16")],
    MazdaCarSpecs(wheelbase=2.7, steerRatio=18.1),  # steer ratio from the 2022 CX-5: same rack hardware
    # This radar does not publish 0x361-0x366 tracks on bus 0.
    dbc_dict={Bus.pt: 'mazda_2017'},
    wmis={WMI.JAPAN_CROSSOVER, WMI.EXPORT_CROSSOVER}, chassis_codes={'KE'}, years={'C', 'D', 'E', 'F', 'G'},  # 2012-16
  )
  MAZDA_CX5 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda CX-5 2017-21")],
    MazdaCarSpecs(wheelbase=2.7, steerRatio=18.1),  # steer ratio from the 2022 CX-5: same rack hardware
    wmis={WMI.JAPAN_CROSSOVER, WMI.EXPORT_CROSSOVER}, chassis_codes={'KF'}, years={'H', 'J', 'K', 'L', 'M'},  # 2017-21
  )
  MAZDA_CX9 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda CX-9 2016-20")],
    MazdaCarSpecs(wheelbase=2.93, steerRatio=17.6),
    # This radar does not publish 0x361-0x366 tracks on bus 0.
    dbc_dict={Bus.pt: 'mazda_2017'},
    wmis={WMI.JAPAN_CROSSOVER, WMI.EXPORT_CROSSOVER}, chassis_codes={'TC'}, years={'G', 'H', 'J', 'K', 'L'},  # 2016-20
  )
  MAZDA_3 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda 3 2017-18")],
    MazdaCarSpecs(wheelbase=2.7, steerRatio=14.0),
    wmis={WMI.JAPAN_PASSENGER, WMI.MEXICO_PASSENGER}, chassis_codes={'BN'}, years={'H', 'J'},  # 2017-18
  )
  MAZDA_6 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda 6 2017-20")],
    MazdaCarSpecs(wheelbase=2.83, steerRatio=15.5),
    wmis={WMI.JAPAN_PASSENGER}, chassis_codes={'GL'}, years={'H', 'J', 'K', 'L', 'M'},  # 2017-21
  )
  MAZDA_CX9_2021 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda CX-9 2021-23", video="https://youtu.be/dA3duO4a0O4")],
    MazdaCarSpecs(wheelbase=2.93, steerRatio=17.6),
    # 2021-23 in North America; export markets kept the TC through 2025 (a JM7 TC S VIN attested)
    wmis={WMI.JAPAN_CROSSOVER, WMI.EXPORT_CROSSOVER}, chassis_codes={'TC'}, years={'M', 'N', 'P', 'S'},
  )
  MAZDA_CX5_2022 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda CX-5 2022-25")],
    MazdaCX5_2022CarSpecs(wheelbase=2.698, steerRatio=18.1),  # learned; factory spec 15.5
    wmis={WMI.JAPAN_CROSSOVER, WMI.EXPORT_CROSSOVER}, chassis_codes={'KF'}, years={'N', 'P', 'R', 'S', 'T'},  # 2022-26; NA 2026 is KM
  )
  MAZDA_CX8_2023 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda CX-8 2023")],
    # Three-row CX-5 derivative on the CX-9 wheelbase (chassis KG): the CX-9 specs stand in. It has no
    # VIN year field to decode, so it fingerprints by firmware alone (docs/zoompilot/mazda-fingerprinting.md).
    MAZDA_CX9_2021.specs,
  )


class LKAS_LIMITS:
  STEER_THRESHOLD = 15
  DISABLE_SPEED = 45    # kph
  ENABLE_SPEED = 52     # kph


# (latAccelFactor, friction) on params.toml's scale for a platform whose params.toml entry is
# borrowed: the CX-5 2022's own learned values, not the CX-9 2021's (docs/zoompilot/lateral-tune.md).
TORQUE_TUNES = {
  CAR.MAZDA_CX5_2022: (1.222, 0.154),
}


# Keep steer-to-zero firmware synchronized with the STEER_TO_ZERO_PLATFORMS EPS entries in fingerprints.py.
STEER_TO_ZERO_EPS_FW = {
  b'K0A1-3210X-A-00\x00\x00\x00\x00\x00\x00\x00\x00\x00',  # CX-8 2023 (Japan)
  b'KBST-3210X-A-00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
  b'KSD5-3210X-C-00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
}

# Platforms that ship the steer-to-zero EPS from the factory: what an unread EPS falls back to.
STEER_TO_ZERO_PLATFORMS = frozenset({CAR.MAZDA_CX5_2022, CAR.MAZDA_CX8_2023})

# The 2016.5-era radar an EPS-swapped older body can keep: no tracks on bus 0, one static frame
# (docs/zoompilot/mazda-longitudinal.md). Stored unpadded and matched with nulls stripped.
G46L_RADAR_FW = {
  b'G46L-67XA1-C',
}


class Buttons:
  NONE = 0
  SET_PLUS = 1
  SET_MINUS = 2
  RESUME = 3
  CANCEL = 4


def platform_from_vin(vin: str) -> str | None:
  """The one platform the VIN's fields identify, or None when the VIN is unknown to
  every platform or ambiguous.

  Shared by the fuzzy firmware fallback and the selected-car/VIN mismatch warning.
  """
  if not is_valid_vin(vin):
    return None

  vin_obj = Vin(vin)
  chassis_code = vin_obj.vds[0:2]
  year = vin_obj.vis[0]

  candidates = {platform for platform in CAR
                if vin_obj.wmi in platform.config.wmis and chassis_code in platform.config.chassis_codes
                and year in platform.config.years}
  return str(next(iter(candidates))) if len(candidates) == 1 else None


def match_fw_to_car_fuzzy(live_fw_versions, vin, offline_fw_versions) -> set[str]:
  # After firmware matching fails, require VIN fields to identify one chassis platform.
  platform = platform_from_vin(vin)
  if platform is not None:
    carlog.error(f"Fingerprinted {platform} by VIN")
    return {platform}

  # Only export VINs without model-year data continue to the EPS-swap fallback.
  if not is_valid_vin(vin) or Vin(vin).wmi != WMI.OCEANIA_EXPORT:
    return set()

  # Export-car swaps require a recognized EPS and an engine that identifies one platform.
  eps_fw = live_fw_versions.get((0x730, None), set())
  if not eps_fw & STEER_TO_ZERO_EPS_FW:
    return set()

  engine_fw = live_fw_versions.get((0x7e0, None), set())
  candidates = {platform for platform, ecus in offline_fw_versions.items()
                if engine_fw & set(ecus.get((Ecu.engine, 0x7e0, None), []))}
  if len(candidates) != 1:
    return set()

  carlog.error(f"Fingerprinted {next(iter(candidates))} by engine firmware behind a steer-to-zero EPS swap")
  return {str(c) for c in candidates}

FW_QUERY_CONFIG = FwQueryConfig(
  fw_version_regex=br"[A-Z0-9-]{11,16}\x00{8,13}",
  requests=[
    # TODO: check data to ensure ABS does not skip ISO-TP frames on bus 0
    Request(
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_RESPONSE],
      bus=0,
    ),
  ],
  match_fw_to_car_fuzzy=match_fw_to_car_fuzzy,
)

DBC = CAR.create_dbc_map()
