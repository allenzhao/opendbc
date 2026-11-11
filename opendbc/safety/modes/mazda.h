#pragma once

#include "opendbc/safety/declarations.h"

// CAN msgs we care about
#define MAZDA_LKAS          0x243U
#define MAZDA_LKAS_HUD      0x440U
#define MAZDA_CRZ_INFO      0x21bU
#define MAZDA_CRZ_CTRL      0x21cU
#define MAZDA_CRZ_BTNS      0x09dU
// Physical TJA button, DBC start bit 11 (byte 1, bit 3); trims without the button hold it low.
#define MAZDA_TJA_BUTTON_BIT 11U
// MRCC ON intent (MODE_Y), DBC start bit 13; the KE's OFF button sets only MODE_X.
#define MAZDA_MODE_Y_BIT 13U
// sunnypilot safety param: the TJA button is the MADS lateral switch
#define MAZDA_PARAM_SP_TJA_BUTTON 1U
#define MAZDA_RADAR_STATIC  0x499U
#define MAZDA_RADAR_TRACK_1 0x361U
#define MAZDA_RADAR_TRACK_2 0x362U
#define MAZDA_RADAR_TRACK_3 0x363U
#define MAZDA_RADAR_TRACK_4 0x364U
#define MAZDA_RADAR_TRACK_5 0x365U
#define MAZDA_RADAR_TRACK_6 0x366U
#define MAZDA_RADAR_UDS     0x764U
#define MAZDA_STEER_TORQUE  0x240U
#define MAZDA_ENGINE_DATA   0x202U
#define MAZDA_PEDALS        0x165U
// Torque Interceptor: its command and, one address up, its feedback. Both travel the OBD-II CAN
// pair that Comma Power carries, which is bus 1 once the panda multiplexes CAN2 onto it.
#define MAZDA_TI_LKAS       0x249U
#define MAZDA_TI_FEEDBACK   0x24aU

// CAN bus numbers
#define MAZDA_MAIN 0
#define MAZDA_AUX  1
#define MAZDA_CAM  2

#define MAZDA_PARAM_LONGITUDINAL 1U
// The measured gen1 EPS envelope; the interface sets it on every Mazda it admits.
#define MAZDA_PARAM_EPS_HW 2U
// A Torque Interceptor is fitted (values' MazdaSafetyFlags.TORQUE_INTERCEPTOR). The panda's
// board code reads the same bit to put CAN2 on the OBD-II pins.
#define MAZDA_PARAM_TI 8U
// TI_FEEDBACK is 50 Hz. Older than this, the interceptor no longer vouches for itself and the
// driver torque it measures is too stale to limit against. The controller stops at 70 ms
// (values' TorqueInterceptorParams.FEEDBACK_TIMEOUT), inside this.
#define MAZDA_TI_FEEDBACK_TIMEOUT_US 100000U
// TI_FEEDBACK.STATE: the interceptor applies its bias in this state only.
#define MAZDA_TI_STATE_RUN 3U

// Keep SET/RES intent fresh until PEDALS reports engagement.
#define MAZDA_ENGAGE_BTN_WINDOW 10U
// Both PEDALS cruise bits low for this many samples is a main-off: values'
// MAIN_OFF_DEBOUNCE_SAMPLES, which carstate counts the same way.
#define MAZDA_MAIN_OFF_DEBOUNCE 10U

static bool mazda_longitudinal = false;
// Declared by the driver: the TJA button owns lateral and MRCC no longer drives the main edge.
static bool mazda_tja_button = false;
static bool mazda_eps_hw = false;
static bool mazda_ti = false;
// The last TI_FEEDBACK frame was well formed and reported RUN with no violation, error or ramp-down.
static bool mazda_ti_healthy = false;
static uint32_t mazda_ti_feedback_ts = 0U;
// The interceptor command's own history: steer_torque_cmd_checks' globals belong to 0x243.
static int mazda_ti_desired_torque_last = 0;
static int mazda_ti_rt_torque_last = 0;
static uint32_t mazda_ti_ts_torque_check_last = 0U;
// Live cruise arming from PEDALS, both longitudinal modes (carstate mrcc_armed_raw).
static bool mazda_acc_armed = false;
static uint32_t mazda_engage_btn_frames = 0U;
static uint32_t mazda_main_off_samples = 0U;

static void mazda_ti_reset_torque_state(void) {
  mazda_ti_desired_torque_last = 0;
  mazda_ti_rt_torque_last = 0;
  mazda_ti_ts_torque_check_last = microsecond_timer_get();
}

// A healthy frame recent enough to act on. Staleness latches until the next healthy frame, so a
// timer wrap cannot make an old one look fresh again: called on every frame the mode hears and
// on every interceptor command, steering or not.
static bool mazda_ti_feedback_fresh(void) {
  if (safety_get_ts_elapsed(microsecond_timer_get(), mazda_ti_feedback_ts) > MAZDA_TI_FEEDBACK_TIMEOUT_US) {
    mazda_ti_healthy = false;
  }
  return mazda_ti_healthy;
}

// The RX check's quality flag for TI_FEEDBACK, the one message that has one. It answers whether
// the frame is well formed; whether the interceptor is fit to take torque is recorded beside it,
// because the RX hook never sees a frame this turns away.
static bool mazda_get_quality_flag_valid(const CANPacket_t *msg) {
  bool valid = true;
  if (msg->addr == MAZDA_TI_FEEDBACK) {
    // The torque byte is sent twice in place of a checksum, every firmware reports a nonzero
    // version, and there are four states.
    valid = (msg->data[0] == msg->data[1]) && (msg->data[2] != 0U) && (msg->data[3] <= MAZDA_TI_STATE_RUN);
    if (!valid) {
      // The frame carries the driver torque both commands are limited against, and it is not
      // sampled. An invalid RX check takes controls_allowed alone; MADS lateral goes with it
      // here, or 0x243 would be admitted against the last readings for as long as the host asks.
      mads_exit_controls(MADS_DISENGAGE_REASON_LAG);
    }
    // RAMP_DOWN: the interceptor is taking its bias to zero by itself and ignores the request
    // until that reads zero. Firmware version 1 has no such byte.
    bool ramp_down = (msg->data[2] > 1U) && (msg->data[6] != 0U);
    // OFF and DRIVER_OVER are ordinary (boot, a latched violation, the driver steering through
    // it): no reason to drop controls, every reason to send no torque.
    mazda_ti_healthy = valid && (msg->data[3] == MAZDA_TI_STATE_RUN) &&
                       (msg->data[4] == 0U) && (msg->data[5] == 0U) && !ramp_down;
    if (mazda_ti_healthy) {
      mazda_ti_feedback_ts = microsecond_timer_get();
    }
  }
  return valid;
}

static bool mazda_mrcc_off_msg_valid(const CANPacket_t *msg) {
  // Exact active-low MRCC master tap. CTR occupies the variable bits in byte 3;
  // all other buttons and payload bits remain pinned.
  return (GET_LEN(msg) == 8U) && (msg->data[0] == 0x00U) &&
         (msg->data[1] == 0x81U) && (msg->data[2] == 0xfeU) &&
         ((msg->data[3] & 0xc3U) == 0xc0U) && (msg->data[4] == 0x00U) &&
         (msg->data[5] == 0x00U) && (msg->data[6] == 0x00U) && (msg->data[7] == 0x00U);
}

// Replaced-radar frames must be one of the stock captures (mazdacan.py): every byte pinned, and on
// the tracks only the counter nibble in byte 7 free. The 0x499 static frame has one capture per
// radar generation; the controller picks the dialect.
typedef struct {
  unsigned int addr;
  uint8_t data[8];
  uint8_t last_byte_mask;
} MazdaRadarCapture;

static bool mazda_radar_capture_msg_valid(const CANPacket_t *msg) {
  static const MazdaRadarCapture MAZDA_RADAR_CAPTURES[] = {
    {MAZDA_RADAR_STATIC,  {0x00U, 0x08U, 0xc0U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U}, 0xffU},  // 2022 family
    {MAZDA_RADAR_STATIC,  {0x00U, 0x98U, 0x40U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U}, 0xffU},  // G46L
    {MAZDA_RADAR_TRACK_1, {0xffU, 0xf7U, 0xfeU, 0xfeU, 0x1fU, 0xc0U, 0x00U, 0x80U}, 0xf0U},
    {MAZDA_RADAR_TRACK_2, {0xffU, 0xf7U, 0xfeU, 0xfeU, 0x1fU, 0xc7U, 0x8cU, 0x80U}, 0xf0U},
    {MAZDA_RADAR_TRACK_3, {0xffU, 0xf7U, 0xfeU, 0xfeU, 0x1fU, 0xc0U, 0x00U, 0x00U}, 0xf0U},
    {MAZDA_RADAR_TRACK_4, {0xffU, 0xf7U, 0xfeU, 0xfeU, 0x1fU, 0xc0U, 0x00U, 0x00U}, 0xf0U},
    {MAZDA_RADAR_TRACK_5, {0xffU, 0xf7U, 0xfeU, 0x7fU, 0xfbU, 0xffU, 0x3fU, 0xc0U}, 0xf0U},
    {MAZDA_RADAR_TRACK_6, {0xffU, 0xf7U, 0xfeU, 0x7fU, 0xfbU, 0xffU, 0x3fU, 0xc0U}, 0xf0U},
  };

  bool valid = false;

  for (unsigned int i = 0U; i < (sizeof(MAZDA_RADAR_CAPTURES) / sizeof(MazdaRadarCapture)); i++) {
    const MazdaRadarCapture *capture = &MAZDA_RADAR_CAPTURES[i];
    if (capture->addr == msg->addr) {
      bool match = (msg->data[7] & capture->last_byte_mask) == capture->data[7];
      for (int j = 0; j < 7; j++) {
        match = match && (msg->data[j] == capture->data[j]);
      }
      valid = valid || match;
    }
  }

  return valid;
}

// track msgs coming from OP so that we know what CAM msgs to drop and what to forward
static void mazda_rx_hook(const CANPacket_t *msg) {
  if (mazda_ti) {
    // keep the staleness latch current while nothing is being sent to the interceptor
    (void)mazda_ti_feedback_fresh();
  }

  if ((int)msg->bus == MAZDA_MAIN) {
    if (msg->addr == MAZDA_ENGINE_DATA) {
      // sample speed: scale by 0.01 to get kph
      int speed = (msg->data[2] << 8) | msg->data[3];
      vehicle_moving = speed > 10; // moving when speed > 0.1 kph
    }

    // With a Torque Interceptor the EPS reports the biased signal here; the driver's torque is
    // the interceptor's own reading of the sensor, sampled below.
    if ((msg->addr == MAZDA_STEER_TORQUE) && !mazda_ti) {
      int torque_driver_new = msg->data[0] - 127U;
      // update array of samples
      update_sample(&torque_driver, torque_driver_new);
    }

    // enter controls on rising edge of ACC, exit controls on ACC off
    if ((msg->addr == MAZDA_CRZ_CTRL) && !mazda_longitudinal) {
      bool cruise_engaged = msg->data[0] & 0x8U;
      pcm_cruise_check(cruise_engaged);
      // With the TJA button owning lateral, MRCC no longer drives the MADS main edge: its
      // falling edge would exit the panda's lateral while the software's MADS stays on.
      if (!mazda_tja_button) {
        acc_main_on = GET_BIT(msg, 17U);
      }
    }

    if (msg->addr == MAZDA_CRZ_BTNS) {
      // TJA-declared cars keep their own lateral switch. Otherwise ON rearms lateral in standby,
      // since a main that stayed on through boot can expire before selfdrived is ready.
      bool lateral_button = mazda_tja_button ? GET_BIT(msg, MAZDA_TJA_BUTTON_BIT) : (acc_main_on && GET_BIT(msg, MAZDA_MODE_Y_BIT));
      mads_button_press = lateral_button ? MADS_BUTTON_PRESSED : MADS_BUTTON_NOT_PRESSED;
    }

    if ((msg->addr == MAZDA_CRZ_BTNS) && mazda_longitudinal) {
      // A physical cancel press always exits controls.
      if (GET_BIT(msg, 0U)) {
        controls_allowed = false;
      }
      // Record SET/RES intent for the engagement qualifier below.
      if (GET_BIT(msg, 2U) || GET_BIT(msg, 4U) || GET_BIT(msg, 5U)) {
        mazda_engage_btn_frames = MAZDA_ENGAGE_BTN_WINDOW;
      } else if (mazda_engage_btn_frames > 0U) {
        mazda_engage_btn_frames -= 1U;
      } else {
      }
    }

    if (msg->addr == MAZDA_ENGINE_DATA) {
      gas_pressed = (msg->data[4] || (msg->data[5] & 0xF0U));
    }

    if (msg->addr == MAZDA_PEDALS) {
      bool brake = (msg->data[0] & 0x10U);
      // The live MRCC arm from the body's own PEDALS bits, the sample mrcc_armed_raw reads, so the
      // MRCC-off exception opens on the frame the controller first sends (CRZ_CTRL lags it).
      mazda_acc_armed = GET_BIT(msg, 2U) || GET_BIT(msg, 3U);
      if (mazda_longitudinal) {
        // Derive cruise state from PEDALS after radar teardown.
        bool cruise_engaged = GET_BIT(msg, 3U);
        bool brake_free = !brake && !brake_pressed_prev;

        // Main mirrors carstate's cruise_available sample for sample: a main that falls on one side
        // only steers MADS into rejected frames (docs/zoompilot/mazda-longitudinal.md).
        if (mazda_tja_button) {
          // the button is the lateral switch; MRCC is cruise only
        } else if (mazda_acc_armed) {
          // The radar takeover gates cruise (controls_allowed below), never main.
          acc_main_on = true;
          mazda_main_off_samples = 0U;
        } else {
          if (mazda_main_off_samples < MAZDA_MAIN_OFF_DEBOUNCE) {
            mazda_main_off_samples += 1U;
          }
          if (mazda_main_off_samples >= MAZDA_MAIN_OFF_DEBOUNCE) {
            acc_main_on = false;
          }
        }

        if (mazda_acc_armed || cruise_engaged_prev || brake_free) {
          // Require recent SET/RES intent on the engaged edge; ACC_ACTIVE alone may acknowledge
          // synthetic traffic rather than a driver request.
          if (cruise_engaged && !cruise_engaged_prev && (mazda_engage_btn_frames > 0U)) {
            controls_allowed = true;
          }
          if (!cruise_engaged) {
            controls_allowed = false;
          }
          cruise_engaged_prev = cruise_engaged;
        }
      }
      brake_pressed = brake;
    }
  }

  if (mazda_ti && ((int)msg->bus == MAZDA_AUX) && (msg->addr == MAZDA_TI_FEEDBACK)) {
    int torque_driver_new = msg->data[0] - 127U;
    update_sample(&torque_driver, torque_driver_new);
  }
}

static bool mazda_is_lka_addr(int addr) {
  return (((unsigned int)addr == MAZDA_LKAS) || ((unsigned int)addr == MAZDA_LKAS_HUD));
}

// The camera owns the LKAS addresses whenever openpilot is not steering. Lateral is its own
// axis under MADS (controls_allowed_lateral); with MADS off it follows cruise. Cruise alone must
// not claim them, or it silences the camera's own TJA/CTS while the dash shows nothing.
static bool mazda_openpilot_controlling(void) {
  return controls_allowed_lateral || (controls_allowed && !m_mads_state.system_enabled);
}

// steer_torque_cmd_checks for the interceptor's command, on its own history. Torque needs
// openpilot steering, as 0x243 does, and an interceptor that reports itself fit; anything else
// admits the zero request only, which keeps the stream the interceptor requires alive.
static bool mazda_ti_torque_cmd_checks(int desired_torque) {
  // values' TorqueInterceptorParams. The interceptor clips at 600 itself.
  const TorqueSteeringLimits MAZDA_TI_STEERING_LIMITS = {
    .max_torque = 600,
    .max_rate_up = 6,
    .max_rate_down = 15,
    .max_rt_delta = 192,
    .driver_torque_multiplier = 40,
    .driver_torque_allowance = 15,
    .type = TorqueDriverLimited,
  };

  bool violation = false;
  // evaluated on every command, steering or not: it latches staleness
  bool fresh = mazda_ti_feedback_fresh();
  bool allowed = mazda_openpilot_controlling() && fresh;
  uint32_t ts = microsecond_timer_get();

  if (allowed) {
    violation |= safety_max_limit_check(desired_torque, MAZDA_TI_STEERING_LIMITS.max_torque, -MAZDA_TI_STEERING_LIMITS.max_torque);
    violation |= driver_limit_check(desired_torque, mazda_ti_desired_torque_last, &torque_driver,
                                    MAZDA_TI_STEERING_LIMITS.max_torque, MAZDA_TI_STEERING_LIMITS.max_rate_up,
                                    MAZDA_TI_STEERING_LIMITS.max_rate_down, MAZDA_TI_STEERING_LIMITS.driver_torque_allowance,
                                    MAZDA_TI_STEERING_LIMITS.driver_torque_multiplier);
    mazda_ti_desired_torque_last = desired_torque;

    violation |= rt_torque_rate_limit_check(desired_torque, mazda_ti_rt_torque_last, MAZDA_TI_STEERING_LIMITS.max_rt_delta);
    if (safety_get_ts_elapsed(ts, mazda_ti_ts_torque_check_last) > MAX_RT_INTERVAL) {
      mazda_ti_rt_torque_last = desired_torque;
      mazda_ti_ts_torque_check_last = ts;
    }
  }

  if (!allowed && (desired_torque != 0)) {
    violation = true;
  }

  // a refusal, or nothing to steer with, restarts the ramp from zero
  if (violation || !allowed) {
    mazda_ti_reset_torque_state();
  }

  return violation;
}

static bool mazda_tx_hook(const CANPacket_t *msg) {
  // Upstream's envelope, the no-param default.
  const TorqueSteeringLimits MAZDA_STEERING_LIMITS = {
    .max_torque = 800,
    .max_rate_up = 10,
    .max_rate_down = 25,
    .max_rt_delta = 300,
    .driver_torque_multiplier = 1,
    .driver_torque_allowance = 15,
    .type = TorqueDriverLimited,
  };

  // The measured EPS envelope: the EPS's ~1000 counts/s hardware slew (12 counts per 12 ms 0x241
  // frame) at 100 Hz, with max_rate_down equal to the controller retreat rate so driver-limit
  // winddown frames remain valid.
  const TorqueSteeringLimits MAZDA_EPS_HW_STEERING_LIMITS = {
    .max_torque = 1200,
    .max_rate_up = 10,
    .max_rate_down = 10,
    .max_rt_delta = 320,
    .driver_torque_multiplier = 15,
    .driver_torque_allowance = 15,
    .type = TorqueDriverLimited,
  };

  // CRZ_INFO.ACCEL_CMD uses 1/1024 m/s2 raw units after removing the offset: 2.0 and -3.5 m/s2.
  const LongitudinalLimits MAZDA_LONG_LIMITS = {
    .max_accel = 2048,
    .min_accel = -3584,
    .inactive_accel = 0,
  };

  bool tx = true;
  bool main_bus = msg->bus == (unsigned char)MAZDA_MAIN;
  bool long_replacement_bus = main_bus || (msg->bus == (unsigned char)MAZDA_CAM);

  if (main_bus && (msg->addr == MAZDA_LKAS)) {
    int desired_torque = (((msg->data[0] & 0x0FU) << 8) | msg->data[1]) - 2048U;

    const TorqueSteeringLimits *limits = &MAZDA_STEERING_LIMITS;
    if (mazda_eps_hw) {
      limits = &MAZDA_EPS_HW_STEERING_LIMITS;
    }
    if (steer_torque_cmd_checks(desired_torque, -1, *limits)) {
      tx = false;
    }
  }

  // Run after steering checks, which reset rate-limit state while disengaged.
  if (main_bus && mazda_is_lka_addr(msg->addr) && !mazda_openpilot_controlling()) {
    tx = false;
  }

  if ((msg->bus == (unsigned char)MAZDA_AUX) && (msg->addr == MAZDA_TI_LKAS)) {
    // The 12-bit request and its copy, each under a clear high nibble, then the interceptor's
    // 3-byte key and the option byte. Only 0x60, normal run, is sent: 0x61 would cold-reset the
    // interceptor and clear its latched violations, 0x62 would force it into bypass.
    uint32_t request = (((uint32_t)msg->data[0] & 0x0FU) << 8) | (uint32_t)msg->data[1];
    uint32_t request_copy = (((uint32_t)msg->data[2] & 0x0FU) << 8) | (uint32_t)msg->data[3];
    bool well_formed = ((msg->data[0] & 0xF0U) == 0U) && ((msg->data[2] & 0xF0U) == 0U) && (request == request_copy) &&
                       (msg->data[4] == 0xc4U) && (msg->data[5] == 0x61U) && (msg->data[6] == 0xceU) && (msg->data[7] == 0x60U);
    int desired_torque = (int)request - 2048;

    // The torque checks run on every frame: they also keep the command history.
    if (mazda_ti_torque_cmd_checks(desired_torque)) {
      tx = false;
    }
    if (!mazda_ti || !well_formed) {
      mazda_ti_reset_torque_state();
      tx = false;
    }
  }

  if (mazda_longitudinal && long_replacement_bus && (msg->addr == MAZDA_CRZ_INFO)) {
    // Allow byte-exact stock standby patterns with the raw 8190 command sentinel.
    bool stock_standby = (msg->data[0] == 0x01U) && (msg->data[1] == 0xffU) &&
                         (msg->data[2] == 0xe3U) && (msg->data[3] == 0xffU) &&
                         ((msg->data[4] & 0xfbU) == 0xc0U) &&
                         ((msg->data[5] & 0x7fU) == 0x00U) &&
                         ((msg->data[6] & 0xf0U) == 0x00U) &&
                         (msg->data[7] == ((0xffU - ((msg->data[0] + msg->data[1] + msg->data[2] + msg->data[3] +
                                                     msg->data[4] + msg->data[5] + msg->data[6]) & 0xffU)) & 0xffU));

    // Assemble the 13-bit ACCEL_CMD unsigned for MISRA 10.1, then remove its 4096 offset.
    uint32_t accel_raw = (((uint32_t)msg->data[2] & 0x3U) << 11) | ((uint32_t)msg->data[3] << 3) | ((uint32_t)msg->data[4] >> 5);
    int desired_accel = (int)accel_raw - 4096;
    if (!stock_standby && longitudinal_accel_checks(desired_accel, MAZDA_LONG_LIMITS)) {
      tx = false;
    }

    // ACC_ACTIVE requires prior controls permission established from the physical SET edge.
    bool acc_active = GET_BIT(msg, 33U);
    if (!controls_allowed && acc_active) {
      tx = false;
    }
  }

  if (mazda_longitudinal && long_replacement_bus && (msg->addr == MAZDA_CRZ_CTRL)) {
    bool cruise_active = GET_BIT(msg, 3U);
    if (!controls_allowed && cruise_active) {
      tx = false;
    }
  }

  bool radar_frame = (msg->addr == MAZDA_RADAR_STATIC) || ((msg->addr >= MAZDA_RADAR_TRACK_1) && (msg->addr <= MAZDA_RADAR_TRACK_6));
  if (mazda_longitudinal && long_replacement_bus && radar_frame) {
    if (!mazda_radar_capture_msg_valid(msg)) {
      tx = false;
    }
  }

  if (mazda_longitudinal && main_bus && (msg->addr == MAZDA_RADAR_UDS)) {
    // Allow tester-present and default/programming session control only.
    bool tester_present = (msg->data[0] == 0x02U) && (msg->data[1] == 0x3eU) && (msg->data[2] == 0x80U);
    bool session_control = (msg->data[0] == 0x02U) && (msg->data[1] == 0x10U) &&
                           ((msg->data[2] == 0x01U) || (msg->data[2] == 0x02U));
    if (!tester_present && !session_control) {
      tx = false;
    }
  }

  if (main_bus && (msg->addr == MAZDA_CRZ_BTNS)) {
    // Permit resume only while controlling and cancel only while not controlling.
    bool cancel_cmd = (msg->data[0] == 0x1U);
    const bool mrcc_off_candidate = !GET_BIT(msg, 16U) && GET_BIT(msg, 15U);
    const bool mrcc_off_cmd = mazda_tja_button && mazda_acc_armed && mazda_mrcc_off_msg_valid(msg);
    if (!controls_allowed && !cancel_cmd && !mrcc_off_cmd) {
      tx = false;
    }
    // An MRCC-off-shaped frame is either the exact narrow exception or invalid; it cannot
    // borrow the normal cancel authorization as a composite command.
    if (mrcc_off_candidate && !mrcc_off_cmd) {
      tx = false;
    }
    // The TJA button is never pressed on the car's side: it would toggle MADS through the
    // rx hook and arm MRCC in the body.
    if (GET_BIT(msg, MAZDA_TJA_BUTTON_BIT)) {
      tx = false;
    }
  }

  return tx;
}

static bool mazda_fwd_hook(int bus_num, int addr) {
  bool block_msg = false;

  if (bus_num == MAZDA_CAM) {
    if (mazda_is_lka_addr(addr)) {
      block_msg = mazda_openpilot_controlling();
    }
  }

  // The interceptor's command belongs to its side channel: never relay it between the car and
  // the camera, in either direction.
  if (mazda_ti && ((unsigned int)addr == MAZDA_TI_LKAS)) {
    block_msg = true;
  }

  return block_msg;
}

static safety_config mazda_init(uint16_t param) {
  mazda_engage_btn_frames = 0U;
  mazda_main_off_samples = 0U;
  mazda_acc_armed = false;

  static const CanMsg MAZDA_TX_MSGS[] = {
    {MAZDA_LKAS, 0, 8, .check_relay = true, .disable_static_blocking = true},
    {MAZDA_CRZ_BTNS, 0, 8, .check_relay = false},
    {MAZDA_LKAS_HUD, 0, 8, .check_relay = true, .disable_static_blocking = true},
  };

  // The interceptor's lists are the two above and below with its command added, selected by
  // MAZDA_PARAM_TI alone: without the bit nothing here changes.
  static const CanMsg MAZDA_TI_TX_MSGS[] = {
    {MAZDA_LKAS, 0, 8, .check_relay = true, .disable_static_blocking = true},
    {MAZDA_CRZ_BTNS, 0, 8, .check_relay = false},
    {MAZDA_LKAS_HUD, 0, 8, .check_relay = true, .disable_static_blocking = true},
    {MAZDA_TI_LKAS, MAZDA_AUX, 8, .check_relay = false},
  };

// Replaced-radar addresses omit relay checks because the radar remains live during boot and
// hand-back. carstate enforces single ownership instead.
  static const CanMsg MAZDA_LONG_TX_MSGS[] = {
    {MAZDA_LKAS, 0, 8, .check_relay = true, .disable_static_blocking = true},
    {MAZDA_CRZ_BTNS, 0, 8, .check_relay = false},
    {MAZDA_LKAS_HUD, 0, 8, .check_relay = true, .disable_static_blocking = true},
    {MAZDA_CRZ_INFO, 0, 8, .check_relay = false},
    {MAZDA_CRZ_CTRL, 0, 8, .check_relay = false},
    {MAZDA_RADAR_STATIC, 0, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_1, 0, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_2, 0, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_3, 0, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_4, 0, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_5, 0, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_6, 0, 8, .check_relay = false},
    {MAZDA_RADAR_UDS, 0, 8, .check_relay = false},
    {MAZDA_CRZ_INFO, MAZDA_CAM, 8, .check_relay = false},
    {MAZDA_CRZ_CTRL, MAZDA_CAM, 8, .check_relay = false},
    {MAZDA_RADAR_STATIC, MAZDA_CAM, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_1, MAZDA_CAM, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_2, MAZDA_CAM, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_3, MAZDA_CAM, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_4, MAZDA_CAM, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_5, MAZDA_CAM, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_6, MAZDA_CAM, 8, .check_relay = false},
  };

  static const CanMsg MAZDA_LONG_TI_TX_MSGS[] = {
    {MAZDA_LKAS, 0, 8, .check_relay = true, .disable_static_blocking = true},
    {MAZDA_CRZ_BTNS, 0, 8, .check_relay = false},
    {MAZDA_LKAS_HUD, 0, 8, .check_relay = true, .disable_static_blocking = true},
    {MAZDA_CRZ_INFO, 0, 8, .check_relay = false},
    {MAZDA_CRZ_CTRL, 0, 8, .check_relay = false},
    {MAZDA_RADAR_STATIC, 0, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_1, 0, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_2, 0, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_3, 0, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_4, 0, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_5, 0, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_6, 0, 8, .check_relay = false},
    {MAZDA_RADAR_UDS, 0, 8, .check_relay = false},
    {MAZDA_CRZ_INFO, MAZDA_CAM, 8, .check_relay = false},
    {MAZDA_CRZ_CTRL, MAZDA_CAM, 8, .check_relay = false},
    {MAZDA_RADAR_STATIC, MAZDA_CAM, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_1, MAZDA_CAM, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_2, MAZDA_CAM, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_3, MAZDA_CAM, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_4, MAZDA_CAM, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_5, MAZDA_CAM, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_6, MAZDA_CAM, 8, .check_relay = false},
    {MAZDA_TI_LKAS, MAZDA_AUX, 8, .check_relay = false},
  };

  static RxCheck mazda_rx_checks[] = {
    {.msg = {{MAZDA_CRZ_CTRL,     0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_CRZ_BTNS,     0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_STEER_TORQUE, 0, 8, 83U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_ENGINE_DATA,  0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_PEDALS,       0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

  // CRZ_CTRL intentionally disappears after radar teardown.
  static RxCheck mazda_long_rx_checks[] = {
    {.msg = {{MAZDA_CRZ_BTNS,     0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_STEER_TORQUE, 0, 8, 83U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_ENGINE_DATA,  0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_PEDALS,       0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

  // TI_FEEDBACK joins the checks: the interceptor must be heard from, and each frame well formed
  // (mazda_get_quality_flag_valid), for cruise controls or MADS lateral to be allowed at all.
  static RxCheck mazda_ti_rx_checks[] = {
    {.msg = {{MAZDA_CRZ_CTRL,     0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_CRZ_BTNS,     0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_STEER_TORQUE, 0, 8, 83U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_ENGINE_DATA,  0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_PEDALS,       0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_TI_FEEDBACK,  MAZDA_AUX, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = false}, { 0 }, { 0 }}},
  };

  static RxCheck mazda_long_ti_rx_checks[] = {
    {.msg = {{MAZDA_CRZ_BTNS,     0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_STEER_TORQUE, 0, 8, 83U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_ENGINE_DATA,  0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_PEDALS,       0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_TI_FEEDBACK,  MAZDA_AUX, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = false}, { 0 }, { 0 }}},
  };

  mazda_longitudinal = GET_FLAG(param, MAZDA_PARAM_LONGITUDINAL);
  mazda_eps_hw = GET_FLAG(param, MAZDA_PARAM_EPS_HW);
  mazda_ti = GET_FLAG(param, MAZDA_PARAM_TI);
  mazda_ti_healthy = false;
  mazda_ti_feedback_ts = 0U;
  mazda_ti_reset_torque_state();
  mazda_tja_button = GET_FLAG(current_safety_param_sp, MAZDA_PARAM_SP_TJA_BUTTON);
  acc_main_on = false;

  return mazda_ti ? (mazda_longitudinal ? BUILD_SAFETY_CFG(mazda_long_ti_rx_checks, MAZDA_LONG_TI_TX_MSGS) :
                                          BUILD_SAFETY_CFG(mazda_ti_rx_checks, MAZDA_TI_TX_MSGS)) :
                    (mazda_longitudinal ? BUILD_SAFETY_CFG(mazda_long_rx_checks, MAZDA_LONG_TX_MSGS) :
                                          BUILD_SAFETY_CFG(mazda_rx_checks, MAZDA_TX_MSGS));
}

const safety_hooks mazda_hooks = {
  .init = mazda_init,
  .rx = mazda_rx_hook,
  .tx = mazda_tx_hook,
  .fwd = mazda_fwd_hook,
  .get_quality_flag_valid = mazda_get_quality_flag_valid,
};
