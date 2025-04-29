/*
  ModbusTCP Client for ClearCode Arduino wrapper

  (c)2021 Alexander Emelianov (a.m.emelianov@gmail.com)
  https://github.com/emelianov/modbus-esp8266

  This code is licensed under the BSD New License. See LICENSE.txt for more info.
*/

#include <Arduino.h>
#include "NvmManager.h"
#include "ClearCore.h"
#include "indicator_light.h"
#include "button.h"


bool is_homed = false;                  // Has the axis been homed
volatile bool is_e_stop = false;               // Is the Emergency Stop currently active
bool is_fingers_down = false;                  // Is the finger actuation currently active
bool is_head_commanded_down = true;            // Is the head currently commanded down

bool io_configured = false;                  // Has the IO been configured

int32_t current_jog_speed = 0;              // Current speed the motor should use for jogging. Steps/second
int32_t current_planish_speed = 0;          // Current speed the motor should use for planishing. Steps/second

uint32_t last_iteration_delta;          // Time spent on the last iteration of the main loop in millis
uint32_t last_iteration_time;           // Time of the last iteration of the main loop in millis

uint32_t loop_num = 0;                  // Number of times the main loop has been called. WILL OVERFLOW
uint32_t last_estop_millis = 0;         // Millis() value at time the last e-stop was pressed

// Memory job values. Not saved to NVRAM until learn sequence completed
uint32_t temp_job_start_pos = 0;           // Start position for learn mode
uint32_t temp_job_end_pos = 0;             // End position for learn mode
uint32_t temp_job_park_pos = 0;            // Park position for learn mode

// saved job values these are retrieved from NVRAM on boot or over-ridden after a learn sequence
uint32_t saved_job_start_pos = 0;           // Start position for learn mode
uint32_t saved_job_end_pos = 0;             // End position for learn mode
uint32_t saved_job_park_pos = 0;            // Park position for learn mode

uint8_t NV_Ram[12];                    // NVram Max Available is `416 bytes of user data

// Reasons to have an E-stop. Defines what conditions need to be met for the e-stop to end
// not currently used because ISR must be void
enum class EstopReason {
  button,
  mandrel_latch,
};

enum class PlanishStates {
  post,                       // Power On Self Test
  begin_homing,               // start axis homing
  wait_for_homing,            // Wait for homing to complete
  idle,                       // Idle await instructions
  manual_jog,                 // manually commanded jog
  e_stop_begin,               // E-stop was activated, secure system and save current config to be resumed
  e_stop_wait,                // E-stop is active, wait for it to end
  resume_from_e_stop,         // E-stop was released. find resume point and go back
  error,                      // unrecoverable error has occurred and machine needs to reboot. should be a dead end state
  wait_for_head,              // Me asf

  job_begin,                  // Job has been started, check initial configuration
  job_begin_lifting_head,     // Job was started with head down, wait for it to lift
  job_jog_to_start,           // Job started, move to saved start pos
  job_jog_to_start_wait,      // Wait for carriage to arrive at start
  job_head_down,              // Engage the planisher
  job_head_down_wait,         // Wait for the planisher to be engaged
  job_planish_to_end,         // Move carriage to the end position
  job_planish_to_end_wait,    // wait for carriage to arrive at end position
  job_planish_to_start,       // In full cycle mode, go back to the start with planisher engaged
  job_planish_to_start_wait,  // Wait for carriage to arrive at start
  job_head_up,                // Raise the head / disengage roller
  job_head_up_wait,           // Wait for head to be up
  job_jog_to_park,            // Move to park position
  job_jog_to_park_wait,       // Wait until carriage arrives at park position

  learn_start_pos,            // Learn has been pressed, set start point
  learn_jog_to_end_pos,       // Manually jogging to end position
  learn_end_pos,              // Learn was pressed again, set end position
  learn_jog_to_park_pos,      // Manually jogging to park position
  learn_park_pos,             // Learn was pressed again, set park position
  saving_job_to_nvram,        // Job is recorded and needs to be saved in NVRAM
};

volatile PlanishStates estop_last_state;  // When an estop is called, this gets set to whatever the current state was.
volatile PlanishStates estop_resume_state;      // When an estop is resolved, this stores where to resume
volatile PlanishStates machine_state;     // the current state of the machine. Volatile because it can be changed by estop ISR

volatile IndicatorLight home_indicator_light;   /// Defined globally, but should not be accessed unless configure_IO has completed without errors
volatile IndicatorLight learn_indicator_light;  /// Defined globally, but should not be accessed unless configure_IO has completed without errors

#define ESTOP_SW ConnectorA12
#define LASER_SW ConnectorA11
#define LEARN_SW ConnectorA10
#define SPEED_POT ConnectorA9
#define MANDREL_LATCH_LMT ConnectorDI8
#define HALF_CYCLE_SW ConnectorDI7
#define HEAD_UP_LMT ConnectorDI6
#define HOME_SW ConnectorIO0
#define CYCLE_SW ConnectorIO1
#define FINGER_SW ConnectorIO2
#define JOG_FWD_SW ConnectorIO3
#define JOG_REV_SW ConnectorIO4
#define HEAD_SW ConnectorIO5

#define CARRIAGE_MOTOR ConnectorM0

// CCIO pin definitions. these are not interchangeable with native io ports.
// please see `configure_io()` to change assignment between ccio and native pins
#define FINGER_ACTUATION CCIOA6 // CCIO pin for finger actuation
#define HEAD_ACTUATION CCIOA4
#define HOME_SW_LIGHT CCIOA0
#define LEARN_SW_LIGHT CCIOA3

#define CCIO1 ConnectorCOM1
#define EXPECTED_NUM_CCIO 1
#define CCIO_TIMEOUT_MS 10000 // Number of ms to wait for CCIO to connect before failing POST

#define CARRIAGE_MOTOR_MAX_ACCEL 50000
#define CARRIAGE_MOTOR_MAX_VEL 4000
#define CARRIAGE_MOTOR_MAX_POS 12000
#define CARRIAGE_MOTOR_MAX_PLANISH_VEL 2000

#define ITERATION_TIME_WARNING_MS 50 // after this many milliseconds stuck on one iteration of the state machine, give a warning.
#define ITERATION_TIME_ERROR_MS 100  // after this many milliseconds stuck on one iteration of the state machine, declare an error
#define SERIAL_ESTABLISH_TIMEOUT 10000 // Number of ms to wait for serial to establish before failing POST

/// Interrupt priority for the periodic interrupt. 0 is highest priority, 7 is lowest.
#define PERIODIC_INTERRUPT_PRIORITY 5

#define ESTOP_COOLDOWN_MS 1000
#define ESTOP_SW_SAFE_STATE 1

#define JOG_FWD_SW_ACTIVE_STATE 0
#define JOG_REV_SW_ACTIVE_STATE 0

#define ADC_RES_BITS 12
#define ADC_MAX_VALUE ((1 << ADC_RES_BITS)-1)

Button LearnButton;
Button HomeButton;
Button CycleButton;
Button FingerButton;
Button HeadButton;

bool configure_io();
void e_stop_handler();
bool motor_movement_checks();
void print_motor_alerts();
void iteration_time_check();
void config_ccio_pin(ClearCorePins target_pin, Connector::ConnectorModes mode, bool initial_state = false);
void set_ccio_pin(ClearCorePins target_pin, bool state);
void ConfigurePeriodicInterrupt(uint32_t frequencyHz);
uint32_t bytes_to_u32(const uint8_t bytes[4], uint32_t offset = 0);
void u32_to_bytes(uint32_t value, uint8_t bytes[4], uint32_t offset = 0);
void read_job_from_nvram();
void save_job_to_nvram();
bool wait_for_motion();
void set_finger_state(bool state);
void set_head_state(bool state);
bool move_motor_with_speed(int32_t position, int32_t speed);
bool move_motor_auto_speed(int32_t position);
void motor_jog(bool reverse);
void update_buttons();
void update_speed_pot();

extern "C" void TCC2_0_Handler(void) __attribute__((
            alias("PeriodicInterrupt")));



void setup() {

  ESTOP_SW.Mode(Connector::INPUT_DIGITAL);
  ESTOP_SW.FilterLength(5, DigitalIn::FILTER_UNIT_SAMPLES);
  ESTOP_SW.InterruptHandlerSet(e_stop_handler, InputManager::InterruptTrigger::CHANGE);
  if (ESTOP_SW.State() != ESTOP_SW_SAFE_STATE) { // make sure e-stop wasn't already depressed before interrupt was registered
    e_stop_handler();
  }

  ConnectorUsb.PortOpen();
  const uint32_t startTime = millis();
  while (!ConnectorUsb && millis() - startTime < SERIAL_ESTABLISH_TIMEOUT)
    continue;

  if (!ConnectorUsb) {
    machine_state = PlanishStates::error;
    return;
  }

  // Debug delay to be able to restart motor before program starts
  uint8_t delay_cycles = 3;
  while (delay_cycles--) {
    delay(1000);
    ConnectorUsb.SendLine(delay_cycles);
  }

  read_job_from_nvram();

  ConnectorUsb.SendLine("job retrieved from NVRAM");
  ConnectorUsb.SendLine(saved_job_start_pos);
  ConnectorUsb.SendLine(saved_job_end_pos);
  ConnectorUsb.SendLine(saved_job_park_pos);

  io_configured = configure_io();

  if (io_configured) {          // if the IO was successfully configured,
    machine_state = PlanishStates::post;
    // do this twice so that the buffer only contains measured values
    update_buttons();
    update_buttons();
  } else {                      // if the IO failed to configure properly
    ConnectorUsb.SendLine("IO was not configured successfully, hanging");
    machine_state = PlanishStates::error;
    return;
  }

  last_iteration_time = millis();
  machine_state = PlanishStates::post;
}


void loop() {
  // If the ESTOP ISR is called after a state change condition is met,
  // but before the state change, the `e_stop_begin` set in the ISR will be overwritten.
  // Fix by checking at the beginning of every cycle
  if (is_e_stop) {
    machine_state = PlanishStates::e_stop_begin;
  }

  loop_num++;

  if (io_configured) {
    // on the first execution it is possible that IO is not configured
    update_buttons();
    if (loop_num%64==0) {
      update_speed_pot();
    }
  }

  last_iteration_time = millis();

  switch (machine_state) {
    case PlanishStates::post:  // Not used anymore but could be in the future
      ConnectorUsb.SendLine("Power on self test");
      machine_state = PlanishStates::idle;
      break;
    case PlanishStates::begin_homing:
      ConnectorUsb.SendLine("begin homing");
      CARRIAGE_MOTOR.EnableRequest(false);
      delay(10); // TODO: remove delays where possible
      CARRIAGE_MOTOR.EnableRequest(true);
      delay(5);
      home_indicator_light.setPattern(LightPattern::BLINK);
      machine_state = PlanishStates::wait_for_homing;
      break;
    case PlanishStates::wait_for_homing:
      if (CARRIAGE_MOTOR.HlfbState() == MotorDriver::HLFB_ASSERTED) {
        // homing done no errors
        ConnectorUsb.SendLine("homing complete");
        CARRIAGE_MOTOR.PositionRefSet(0);
        is_homed = true;
        home_indicator_light.setPattern(LightPattern::ON);
        machine_state = PlanishStates::idle;
      } else if (CARRIAGE_MOTOR.StatusReg().bit.AlertsPresent) {
        // motor has an error

        if(CARRIAGE_MOTOR.AlertReg().bit.MotionCanceledMotorDisabled) {
          // TODO: If homing requirement for jogging gets removed, this needs updating
          ConnectorUsb.SendLine("Motor was commanded to move before enabling."
                                "depending on how the code turns out this might not be problematic");
        }

        ConnectorUsb.SendLine("Motor alert detected during homing.");
        print_motor_alerts();
        machine_state = PlanishStates::error;
        home_indicator_light.setPattern(LightPattern::STROBE);
      } else {
        // still homing
      }
      break;
    case PlanishStates::idle:
      // if (loop_num%128==0) {
      //   ConnectorUsb.Send("CARRIAGE_MOTOR.PositionRefCommanded() == ");
      //   ConnectorUsb.SendLine(CARRIAGE_MOTOR.PositionRefCommanded());
      // }
      if (!is_e_stop && MANDREL_LATCH_LMT.State()) {
        if (HomeButton.is_rising()) {
          machine_state = PlanishStates::begin_homing;
          break;
        }
        if (FingerButton.is_rising()) {
          const bool new_finger_state = !is_fingers_down;
          ConnectorUsb.SendLine(new_finger_state ? "Engaging fingers" : "Disengaging fingers");
          set_finger_state(new_finger_state);
        }
        if (HeadButton.is_changing()) {
          // Despite the '==' in that expression, this check is to see if there is a MISMATCH between
          // measured and commanded head state.
          set_head_state(HeadButton.get_current_state());
          machine_state = PlanishStates::wait_for_head;
          break;
        }
        if (is_homed) {
          if (JOG_FWD_SW.State() == JOG_FWD_SW_ACTIVE_STATE
              || JOG_REV_SW.State() == JOG_REV_SW_ACTIVE_STATE) {
            machine_state = PlanishStates::manual_jog;
            break;
          }
          if (LearnButton.is_rising()) {
            machine_state = PlanishStates::learn_start_pos;
            break;
          }
          if (CycleButton.is_rising()) {
            if (is_fingers_down) {
              machine_state = PlanishStates::job_begin;
              break;
            } else {
              ConnectorUsb.SendLine("Tried to start cycle without fingers engaged");
            }
          }
        }
      }
      break;
    case PlanishStates::wait_for_head:
      if (loop_num%512==0) {
        ConnectorUsb.Send("Waiting for head. Currently commanded ");
        ConnectorUsb.SendLine(is_head_commanded_down ? "down." : "up.");
      }
      if (is_head_commanded_down != HEAD_UP_LMT.State()) {
        machine_state = PlanishStates::idle;
      }
      break;  
    case PlanishStates::manual_jog:
      if (!is_e_stop && MANDREL_LATCH_LMT.State() && is_homed) {
        if (JOG_FWD_SW.State() == JOG_FWD_SW_ACTIVE_STATE) {
          motor_jog(false);
          break;
        }
        if (JOG_REV_SW.State() == JOG_REV_SW_ACTIVE_STATE) {
          motor_jog(true);
          break;
        }
      }
      machine_state = PlanishStates::idle;
      CARRIAGE_MOTOR.MoveStopDecel();
      break;
    case PlanishStates::e_stop_begin:

      break;
    case PlanishStates::e_stop_wait:
      if ((ESTOP_SW.State() == ESTOP_SW_SAFE_STATE) && (millis() - last_estop_millis > ESTOP_COOLDOWN_MS)) {
        ConnectorUsb.SendLine("Emergency Stop Released, returning to idle");
        machine_state = PlanishStates::idle;
        is_e_stop = false;
        break;
      }
      break;
    case PlanishStates::resume_from_e_stop:
      break;
    case PlanishStates::error:
      while (true) {
        ConnectorUsb.SendLine("reached error state");
        delay(1000);
      }
      break;
    case PlanishStates::job_begin:
      set_head_state(false);
      if (HEAD_UP_LMT.State()) {
        ConnectorUsb.SendLine("Job started head was already up");
        machine_state = PlanishStates::job_jog_to_start;
      } else {
        ConnectorUsb.SendLine("Job was started but the head was already down, raising");
        machine_state = PlanishStates::job_begin_lifting_head;
      }
      break;
    case PlanishStates::job_begin_lifting_head:
      if (is_e_stop || !MANDREL_LATCH_LMT.State() || !is_homed) {
        // TODO: this should be recoverable
        ConnectorUsb.SendLine("preconditions failed since starting job");
        machine_state = PlanishStates::error;
        break;
      }
      if (HEAD_UP_LMT.State()) {
        machine_state = PlanishStates::job_jog_to_start;
      }
      if (loop_num%512==0) {
        ConnectorUsb.SendLine("Waiting for head to rise for job start");
      }
      break;
    case PlanishStates::job_jog_to_start:
      ConnectorUsb.SendLine("Jog to start");
      move_motor_auto_speed(saved_job_start_pos);
      machine_state = PlanishStates::job_jog_to_start_wait;
      break;
    case PlanishStates::job_jog_to_start_wait:
      CARRIAGE_MOTOR.VelMax(current_jog_speed);
      if (wait_for_motion()) {
        machine_state = PlanishStates::job_head_down;
      }
      if (loop_num%512==0) {
        ConnectorUsb.SendLine("waiting for jog to start");
      }
      break;
    case PlanishStates::job_head_down:
      ConnectorUsb.SendLine("Lowering head");
      set_head_state(true);
      machine_state = PlanishStates::job_head_down_wait;
      break;
    case PlanishStates::job_head_down_wait:
      if (is_e_stop || !MANDREL_LATCH_LMT.State() || !is_homed) {
        // TODO: this should be recoverable
        ConnectorUsb.SendLine("preconditions failed since starting job");
        machine_state = PlanishStates::error;
        break;
      }
      if (!HEAD_UP_LMT.State()) {
        machine_state = PlanishStates::job_planish_to_end;
      }
      if (loop_num%512==0) {
        ConnectorUsb.SendLine("Waiting for head to lower to begin planish");
      }
      break;
    case PlanishStates::job_planish_to_end:
      ConnectorUsb.SendLine("Planishing to end position");
      move_motor_auto_speed(saved_job_end_pos);
      machine_state = PlanishStates::job_planish_to_end_wait;
      break;
    case PlanishStates::job_planish_to_end_wait:
      CARRIAGE_MOTOR.VelMax(current_planish_speed);
      if (wait_for_motion()) {
        if (HALF_CYCLE_SW.State()) {
          ConnectorUsb.SendLine("half cycle detected, go to park");
          // if only doing a half-cycle, the single pass thus far is sufficient. retract head and return to park
          machine_state = PlanishStates::job_head_up;
        } else {
          ConnectorUsb.SendLine("full cycle detected, do second pass");
          // otherwise do the other pass
          machine_state = PlanishStates::job_planish_to_start;
        }
      }
      if (loop_num%512==0) {
        ConnectorUsb.SendLine("Planishing to end position");
      }
      break;
    case PlanishStates::job_planish_to_start:
      ConnectorUsb.SendLine("Begin planishing to start position");
      move_motor_auto_speed(saved_job_start_pos);
      machine_state = PlanishStates::job_planish_to_start_wait;
      break;
    case PlanishStates::job_planish_to_start_wait:
      CARRIAGE_MOTOR.VelMax(current_planish_speed);
      if (wait_for_motion()) {
        machine_state = PlanishStates::job_head_up;
      }
      if (loop_num%512==0) {
        ConnectorUsb.SendLine("Planishing to start position");
      }
      break;
    case PlanishStates::job_head_up:
      ConnectorUsb.SendLine("Raising head");
      set_head_state(false);
      machine_state = PlanishStates::job_head_up_wait;
      break;
    case PlanishStates::job_head_up_wait:
      if (is_e_stop || !MANDREL_LATCH_LMT.State() || !is_homed) {
        // TODO: this should be recoverable
        ConnectorUsb.SendLine("preconditions failed since starting job");
        machine_state = PlanishStates::error;
        break;
      }
      if (HEAD_UP_LMT.State()) {
        machine_state = PlanishStates::job_jog_to_park;
      }
      if (loop_num%512==0) {
        ConnectorUsb.SendLine("Waiting for head to raise to park");
      }
      break;
    case PlanishStates::job_jog_to_park:
      ConnectorUsb.SendLine("Begin jogging to park position");
      move_motor_auto_speed(saved_job_park_pos);
      machine_state = PlanishStates::job_jog_to_park_wait;
      break;
    case PlanishStates::job_jog_to_park_wait:
      CARRIAGE_MOTOR.VelMax(current_jog_speed);
      if (wait_for_motion()) {
        machine_state = PlanishStates::idle;
      }
      if (loop_num%512==0) {
        ConnectorUsb.SendLine("Jogging to park position");
      }
      break;
    case PlanishStates::learn_start_pos:
      learn_indicator_light.setPattern(LightPattern::FLASH1);
      ConnectorUsb.SendLine("learn start pos");
      temp_job_start_pos = CARRIAGE_MOTOR.PositionRefCommanded();
      machine_state = PlanishStates::learn_jog_to_end_pos;
      break;
    case PlanishStates::learn_jog_to_end_pos:
      if (!is_e_stop && MANDREL_LATCH_LMT.State() && is_homed) {
        if (LearnButton.is_rising()) {
          machine_state = PlanishStates::learn_end_pos;
          break;
        }
        if (JOG_FWD_SW.State() == JOG_FWD_SW_ACTIVE_STATE) {
          motor_jog(false);
          break;
        }
        if (JOG_REV_SW.State() == JOG_REV_SW_ACTIVE_STATE) {
          motor_jog(true);
          break;
        }
      }
      CARRIAGE_MOTOR.MoveStopDecel();
      break;
    case PlanishStates::learn_end_pos:
      learn_indicator_light.setPattern(LightPattern::FLASH2);
      ConnectorUsb.SendLine("learn end pos");
      temp_job_end_pos = CARRIAGE_MOTOR.PositionRefCommanded();
      machine_state = PlanishStates::learn_jog_to_park_pos;
      break;
    case PlanishStates::learn_jog_to_park_pos:
      if (!is_e_stop && MANDREL_LATCH_LMT.State() && is_homed) {
        if (LearnButton.is_rising()) {
          machine_state = PlanishStates::learn_park_pos;
          break;
        }
        if (JOG_FWD_SW.State() == JOG_FWD_SW_ACTIVE_STATE) {
          motor_jog(false);
          break;
        }
        if (JOG_REV_SW.State() == JOG_REV_SW_ACTIVE_STATE) {
          motor_jog(true);
          break;
        }
      }
      CARRIAGE_MOTOR.MoveStopDecel();
      break;
    case PlanishStates::learn_park_pos:
      learn_indicator_light.setPattern(LightPattern::FLASH3);
      ConnectorUsb.SendLine("learn park pos");
      temp_job_park_pos = CARRIAGE_MOTOR.PositionRefCommanded();
      machine_state = PlanishStates::saving_job_to_nvram;
      break;
    case PlanishStates::saving_job_to_nvram:
      ConnectorUsb.SendLine("saving job to nvram");
      ConnectorUsb.SendLine(temp_job_start_pos);
      ConnectorUsb.SendLine(temp_job_end_pos);
      ConnectorUsb.SendLine(temp_job_park_pos);
      save_job_to_nvram();
      learn_indicator_light.setPattern(LightPattern::OFF);

      machine_state = PlanishStates::idle;
      break;
  }
}

/**
 * Configures IO, go figure
 * @return True for success
 */
bool configure_io() {

  LASER_SW.Mode(Connector::INPUT_DIGITAL);
  LEARN_SW.Mode(Connector::INPUT_DIGITAL);
  SPEED_POT.Mode(Connector::INPUT_ANALOG);
  MANDREL_LATCH_LMT.Mode(Connector::INPUT_DIGITAL);
  HALF_CYCLE_SW.Mode(Connector::INPUT_DIGITAL);
  HEAD_UP_LMT.Mode(Connector::INPUT_DIGITAL);
  HOME_SW.Mode(Connector::INPUT_DIGITAL);
  CYCLE_SW.Mode(Connector::INPUT_DIGITAL);
  FINGER_SW.Mode(Connector::INPUT_DIGITAL);
  JOG_FWD_SW.Mode(Connector::INPUT_DIGITAL);
  JOG_REV_SW.Mode(Connector::INPUT_DIGITAL);
  HEAD_SW.Mode(Connector::INPUT_DIGITAL);

  AdcMgr.AdcResolution(ADC_RES_BITS);

  CCIO1.Mode(Connector::CCIO);
  CCIO1.PortOpen();

  if (CcioMgr.LinkBroken()) {
    uint32_t lastStatusTime = Milliseconds();
    ConnectorUsb.SendLine("The CCIO-8 link is broken!");

    while (CcioMgr.LinkBroken() && Milliseconds() - lastStatusTime < CCIO_TIMEOUT_MS) {
      if (Milliseconds() - lastStatusTime > 1000) {
        ConnectorUsb.SendLine("The CCIO-8 link is still broken!");
        lastStatusTime = Milliseconds();
      }
    }
    if (CcioMgr.LinkBroken()) {
      ConnectorUsb.SendLine("Timed out waiting for CCIO");
      return false;
    }
    ConnectorUsb.SendLine("The CCIO-8 link is online again!");
  }

  if (CcioMgr.CcioCount() != EXPECTED_NUM_CCIO) {
    ConnectorUsb.Send("Expected to find exactly one CCIO connector, found ");
    ConnectorUsb.SendLine(CcioMgr.CcioCount());
    return false;
  }

  config_ccio_pin(FINGER_ACTUATION, Connector::OUTPUT_DIGITAL);
  config_ccio_pin(HEAD_ACTUATION, Connector::OUTPUT_DIGITAL);
  config_ccio_pin(HOME_SW_LIGHT, Connector::OUTPUT_DIGITAL);
  config_ccio_pin(LEARN_SW_LIGHT, Connector::OUTPUT_DIGITAL);

  home_indicator_light.setPin(CcioMgr.PinByIndex(HOME_SW_LIGHT));
  home_indicator_light.setPeriod(50);
  home_indicator_light.setPattern(LightPattern::OFF);
  learn_indicator_light.setPin(CcioMgr.PinByIndex(LEARN_SW_LIGHT));
  learn_indicator_light.setPeriod(50);
  learn_indicator_light.setPattern(LightPattern::OFF);

  MotorMgr.MotorInputClocking(MotorManager::CLOCK_RATE_NORMAL);
  MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL, Connector::CPM_MODE_STEP_AND_DIR);

  CARRIAGE_MOTOR.HlfbMode(MotorDriver::HLFB_MODE_HAS_BIPOLAR_PWM);
  CARRIAGE_MOTOR.HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);


  update_speed_pot();
  CARRIAGE_MOTOR.VelMax(current_jog_speed);
  CARRIAGE_MOTOR.AccelMax(CARRIAGE_MOTOR_MAX_ACCEL);

  set_finger_state(false);
  set_head_state(false);

  ConfigurePeriodicInterrupt(1000);

  return true;
}

/**
 * After commanding motion as part of a job, this is just a helper for waiting for that motion to complete.
 * It will return true when completed successfully, and false if not done or has an error.
 * Error states are automatically dealt with, assuming the next thing that happens is
 * a new iteration of the state loop.
 */
bool wait_for_motion() {
  if (CARRIAGE_MOTOR.HlfbState() == MotorDriver::HLFB_ASSERTED
      && CARRIAGE_MOTOR.StepsComplete()) {
    ConnectorUsb.SendLine("Motor has completed motion");
    return true;
  }
  if (CARRIAGE_MOTOR.StatusReg().bit.AlertsPresent) {
    // motor has an error
    ConnectorUsb.SendLine("Motor alert detected during homing.");
    print_motor_alerts();
    machine_state = PlanishStates::error;
    return false;
  }

  // TODO: make sure EVERY TIME something is being waited for in the main loop that setting estop to true will abort the wait. That means check everywhere else

  if (is_e_stop || !MANDREL_LATCH_LMT.State() || !is_homed) {
    // TODO: this should be recoverable, and treated as an e-stop
    ConnectorUsb.SendLine("preconditions failed since starting job");
    machine_state = PlanishStates::error;
    return false;
  }
  return false;
}

/**
 * Interrupt handler gets automatically called every ms
 */
extern "C" void PeriodicInterrupt(void) {
  // These calls just update the indicator light
  // logic so they can check if they need to be changed
  home_indicator_light.tick();
  learn_indicator_light.tick();

  // Check the time since the last loop was run.
  // If it's been too long, then something is wrong.
  iteration_time_check();

  // Acknowledge the interrupt to clear the flag and wait for the next interrupt.
  TCC2->INTFLAG.reg = TCC_INTFLAG_MASK; // This is critical
}

/**
 * Check interval between calls.
 * This should be called regularly
 */
void iteration_time_check() {
  last_iteration_delta = millis()-last_iteration_time;

  if (last_iteration_delta >= ITERATION_TIME_ERROR_MS) {
    ConnectorUsb.Send("Last iteration of the state machine took more than `ITERATION_TIME_ERROR_MS` (");
    ConnectorUsb.Send(ITERATION_TIME_ERROR_MS);
    ConnectorUsb.Send("ms) to complete. This is likely a bug. Last iteration took ");
    ConnectorUsb.Send(last_iteration_delta);
    ConnectorUsb.Send("ms. Engaging E-Stop and stopping execution\n");
    machine_state = PlanishStates::error;
  } else if (last_iteration_delta >= ITERATION_TIME_WARNING_MS) {
    ConnectorUsb.Send("Last iteration of the state machine took more than `ITERATION_TIME_WARNING_MS` (");
    ConnectorUsb.Send(ITERATION_TIME_WARNING_MS);
    ConnectorUsb.Send("ms) to complete. This is likely a bug. Last iteration took ");
    ConnectorUsb.Send(last_iteration_delta);
    ConnectorUsb.Send("ms. Continuing execution");
  }
}

/**
 * Only used to configure the ms interrupt.
 * Should only be called once other IO is configured.
 * If you are confused, don't worry, this won't make sense without knowing the
 * internal architecture of the hardware. It was stolen from
 * https://teknic-inc.github.io/ClearCore-library/_periodic_interrupt_8cpp-example.html
 *
 * @param frequencyHz
 */
void ConfigurePeriodicInterrupt(const uint32_t frequencyHz) {
    // Enable the TCC2 peripheral.
    // TCC2 and TCC3 share their clock configuration and they
    // are already configured to be clocked at 120 MHz from GCLK0.
    CLOCK_ENABLE(APBCMASK, TCC2_);

    // Disable TCC2.
    TCC2->CTRLA.bit.ENABLE = 0;
    SYNCBUSY_WAIT(TCC2, TCC_SYNCBUSY_ENABLE);

    // Reset the TCC module so we know we are starting from a clean state.
    TCC2->CTRLA.bit.SWRST = 1;
    while (TCC2->CTRLA.bit.SWRST) {}

    // If the frequency requested is zero, disable the interrupt and bail out.
    if (!frequencyHz) {
        NVIC_DisableIRQ(TCC2_0_IRQn);
        return;
    }

    // Determine the clock prescaler and period value needed to achieve the
    // requested frequency.
    uint32_t period = (CPU_CLK + frequencyHz / 2) / frequencyHz;
    uint8_t prescale;
    // Make sure period is >= 1.
    period = max(period, 1U);

    // Prescale values 0-4 map to prescale divisors of 1-16,
    // dividing by 2 each increment.
    for (prescale = TCC_CTRLA_PRESCALER_DIV1_Val;
            prescale < TCC_CTRLA_PRESCALER_DIV16_Val && (period - 1) > UINT16_MAX;
            prescale++) {
        period = period >> 1;
    }
    // Prescale values 5-7 map to prescale divisors of 64-1024,
    // dividing by 4 each increment.
    for (; prescale < TCC_CTRLA_PRESCALER_DIV1024_Val && (period - 1) > UINT16_MAX;
            prescale++) {
        period = period >> 2;
    }
    // If we have maxed out the prescaler and the period is still too big,
    // use the maximum period. This results in a ~1.788 Hz interrupt.
    if (period > UINT16_MAX) {
        TCC2->PER.reg = UINT16_MAX;
    }
    else {
        TCC2->PER.reg = period - 1;
    }
    TCC2->CTRLA.bit.PRESCALER = prescale;

    // Interrupt every period on counter overflow.
    TCC2->INTENSET.bit.OVF = 1;
    // Enable TCC2.
    TCC2->CTRLA.bit.ENABLE = 1;

    // Set the interrupt priority and enable it.
    NVIC_SetPriority(TCC2_0_IRQn, PERIODIC_INTERRUPT_PRIORITY);
    NVIC_EnableIRQ(TCC2_0_IRQn);
}


void config_ccio_pin(
  const ClearCorePins target_pin,
  const Connector::ConnectorModes mode,
  const bool initial_state)
{
  CcioPin *pin_pointer = CcioMgr.PinByIndex(target_pin);
  if (pin_pointer != nullptr) {
    pin_pointer->Mode(mode);
    pin_pointer->State(initial_state);
  }
}

void set_ccio_pin(
  const ClearCorePins target_pin,
  const bool state)
{
  CcioPin *pin_pointer = CcioMgr.PinByIndex(target_pin);
  if (pin_pointer != nullptr) {
    pin_pointer->State(state);
  }
}

void e_stop_handler() {
  is_e_stop = true;
  estop_last_state = machine_state;
  CARRIAGE_MOTOR.MoveStopAbrupt();
  machine_state = PlanishStates::e_stop_begin;
  last_estop_millis = millis();
  ConnectorUsb.SendLine("Emergency Stop");
}



/**
 * Prints active alerts.
 *
 * @pre requires "CARRIAGE_MOTOR" to be defined as a ClearCore motor connector
 */
void print_motor_alerts(){
  // report status of alerts
  ConnectorUsb.SendLine("ClearPath Alerts present: ");
  if(CARRIAGE_MOTOR.AlertReg().bit.MotionCanceledInAlert){
    ConnectorUsb.SendLine("    MotionCanceledInAlert "); }
  if(CARRIAGE_MOTOR.AlertReg().bit.MotionCanceledPositiveLimit){
    ConnectorUsb.SendLine("    MotionCanceledPositiveLimit "); }
  if(CARRIAGE_MOTOR.AlertReg().bit.MotionCanceledNegativeLimit){
    ConnectorUsb.SendLine("    MotionCanceledNegativeLimit "); }
  if(CARRIAGE_MOTOR.AlertReg().bit.MotionCanceledSensorEStop){
    ConnectorUsb.SendLine("    MotionCanceledSensorEStop "); }
  if(CARRIAGE_MOTOR.AlertReg().bit.MotionCanceledMotorDisabled){
    ConnectorUsb.SendLine("    MotionCanceledMotorDisabled "); }
  if(CARRIAGE_MOTOR.AlertReg().bit.MotorFaulted){
    ConnectorUsb.SendLine("    MotorFaulted ");
  }
}



/**
 * Convert bytes to u32. Big endian, little endian?
 * I don't know, but it works with `u32_to_bytes()`
 * @param bytes Const; The bytes to turn into a u32
 * @param offset Offset for accessing array
 * @return the u32
 */
uint32_t bytes_to_u32(const uint8_t *bytes, const uint32_t offset) {
  return
      bytes[offset] << 24 |
      bytes[offset + 1] << 16 |
      bytes[offset + 2] << 8 |
      bytes[offset + 3];
}

/**
 * Convert uint32_t into bytes. Big endian, little endian?
 * I don't know, but it works with `bytes_to_u32()`
 * @param value The u32 to convert
 * @param bytes Where that converted u32 should go
 * @param offset Offset for writing to array
 */
void u32_to_bytes(const uint32_t value, uint8_t *bytes, const uint32_t offset) {
  bytes[offset] = (value >> 24) & 0xFF;
  bytes[offset+1] = (value >> 16) & 0xFF;
  bytes[offset+2] = (value >> 8) & 0xFF;
  bytes[offset+3] = value & 0xFF;
}

/**
 * Takes the job from NVRAM and puts it into `saved_job_start_pos`, `saved_job_end_pos`, and `saved_job_park_pos`
 */
void read_job_from_nvram() {
  NvmManager::Instance().BlockRead(NvmManager::NVM_LOC_USER_START, (sizeof NV_Ram), &NV_Ram[0]);
  saved_job_start_pos = bytes_to_u32(NV_Ram, 0);
  saved_job_end_pos = bytes_to_u32(NV_Ram, 4);
  saved_job_park_pos = bytes_to_u32(NV_Ram, 8);
}

/**
 * Take job from `temp_job_start_pos`, `temp_job_end_pos`, `temp_job_park_pos` and save it to NVRAM as well as
 * `saved_job_start_pos`, `saved_job_end_pos`, and `saved_job_park_pos`
 *
 */
void save_job_to_nvram() {
  saved_job_start_pos = temp_job_start_pos;
  saved_job_end_pos = temp_job_end_pos;
  saved_job_park_pos = temp_job_park_pos;

  u32_to_bytes(saved_job_start_pos, NV_Ram, 0);
  u32_to_bytes(saved_job_end_pos, NV_Ram, 4);
  u32_to_bytes(saved_job_park_pos, NV_Ram, 8);
  NvmManager::Instance().BlockWrite(NvmManager::NVM_LOC_USER_START, (sizeof NV_Ram), &NV_Ram[0]);
}

/**
 * Setter for finger state that caches state
 * @param state true for engaged
 */
void set_finger_state(const bool state) {
  is_fingers_down = state;
  set_ccio_pin(FINGER_ACTUATION, state);
}

/**
 * Setter for head state that caches state
 * @param state true for down
 */
void set_head_state(const bool state) {
  is_head_commanded_down = state;
  set_ccio_pin(HEAD_ACTUATION, state);
}

/**
 * Very simple helper function to avoid forgetting to call velmax
 * @param position
 * @param speed
 * @return whatever `CARRIAGE_MOTOR.Move` returns. (no one knows atm)
 */
bool move_motor_with_speed(const int32_t position, const int32_t speed) {
  ConnectorUsb.Send("move_motor_with_speed(");
  ConnectorUsb.Send(position);
  ConnectorUsb.Send(", ");
  ConnectorUsb.Send(speed);
  ConnectorUsb.Send(");");
  CARRIAGE_MOTOR.VelMax(speed);
  return CARRIAGE_MOTOR.Move(position, StepGenerator::MOVE_TARGET_ABSOLUTE);
}

/**
 * Very simple helper function to avoid forgetting to call velmax, but this time, speed is calculated
 * @param position
 * @return whatever `CARRIAGE_MOTOR.Move` returns. (no one knows atm)
 */
bool move_motor_auto_speed(const int32_t position) {
  ConnectorUsb.Send("move_motor_auto_speed(");
  ConnectorUsb.Send(position);
  ConnectorUsb.Send(");");
  const int32_t speed = HEAD_UP_LMT.State() ? current_jog_speed : current_planish_speed;
  CARRIAGE_MOTOR.VelMax(speed);
  return CARRIAGE_MOTOR.Move(position, StepGenerator::MOVE_TARGET_ABSOLUTE);
}

void motor_jog(const bool reverse) {
  if (HEAD_UP_LMT.State()) {
    CARRIAGE_MOTOR.MoveVelocity(reverse ? -current_jog_speed : current_jog_speed);
  } else {
    CARRIAGE_MOTOR.MoveVelocity(reverse ? -current_planish_speed : current_planish_speed);
  }
}

/**
 * Updates the button states at once. This is done to make sure that on
 * every iteration of the state machine loop, the button can be guaranteed
 * to have risen THAT loop and not previously.
 *
 * @pre IO has been configured
 */
void update_buttons() {
  LearnButton.new_reading(LEARN_SW.State());
  HomeButton.new_reading(HOME_SW.State());
  CycleButton.new_reading(CYCLE_SW.State());
  FingerButton.new_reading(FINGER_SW.State());
  HeadButton.new_reading(HEAD_SW.State());
}

/**
 * Takes reading from speed pot and sets `current_planish_speed` and `current_jog_speed` to values
 * scaled from their max by the pot
 *
 * @pre IO is configured
 */
void update_speed_pot() {
  const int16_t analog_reading = SPEED_POT.State();
  const float pot_percent = static_cast<float>(analog_reading) / ADC_MAX_VALUE;

  const float temp_jog_speed = pot_percent * CARRIAGE_MOTOR_MAX_VEL;
  const float temp_planish_speed = pot_percent * CARRIAGE_MOTOR_MAX_PLANISH_VEL;

  current_planish_speed = static_cast<int32_t>(temp_planish_speed);
  current_jog_speed = static_cast<int32_t>(temp_jog_speed);
}

/**
 * Gets the state the machine should return to if it was interupted.
 * For example if it was interupted while waiting for the motor to finish moving,
 * it can just go back to waiting after an estop is cleared because
 * the motor will have stopped. Instead it must backtrack and resume from the
 * previous state that issued the move command
 * @param last_state The last known state before state machine was interrupted
 * @return The state to resume from
 */
PlanishStates e_stop_resume(const PlanishStates last_state) {

  // !! I am purposefully omitting a default case.
  // even though it saves some lines, I would rather not forget new states
  switch (last_state) {
    case PlanishStates::post:
      return PlanishStates::post;
    case PlanishStates::begin_homing:
      return PlanishStates::begin_homing;
    case PlanishStates::wait_for_homing:
      // If interrupted during homing the motor is actually disabled,
      // as any move commands to stop it do not work.
      // Therefore it needs to restart the homing process
      return PlanishStates::begin_homing;
    case PlanishStates::idle:
      return PlanishStates::idle;
    case PlanishStates::manual_jog:
      // While the manual jog state will already send the state
      // back to idle if no jog is commanded, I'd rather not risk it.
      // This is totally recoverable anyway
      return PlanishStates::idle;
    case PlanishStates::e_stop_begin:
      // if the e_stop state gets interrupted by an
      // e-stop I think it's safe to have a breakdown
      return PlanishStates::error;
    case PlanishStates::e_stop_wait:
      break;
    case PlanishStates::resume_from_e_stop:
      break;
    case PlanishStates::error:
      // error state only exists to be unrecoverable
      return PlanishStates::error;
    case PlanishStates::wait_for_head:
      // TODO: Ensure substate is maintained and will resume with the correct command
      return PlanishStates::wait_for_head;
    case PlanishStates::job_begin:
      break;
    case PlanishStates::job_begin_lifting_head:
      break;
    case PlanishStates::job_jog_to_start:
      break;
    case PlanishStates::job_jog_to_start_wait:
      break;
    case PlanishStates::job_head_down:
      break;
    case PlanishStates::job_head_down_wait:
      break;
    case PlanishStates::job_planish_to_end:
      break;
    case PlanishStates::job_planish_to_end_wait:
      break;
    case PlanishStates::job_planish_to_start:
      break;
    case PlanishStates::job_planish_to_start_wait:
      break;
    case PlanishStates::job_head_up:
      break;
    case PlanishStates::job_head_up_wait:
      break;
    case PlanishStates::job_jog_to_park:
      break;
    case PlanishStates::job_jog_to_park_wait:
      break;
    case PlanishStates::learn_start_pos:
      break;
    case PlanishStates::learn_jog_to_end_pos:
      break;
    case PlanishStates::learn_end_pos:
      break;
    case PlanishStates::learn_jog_to_park_pos:
      break;
    case PlanishStates::learn_park_pos:
      break;
    case PlanishStates::saving_job_to_nvram:
      break;
  }
  return PlanishStates::error;
}

void secure_system(const PlanishStates last_state) {
  switch (last_state) {
    case PlanishStates::post:
      // E-stop during post isn't worth recovering. Recover from what?
      estop_resume_state = PlanishStates::error;
      break;
    case PlanishStates::begin_homing:
    case PlanishStates::wait_for_homing:
      home_indicator_light.setPattern(LightPattern::OFF);
      CARRIAGE_MOTOR.EnableRequest(false);
      is_homed = false;
      estop_resume_state = PlanishStates::idle;
      break;
    case PlanishStates::idle:
    case PlanishStates::manual_jog:
      // these are both always recoverable
      break;
    case PlanishStates::e_stop_begin:
      break;
    case PlanishStates::e_stop_wait:
      break;
    case PlanishStates::resume_from_e_stop:
      break;
    case PlanishStates::error:
      break;
    case PlanishStates::wait_for_head:
      break;
    case PlanishStates::job_begin:
      break;
    case PlanishStates::job_begin_lifting_head:
      break;
    case PlanishStates::job_jog_to_start:
      break;
    case PlanishStates::job_jog_to_start_wait:
      break;
    case PlanishStates::job_head_down:
      break;
    case PlanishStates::job_head_down_wait:
      break;
    case PlanishStates::job_planish_to_end:
      break;
    case PlanishStates::job_planish_to_end_wait:
      break;
    case PlanishStates::job_planish_to_start:
      break;
    case PlanishStates::job_planish_to_start_wait:
      break;
    case PlanishStates::job_head_up:
      break;
    case PlanishStates::job_head_up_wait:
      break;
    case PlanishStates::job_jog_to_park:
      break;
    case PlanishStates::job_jog_to_park_wait:
      break;
    case PlanishStates::learn_start_pos:
    case PlanishStates::learn_jog_to_end_pos:
    case PlanishStates::learn_end_pos:
    case PlanishStates::learn_jog_to_park_pos:
    case PlanishStates::learn_park_pos:
      // An estop here will cancel the learn operation
      learn_indicator_light.setPattern(LightPattern::OFF);


      estop_resume_state = PlanishStates::idle;
      break;
    case PlanishStates::saving_job_to_nvram:
      break;
  }
}