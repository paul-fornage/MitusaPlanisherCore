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


bool is_homed = false;                  // Has the axis been homed
volatile bool is_e_stop = false;                // Is the Emergency Stop currently active

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

enum PlanishStates {
  post,                   // Power On Self Test
  begin_homing,           // start axis homing
  wait_for_homing,        // Wait for homing to complete
  idle,                   // Idle await instructions
  manual_jog,             // manually commanded jog
  e_stop,                 // E-stop currently active
  error,                  // unrecoverable error has occurred and machine needs to reboot. should be a dead end state

  job_start,              // Job started, move to saved start pos
  job_planish_1,          // Start position reached, engage roller and move to end pos
  job_planish_2,          // In full cycle only, do another pass going back to the beginning
  job_park,               // Planish finished, disengage roller and jog to park

  learn_start_pos,        // Learn has been pressed, set start point
  learn_jog_to_end_pos,   // Manually jogging to end position
  learn_end_pos,          // Learn was pressed again, set end position
  learn_jog_to_park_pos,  // Manually jogging to park position
  learn_park_pos,         // Learn was pressed again, set park position
  saving_job_to_nvram,      // Job is recorded and needs to be saved in NVRAM
};

volatile PlanishStates machine_state;

volatile IndicatorLight home_indicator_light;   /// Defined globally, but should not be accessed unless configure_IO has completed without errors
volatile IndicatorLight learn_indicator_light;  /// Defined globally, but should not be accessed unless configure_IO has completed without errors

#define ESTOP_SW ConnectorA12
#define LASER_SW ConnectorA11
#define LEARN_SW ConnectorA10
#define SPEED_POT ConnectorA9
#define MANDREL_LATCH_LMT ConnectorDI8
#define SINGLE_CYCLE_SW ConnectorDI7
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
#define CARRIAGE_MOTOR_MAX_VEL 5000
#define CARRIAGE_MOTOR_MAX_POS 12000

#define ITERATION_TIME_WARNING_MS 100 // after this many milliseconds stuck on one iteration of the state machine, give a warning.
#define ITERATION_TIME_ERROR_MS 1000  // after this many milliseconds stuck on one iteration of the state machine, declare an error
#define SERIAL_ESTABLISH_TIMEOUT 10000 // Number of ms to wait for serial to establish before failing POST

/// Interrupt priority for the periodic interrupt. 0 is highest priority, 7 is lowest.
#define PERIODIC_INTERRUPT_PRIORITY 5

#define ESTOP_COOLDOWN_MS 1000
#define ESTOP_SW_SAFE_STATE 1

#define JOG_FWD_SW_ACTIVE_STATE 0
#define JOG_REV_SW_ACTIVE_STATE 0



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
    machine_state = error;
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

  last_iteration_time = millis();
  machine_state = post;

}


void loop() {
  loop_num++;

  // TODO: Check and latch input button states if high

  iteration_time_check();
  switch (machine_state) {
    case post:
      ConnectorUsb.SendLine("Power on self test");
      // Configure the IO and make sure it was successful
      if (!configure_io()) {
        machine_state = error;
        break;
      }
      machine_state = idle;
      break;
    case begin_homing:
      ConnectorUsb.SendLine("begin homing");
      CARRIAGE_MOTOR.EnableRequest(false);
      delay(10);
      CARRIAGE_MOTOR.EnableRequest(true);
      home_indicator_light.setPattern(LightPattern::BLINK);
      machine_state = wait_for_homing;
      break;
    case wait_for_homing:
      if (CARRIAGE_MOTOR.HlfbState() == MotorDriver::HLFB_ASSERTED) {
        // homing done no errors
        ConnectorUsb.SendLine("homing complete");
        CARRIAGE_MOTOR.PositionRefSet(0);
        is_homed = true;
        home_indicator_light.setPattern(LightPattern::ON);
        machine_state = idle;
      } else if (CARRIAGE_MOTOR.StatusReg().bit.AlertsPresent) {
        // motor has an error
        ConnectorUsb.SendLine("Motor alert detected during homing.");
        print_motor_alerts();
        machine_state = error;
        home_indicator_light.setPattern(LightPattern::STROBE);
      } else {
        // still homing
      }
      break;
    case idle:
      // if (loop_num%128==0) {
      //   ConnectorUsb.Send("CARRIAGE_MOTOR.PositionRefCommanded() == ");
      //   ConnectorUsb.SendLine(CARRIAGE_MOTOR.PositionRefCommanded());
      // }
      if (!is_e_stop && MANDREL_LATCH_LMT.State()) {
        if (HOME_SW.InputRisen() || HOME_SW.State()) {
          machine_state = begin_homing;
          break;
        }
        if (is_homed) {
          if (JOG_FWD_SW.State() == JOG_FWD_SW_ACTIVE_STATE
              || JOG_REV_SW.State() == JOG_REV_SW_ACTIVE_STATE) {
            machine_state = manual_jog;
          }
          if (LEARN_SW.InputRisen() || LEARN_SW.State()) {
            machine_state = learn_start_pos;
          }
        }
      }


      break;
    case manual_jog:
      if (!is_e_stop && MANDREL_LATCH_LMT.State() && is_homed) {
        if (JOG_FWD_SW.State() == JOG_FWD_SW_ACTIVE_STATE) {
          CARRIAGE_MOTOR.MoveVelocity(500);
          break;
        }
        if (JOG_REV_SW.State() == JOG_REV_SW_ACTIVE_STATE) {
          CARRIAGE_MOTOR.MoveVelocity(-500);
          break;
        }
      }
      machine_state = idle;
      CARRIAGE_MOTOR.MoveStopDecel();
      break;
    case e_stop:
      if ((ESTOP_SW.State() == ESTOP_SW_SAFE_STATE) && (millis() - last_estop_millis > ESTOP_COOLDOWN_MS)) {
        ConnectorUsb.SendLine("Emergency Stop Released, returning to idle");
        machine_state = idle;
        is_e_stop = false;
        break;
      }
      break;
    case error:
      while (true) {
        ConnectorUsb.SendLine("reached error state");
        delay(1000);
      }
      break;
    case job_start:
      break;
    case job_planish_1:
      break;
    case job_planish_2:
      break;
    case job_park:
      break;
    case learn_start_pos:
      learn_indicator_light.setPattern(LightPattern::FLASH1);
      ConnectorUsb.SendLine("learn start pos");
      temp_job_start_pos = CARRIAGE_MOTOR.PositionRefCommanded();
      machine_state = learn_jog_to_end_pos;
      break;
    case learn_jog_to_end_pos:
      if (!is_e_stop && MANDREL_LATCH_LMT.State() && is_homed) {
        if (LEARN_SW.InputRisen() || LEARN_SW.State()) {
          machine_state = learn_end_pos;
          break;
        }
        if (JOG_FWD_SW.State() == JOG_FWD_SW_ACTIVE_STATE) {
          CARRIAGE_MOTOR.MoveVelocity(500);
          break;
        }
        if (JOG_REV_SW.State() == JOG_REV_SW_ACTIVE_STATE) {
          CARRIAGE_MOTOR.MoveVelocity(-500);
          break;
        }
      }
      CARRIAGE_MOTOR.MoveStopDecel();
      break;
    case learn_end_pos:
      learn_indicator_light.setPattern(LightPattern::FLASH2);
      ConnectorUsb.SendLine("learn end pos");
      temp_job_end_pos = CARRIAGE_MOTOR.PositionRefCommanded();
      machine_state = learn_jog_to_park_pos;
      break;
    case learn_jog_to_park_pos:
      if (!is_e_stop && MANDREL_LATCH_LMT.State() && is_homed) {
        if (LEARN_SW.InputRisen() || LEARN_SW.State()) {
          machine_state = learn_park_pos;
          break;
        }
        if (JOG_FWD_SW.State() == JOG_FWD_SW_ACTIVE_STATE) {
          CARRIAGE_MOTOR.MoveVelocity(500);
          break;
        }
        if (JOG_REV_SW.State() == JOG_REV_SW_ACTIVE_STATE) {
          CARRIAGE_MOTOR.MoveVelocity(-500);
          break;
        }
      }
      CARRIAGE_MOTOR.MoveStopDecel();
      break;
    case learn_park_pos:
      learn_indicator_light.setPattern(LightPattern::FLASH3);
      ConnectorUsb.SendLine("learn park pos");
      temp_job_park_pos = CARRIAGE_MOTOR.PositionRefCommanded();
      machine_state = saving_job_to_nvram;
      break;
    case saving_job_to_nvram:
      ConnectorUsb.SendLine("saving job to nvram");
      ConnectorUsb.SendLine(temp_job_start_pos);
      ConnectorUsb.SendLine(temp_job_end_pos);
      ConnectorUsb.SendLine(temp_job_park_pos);
      save_job_to_nvram();
      learn_indicator_light.setPattern(LightPattern::OFF);

      machine_state = idle;
      break;
  }

  // Unlatch input button states
  LEARN_SW.InputRisen();
  HOME_SW.InputRisen();
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
  SINGLE_CYCLE_SW.Mode(Connector::INPUT_DIGITAL);
  HEAD_UP_LMT.Mode(Connector::INPUT_DIGITAL);
  HOME_SW.Mode(Connector::INPUT_DIGITAL);
  CYCLE_SW.Mode(Connector::INPUT_DIGITAL);
  FINGER_SW.Mode(Connector::INPUT_DIGITAL);
  JOG_FWD_SW.Mode(Connector::INPUT_DIGITAL);
  JOG_REV_SW.Mode(Connector::INPUT_DIGITAL);
  HEAD_SW.Mode(Connector::INPUT_DIGITAL);


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


  CARRIAGE_MOTOR.VelMax(CARRIAGE_MOTOR_MAX_VEL);
  CARRIAGE_MOTOR.AccelMax(CARRIAGE_MOTOR_MAX_ACCEL);

  ConfigurePeriodicInterrupt(1000);

  return true;
}


/**
 * Interrupt handler gets automatically called every ms
 */
extern "C" void PeriodicInterrupt(void) {
  // These calls just update the indicator light
  // logic so they can check if they need to be changed
  home_indicator_light.tick();
  learn_indicator_light.tick();


  // Acknowledge the interrupt to clear the flag and wait for the next interrupt.
  TCC2->INTFLAG.reg = TCC_INTFLAG_MASK; // This is critical
}

/**
 * Check interval between calls.
 * This should be called right before every execution of the state machine
 */
void iteration_time_check() {
  last_iteration_delta = millis()-last_iteration_time;
  last_iteration_time = millis();
  if (last_iteration_delta >= ITERATION_TIME_ERROR_MS) {
    ConnectorUsb.Send("Last iteration of the state machine took more than `ITERATION_TIME_ERROR_MS` (");
    ConnectorUsb.Send(ITERATION_TIME_ERROR_MS);
    ConnectorUsb.Send("ms) to complete. This is likely a bug. Last iteration took ");
    ConnectorUsb.Send(last_iteration_delta);
    ConnectorUsb.Send("ms. Engaging E-Stop and stopping execution\n");
    machine_state = e_stop;
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
  CARRIAGE_MOTOR.MoveStopAbrupt();
  if (machine_state == wait_for_homing || machine_state == begin_homing) {
    // If it was homing the only way to stop it is by disabling the motor,
    // However disabling the motor will cause it to lose home and would therefore
    // be unrecoverable. This is fine for the homing sequence but would ruin a
    // part if it was mid sequence
    CARRIAGE_MOTOR.EnableRequest(false);
    is_homed = false;
    home_indicator_light.setPattern(LightPattern::OFF);
  }
  machine_state = e_stop; // TODO: Make this recoverable
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
 * Command step pulses to move the motor's current position to the absolute
 * position specified by "position"
 * Prints the move status to the USB serial port
 * Returns when HLFB asserts (indicating the motor has reached the commanded
 * position)
 *
 * @param position The absolute position, in step pulses, to move to
 *
 * @return bool: whether the move was successfully triggered.
 */
bool MoveAbsolutePosition(const int32_t position) {
    // Check if a motor alert is currently preventing motion
    // Clear alert if configured to do so 
    if (CARRIAGE_MOTOR.StatusReg().bit.AlertsPresent) {
        ConnectorUsb.SendLine("Motor alert detected.");       
        print_motor_alerts();
        ConnectorUsb.SendLine("Move canceled.");
        ConnectorUsb.SendLine();
        return false;
    }

    ConnectorUsb.Send("Moving to absolute position: ");
    ConnectorUsb.SendLine(position);

    // Command the move of absolute distance
    CARRIAGE_MOTOR.Move(position, MotorDriver::MOVE_TARGET_ABSOLUTE);

    // Waits for HLFB to assert (signaling the move has successfully completed)
    ConnectorUsb.SendLine("Moving.. Waiting for HLFB");
    while ( (!CARRIAGE_MOTOR.StepsComplete() || CARRIAGE_MOTOR.HlfbState() != MotorDriver::HLFB_ASSERTED) &&
            !CARRIAGE_MOTOR.StatusReg().bit.AlertsPresent) {}
    // Check if motor alert occurred during move
    // Clear alert if configured to do so
    if (CARRIAGE_MOTOR.StatusReg().bit.AlertsPresent) {
        ConnectorUsb.SendLine("Motor alert detected.");
        print_motor_alerts();
        ConnectorUsb.SendLine("Motion may not have completed as expected. Proceed with caution.");
        ConnectorUsb.SendLine();
        return false;
    }
    ConnectorUsb.SendLine("Move Done");
    return true;
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
