/*
  ModbusTCP Client for ClearCode Arduino wrapper

  (c)2021 Alexander Emelianov (a.m.emelianov@gmail.com)
  https://github.com/emelianov/modbus-esp8266

  This code is licensed under the BSD New License. See LICENSE.txt for more info.
*/

#include <Arduino.h>

#include "ClearCore.h"
#include "indicator_light.h"


bool is_homed = false;                  // Has the axis been homed
volatile bool is_e_stop;                // Is the Emergency Stop currently active

uint32_t last_iteration_delta;          // Time spent on the last iteration of the main loop in millis
uint32_t last_iteration_time;           // Time of the last iteration of the main loop in millis

enum PlanishStates {
  post,                   // Power On Self Test
  begin_homing,           // start axis homing
  wait_for_homing,        // Wait for homing to complete
  idle,                   // Idle await instructions
  manual_jog,             // manually commanded jog
  e_stop,                 // E-stop currently active
  error,                  // unrecoverable error has occurred and machine needs to reboot. should be a dead end state

  job_start,              // Job started, move to saved start pos
  job_planish,            // Start position reached, engage roller and move to end pos
  job_park,               // Planish finished, disengage roller and jog to park

  learn_start_pos,        // Learn has been pressed, set start point
  learn_jog_to_end_pos,   // Manually jogging to end position
  learn_end_pos,          // Learn was pressed again, set end position
  learn_jog_to_park_pos,  // Manually jogging to park position
  learn_park_pos,         // Learn was pressed again, set park position
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

#define ITERATION_TIME_WARNING_MS 100 // after this many milliseconds stuck on one iteration of the state machine, give a warning.
#define ITERATION_TIME_ERROR_MS 1000  // after this many milliseconds stuck on one iteration of the state machine, declare an error
#define SERIAL_ESTABLISH_TIMEOUT 10000 // Number of ms to wait for serial to establish before failing POST

/// Interrupt priority for the periodic interrupt. 0 is highest priority, 7 is lowest.
#define PERIODIC_INTERRUPT_PRIORITY 5

bool configure_io();
void e_stop_handler();
bool motor_movement_checks();
void print_motor_alerts();
void iteration_time_check();
void config_ccio_pin(ClearCorePins target_pin, Connector::ConnectorModes mode, bool initial_state = false);
void set_ccio_pin(ClearCorePins target_pin, bool state);
void ConfigurePeriodicInterrupt(uint32_t frequencyHz);

extern "C" void TCC2_0_Handler(void) __attribute__((
            alias("PeriodicInterrupt")));



void setup() {

  ESTOP_SW.InterruptHandlerSet(e_stop_handler, InputManager::InterruptTrigger::RISING);

  Serial.begin(115200);
  const uint32_t timeout = SERIAL_ESTABLISH_TIMEOUT;
  const uint32_t startTime = millis();
  while (!Serial && millis() - startTime < timeout)
    continue;

  if (!Serial) {
    machine_state = error;
    return;
  }

  // Debug delay to be able to restart motor before program starts
  uint8_t delay_cycles = 10;
  while (delay_cycles--) {
    delay(1000);
    Serial.println(delay_cycles);
  }

  last_iteration_time = millis();
  machine_state = post;
}


void loop() {

  // TODO: Check and latch input button states if high

  iteration_time_check();
  switch (machine_state) {
    case post:
      Serial.println("Power on self test");
      bool io_configure_result = configure_io();
      if (!io_configure_result) {
        machine_state = error;
        break;
      }
      machine_state = idle;
      break;
    case begin_homing:
      if (!is_e_stop) { // TODO: More checks
        Serial.println("begin homing");
        CARRIAGE_MOTOR.EnableRequest(true);
        machine_state = wait_for_homing;
      } else {
        Serial.println("unsafe to home");
        delay(10);
      }
      break;
    case wait_for_homing:
      if (CARRIAGE_MOTOR.HlfbState() == MotorDriver::HLFB_ASSERTED) {
        // homing done no errors
        Serial.println("homing complete");
        is_homed = true;
        machine_state = idle;
      } else if (CARRIAGE_MOTOR.StatusReg().bit.AlertsPresent) {
        // motor has an error
        Serial.println("Motor alert detected during homing.");
        print_motor_alerts();
        machine_state = error;
      } else {
        // still homing
      }
      break;
    case idle:
      if (HOME_SW.InputRisen()) {
        machine_state = begin_homing;
        break;
      }
      while (true) {
        Serial.println("ready in idle");
        delay(1000);
      }
      break;
    case manual_jog:
      break;
    case e_stop:
      while (true) {
        Serial.println("e-stop");
        delay(1000);
      }
      break;
    case error:
      while (true) {
        Serial.println("reached error state");
        delay(1000);
      }
      break;
    case job_start:
      break;
    case job_planish:
      break;
    case job_park:
      break;
    case learn_start_pos:
      break;
    case learn_jog_to_end_pos:
      break;
    case learn_end_pos:
      break;
    case learn_jog_to_park_pos:
      break;
    case learn_park_pos:
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


  ESTOP_SW.Mode(Connector::INPUT_DIGITAL);
  ESTOP_SW.InterruptHandlerSet(e_stop_handler);

  ESTOP_SW.FilterLength(16, DigitalIn::FILTER_UNIT_SAMPLES);

  CCIO1.Mode(Connector::CCIO);
  CCIO1.PortOpen();

  if (CcioMgr.LinkBroken()) {
    uint32_t lastStatusTime = Milliseconds();
    Serial.println("The CCIO-8 link is broken!");

    while (CcioMgr.LinkBroken() && Milliseconds() - lastStatusTime < CCIO_TIMEOUT_MS) {
      if (Milliseconds() - lastStatusTime > 1000) {
        Serial.println("The CCIO-8 link is still broken!");
        lastStatusTime = Milliseconds();
      }
    }
    if (CcioMgr.LinkBroken()) {
      Serial.println("Timed out waiting for CCIO");
      return false;
    }
    Serial.println("The CCIO-8 link is online again!");
  }

  if (CcioMgr.CcioCount() != EXPECTED_NUM_CCIO) {
    Serial.print("Expected to find exactly one CCIO connector, found ");
    Serial.println(CcioMgr.CcioCount());
    return false;
  }

  config_ccio_pin(FINGER_ACTUATION, Connector::OUTPUT_DIGITAL);
  config_ccio_pin(HEAD_ACTUATION, Connector::OUTPUT_DIGITAL);
  config_ccio_pin(HOME_SW_LIGHT, Connector::OUTPUT_DIGITAL);
  config_ccio_pin(LEARN_SW_LIGHT, Connector::OUTPUT_DIGITAL);

  home_indicator_light.setPin(CcioMgr.PinByIndex(HOME_SW_LIGHT));
  learn_indicator_light.setPin(CcioMgr.PinByIndex(LEARN_SW_LIGHT));

  MotorMgr.MotorInputClocking(MotorManager::CLOCK_RATE_NORMAL);
  MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL, Connector::CPM_MODE_STEP_AND_DIR);

  // Set the motor's HLFB mode to bipolar PWM
  CARRIAGE_MOTOR.HlfbMode(MotorDriver::HLFB_MODE_HAS_BIPOLAR_PWM);
  // Set the HFLB carrier frequency to 482 Hz
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
    Serial.println("Last iteration of the state machine took more than "
                   "`ITERATION_TIME_ERROR_MS` (" + String(ITERATION_TIME_ERROR_MS) + "ms) to complete. "
                   "This is likely a bug. Last iteration took " + String(last_iteration_delta) + "ms. "
                   "Engaging E-Stop and stopping execution");
    machine_state = e_stop;
  } else if (last_iteration_delta >= ITERATION_TIME_WARNING_MS) {
    Serial.println("Last iteration of the state machine took more than "
                   "`ITERATION_TIME_WARNING_MS` (" + String(ITERATION_TIME_WARNING_MS) + "ms) to complete. "
                   "This is likely a bug. Last iteration took " + String(last_iteration_delta) + "ms. "
                   "Continuing execution");
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
  machine_state = e_stop;
  CARRIAGE_MOTOR.MoveStopAbrupt();
  Serial.println("Emergency Stop");
}



/**
 * Prints active alerts.
 *
 * @pre requires "CARRIAGE_MOTOR" to be defined as a ClearCore motor connector
 */
void print_motor_alerts(){
  // report status of alerts
  Serial.println("ClearPath Alerts present: ");
  if(CARRIAGE_MOTOR.AlertReg().bit.MotionCanceledInAlert){
    Serial.println("    MotionCanceledInAlert "); }
  if(CARRIAGE_MOTOR.AlertReg().bit.MotionCanceledPositiveLimit){
    Serial.println("    MotionCanceledPositiveLimit "); }
  if(CARRIAGE_MOTOR.AlertReg().bit.MotionCanceledNegativeLimit){
    Serial.println("    MotionCanceledNegativeLimit "); }
  if(CARRIAGE_MOTOR.AlertReg().bit.MotionCanceledSensorEStop){
    Serial.println("    MotionCanceledSensorEStop "); }
  if(CARRIAGE_MOTOR.AlertReg().bit.MotionCanceledMotorDisabled){
    Serial.println("    MotionCanceledMotorDisabled "); }
  if(CARRIAGE_MOTOR.AlertReg().bit.MotorFaulted){
    Serial.println("    MotorFaulted ");
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
        Serial.println("Motor alert detected.");       
        print_motor_alerts();
        Serial.println("Move canceled.");
        Serial.println();
        return false;
    }

    Serial.print("Moving to absolute position: ");
    Serial.println(position);

    // Command the move of absolute distance
    CARRIAGE_MOTOR.Move(position, MotorDriver::MOVE_TARGET_ABSOLUTE);

    // Waits for HLFB to assert (signaling the move has successfully completed)
    Serial.println("Moving.. Waiting for HLFB");
    while ( (!CARRIAGE_MOTOR.StepsComplete() || CARRIAGE_MOTOR.HlfbState() != MotorDriver::HLFB_ASSERTED) &&
            !CARRIAGE_MOTOR.StatusReg().bit.AlertsPresent) {}
    // Check if motor alert occurred during move
    // Clear alert if configured to do so
    if (CARRIAGE_MOTOR.StatusReg().bit.AlertsPresent) {
        Serial.println("Motor alert detected.");
        print_motor_alerts();
        Serial.println("Motion may not have completed as expected. Proceed with caution.");
        Serial.println();
        return false;
    }
    Serial.println("Move Done");
    return true;
}

