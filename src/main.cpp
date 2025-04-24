/*
  ModbusTCP Client for ClearCode Arduino wrapper

  (c)2021 Alexander Emelianov (a.m.emelianov@gmail.com)
  https://github.com/emelianov/modbus-esp8266

  This code is licensed under the BSD New License. See LICENSE.txt for more info.
*/

#include <Arduino.h>

#include "ClearCore.h"


bool is_mandrel_latch_closed;           // Mandrel latch sensor reading. True for closed and safe
bool is_fingers_down;                   // Are the workpiece holding fingers commanded down
bool is_homed = false;                  // Has the axis been homed
bool is_fault;                          // Software fault detection. Indicates an unexpected condition
bool is_ready_for_cycle;                // Is the clearcore ready to execute its program
volatile bool is_e_stop;                // Is the Emergency Stop currently active
bool is_job_active;                     // Is the clearcore executing its program
bool is_ready_for_manual_control;       // Is the clearcore ready for manual control commands
bool is_roller_down;                    // Is the roller down/engaged
bool is_commanded_pos;                  // Has the HMI requested a new commanded position?
bool is_axis_homing_button_latched;     // 'Run axis homing sequence' button latch state
bool is_set_job_start_button_latched;   // 'Set start to current position' button latch state
bool is_set_job_end_button_latched;     // 'Set end to current position' button latch state
bool is_set_job_park_button_latched;    // 'Set park to current position' button latch state
bool commanded_fingers;                 // Finger state commanded by the HMI, true means engaged, false means disengaged
bool commanded_roller;                  // Roller state commanded by the HMI, true means engaged, false means disengaged

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


#define ESTOP_SW ConnectorDI6
#define CYCLE_START_SW ConnectorA11
#define CYCLE_START_LIGHT ConnectorIO2

// CCIO pin definitions. these are not interchangeable with native io ports.
// please see `configure_io()` to change assignment between ccio and native pins
#define FINGER_ACTUATION CCIOA6 // CCIO pin for finger actuation
#define HEAD_ACTUATION CCIOA4
#define HOME_BUTTON_LIGHT CCIOA0
#define LEARN_BUTTON_LIGHT CCIOA3
#define CARRIAGE_MOTOR ConnectorM0

#define CCIO1 ConnectorCOM1
#define EXPECTED_NUM_CCIO 1
#define CCIO_TIMEOUT_MS 10000 // Number of ms to wait for CCIO to connect before failing POST

#define CARRIAGE_MOTOR_MAX_ACCEL 50000
#define CARRIAGE_MOTOR_MAX_VEL 5000

#define ITERATION_TIME_WARNING_MS 100 // after this many milliseconds stuck on one iteration of the state machine, give a warning.
#define ITERATION_TIME_ERROR_MS 1000  // after this many milliseconds stuck on one iteration of the state machine, declare an error
#define SERIAL_ESTABLISH_TIMEOUT 10000 // Number of ms to wait for serial to establish before failing POST

bool configure_io();
void e_stop_handler();
bool motor_movement_checks();
void print_motor_alerts();
void iteration_time_check();
void config_ccio_pin(ClearCorePins target_pin, Connector::ConnectorModes mode, bool initial_state = false);
void set_ccio_pin(ClearCorePins target_pin, bool state);


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
      const bool io_configure_result = configure_io();
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
      if (CARRIAGE_MOTOR.HlfbState() != MotorDriver::HLFB_ASSERTED &&
            !CARRIAGE_MOTOR.StatusReg().bit.AlertsPresent)
      { // motor still homing, no errors
        delay(1);
      } else {
        if (CARRIAGE_MOTOR.StatusReg().bit.AlertsPresent) { // motor has an error
          Serial.println("Motor alert detected during homing.");
          print_motor_alerts();
          machine_state = error;
        } else { // homing done no errors
          Serial.println("homing complete");
          is_homed = true;
          machine_state = idle;
        }
      }
      break;
    case idle:
      if (is_axis_homing_button_latched) {
        machine_state = begin_homing;
        is_axis_homing_button_latched = false;
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

  // TODO: Unlatch input button states

}

/**
 * Configures IO, go figure
 * @return True for success
 */
bool configure_io() {
  CYCLE_START_SW.Mode(Connector::INPUT_DIGITAL);
  CYCLE_START_LIGHT.Mode(Connector::OUTPUT_DIGITAL);

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
  config_ccio_pin(HOME_BUTTON_LIGHT, Connector::OUTPUT_DIGITAL);
  config_ccio_pin(LEARN_BUTTON_LIGHT, Connector::OUTPUT_DIGITAL);

  MotorMgr.MotorInputClocking(MotorManager::CLOCK_RATE_NORMAL);
  MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL, Connector::CPM_MODE_STEP_AND_DIR);

  // Set the motor's HLFB mode to bipolar PWM
  CARRIAGE_MOTOR.HlfbMode(MotorDriver::HLFB_MODE_HAS_BIPOLAR_PWM);
  // Set the HFLB carrier frequency to 482 Hz
  CARRIAGE_MOTOR.HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);


  CARRIAGE_MOTOR.VelMax(CARRIAGE_MOTOR_MAX_VEL);
  CARRIAGE_MOTOR.AccelMax(CARRIAGE_MOTOR_MAX_ACCEL);
  return true;
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

bool home_axis() {
  CARRIAGE_MOTOR.EnableRequest(true);

  while (CARRIAGE_MOTOR.HlfbState() != MotorDriver::HLFB_ASSERTED &&
            !CARRIAGE_MOTOR.StatusReg().bit.AlertsPresent) {}

  if (CARRIAGE_MOTOR.StatusReg().bit.AlertsPresent) {
    Serial.println("Motor alert detected during homing.");
    print_motor_alerts();
    return false;
  }
  return true;
}



/**
 * helper function for motor safety checks
 * @return true if motor movement checks pass
 */
bool motor_movement_checks() {
  return !is_e_stop &&
    is_mandrel_latch_closed &&
    is_homed;
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
bool MoveAbsolutePosition(int32_t position) {
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

/**
 * TODO: Maybe make a light manger with an enum for blink periods/on/off
 * For a given blink period, get the state that the light should currently be at.
 * For a period of `n` ms, the light will be on for `n` ms, and off for `n` ms.
 * @param period the time in ms for the on time/off time of the light.
 * @return The calculated state the light should currently be at
 */
bool blink_state(const uint32_t period) {
  return (millis() % (2*period)) < period;
}