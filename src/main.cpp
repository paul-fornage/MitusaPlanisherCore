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
bool is_homed;                          // Has the axis been homed
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


uint16_t actual_position;         // Hreg R   actual position of the axis measured in hundredths of an inch
uint16_t commanded_position;      // Hreg R/W commanded position of the axis measured in hundredths of an inch;
                                    // will not be acted upon if is_commanded_pos is not set
uint16_t job_progress;            // Hreg R   progress of the current job: (job_progress / 65536) * 100%
uint16_t job_start_pos;           // Hreg R   saved start position of the axis measured in hundredths of an inch
uint16_t job_end_pos;             // Hreg R   saved end position of the axis measured in hundredths of an inch
uint16_t job_park_pos;            // Hreg R   saved park position of the axis measured in hundredths of an inch
uint16_t min_pos;                 // Hreg R   minimum absolute position of the axis measured in hundredths of an inch
uint16_t max_pos;                 // Hreg R   maximum absolute position of the axis measured in hundredths of an inch
uint16_t jog_speed;               // Hreg R/W target speed while jogging manually or during disengaged portions of the job sequence
uint16_t planish_speed;           // Hreg R/W target speed during engaged portions of the job sequence
uint16_t fault_code;              // Hreg R   fault code; 0 indicates normal operation
uint16_t heartbeat_in;            // Hreg W   Contains an arbitrary value set by the HMI
uint16_t heartbeat_out;           // Hreg R   This should always be equal to `(heartbeat_in*2)%65536`


enum PlanishStates {
  post,
  homing,
  idle,
  manual_jog,
  e_stop,

};


#define ESTOP_SW ConnectorA12 /// These should match \/
#define ESTOP_PIN CLEARCORE_PIN_A12 /// ^ should be set to the same semantic connector pin as ESTOP_SW
#define CYCLE_START_SW ConnectorA11
#define CYCLE_START_LIGHT ConnectorIO2
#define FINGER_ACTUATION ConnectorIO0
#define ROLLER_ACTUATION ConnectorIO1
#define CARRIAGE_MOTOR ConnectorM0

#define HMI_CONNECTION_TRIES_BEFORE_ERROR 5
#define CARRIAGE_MOTOR_MAX_ACCEL 50000
#define CARRIAGE_MOTOR_MAX_VEL 5000

void configure_io();
void e_stop_handler();
bool read_coils();
bool read_registers();
bool motor_movement_checks();
void print_motor_alerts();


void setup() {
  Serial.begin(115200);
  const uint32_t timeout = 5000;
  const uint32_t startTime = millis();
  while (!Serial && millis() - startTime < timeout)
    continue;

  ESTOP_SW.InterruptHandlerSet(e_stop_handler);


  // TODO: https://teknic-inc.github.io/ClearCore-library/_move_position_absolute_8cpp-example.html

  MotorMgr.MotorInputClocking(MotorManager::CLOCK_RATE_NORMAL);
  MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL, Connector::CPM_MODE_STEP_AND_DIR);

  // Set the motor's HLFB mode to bipolar PWM
  CARRIAGE_MOTOR.HlfbMode(MotorDriver::HLFB_MODE_HAS_BIPOLAR_PWM);
  // Set the HFLB carrier frequency to 482 Hz
  CARRIAGE_MOTOR.HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);


  CARRIAGE_MOTOR.VelMax(CARRIAGE_MOTOR_MAX_VEL);
  CARRIAGE_MOTOR.AccelMax(CARRIAGE_MOTOR_MAX_ACCEL);
}



void loop() {
  if (is_e_stop) {
    Serial.println("Emergency Stop");
  } else {
    Serial.println("Normal Operation");
  }

  delay(10);

}


void configure_io() {
  ESTOP_SW.Mode(Connector::INPUT_DIGITAL);
  CYCLE_START_SW.Mode(Connector::INPUT_DIGITAL);
  CYCLE_START_LIGHT.Mode(Connector::OUTPUT_DIGITAL);
  FINGER_ACTUATION.Mode(Connector::OUTPUT_DIGITAL);
  ROLLER_ACTUATION.Mode(Connector::OUTPUT_DIGITAL);
  CARRIAGE_MOTOR.EStopConnector(ESTOP_PIN); // TODO: https://teknic-inc.github.io/ClearCore-library/class_clear_core_1_1_motor_driver.html#a7294f84ac78e9ea31ad6cedab585ec03
}

void e_stop_handler() {
  is_e_stop = true;
  CARRIAGE_MOTOR.MoveStopAbrupt();
  Serial.println("Emergency Stop");
}




bool move_to_position(uint16_t new_position) {
  if (new_position > max_pos) {
    new_position = max_pos;
  } else if (new_position < min_pos) {
    new_position = min_pos;
  }
  if (new_position == actual_position) {
    return true;
  }
  commanded_position = new_position;
  if(motor_movement_checks()) {
    CARRIAGE_MOTOR.Move(commanded_position, StepGenerator::MOVE_TARGET_ABSOLUTE);
  }
}

/**
 *
 * @return true for success
 */
bool handle_axis_home_button() {
  is_homed = false;
  if (!is_e_stop &&
    !ESTOP_SW.State() && // Ideally this is redundant, but really would hate to miss an interrupt and fail to ESTOP
    is_mandrel_latch_closed) { // TODO: should mandrel latch need to be closed for motor movement?
    CARRIAGE_MOTOR.EnableRequest(false);

    CARRIAGE_MOTOR.EnableRequest(true);
    // TODO: Does homing go both ways? Can I change the direction it starts homing?

    while (CARRIAGE_MOTOR.HlfbState() != MotorDriver::HLFB_ASSERTED &&
            !CARRIAGE_MOTOR.StatusReg().bit.AlertsPresent) {}
    // Check if motor alert occurred during enabling
    // Clear alert if configured to do so
    if (CARRIAGE_MOTOR.StatusReg().bit.AlertsPresent) {
      Serial.println("Motor alert detected.");
      print_motor_alerts();
    } else {
      Serial.println("Motor Ready");
    }
    return true;
  }

  return false;
}

/**
 * helper function for motor safety checks
 * @return true if motor movement checks pass
 */
bool motor_movement_checks() {
  return !is_e_stop &&
    !ESTOP_SW.State() && // Ideally this is redundant, but really would hate to miss an interrupt and fail to ESTOP
    is_mandrel_latch_closed &&
    is_homed;
}


/*------------------------------------------------------------------------------
 * PrintAlerts
 *
 *    Prints active alerts.
 *
 * Parameters:
 *    requires "motor" to be defined as a ClearCore motor connector
 *
 * Returns: 
 *    none
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

/*------------------------------------------------------------------------------
 * MoveAbsolutePosition
 *
 *    Command step pulses to move the motor's current position to the absolute
 *    position specified by "position"
 *    Prints the move status to the USB serial port
 *    Returns when HLFB asserts (indicating the motor has reached the commanded
 *    position)
 *
 * Parameters:
 *    int position  - The absolute position, in step pulses, to move to
 *
 * Returns: True/False depending on whether the move was successfully triggered.
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
