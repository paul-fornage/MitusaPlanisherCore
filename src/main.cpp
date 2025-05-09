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
#include "actuator.h"
#include "FaultCodes.h"
#include <Ethernet.h>       // Ethernet library v2 is required
#include <ModbusAPI.h>
#include <ModbusTCPTemplate.h>
#include <utility>
#include "HmiReg.h"
#include "RegisterDefinitions.h"


// TODO: Bug when booting with e-stop on
// TODO: E-stop ISR
// TODO: https://piolabs.com/blog/insights/unit-testing-part-1.html#introduction, https://docs.platformio.org/en/latest/advanced/unit-testing/frameworks/doctest.html




// ModBus TCP stuff

// TODO: DHCP as option

class ModbusEthernet : public ModbusAPI<ModbusTCPTemplate<EthernetServer, EthernetClient>> {};
const IPAddress remote(192, 168, 1, 100);  // Address of Modbus Slave device
byte mac[] = { 0x24, 0x15, 0x10, 0xB0, 0x45, 0xA4 }; // MAC address is ignored but because of C++ types, you still need to give it garbage
IPAddress ip(192, 168, 0, 178); // The IP address will be dependent on your local network
ModbusEthernet mb;               // Declare ModbusTCP instance

bool is_HMI_comm_good = false;

Button HmiIsRthButton(false, false); // TODO (this is really just `HmiIsCommandedPosButton` but with position set at 0)
Button HmiIsAxisHomingButton(false, false);
Button HmiIsSetJobStartButton(false, false);
Button HmiIsSetJobEndButton(false, false);
Button HmiIsSetJobParkButton(false, false);
Button HmiCommitJobButton(false, false);
Button HmiIsCommandedFingersUpButton(false, false);
Button HmiIsCommandedFingersDownButton(false, false);
Button HmiIsCommandedRollerUpButton(false, false);
Button HmiIsCommandedRollerDownButton(false, false);
Button HmiIsCommandedPosButton(false, false); // TODO

HmiReg<bool> hmi_is_rth_state(CoilAddr::IS_RTH_BUTTON_LATCHED, false);
HmiReg<bool> hmi_is_axis_homing_state(CoilAddr::IS_AXIS_HOMING_BUTTON_LATCHED, false);
HmiReg<bool> hmi_is_set_job_start_state(CoilAddr::IS_SET_JOB_START_BUTTON_LATCHED, false);
HmiReg<bool> hmi_is_set_job_end_state(CoilAddr::IS_SET_JOB_END_BUTTON_LATCHED, false);
HmiReg<bool> hmi_is_set_job_park_state(CoilAddr::IS_SET_JOB_PARK_BUTTON_LATCHED, false);
HmiReg<bool> hmi_commit_job_state(CoilAddr::IS_COMMIT_JOB_BUTTON_LATCHED, false);
HmiReg<bool> hmi_finger_up_state(CoilAddr::IS_FINGER_UP_LATCHED, false);
HmiReg<bool> hmi_finger_down_state(CoilAddr::IS_FINGER_DOWN_LATCHED, false);
HmiReg<bool> hmi_roller_up_state(CoilAddr::IS_ROLLER_UP_LATCHED, false);
HmiReg<bool> hmi_roller_down_state(CoilAddr::IS_ROLLER_DOWN_LATCHED, false);
HmiReg<bool> hmi_is_commanded_pos_state(CoilAddr::IS_COMMANDED_POS_LATCHED, false);

HmiReg<uint16_t> HmiCommandedPosition(HregAddr::HMI_COMMANDED_POSITION_REG_ADDR, 0);

auto fault_code = FaultCodes::None;

bool is_homed = false;                  // Has the axis been homed
volatile bool is_e_stop = false;               // Is the Emergency Stop currently active

bool io_configured = false;                  // Has the IO been configured
bool is_mandrel_safe = false;

uint32_t last_modbus_print = 0;

int32_t current_jog_speed = 0;              // Current speed the motor should use for jogging. Steps/second
int32_t current_planish_speed = 0;          // Current speed the motor should use for planishing. Steps/second

uint32_t last_iteration_delta;          // Time spent on the last iteration of the main loop in millis
uint32_t last_iteration_time;           // Time of the last iteration of the main loop in millis

uint32_t loop_num = 0;                  // Number of times the main loop has been called. WILL OVERFLOW
uint32_t last_estop_millis = 0;         // Millis() value at time the last e-stop was pressed

uint32_t homing_disable_time = 0;        // Time the motor was disabled for homing in millis

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
  NONE,               // Should not happen
  button,             // E-Stop connector read high, could be a button or a laser switch or something
  mandrel_latch,      // An operation was in progress that needed the mandrel latch to be down,
                      // but it was opened while the operation was in progress
  internal_error,     // The code reached one of those 'this should not be possible' comments. Require reboot.
  motor_error,        // The motor has an error, this can only be rectified by re-homing
};

enum class PlanishState {
  post,                       // Power On Self Test
  begin_homing,               // start axis homing
  homing_wait_for_disable,    // during homing the motor needs to wait between disabling and re-enabling
  wait_for_homing,            // Wait for homing to complete
  idle,                       // Idle await instructions
  manual_jog,                 // manually commanded jog
  e_stop_begin,               // E-stop was activated, secure system and save current config to be resumed
  e_stop_wait,                // E-stop is active, wait for it to end
  error,                      // unrecoverable error has occurred and machine needs to reboot. should be a dead end state
  wait_for_head,              // Me asf. This is for when the head position is changed from the idle state, not for jobs
  wait_for_fingers,           // This is for when the finger position is changed from the idle state, not for jobs

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

volatile PlanishState estop_last_state;  // When an estop is called, this gets set to whatever the current state was.
volatile PlanishState estop_resume_state;      // When an estop is resolved, this stores where to resume
volatile PlanishState machine_state;     // the current state of the machine. Volatile because it can be changed by estop ISR

volatile IndicatorLight home_indicator_light;   /// Defined globally, but should not be accessed unless configure_IO has completed without errors
volatile IndicatorLight learn_indicator_light;  /// Defined globally, but should not be accessed unless configure_IO has completed without errors

volatile EstopReason estop_reason = EstopReason::NONE;

/**
 * when defined the motor will not move and
 * any commands meant for the motor will be logged/simulated
 */
// #define TEST_MODE_DISABLE_MOTOR


#ifdef TEST_MODE_DISABLE_MOTOR
// Wrapper for commands to the motor that will print them instead if motor is disabled for test mode
#define MOTOR_COMMAND(code) ConnectorUsb.SendLine(#code)
#define MOTOR_ASSERTED true
#define MOTOR_HAS_ERRORS false
#define MOTOR_ERROR_COMMANDED_WHEN_DISABLED false
#define MOTOR_STEPS_COMPLETE true
#define MOTOR_COMMANDED_POSITION 0
#else
// Wrapper for commands to the motor that will print them instead if motor is disabled for test mode
#define MOTOR_COMMAND(code) code
#define MOTOR_ASSERTED (CARRIAGE_MOTOR.HlfbState() == MotorDriver::HLFB_ASSERTED)
#define MOTOR_HAS_ERRORS (CARRIAGE_MOTOR.StatusReg().bit.AlertsPresent)
#define MOTOR_ERROR_COMMANDED_WHEN_DISABLED (CARRIAGE_MOTOR.AlertReg().bit.MotionCanceledMotorDisabled)
#define MOTOR_STEPS_COMPLETE CARRIAGE_MOTOR.StepsComplete()
#define MOTOR_COMMANDED_POSITION CARRIAGE_MOTOR.PositionRefCommanded()

#define CARRIAGE_MOTOR ConnectorM0
#endif

#define ESTOP_SW ConnectorA12
#define FINGER_DOWN_LMT ConnectorA11
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
#define CARRIAGE_MOTOR_MAX_VEL 10000
// not a safety limit, just what should the minimum position on the speed selector represent in jog mode
#define CARRIAGE_MOTOR_MIN_VEL 100
#define CARRIAGE_MOTOR_MAX_POS 12000
#define CARRIAGE_MOTOR_MAX_PLANISH_VEL 4000
// not a safety limit, just what should the minimum position on the speed selector represent in planish mode
#define CARRIAGE_MOTOR_MIN_PLANISH_VEL 50

#define ITERATION_TIME_WARNING_MS 50 // after this many milliseconds stuck on one iteration of the state machine, give a warning.
#define ITERATION_TIME_ERROR_MS 100  // after this many milliseconds stuck on one iteration of the state machine, declare an error
#define SERIAL_ESTABLISH_TIMEOUT 5000 // Number of ms to wait for serial to establish before failing POST
#define HMI_CONNECTION_TRIES_BEFORE_ERROR 5

/// Interrupt priority for the periodic interrupt. 0 is highest priority, 7 is lowest.
#define PERIODIC_INTERRUPT_PRIORITY 5

#define ESTOP_COOLDOWN_MS 1000
#define ESTOP_SW_SAFE_STATE 1
#define MANDREL_LATCH_LMT_SAFE_STATE 1 // MANDREL_LATCH_LMT.State() should return this when it's safe/down/engaged

#define JOG_FWD_SW_ACTIVE_STATE 0
#define JOG_REV_SW_ACTIVE_STATE 0

#define ADC_RES_BITS 12
#define ADC_MAX_VALUE ((1 << ADC_RES_BITS)-1)

#define MOTOR_EN_DIS_DELAY_MS 10    // during homing the motor needs to be disabled for this many ms to actually do it

#define STATE_MACHINE_LOOPS_POT_UPDATE_INTERVAL 32 // number of loops of the main state machine between speed pot updates
#define STATE_MACHINE_LOOPS_HMI_UPDATE_INTERVAL 32 // number of loops of the main state machine between Modbus Hmi updates
#define STATE_MACHINE_LOOPS_LOG_INTERVAL 1024 // number of loops of the main state machine between
// logging from 'wait' states. Too low will flood the logs

#define PERIODIC_PRINT(statements) if (loop_num%STATE_MACHINE_LOOPS_LOG_INTERVAL==0) { statements }

// TODO: Verify this WAGNER
#define STEPS_PER_REV 800
#define GEARBOX_RATIO 100.0 // number of input revs for one output revs
#define RACK_TEETH_PER_INCH 4.0
#define PINION_TEETH_PER_REV 24 // teeth on the pinion gear that interfaces with the rack


Button LearnButton;
Button HomeButton;
Button CycleButton;
Button FingerButton;
Button HeadButton;

SensedActuator Fingers;
SensedActuator Head;

bool configure_io();
void e_stop_button_handler();
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
bool move_motor_with_speed(int32_t position, int32_t speed);
bool move_motor_auto_speed(int32_t position);
void motor_jog(bool reverse);
void update_buttons();
void update_speed_pot();
void e_stop_handler(EstopReason reason);
PlanishState secure_system(PlanishState last_state);
PlanishState state_machine(PlanishState state_in);
const char *get_estop_reason_name(EstopReason state);
void check_modbus();
std::pair<uint16_t, bool> job_progress(PlanishState state_to_check);
void mb_read_hreg(HmiReg<uint16_t> *reg);
void mb_read_coil(HmiReg<bool> *reg);
void mb_write_coil(uint16_t address, bool val);
void mb_write_hreg(uint16_t address, uint16_t val);
double steps_to_f64_inch(uint32_t steps);
uint16_t steps_to_hundreths(uint32_t steps);
uint16_t steps_per_sec_to_inches_per_minute(uint32_t steps_per_second);
const char *get_state_name(PlanishState state);
void update_modbus_buttons();

extern "C" void TCC2_0_Handler(void) __attribute__((
            alias("PeriodicInterrupt")));




void setup() {

  ESTOP_SW.Mode(Connector::INPUT_DIGITAL);
//  ESTOP_SW.FilterLength(5, DigitalIn::FILTER_UNIT_SAMPLES);

  ConnectorUsb.PortOpen();
  const uint32_t startTime = millis();
  while (!ConnectorUsb && ((millis() - startTime) < SERIAL_ESTABLISH_TIMEOUT))
    continue;

  if (!ConnectorUsb) {
//    e_stop_handler(EstopReason::internal_error);
//    return;
  }

  // Debug delay to be able to restart motor before program starts
//  delay(2000);

  if (!Ethernet.begin(mac)) {
    ConnectorUsb.SendLine("DHCP failed, using static IP");
  } else {
    ConnectorUsb.Send("DHCP gives IP: ");
    const auto temp_ip = Ethernet.localIP();
    ConnectorUsb.Send(temp_ip[0]);
    ConnectorUsb.SendChar('.');
    ConnectorUsb.Send(temp_ip[1]);
    ConnectorUsb.SendChar('.');
    ConnectorUsb.Send(temp_ip[2]);
    ConnectorUsb.SendChar('.');
    ConnectorUsb.SendLine(temp_ip[3]);
  }



  read_job_from_nvram();

  ConnectorUsb.SendLine("job retrieved from NVRAM");
  ConnectorUsb.SendLine(saved_job_start_pos);
  ConnectorUsb.SendLine(saved_job_end_pos);
  ConnectorUsb.SendLine(saved_job_park_pos);

  io_configured = configure_io();

  if (!io_configured) {           // if the IO failed to configure properly
    ConnectorUsb.SendLine("IO was not configured successfully, hanging");
    e_stop_handler(EstopReason::internal_error);
    return;
  }
  // do this twice so that the buffer only contains measured values
  update_buttons();
  update_buttons();

  last_iteration_time = millis();
  machine_state = PlanishState::post;

//  ESTOP_SW.InterruptHandlerSet(e_stop_button_handler, InputManager::InterruptTrigger::CHANGE);



}


void loop() {
  is_mandrel_safe = MANDREL_LATCH_LMT.State() == MANDREL_LATCH_LMT_SAFE_STATE;

  if (ESTOP_SW.State() != ESTOP_SW_SAFE_STATE) { // make sure e-stop wasn't already depressed before interrupt was registered
    ConnectorUsb.SendLine("E-Stop was triggered by button from the main loop check");
    e_stop_handler(EstopReason::button);
  }

  // If the ESTOP ISR is called after a state change condition is met,
  // but before the state change, the `e_stop_begin` set in the ISR will be overwritten.
  // Fix by checking at the beginning of every cycle
  if (is_e_stop) {
    // Also make sure that the state wasn't already set to something more severe
    if (machine_state != PlanishState::e_stop_wait && machine_state != PlanishState::error) {
      machine_state = PlanishState::e_stop_begin;
      MOTOR_COMMAND(CARRIAGE_MOTOR.MoveStopAbrupt(););
    }
  }
  loop_num++;
  for (uint8_t i=0; i<32; i++){
    mb.task(); // modbus tick
  }
  last_iteration_time = millis();

  PERIODIC_PRINT(
    if (!is_HMI_comm_good) {
      ConnectorUsb.SendLine("HMI communication not established");
    }
  )

  // state machine state machine machine state
  // Run one instance of the state machine
  machine_state = state_machine(machine_state);

  // both of these calls rely on io being configured but it
  update_buttons();
  if (loop_num%STATE_MACHINE_LOOPS_POT_UPDATE_INTERVAL==0) {
    update_speed_pot();
  }
  if (loop_num%STATE_MACHINE_LOOPS_HMI_UPDATE_INTERVAL==0) {
    check_modbus();
  }
}
// TODO: Probably too many mb.task() calls
/**
 * Checks modbus
 * @pre IO configured successfully
 */
void check_modbus() {
  if (mb.isConnected(remote)) {   // Check if connection to Modbus Slave is established
    is_HMI_comm_good = true;
    mb_write_coil(CoilAddr::IS_MANDREL_LATCH_CLOSED, is_mandrel_safe);  // Initiate Read Hreg from Modbus Slave
    mb_write_coil(CoilAddr::IS_FINGERS_DOWN, Fingers.get_measured_state());
    mb_write_coil(CoilAddr::IS_HOMED, is_homed);
    mb_write_coil(CoilAddr::IS_FAULT, machine_state==PlanishState::error);
    // TODO: Abstract those lists of conditions
    mb_write_coil(CoilAddr::IS_READY_FOR_CYCLE, Fingers.is_fully_engaged() && is_homed && is_mandrel_safe && machine_state == PlanishState::idle);
    mb_write_coil(CoilAddr::IS_E_STOP, is_e_stop);
    mb_write_coil(CoilAddr::IS_JOB_ACTIVE, job_progress(machine_state).first);
    mb_write_coil(CoilAddr::IS_READY_FOR_MANUAL_CONTROL, is_mandrel_safe && is_homed && machine_state == PlanishState::idle);
    mb_write_coil(CoilAddr::IS_ROLLER_DOWN, Head.get_measured_state());
    mb_write_coil(CoilAddr::CC_COMMANDED_FINGERS, Fingers.get_commanded_state());
    mb_write_coil(CoilAddr::CC_COMMANDED_ROLLER, Head.get_commanded_state());

    if (millis() - last_modbus_print > 1000) {
      last_modbus_print = millis();
    }
    update_modbus_buttons();

    mb_write_hreg(HregAddr::CC_COMMANDED_POSITION_REG_ADDR, steps_to_hundreths(CARRIAGE_MOTOR.PositionRefCommanded()));
    mb_write_hreg(HregAddr::JOB_PROGRESS_REG_ADDR, job_progress(machine_state).second);
    mb_write_hreg(HregAddr::JOB_START_POS_REG_ADDR, steps_to_hundreths(saved_job_start_pos));
    mb_write_hreg(HregAddr::JOB_END_POS_REG_ADDR, steps_to_hundreths(saved_job_end_pos));
    mb_write_hreg(HregAddr::JOB_PARK_POS_REG_ADDR, steps_to_hundreths(saved_job_park_pos));
    mb_write_hreg(HregAddr::JOG_SPEED_REG_ADDR, steps_per_sec_to_inches_per_minute(current_jog_speed));
    mb_write_hreg(HregAddr::PLANISH_SPEED_REG_ADDR, steps_per_sec_to_inches_per_minute(current_planish_speed));
    mb_write_hreg(HregAddr::FAULT_CODE_REG_ADDR, fault_code);
    mb_read_hreg(&HmiCommandedPosition);
    
  } else {
    is_HMI_comm_good = false;
    mb.connect(remote);           // Try to connect if not connected
  }
}

void mb_read_hreg(HmiReg<uint16_t> *reg) {
  mb.readHreg(remote, reg->address, &reg->value);
}

void mb_read_unlatch_coil(HmiReg<bool> *reg) {
  mb_read_coil(reg);
  mb_write_coil(reg->address, false);
}

void mb_read_coil(HmiReg<bool> *reg) {
  mb.readCoil(remote, reg->address, &reg->value);
}

void mb_write_coil(const uint16_t address, const bool val) {
  mb.writeCoil(remote, address, val);
}

void mb_write_hreg(const uint16_t address, const uint16_t val) {
  mb.writeCoil(remote, address, val);
}

void update_modbus_buttons() {
  mb_read_unlatch_coil(&hmi_is_rth_state);
  mb_read_unlatch_coil(&hmi_is_axis_homing_state);
  mb_read_unlatch_coil(&hmi_is_set_job_start_state);
  mb_read_unlatch_coil(&hmi_is_set_job_end_state);
  mb_read_unlatch_coil(&hmi_is_set_job_park_state);
  mb_read_unlatch_coil(&hmi_finger_up_state);
  mb_read_unlatch_coil(&hmi_finger_down_state);
  mb_read_unlatch_coil(&hmi_roller_up_state);
  mb_read_unlatch_coil(&hmi_roller_down_state);
  mb_read_unlatch_coil(&hmi_is_commanded_pos_state);
}

/**
 * Get the progress of the current job, if there is one
 * @param state_to_check current machine state
 * @return Pair(progress 0..2^16, is there even a job going on)
 */
std::pair<uint16_t, bool> job_progress(const PlanishState state_to_check) {
  uint16_t progress = 0;
  switch (state_to_check) {
    case PlanishState::job_begin:
    case PlanishState::job_begin_lifting_head:
      progress = 2;
      break;
    case PlanishState::job_jog_to_start:
    case PlanishState::job_jog_to_start_wait:
      progress = 4;
      break;
    case PlanishState::job_head_down:
    case PlanishState::job_head_down_wait:
      progress = 6;
      break;
    case PlanishState::job_planish_to_end:
    case PlanishState::job_planish_to_end_wait:
      progress = 8;
      break;
    case PlanishState::job_planish_to_start:
    case PlanishState::job_planish_to_start_wait:
      progress = 10;
      break;
    case PlanishState::job_head_up:
    case PlanishState::job_head_up_wait:
      progress = 12;
      break;
    case PlanishState::job_jog_to_park:
    case PlanishState::job_jog_to_park_wait:
      progress = 15;
      break;
    default:
      return std::make_pair(0, false);
  }
  return std::make_pair(progress << 12, true);
}

PlanishState state_machine(const PlanishState state_in) {
  switch (state_in) {
    case PlanishState::post:  // Not used anymore but could be in the future
      PERIODIC_PRINT(ConnectorUsb.SendLine("post state"););
      is_HMI_comm_good = false;
      if (Ethernet.linkStatus() == LinkOFF) {
        PERIODIC_PRINT(ConnectorUsb.SendLine("Eth link off"););
        return PlanishState::post;
      }
      mb.client();
      if(!mb.isConnected(remote)) {
        mb.connect(remote);
        ConnectorUsb.SendChar('.');
        return PlanishState::post;
      }
      is_HMI_comm_good = true;
      ConnectorUsb.SendLine("POSTed up. HMI seems connected");
      return PlanishState::idle;

    case PlanishState::begin_homing:
      ConnectorUsb.SendLine("begin homing");

      MOTOR_COMMAND(CARRIAGE_MOTOR.EnableRequest(false));
      is_homed = false;

      homing_disable_time = millis();
      return PlanishState::homing_wait_for_disable;

    case PlanishState::homing_wait_for_disable:
      if (millis() - homing_disable_time > MOTOR_EN_DIS_DELAY_MS) {
        MOTOR_COMMAND(CARRIAGE_MOTOR.EnableRequest(true););
        home_indicator_light.setPattern(LightPattern::BLINK);
        return PlanishState::wait_for_homing;
      }
      return PlanishState::homing_wait_for_disable;

    case PlanishState::wait_for_homing:
      if (!is_mandrel_safe) {
        e_stop_handler(EstopReason::mandrel_latch);
        return PlanishState::e_stop_begin;
      }

      if (MOTOR_ASSERTED) {
        // homing done no errors
        ConnectorUsb.SendLine("homing complete");
        // the position saved in the local motor instance is not neccesarily the same of the motor FW.
        // this line ensures they are set to the same thing, but when there seems to be a rounding error between the two
        // can confirm they don't desync over the course of at least ~20,000 steps
        MOTOR_COMMAND(CARRIAGE_MOTOR.PositionRefSet(0););
        is_homed = true;
        home_indicator_light.setPattern(LightPattern::ON);
        return PlanishState::idle;
      }

      if (MOTOR_HAS_ERRORS) {
        if(MOTOR_ERROR_COMMANDED_WHEN_DISABLED) {
          ConnectorUsb.SendLine("Motor was commanded to move before enabling."
                                "depending on how the code turns out this might not be problematic");
        }
        ConnectorUsb.SendLine("Motor alert detected during homing.");
        print_motor_alerts();
        e_stop_handler(EstopReason::motor_error);
      }
      return PlanishState::wait_for_homing;

    case PlanishState::idle:
      if (HomeButton.is_rising() || HmiIsAxisHomingButton.is_rising()) {
        if (is_mandrel_safe) {
          if (Head.is_fully_disengaged()) {
            return PlanishState::begin_homing;
          } else {
            ConnectorUsb.SendLine("Tried to home while head engaged");
          }
        } else {
          ConnectorUsb.SendLine("Tried to home without mandrel latch engaged");
        }
      }

      if (FingerButton.is_rising()
          || HmiIsCommandedFingersUpButton.is_rising()
          || HmiIsCommandedFingersDownButton.is_rising())
        {
        bool new_finger_state; // < doesn't necessarily get acted on or set.
        if(HmiIsCommandedFingersUpButton.is_rising()) {
          new_finger_state = true;
        } else if (HmiIsCommandedFingersDownButton.is_rising()) {
          new_finger_state = false;
        } else {
          new_finger_state = !Fingers.get_commanded_state();
        }
        if (is_mandrel_safe || !new_finger_state) {
          if(Head.is_fully_disengaged() || new_finger_state) {
            // make sure the head is up and not engaged, and also not in the process of lowering.
            // OR the user is trying to engage the fingers because the head somehow was already down
            ConnectorUsb.SendLine(new_finger_state ? "Engaging fingers" : "Disengaging fingers");
            Fingers.set_commanded_state(new_finger_state);
            return PlanishState::wait_for_fingers;
          } else {
            ConnectorUsb.SendLine("Tried to disengage fingers while head was down or lowering");
          }
        } else {
          ConnectorUsb.SendLine("tried to engage fingers without mandrel latch engaged");
        }
      }

      if (HeadButton.is_changing()
          || HmiIsCommandedRollerUpButton.is_rising()
          || HmiIsCommandedRollerDownButton.is_rising())
        {
        // Although the head switch is not momentary, changes should only be made when the user commands it, to avoid
        // unexpected moves when relinquishing head control to user after a job

        bool new_head_state; // < doesn't necessarily get acted on or set.
        if (HmiIsCommandedRollerUpButton.is_rising()) {
          new_head_state = true;
        } else if (HmiIsCommandedRollerDownButton.is_rising()) {
          new_head_state = false;
        } else {
          new_head_state = HeadButton.get_current_state();
        }

        if((is_mandrel_safe && Fingers.is_fully_engaged()) || !new_head_state) {
          // if safety checks for lowering head pass, or the user is trying to raise the head
          Head.set_commanded_state(new_head_state);
          return PlanishState::wait_for_head;
        } else {
          ConnectorUsb.SendLine("Tried to lower head while fingers not down or mandrel latch not engaged");
          return PlanishState::idle;
        }
      }

      if (LearnButton.is_rising()) {
        if (is_mandrel_safe) {
          if (Head.is_fully_disengaged()) {
            if (is_homed) {
              return PlanishState::learn_start_pos;
            } else {
              ConnectorUsb.SendLine("Tried to start learn without homed");
            }
          } else {
            ConnectorUsb.SendLine("Tried to learn job while the head was not fully down");
          }
        } else {
          ConnectorUsb.SendLine("Tried to learn job without mandrel being in safe position");
        }
      }

      if (JOG_FWD_SW.State() == JOG_FWD_SW_ACTIVE_STATE
          || JOG_REV_SW.State() == JOG_REV_SW_ACTIVE_STATE) {
        if (is_mandrel_safe) {
          if (is_homed) {
            // This is weird, while there is nothing that depends on the motor's absolute position for this,
            // it needs to be enabled, and enabling is what starts the homing process.
            // in this sense, `is_homed` is just checking if the motor is enabled,
            // because otherwise it will put the motor FW in an error state
            return PlanishState::manual_jog;
          } else {
            ConnectorUsb.SendLine("Tried to start jog without being homed");
          }
        } else {
          ConnectorUsb.SendLine("Tried to jog without mandrel switch engaged");
        }
      }

      if (HmiIsSetJobStartButton.is_rising()) {
        if (MOTOR_ASSERTED && MOTOR_STEPS_COMPLETE) {
          temp_job_start_pos = CARRIAGE_MOTOR.PositionRefCommanded();
          ConnectorUsb.SendLine("saved start position");
        } else {
          ConnectorUsb.SendLine("HMI tried to save current position as job start, but the carriage is still in motion!");
        }
      }
      if (HmiIsSetJobEndButton.is_rising()) {
        if (MOTOR_ASSERTED && MOTOR_STEPS_COMPLETE) {
          temp_job_end_pos = CARRIAGE_MOTOR.PositionRefCommanded();
          ConnectorUsb.SendLine("saved end position");
        } else {
          ConnectorUsb.SendLine("HMI tried to save current position as job end, but the carriage is still in motion!");
        }
      }
      if (HmiIsSetJobParkButton.is_rising()) {
        if (MOTOR_ASSERTED && MOTOR_STEPS_COMPLETE) {
          temp_job_park_pos = CARRIAGE_MOTOR.PositionRefCommanded();
          ConnectorUsb.SendLine("saved park position");
        } else {
          ConnectorUsb.SendLine("HMI tried to save current position as job park, but the carriage is still in motion!");
        }
      }
      if (HmiCommitJobButton.is_rising()) {
        save_job_to_nvram();
        ConnectorUsb.SendLine("Saved job to NVRAM and memory");
      }

      if (CycleButton.is_rising()) {
        if (Fingers.is_fully_engaged() && is_homed && is_mandrel_safe) {
          return PlanishState::job_begin;
        } else if (!Fingers.is_fully_engaged()) {
          ConnectorUsb.SendLine("Tried to start cycle without fingers engaged");
        } else if (!is_homed){
          ConnectorUsb.SendLine("Tried to start cycle without being homed");
        } else {
          ConnectorUsb.SendLine("Tried to start cycle without Mandrel latch being engaged");
        }
      }
      return PlanishState::idle;

    case PlanishState::wait_for_head:
      if (loop_num%STATE_MACHINE_LOOPS_LOG_INTERVAL==0) {
        ConnectorUsb.Send("Waiting for head. Currently commanded ");
        ConnectorUsb.SendLine(Head.get_commanded_state() ? "down." : "up.");
      }
      if (!is_mandrel_safe && Head.get_commanded_state()) {
        // if the mandrel latch is not engaged and the head is being lowered
        e_stop_handler(EstopReason::mandrel_latch);
        return PlanishState::e_stop_begin;
      }

      // if head was changed from manual control, they should be able to 'undo' it without waiting for completion
      if (HeadButton.is_changing()) {
        const bool new_head_state = HeadButton.get_current_state(); // < doesn't necessarily get acted on or set.
        if(Fingers.is_fully_engaged() || !new_head_state) {
          // Make sure the fingers are down or the user is trying to raise the head.
          // Basically don't let them lower it without fingers engaged
          Head.set_commanded_state(new_head_state);
          return PlanishState::wait_for_head;
        } else {
          ConnectorUsb.SendLine("Tried to lower head while fingers not down");
          return PlanishState::idle;
        }
      }
      if (!Head.is_mismatch()) {
        return PlanishState::idle;
      }
      return PlanishState::wait_for_head;

    case PlanishState::wait_for_fingers:

      if (loop_num%STATE_MACHINE_LOOPS_LOG_INTERVAL==0) {
        ConnectorUsb.Send("Waiting for fingers. Currently commanded ");
        ConnectorUsb.SendLine(Fingers.get_commanded_state() ? "down." : "up.");
      }
      if (!is_mandrel_safe) {
        e_stop_handler(EstopReason::mandrel_latch);
        return PlanishState::e_stop_begin;
      }

      if (FingerButton.is_rising()) { // still let the user change the finger state even if in motion
        const bool new_finger_state = !Fingers.get_commanded_state(); // < doesn't necessarily get acted on or set.
        ConnectorUsb.SendLine(new_finger_state ? "Engaging fingers" : "Disengaging fingers");
        Fingers.set_commanded_state(new_finger_state);
        return PlanishState::wait_for_fingers;
      }

      if (!Fingers.is_mismatch()) {
        return PlanishState::idle;
      }
      return PlanishState::wait_for_fingers;

    case PlanishState::manual_jog:
      if (is_mandrel_safe && is_homed) {
        if (JOG_FWD_SW.State() == JOG_FWD_SW_ACTIVE_STATE) {
          motor_jog(false);
          return PlanishState::manual_jog;
        }
        if (JOG_REV_SW.State() == JOG_REV_SW_ACTIVE_STATE) {
          motor_jog(true);
          return PlanishState::manual_jog;
        }
      }

      // if any of the following conditions are met;
      // 1. the motor is not homed
      // 2. the mandrel limit switch is not in the safe position
      // 3. neither the forward nor reverse jog is being commanded
      MOTOR_COMMAND(CARRIAGE_MOTOR.MoveStopDecel(););

      return PlanishState::idle;

    case PlanishState::e_stop_begin:
      estop_resume_state = secure_system(estop_last_state);
      ConnectorUsb.Send("secure_system returns");
      ConnectorUsb.Send(get_state_name(estop_resume_state));
      return PlanishState::e_stop_wait;

    case PlanishState::e_stop_wait:
      switch (estop_reason) {
        case EstopReason::NONE:
          ConnectorUsb.SendLine("E-Stop was triggered without setting the reason, changing to error");
          estop_reason = EstopReason::internal_error;
          return PlanishState::e_stop_wait;
        case EstopReason::button:
          if ((ESTOP_SW.State() == ESTOP_SW_SAFE_STATE)
              && (millis() - last_estop_millis > ESTOP_COOLDOWN_MS))
          { // if the button was released and the estop has been active for over a second
            ConnectorUsb.SendLine("Emergency Stop Released, resuming");
            is_e_stop = false;
            return estop_resume_state;
          }
          return PlanishState::e_stop_wait;
        case EstopReason::mandrel_latch:
          if ((ESTOP_SW.State() == ESTOP_SW_SAFE_STATE)
              && (millis() - last_estop_millis > ESTOP_COOLDOWN_MS)
              && (is_mandrel_safe))
          {
            // if the latch was fixed and the button hasn't been pressed,
            // and the estop was activated more than `ESTOP_COOLDOWN_MS` ago
            ConnectorUsb.SendLine("Mandrel put back, resuming");
            is_e_stop = false;
            return estop_resume_state;
          }
          return PlanishState::e_stop_wait;
        case EstopReason::internal_error:
          // unrecoverable by design
          home_indicator_light.setPattern(LightPattern::STROBE);
          learn_indicator_light.setPattern(LightPattern::STROBE);
          return PlanishState::error;
        case EstopReason::motor_error:
          if ((ESTOP_SW.State() == ESTOP_SW_SAFE_STATE)
              && (millis() - last_estop_millis > ESTOP_COOLDOWN_MS)
              && (is_mandrel_safe)
              && HomeButton.is_rising())
          {
            // if the latch was fixed and the button hasn't been pressed,
            // and the estop was activated more than `ESTOP_COOLDOWN_MS` ago, AND the user is homing again
            ConnectorUsb.SendLine("Motor error cleared by homing, resuming");
            home_indicator_light.setPattern(LightPattern::OFF);
            is_e_stop = false;
            return PlanishState::begin_homing; // this estop can only be recovered from if they press the home button
          }
          home_indicator_light.setPattern(LightPattern::STROBE);
          return PlanishState::e_stop_wait;
      }
      // should be unreachable
      return PlanishState::e_stop_wait;

    case PlanishState::error:
      fault_code = FaultCodes::Error;
      home_indicator_light.setPattern(LightPattern::STROBE);
      learn_indicator_light.setPattern(LightPattern::STROBE);
      check_modbus(); // last words, if able
      while (true) {
        ConnectorUsb.SendLine("reached error state.");
        ConnectorUsb.Send("estop reason: ");
        ConnectorUsb.SendLine(get_estop_reason_name(estop_reason));
        delay(1000);
      }

    case PlanishState::job_begin:
      Head.set_commanded_state(false); // head is supposed to already be raised, but this needs to be called
      // despite the sensor check in case it was in the proccess of lowering already
      if (Head.is_fully_disengaged()) {
        ConnectorUsb.SendLine("Job started head was already up");
        return PlanishState::job_jog_to_start;
      } else {
        ConnectorUsb.SendLine("Job was started but the head was already down, raising");
        return PlanishState::job_begin_lifting_head;
      }

    case PlanishState::job_begin_lifting_head:
      if (!is_mandrel_safe) {
        ConnectorUsb.SendLine("mandrel limit precondition failed since starting job");
        e_stop_handler(EstopReason::mandrel_latch);
        return PlanishState::e_stop_begin;
      }
      if (Head.is_fully_disengaged()) {
        return PlanishState::job_jog_to_start;
      }
      if (loop_num%STATE_MACHINE_LOOPS_LOG_INTERVAL==0) {
        ConnectorUsb.SendLine("Waiting for head to rise for job start");
      }
      return PlanishState::job_begin_lifting_head;

    case PlanishState::job_jog_to_start:
      ConnectorUsb.SendLine("Jog to start");
      move_motor_auto_speed(saved_job_start_pos);
      return PlanishState::job_jog_to_start_wait;

    case PlanishState::job_jog_to_start_wait:
      MOTOR_COMMAND(CARRIAGE_MOTOR.VelMax(current_jog_speed););
      if (wait_for_motion()) {
        return PlanishState::job_head_down;
      }
      if (loop_num%STATE_MACHINE_LOOPS_LOG_INTERVAL==0) {
        ConnectorUsb.SendLine("waiting for jog to start");
      }
      return PlanishState::job_jog_to_start_wait;

    case PlanishState::job_head_down:
      ConnectorUsb.SendLine("Lowering head");
      Head.set_commanded_state(true);
      return PlanishState::job_head_down_wait;

    case PlanishState::job_head_down_wait:
      if (!is_mandrel_safe) {
        ConnectorUsb.SendLine("mandrel limit precondition failed since starting job");
        e_stop_handler(EstopReason::mandrel_latch);
        return PlanishState::e_stop_begin;
      }
      if (Head.is_fully_engaged()) {
        return PlanishState::job_planish_to_end;
      }
      if (loop_num%STATE_MACHINE_LOOPS_LOG_INTERVAL==0) {
        ConnectorUsb.SendLine("Waiting for head to lower to begin planish");
      }
      return PlanishState::job_head_down_wait;

    case PlanishState::job_planish_to_end:
      ConnectorUsb.SendLine("Planishing to end position");
      move_motor_auto_speed(saved_job_end_pos);
      return PlanishState::job_planish_to_end_wait;

    case PlanishState::job_planish_to_end_wait:
      MOTOR_COMMAND(CARRIAGE_MOTOR.VelMax(current_planish_speed););
      if (wait_for_motion()) {
        if (HALF_CYCLE_SW.State()) {
          ConnectorUsb.SendLine("half cycle detected, go to park");
          // if only doing a half-cycle, the single pass thus far is sufficient. retract head and return to park
          return PlanishState::job_head_up;
        } else {
          ConnectorUsb.SendLine("full cycle detected, do second pass");
          // otherwise do the other pass
          return PlanishState::job_planish_to_start;
        }
      }
      if (loop_num%STATE_MACHINE_LOOPS_LOG_INTERVAL==0) {
        ConnectorUsb.SendLine("Planishing to end position");
      }
      return PlanishState::job_planish_to_end_wait;

    case PlanishState::job_planish_to_start:
      ConnectorUsb.SendLine("Begin planishing to start position");
      move_motor_auto_speed(saved_job_start_pos);
      return PlanishState::job_planish_to_start_wait;

    case PlanishState::job_planish_to_start_wait:
      MOTOR_COMMAND(CARRIAGE_MOTOR.VelMax(current_planish_speed););
      if (wait_for_motion()) {
        return PlanishState::job_head_up;
      }
      if (loop_num%STATE_MACHINE_LOOPS_LOG_INTERVAL==0) {
        ConnectorUsb.SendLine("Planishing to start position");
      }
      return PlanishState::job_planish_to_start_wait;

    case PlanishState::job_head_up:
      ConnectorUsb.SendLine("Raising head");
      Head.set_commanded_state(false);
      return PlanishState::job_head_up_wait;

    case PlanishState::job_head_up_wait:
      if (!is_mandrel_safe) {
        ConnectorUsb.SendLine("mandrel limit precondition failed since starting job");
        e_stop_handler(EstopReason::mandrel_latch);
        return PlanishState::e_stop_begin;
      }
      if (Head.is_fully_disengaged()) {
        return PlanishState::job_jog_to_park;
      }
      if (loop_num%STATE_MACHINE_LOOPS_LOG_INTERVAL==0) {
        ConnectorUsb.SendLine("Waiting for head to raise to park");
      }
      return PlanishState::job_head_up_wait;

    case PlanishState::job_jog_to_park:
      ConnectorUsb.SendLine("Begin jogging to park position");
      move_motor_auto_speed(saved_job_park_pos);
      return PlanishState::job_jog_to_park_wait;

    case PlanishState::job_jog_to_park_wait:
      MOTOR_COMMAND(CARRIAGE_MOTOR.VelMax(current_jog_speed););
      if (wait_for_motion()) {
        return PlanishState::idle;
      }
      if (loop_num%STATE_MACHINE_LOOPS_LOG_INTERVAL==0) {
        ConnectorUsb.SendLine("Jogging to park position");
      }
      return PlanishState::job_jog_to_park_wait;

    case PlanishState::learn_start_pos:
      learn_indicator_light.setPattern(LightPattern::FLASH1);
      ConnectorUsb.SendLine("learn start pos");
      temp_job_start_pos = MOTOR_COMMANDED_POSITION;
      return PlanishState::learn_jog_to_end_pos;

    case PlanishState::learn_jog_to_end_pos:
      if (!is_mandrel_safe) {
        e_stop_handler(EstopReason::mandrel_latch);
        return PlanishState::e_stop_begin;
      }
      if (LearnButton.is_rising()) {
        return PlanishState::learn_end_pos;
      }
      if (JOG_FWD_SW.State() == JOG_FWD_SW_ACTIVE_STATE) {
        motor_jog(false);
      } else if (JOG_REV_SW.State() == JOG_REV_SW_ACTIVE_STATE) {
        motor_jog(true);
      } else {
        MOTOR_COMMAND(CARRIAGE_MOTOR.MoveStopDecel(););
      }
      return PlanishState::learn_jog_to_end_pos;

    case PlanishState::learn_end_pos:
      learn_indicator_light.setPattern(LightPattern::FLASH2);
      ConnectorUsb.SendLine("learn end pos");
      temp_job_end_pos = MOTOR_COMMANDED_POSITION;
      return PlanishState::learn_jog_to_park_pos;

    case PlanishState::learn_jog_to_park_pos:
      if (!is_mandrel_safe) {
        e_stop_handler(EstopReason::mandrel_latch);
        return PlanishState::e_stop_begin;
      }
      if (LearnButton.is_rising()) {
        return PlanishState::learn_park_pos;
      }
      if (JOG_FWD_SW.State() == JOG_FWD_SW_ACTIVE_STATE) {
        motor_jog(false);
      } else if (JOG_REV_SW.State() == JOG_REV_SW_ACTIVE_STATE) {
        motor_jog(true);
      } else {
        MOTOR_COMMAND(CARRIAGE_MOTOR.MoveStopDecel(););
      }
      return PlanishState::learn_jog_to_park_pos;

    case PlanishState::learn_park_pos:
      learn_indicator_light.setPattern(LightPattern::FLASH3);
      ConnectorUsb.SendLine("learn park pos");
      temp_job_park_pos = MOTOR_COMMANDED_POSITION;
      return PlanishState::saving_job_to_nvram;

    case PlanishState::saving_job_to_nvram:
      ConnectorUsb.SendLine("saving job to nvram");
      ConnectorUsb.SendLine(temp_job_start_pos);
      ConnectorUsb.SendLine(temp_job_end_pos);
      ConnectorUsb.SendLine(temp_job_park_pos);
      save_job_to_nvram();
      learn_indicator_light.setPattern(LightPattern::OFF);
      return PlanishState::idle;


  }
  // this means the state machine did not already give a state
  ConnectorUsb.SendLine("State machine iteration executed without explicitly returning the next state");
  ConnectorUsb.Send("Previous state: ");
  ConnectorUsb.SendLine(get_state_name(state_in));
  e_stop_handler(EstopReason::internal_error);
  return PlanishState::e_stop_begin;
}

/**
 * Configures IO, go figure
 * @return True for success
 */
bool configure_io() {

  FINGER_DOWN_LMT.Mode(Connector::INPUT_DIGITAL);
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

  Fingers.set_actuator_pin(CcioMgr.PinByIndex(FINGER_ACTUATION));
  Fingers.set_sense_pin(&FINGER_DOWN_LMT);
  Fingers.set_commanded_state(false);
//  Fingers.set_commanded_state(Fingers.get_measured_state());
  // TODO: what is the desired behavior for finger and head state on boot
  Head.set_actuator_pin(CcioMgr.PinByIndex(HEAD_ACTUATION));
  Head.set_sense_pin(&HEAD_UP_LMT, true);
//  Head.set_commanded_state(Head.get_measured_state());
  Head.set_commanded_state(false);

  home_indicator_light.setPin(CcioMgr.PinByIndex(HOME_SW_LIGHT));
  home_indicator_light.setPeriod(50);
  home_indicator_light.setPattern(LightPattern::OFF);
  home_indicator_light.setInverted(true);
  learn_indicator_light.setPin(CcioMgr.PinByIndex(LEARN_SW_LIGHT));
  learn_indicator_light.setPeriod(50);
  learn_indicator_light.setPattern(LightPattern::OFF);
  learn_indicator_light.setInverted(true);

  MOTOR_COMMAND(MotorMgr.MotorInputClocking(MotorManager::CLOCK_RATE_NORMAL););
  MOTOR_COMMAND(MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL, Connector::CPM_MODE_STEP_AND_DIR););

  MOTOR_COMMAND(CARRIAGE_MOTOR.HlfbMode(MotorDriver::HLFB_MODE_HAS_BIPOLAR_PWM););
  MOTOR_COMMAND(CARRIAGE_MOTOR.HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ););

  update_speed_pot();
  MOTOR_COMMAND(CARRIAGE_MOTOR.VelMax(current_jog_speed););
  MOTOR_COMMAND(CARRIAGE_MOTOR.AccelMax(CARRIAGE_MOTOR_MAX_ACCEL););



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
  if (MOTOR_ASSERTED && MOTOR_STEPS_COMPLETE) {
    ConnectorUsb.SendLine("Motor has completed motion");
    return true;
  }
  if (MOTOR_HAS_ERRORS) {
    // motor has an error
    ConnectorUsb.SendLine("Motor alert detected during homing.");
    print_motor_alerts();
    e_stop_handler(EstopReason::motor_error);
    return false;
  }

  if (!is_mandrel_safe) {
    e_stop_handler(EstopReason::mandrel_latch);
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
  if (machine_state == PlanishState::error) {
    // error is intentionally a dead end
    return;
  }

  if (last_iteration_delta >= ITERATION_TIME_ERROR_MS) {
    ConnectorUsb.Send("Last iteration of the state machine took more than `ITERATION_TIME_ERROR_MS` (");
    ConnectorUsb.Send(ITERATION_TIME_ERROR_MS);
    ConnectorUsb.Send("ms) to complete. This is likely a bug. Last iteration took ");
    ConnectorUsb.Send(last_iteration_delta);
    ConnectorUsb.Send("ms. Engaging E-Stop and stopping execution. Last state: ");
    ConnectorUsb.SendLine(get_state_name(machine_state));
    fault_code = FaultCodes::SmTimeout;
    e_stop_handler(EstopReason::internal_error);
  } else if (last_iteration_delta >= ITERATION_TIME_WARNING_MS) {
    ConnectorUsb.Send("Last iteration of the state machine took more than `ITERATION_TIME_WARNING_MS` (");
    ConnectorUsb.Send(ITERATION_TIME_WARNING_MS);
    ConnectorUsb.Send("ms) to complete. This is likely a bug. Last iteration took ");
    ConnectorUsb.Send(last_iteration_delta);
    ConnectorUsb.Send("ms. Continuing execution. Last state: ");
    ConnectorUsb.SendLine(get_state_name(machine_state));
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

/**
 * Just a wrapper for `e_stop_handler` that 'curries' the E-stop reason for the ISR
 */
void e_stop_button_handler() {
  e_stop_handler(EstopReason::button);
  ConnectorUsb.SendLine("ISR");
}

/**
 * Latch E-stop on.
 * @param reason Why I pulled you over today. Defines what should happen to release the e-stop
 */
void e_stop_handler(const EstopReason reason) {
  if (!is_e_stop) { // prevent the handler from being called when e-stop already active
    estop_reason = reason;
    is_e_stop = true;
    estop_last_state = machine_state;
    MOTOR_COMMAND(CARRIAGE_MOTOR.MoveStopAbrupt(););
    machine_state = PlanishState::e_stop_begin;
    last_estop_millis = millis();
    ConnectorUsb.SendLine("Emergency Stop");
  } else {
    ConnectorUsb.SendLine("Emergency Stop Handler called when e-stop was already active");
  }
}

/**
 * Prints active alerts.
 *
 * @pre requires "CARRIAGE_MOTOR" to be defined as a ClearCore motor connector
 */
void print_motor_alerts(){
  // report status of alerts
#ifdef TEST_MODE_DISABLE_MOTOR
  ConnectorUsb.SendLine("print_motor_alerts()");
#else
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
#endif
}

/**
 * Convert bytes to u32. Big endian, little endian?
 * I don't know, but it works with `u32_to_bytes()`
 *
 * @warning `bytes[offset+3]` needs to be a valid address
 *
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
 *
 * @warning `bytes[offset+3]` needs to be a valid address
 *
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
  MOTOR_COMMAND(CARRIAGE_MOTOR.VelMax(speed););
  return MOTOR_COMMAND(CARRIAGE_MOTOR.Move(position, StepGenerator::MOVE_TARGET_ABSOLUTE););
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
  const int32_t speed = Head.is_fully_disengaged() ? current_jog_speed : current_planish_speed;
  MOTOR_COMMAND(CARRIAGE_MOTOR.VelMax(speed););
  return MOTOR_COMMAND(CARRIAGE_MOTOR.Move(position, StepGenerator::MOVE_TARGET_ABSOLUTE););
}

void motor_jog(const bool reverse) {
  if (Head.is_fully_disengaged()) {
#ifdef TEST_MODE_DISABLE_MOTOR
    ConnectorUsb.Send("Jog mode CARRIAGE_MOTOR.MoveVelocity(");
    ConnectorUsb.Send(reverse ? -current_jog_speed : current_jog_speed);
    ConnectorUsb.SendLine(");");
#else
    CARRIAGE_MOTOR.MoveVelocity(reverse ? -current_jog_speed : current_jog_speed);
#endif
  } else {
#ifdef TEST_MODE_DISABLE_MOTOR
    ConnectorUsb.Send("Planish Mode CARRIAGE_MOTOR.MoveVelocity(");
    ConnectorUsb.Send(reverse ? -current_planish_speed : current_planish_speed);
    ConnectorUsb.SendLine(");");
#else
    CARRIAGE_MOTOR.MoveVelocity(reverse ? -current_planish_speed : current_planish_speed);
#endif
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

  HmiIsRthButton.new_reading(hmi_is_rth_state.value);
  HmiIsAxisHomingButton.new_reading(hmi_is_axis_homing_state.value);
  HmiIsSetJobStartButton.new_reading(hmi_is_set_job_start_state.value);
  HmiIsSetJobEndButton.new_reading(hmi_is_set_job_end_state.value);
  HmiIsSetJobParkButton.new_reading(hmi_is_set_job_park_state.value);
  HmiIsCommandedFingersUpButton.new_reading(hmi_finger_up_state.value);
  HmiIsCommandedFingersDownButton.new_reading(hmi_finger_down_state.value);
  HmiIsCommandedRollerUpButton.new_reading(hmi_roller_up_state.value);
  HmiIsCommandedRollerDownButton.new_reading(hmi_roller_down_state.value);
  HmiIsCommandedPosButton.new_reading(hmi_is_commanded_pos_state.value);
  HmiCommitJobButton.new_reading(hmi_commit_job_state.value);
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

  float temp_jog_speed = (pot_percent * CARRIAGE_MOTOR_MAX_VEL) + CARRIAGE_MOTOR_MIN_VEL;
  float temp_planish_speed = (pot_percent * CARRIAGE_MOTOR_MAX_PLANISH_VEL) + CARRIAGE_MOTOR_MIN_PLANISH_VEL;

  if (temp_jog_speed > CARRIAGE_MOTOR_MAX_VEL) {
    temp_jog_speed = CARRIAGE_MOTOR_MAX_VEL;
  }
  if (temp_planish_speed > CARRIAGE_MOTOR_MAX_PLANISH_VEL) {
    temp_planish_speed = CARRIAGE_MOTOR_MAX_PLANISH_VEL;
  }

  current_planish_speed = static_cast<int32_t>(temp_planish_speed);
  current_jog_speed = static_cast<int32_t>(temp_jog_speed);
}


/**
 * Handle stopping and securing every part of the system.
 * @param last_state The state of the system before the e-stop
 * @return The state to resume to once the E-stop cause was rectified
 */
PlanishState secure_system(const PlanishState last_state) {
  ConnectorUsb.Send("secure_system called with state: ");
  ConnectorUsb.Send(get_state_name(last_state));
  switch (last_state) {
    case PlanishState::post:
      // can happen if E-stop ISR goes off erroneously on boot
      return PlanishState::post;

    case PlanishState::begin_homing:
    case PlanishState::homing_wait_for_disable:
    case PlanishState::wait_for_homing:
      // special case where telling the motor to stop won't stop it. it needs to be disabled.
      home_indicator_light.setPattern(LightPattern::OFF);
      MOTOR_COMMAND(CARRIAGE_MOTOR.EnableRequest(false););
      is_homed = false;
      return PlanishState::idle;

    case PlanishState::idle:
      return PlanishState::idle;

    case PlanishState::manual_jog:
      return PlanishState::idle;

    case PlanishState::e_stop_begin:
    case PlanishState::e_stop_wait:
      // This is not a valid state. The state before the E-stop ran should never be e-stop
      return PlanishState::error;

    case PlanishState::error:
      // This would mean the machine was in an error state before the e-stop.
      return PlanishState::error;

    case PlanishState::wait_for_head:
      if (Head.is_mismatch() && Head.get_commanded_state()) {
        // if the head was being commanded down and hasn't reached down yet
        Head.set_commanded_state(false);
      } // if the head was not in motion, or was already raising, don't do anything
      return PlanishState::idle;

    case PlanishState::wait_for_fingers:
      if (Fingers.is_mismatch() && Fingers.get_commanded_state()) {
        // if the fingers were being commanded down and hadn't reached down yet
        Fingers.set_commanded_state(false);
      } // if the fingers were not in motion, or were already raising, don't do anything
      return PlanishState::idle;

    case PlanishState::job_begin:
      return PlanishState::job_begin;

    case PlanishState::job_begin_lifting_head:
      // this is safe to resume, stateless
      return PlanishState::job_begin_lifting_head;

    case PlanishState::job_jog_to_start:
    case PlanishState::job_jog_to_start_wait:
      return PlanishState::job_jog_to_start;

    case PlanishState::job_head_down:
      return PlanishState::job_head_down;

    case PlanishState::job_head_down_wait:
      if (Head.is_mismatch()) {
        // if there is a mismatch between the commanded state and the measured state
        Head.set_commanded_state(false); // if the head was in motion, raise it
      } // if the head was not in motion, then it's fine to keep where it is
      // resume by lowering the head again
      return PlanishState::job_head_down;

    case PlanishState::job_planish_to_end:
    case PlanishState::job_planish_to_end_wait:
      return PlanishState::job_planish_to_end;

    case PlanishState::job_planish_to_start:
    case PlanishState::job_planish_to_start_wait:
      return PlanishState::job_planish_to_start;

    case PlanishState::job_head_up:
      return PlanishState::job_head_up;

    case PlanishState::job_head_up_wait:
      return PlanishState::job_head_up_wait;

    case PlanishState::job_jog_to_park:
    case PlanishState::job_jog_to_park_wait:
      return PlanishState::job_jog_to_park;

    case PlanishState::learn_start_pos:
    case PlanishState::learn_jog_to_end_pos:
    case PlanishState::learn_end_pos:
    case PlanishState::learn_jog_to_park_pos:
    case PlanishState::learn_park_pos:
      // An estop here will cancel the learn operation
      learn_indicator_light.setPattern(LightPattern::OFF);
      return PlanishState::idle;

    case PlanishState::saving_job_to_nvram:
      return PlanishState::saving_job_to_nvram;
  }
  return PlanishState::error;
}


/**
 * For debugging.
 * @param state
 * @return the name of the state in the codebase.
 */
const char *get_state_name(const PlanishState state) {
#define STATE_NAME(state) case PlanishState::state: return #state;
  switch (state) {
    STATE_NAME(post);
    STATE_NAME(begin_homing);
    STATE_NAME(homing_wait_for_disable);
    STATE_NAME(wait_for_homing);
    STATE_NAME(idle);
    STATE_NAME(manual_jog);
    STATE_NAME(e_stop_begin);
    STATE_NAME(e_stop_wait);
    STATE_NAME(error);
    STATE_NAME(wait_for_fingers);
    STATE_NAME(wait_for_head);
    STATE_NAME(job_begin);
    STATE_NAME(job_begin_lifting_head);
    STATE_NAME(job_jog_to_start);
    STATE_NAME(job_jog_to_start_wait);
    STATE_NAME(job_head_down);
    STATE_NAME(job_head_down_wait);
    STATE_NAME(job_planish_to_end);
    STATE_NAME(job_planish_to_end_wait);
    STATE_NAME(job_planish_to_start);
    STATE_NAME(job_planish_to_start_wait);
    STATE_NAME(job_head_up);
    STATE_NAME(job_head_up_wait);
    STATE_NAME(job_jog_to_park);
    STATE_NAME(job_jog_to_park_wait);
    STATE_NAME(learn_start_pos);
    STATE_NAME(learn_jog_to_end_pos);
    STATE_NAME(learn_end_pos);
    STATE_NAME(learn_jog_to_park_pos);
    STATE_NAME(learn_park_pos);
    STATE_NAME(saving_job_to_nvram);
  }
  return "INVALID_STATE";
}

/**
 * For debugging.
 * @param state
 * @return the name of the state in the codebase.
 */
const char *get_estop_reason_name(const EstopReason state) {
#define REASON_NAME(reason) case EstopReason::reason: return #reason;
  switch (state) {
    REASON_NAME(NONE)
    REASON_NAME(button)
    REASON_NAME(mandrel_latch)
    REASON_NAME(internal_error)
    REASON_NAME(motor_error)
  }
  return "INVALID_STATE";
}

double steps_to_f64_inch(const uint32_t steps) {
  const double motor_revs = static_cast<double>(steps)/STEPS_PER_REV;
  const double pinion_revs = motor_revs * GEARBOX_RATIO;
  const double teeth = pinion_revs / PINION_TEETH_PER_REV;
  const double inches = teeth / RACK_TEETH_PER_INCH;
  return inches;
}

uint16_t steps_to_hundreths(const uint32_t steps) {
  const double hundreths = steps_to_f64_inch(steps) * 100;

  return static_cast<uint16_t>(hundreths);
}

uint16_t steps_per_sec_to_inches_per_minute(const uint32_t steps_per_second) {
  const double inches_per_second = steps_to_f64_inch(steps_per_second);
  const double inches_per_minute = inches_per_second * 60;

  return static_cast<uint16_t>(inches_per_minute);
}