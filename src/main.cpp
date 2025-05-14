/*
  Paul Fornage
*/

#include <Arduino.h>
#include "NvmManager.h"
#include "ClearCore.h"
#include "indicator_light.h"
#include "button.h"
#include "actuator.h"
#include "FaultCodes.h"
#include <Ethernet.h>       // Ethernet library v2 is required
#define MODBUSIP_TIMEOUT 100
#define MODBUSIP_MAX_READMS 50
#include <ModbusAPI.h>
#include <ModbusTCPTemplate.h>
#include <utility>
#include "helpers.h"
#include "MbButton.h"
#include "RegisterDefinitions.h"
#include "Message.h"


// TODO: E-stop ISR

// TODO: Maybe set job parameters from GCODE

// TODO: logger

// TODO: Save translation function with register. (EG steps to inches). Maybe even types with units? Rust momen

// TODO: Register sync dev tool

// TODO: read status reg: https://teknic-inc.github.io/ClearCore-library/_clear_core_status_register_8cpp-example.html

// TODO: HMI reset temp job reg to saved value. (discard/reset button)

// TODO: Job progress percent based on steps/total steps

// TODO: Estop button during job should also pause the job


// ModBus TCP stuff
uint8_t mac[6] = {0x24, 0x15, 0x10, 0xB0, 0x45, 0xA4}; // MAC address is ignored but because of C++ types, you still need to give it garbage

EthernetTcpClient client;

bool use_dhcp = true; // this will get disabled at runtime if DHCP fails `dhcp_attempts` times
#define MAX_DHCP_ATTEMPTS 5 // how many times to try DHCP in a row before going static
uint8_t dhcp_attempts = MAX_DHCP_ATTEMPTS; // this gets reset when DHCP is successful

// not used in DHCP
const IPAddress ip(192, 168, 1, 17); // Local IP for non DHCP mode

// Instance will not be deleted, disabling warning
// ReSharper disable once CppPolymorphicClassWithNonVirtualPublicDestructor
class ModbusEthernet : public ModbusAPI<ModbusTCPTemplate<EthernetServer, EthernetClient>> {};
ModbusEthernet mb;  //ModbusTCP object

Message::MessageClass HmiMessage;

MbButton HmiAxisHomingButton(CoilAddr::IS_AXIS_HOMING_BUTTON_LATCHED);
MbButton HmiSetJobStartButton(CoilAddr::IS_SET_JOB_START_BUTTON_LATCHED);
MbButton HmiSetJobEndButton(CoilAddr::IS_SET_JOB_END_BUTTON_LATCHED);
MbButton HmiSetJobParkButton(CoilAddr::IS_SET_JOB_PARK_BUTTON_LATCHED);
MbButton HmiCommitJobButton(CoilAddr::IS_COMMIT_JOB_BUTTON_LATCHED);
MbButton HmiCommandedFingersUpButton(CoilAddr::IS_FINGER_UP_LATCHED);
MbButton HmiCommandedFingersDownButton(CoilAddr::IS_FINGER_DOWN_LATCHED);
MbButton HmiCommandedRollerUpButton(CoilAddr::IS_ROLLER_UP_LATCHED);
MbButton HmiCommandedRollerDownButton(CoilAddr::IS_ROLLER_DOWN_LATCHED);
MbButton HmiCommandedPosButton(CoilAddr::IS_COMMANDED_POS_LATCHED);
MbButton HmiStartCycleButton(CoilAddr::IS_START_CYCLE_BUTTON_LATCHED);
MbButton HmiCancelCycleButton(CoilAddr::IS_CANCEL_CYCLE_BUTTON_LATCHED);
MbButton HmiPauseCycleButton(CoilAddr::IS_PAUSE_CYCLE_BUTTON_LATCHED);

auto fault_code = FaultCodes::None;

bool is_homed = false;                  // Has the axis been homed
volatile bool is_e_stop = false;               // Is the Emergency Stop currently active

bool io_configured = false;                  // Has the IO been configured
char message_buffer[64] = "";

volatile bool timeout_warning_since_last_loop = false; // has a warning been printed since the last loop?
volatile bool timeout_error_since_last_loop = false; // has an error been printed since the last loop?

uint32_t last_modbus_read = 0;
uint32_t time_since_last_modbus_read = 0;

int32_t current_jog_speed = 100;              // Current speed the motor should use for jogging. Steps/second
int32_t current_planish_speed = 100;          // Current speed the motor should use for planishing. Steps/second

uint32_t last_iteration_delta;          // Time spent on the last iteration of the main loop in millis
uint32_t last_iteration_time;           // Time of the last iteration of the main loop in millis

uint32_t loop_num = 0;                  // Number of times the main loop has been called. WILL OVERFLOW
uint32_t last_estop_millis = 0;         // Millis() value at time the last e-stop was pressed

uint32_t homing_disable_time = 0;        // Time the motor was disabled for homing in millis

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
  manual_jog_absolute,        // manually commanded jog to absolute position
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
  job_paused,                 // The job is paused and not canceled. This needs a global variable for what state it was previously in
  job_cancel,                 // Return the machine to a safe and static state and then return to idle
};

volatile PlanishState estop_last_state;  // When an estop is called, this gets set to whatever the current state was.
volatile PlanishState estop_resume_state;      // When an estop is resolved, this stores where to resume
volatile PlanishState machine_state;     // the current state of the machine. Volatile because it can be changed by estop ISR
PlanishState job_resume_state;      // the state of the job to resume to after pausing.

volatile EstopReason estop_reason = EstopReason::NONE;

/**
 * when defined the motor will not move and
 * any commands meant for the motor will be logged/simulated
 */
// #define TEST_MODE_DISABLE_MOTOR


#ifdef TEST_MODE_DISABLE_MOTOR
// Wrapper for commands to the motor that will print them instead if motor is disabled for test mode
#define MOTOR_COMMAND(code) USB_PRINTLN(#code)
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
#define MANDREL_LATCH_LMT ConnectorDI8
#define HEAD_UP_LMT ConnectorDI6


// CCIO pin definitions. these are not interchangeable with native io ports.
// please see `configure_io()` to change assignment between ccio and native pins
#define FINGER_ACTUATION CCIOA6 // CCIO pin for finger actuation
#define HEAD_ACTUATION CCIOA4


#define CCIO1 ConnectorCOM1
#define EXPECTED_NUM_CCIO 1
#define CCIO_TIMEOUT_MS 10000 // Number of ms to wait for CCIO to connect before failing POST

#define CARRIAGE_MOTOR_MAX_ACCEL 50000
#define CARRIAGE_MOTOR_MAX_VEL 10000
// not a safety limit, just what should the minimum position on the speed selector represent in jog mode
#define CARRIAGE_MOTOR_MIN_VEL 200
#define CARRIAGE_MOTOR_MAX_POS 12000
#define CARRIAGE_MOTOR_MAX_PLANISH_VEL 4000
// not a safety limit, just what should the minimum position on the speed selector represent in planish mode
#define CARRIAGE_MOTOR_MIN_PLANISH_VEL 100

#define ENABLE_ITERATION_TIME_CHECK
#define ITERATION_TIME_WARNING_MS 500 // after this many milliseconds stuck on one iteration of the state machine, give a warning.
#define ITERATION_TIME_ERROR_MS 1000  // after this many milliseconds stuck on one iteration of the state machine, declare an error
#define SERIAL_ESTABLISH_TIMEOUT 5000 // Number of ms to wait for serial to establish before failing POST
#define HMI_CONNECTION_TRIES_BEFORE_ERROR 5
#define MODBUS_CHECK_INTERVAL_MS 100

/// Interrupt priority for the periodic interrupt. 0 is highest priority, 7 is lowest.
#define PERIODIC_INTERRUPT_PRIORITY 5

#define ESTOP_COOLDOWN_MS 1000
#define ESTOP_SW_SAFE_STATE 1
#define MANDREL_LATCH_LMT_SAFE_STATE 1 // MANDREL_LATCH_LMT.State() should return this when it's safe/down/engaged
#define IS_MANDREL_SAFE (MANDREL_LATCH_LMT.State() == MANDREL_LATCH_LMT_SAFE_STATE)

#define JOG_FWD_SW_ACTIVE_STATE 0
#define JOG_REV_SW_ACTIVE_STATE 0

#define ADC_RES_BITS 12
#define ADC_MAX_VALUE ((1 << ADC_RES_BITS)-1)

#define MOTOR_EN_DIS_DELAY_MS 10    // during homing the motor needs to be disabled for this many ms to actually do it

#define STATE_MACHINE_LOOPS_POT_UPDATE_INTERVAL 32 // number of loops of the main state machine between speed pot updates
#define STATE_MACHINE_LOOPS_HMI_UPDATE_INTERVAL 32 // number of loops of the main state machine between Modbus Hmi updates.
#define STATE_MACHINE_LOOPS_LOG_INTERVAL 1024 // number of loops of the main state machine between
// logging from 'wait' states. Too low will flood the logs

#define IS_JOG_FWD_ACTIVE (mb.Coil(CoilAddr::IS_JOG_POS_PRESSED))
#define IS_JOG_REV_ACTIVE (mb.Coil(CoilAddr::IS_JOG_NEG_PRESSED))

#define PERIODIC_PRINT(statements) if (loop_num%STATE_MACHINE_LOOPS_LOG_INTERVAL==0) { statements }

#define USB_PRINT(statements) ConnectorUsb.Send(statements);
#define USB_PRINTLN(statements) ConnectorUsb.SendLine(statements);
#define USB_PRINT_CHAR(statements) ConnectorUsb.SendChar(statements);

std::pair<PlanishState, bool> state_change;
/**
 * Wrapper of the function with the same name.
 */
#define COMMON_JOB_TASKS()\
state_change = common_job_tasks();\
if(state_change.second) {return state_change.first;}\

SensedActuator Fingers;
SensedActuator Head;

bool configure_io();
void e_stop_button_handler();
void print_motor_alerts();
void iteration_time_check();
void config_ccio_pin(ClearCorePins target_pin, Connector::ConnectorModes mode, bool initial_state = false);
void set_ccio_pin(ClearCorePins target_pin, bool state);
void ConfigurePeriodicInterrupt(uint32_t frequencyHz);
void read_job_from_nvram();
void save_job_to_nvram();
bool wait_for_motion();
bool move_motor_with_speed(int32_t position, int32_t speed);
bool move_motor_auto_speed(int32_t position);
void motor_jog(bool reverse);
void update_buttons();
void e_stop_handler(EstopReason reason);
PlanishState secure_system(PlanishState last_state);
PlanishState state_machine(PlanishState state_in);
const char *get_estop_reason_name(EstopReason state);
void check_modbus();
std::pair<uint16_t, bool> job_progress(PlanishState state_to_check);
void mb_read_unlatch_coil(MbButton *button);
const char *get_state_name(PlanishState state);
bool ethernet_setup();
bool cbConn(IPAddress ip);
PlanishState job_pause_get_resume_state(PlanishState state_before_pause);
std::pair<PlanishState, bool> common_job_tasks();
void pause_job(PlanishState state_before_pause);
inline void hmi_print(const char* message, uint16_t time_to_print);
inline void combined_print(const char* message, uint16_t time_to_print);
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

  const bool eth_setup_succeed = ethernet_setup();
  USB_PRINTLN(eth_setup_succeed?"ethernet set up correctly":"ethernet link failed")

  mb.server();
  mb.onConnect(cbConn);

  // Add all Coil Registers
  mb.addCoil(0, false, CoilAddr::NULL_TERM);

  // Add all Holding Registers
  mb.addHreg(0, 0, HregAddr::NULL_TERM);

  read_job_from_nvram();

  USB_PRINTLN("job retrieved from NVRAM");
  USB_PRINTLN(saved_job_start_pos);
  USB_PRINTLN(saved_job_end_pos);
  USB_PRINTLN(saved_job_park_pos);

  io_configured = configure_io();

  if (!io_configured) {           // if the IO failed to configure properly
    USB_PRINTLN("IO was not configured successfully, hanging");
    e_stop_handler(EstopReason::internal_error);
    return;
  }
  // do this twice so that the buffer only contains measured values
  update_buttons();
  update_buttons();


  last_iteration_time = millis();
  last_modbus_read = millis();
  machine_state = PlanishState::post;

//  ESTOP_SW.InterruptHandlerSet(e_stop_button_handler, InputManager::InterruptTrigger::CHANGE);
}


void loop() {
  timeout_warning_since_last_loop = false;
  timeout_error_since_last_loop = false;
  if (ESTOP_SW.State() != ESTOP_SW_SAFE_STATE) { // make sure e-stop wasn't already depressed before interrupt was registered
    combined_print("E-Stop button pressed, entering e-stop mode. ", 10000);
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

  last_iteration_time = millis();


  mb.task();

  // state machine state machine machine state
  // Run one instance of the state machine
  machine_state = state_machine(machine_state);

  // both of these calls rely on io being configured but it
  update_buttons();


  time_since_last_modbus_read = millis()-last_modbus_read;
  if (time_since_last_modbus_read > MODBUS_CHECK_INTERVAL_MS) {
    check_modbus();
    last_modbus_read = millis();
    if (time_since_last_modbus_read > MODBUS_CHECK_INTERVAL_MS + 100) {
      USB_PRINTLN("Modbus check falling behind interval");
    }
  }
}

void printIp(IPAddress ip) {
  USB_PRINT(ip[0]);
  USB_PRINT('.');
  USB_PRINT(ip[1]);
  USB_PRINT('.');
  USB_PRINT(ip[2]);
  USB_PRINT('.');
  USB_PRINT(ip[3]);
}

bool ethernet_setup() {
  if (Ethernet.linkStatus() == EthernetLinkStatus::LinkOFF) {
    USB_PRINTLN("Ethernet has no link, `ethernet_setup()` fails. Trying again")
    delay(500); // TODO: fix this
    if (Ethernet.linkStatus() == EthernetLinkStatus::LinkOFF) {
      USB_PRINTLN("Ethernet has no link, `ethernet_setup()` fails")
      return false;
    }
  }

  while (use_dhcp && dhcp_attempts > 0) {
    if (!Ethernet.begin(mac)) {
      dhcp_attempts--;
      USB_PRINT("DHCP failed. ");
      USB_PRINT(dhcp_attempts);
      USB_PRINTLN(" attempts remaining");
    } else {
      dhcp_attempts = MAX_DHCP_ATTEMPTS;
      USB_PRINT("DHCP gives IP: ");
      const auto temp_ip = Ethernet.localIP();
      printIp(temp_ip);
      USB_PRINTLN();
      return true;
    }
  }
  Ethernet.begin(mac, ip);
  USB_PRINT("Ethernet set up static on ");
  printIp(ip);
  USB_PRINTLN();
  return true;
}

bool cbConn(IPAddress ip) {
  USB_PRINT("Connection from: ")
  printIp(std::move(ip));
  USB_PRINTLN();
  return true;
}

/**
 * Checks modbus
 * Happens after reading state from hmi, and before writing
 * @pre IO configured successfully
 */
void check_modbus() {
  mb.Coil(CoilAddr::IS_MANDREL_LATCH_CLOSED, IS_MANDREL_SAFE);
  mb.Coil(CoilAddr::IS_FINGERS_DOWN, Fingers.get_measured_state());
  mb.Coil(CoilAddr::IS_ROLLER_DOWN, Head.get_measured_state());
  mb.Coil(CoilAddr::IS_HOMED, is_homed);
  mb.Coil(CoilAddr::IS_FAULT, machine_state==PlanishState::error);

  mb.Coil(CoilAddr::IS_E_STOP, is_e_stop);
  mb.Coil(CoilAddr::IS_JOB_ACTIVE, job_progress(machine_state).second || machine_state==PlanishState::job_paused);

  mb.Coil(CoilAddr::CC_COMMANDED_FINGERS, Fingers.get_commanded_state());
  mb.Coil(CoilAddr::CC_COMMANDED_ROLLER, Head.get_commanded_state());
  mb.Coil(CoilAddr::IS_JOB_PAUSED, machine_state==PlanishState::job_paused);
  mb.Coil(CoilAddr::SHOW_MESSAGE, HmiMessage.is_active());

  mb.Hreg(HregAddr::CC_COMMANDED_POSITION_REG_ADDR, steps_to_hundreths(MOTOR_COMMANDED_POSITION));

  if (machine_state == PlanishState::job_paused) {
    mb.Hreg(HregAddr::JOB_PROGRESS_REG_ADDR, job_progress(job_resume_state).first);
  } else {
    mb.Hreg(HregAddr::JOB_PROGRESS_REG_ADDR, job_progress(machine_state).first);
  }

  mb.Hreg(HregAddr::JOB_START_POS_REG_ADDR, steps_to_hundreths(saved_job_start_pos));
  mb.Hreg(HregAddr::JOB_END_POS_REG_ADDR, steps_to_hundreths(saved_job_end_pos));
  mb.Hreg(HregAddr::JOB_PARK_POS_REG_ADDR, steps_to_hundreths(saved_job_park_pos));
  mb.Hreg(HregAddr::FAULT_CODE_REG_ADDR, static_cast<uint16_t>(fault_code));
  mb.Hreg(HregAddr::CURRENT_STATE_REG_ADDR, static_cast<uint16_t>(machine_state));


  const uint16_t planish_speed_temp_hpm /*Hundreths per minute*/ = mb.Hreg(HregAddr::PLANISH_SPEED_REG_ADDR);
  const uint16_t planish_speed_temp_sps /*Steps per second*/ = hundreths_per_minute_to_steps_per_sec(planish_speed_temp_hpm);
  if (planish_speed_temp_sps < CARRIAGE_MOTOR_MIN_PLANISH_VEL) {
    current_planish_speed = CARRIAGE_MOTOR_MIN_PLANISH_VEL;
    mb.Hreg(HregAddr::PLANISH_SPEED_REG_ADDR,
      steps_per_sec_to_hundreths_per_minute(CARRIAGE_MOTOR_MIN_PLANISH_VEL));
  } else if (planish_speed_temp_sps > CARRIAGE_MOTOR_MAX_PLANISH_VEL) {
    current_planish_speed = CARRIAGE_MOTOR_MAX_PLANISH_VEL;
    mb.Hreg(HregAddr::PLANISH_SPEED_REG_ADDR,
      steps_per_sec_to_hundreths_per_minute(CARRIAGE_MOTOR_MAX_PLANISH_VEL));
  } else {
    current_planish_speed = planish_speed_temp_sps;
  }
  const uint16_t jog_speed_temp_hpm /*Hundreths per minute*/ = mb.Hreg(HregAddr::JOG_SPEED_REG_ADDR);
  const uint16_t jog_speed_temp_sps /*Steps per second*/ = hundreths_per_minute_to_steps_per_sec(jog_speed_temp_hpm);
  if (jog_speed_temp_sps < CARRIAGE_MOTOR_MIN_VEL) {
    current_jog_speed = CARRIAGE_MOTOR_MIN_VEL;
    mb.Hreg(HregAddr::JOG_SPEED_REG_ADDR,
      steps_per_sec_to_hundreths_per_minute(CARRIAGE_MOTOR_MIN_VEL));
  } else if (jog_speed_temp_sps > CARRIAGE_MOTOR_MAX_VEL) {
    current_jog_speed = CARRIAGE_MOTOR_MAX_VEL;
    mb.Hreg(HregAddr::JOG_SPEED_REG_ADDR,
      steps_per_sec_to_hundreths_per_minute(CARRIAGE_MOTOR_MAX_VEL));
  } else {
    current_jog_speed = jog_speed_temp_sps;
  }

  if (HmiMessage.is_active() && HmiMessage.has_been_updated()) {
    const auto values = HmiMessage.get_message_u16();
    for (uint8_t i = 0; i < Message::MessageClass::message_u16_len_max; i++) {
      mb.Hreg(i+HregAddr::MESSAGE_START, values[i]);
    }
    HmiMessage.unlatch_updated();
  }
}

/**
 * reads the value associated with the button from the MB register and if its latched (true), it gets reset to false
 * updates the button with the reading that was taken.
 *
 *
 * @param button
 */
void mb_read_unlatch_coil(MbButton *button) {
  const uint16_t addr = button->get_address();
  const bool temp = mb.Coil(addr);
  button->new_reading(temp);
  if (temp) {
    USB_PRINT("unlatching coil at address ");
    USB_PRINTLN(addr)
    mb.Coil(addr, false);
  }
}

inline void hmi_print(const char* message, const uint16_t time_to_print) {
  HmiMessage.set_message(message, time_to_print);
}

/**
 * Print message to USB serial, and put the message on the HMI for `time_to_print` ms
 */
inline void combined_print(const char* message, const uint16_t time_to_print) {
  HmiMessage.set_message(message, time_to_print);
  USB_PRINTLN(message);
}

/**
 * Get the progress of the current job, if there is one
 * @param state_to_check current machine state
 * @return Pair(progress 0..99, is there even a job going on)
 */
std::pair<uint16_t, bool> job_progress(const PlanishState state_to_check) {
  uint16_t progress = 0;
  switch (state_to_check) {
    case PlanishState::job_begin:
    case PlanishState::job_begin_lifting_head:
      progress = 10;
      break;
    case PlanishState::job_jog_to_start:
    case PlanishState::job_jog_to_start_wait:
      progress = 15;
      break;
    case PlanishState::job_head_down:
    case PlanishState::job_head_down_wait:
      progress = 18;
      break;
    case PlanishState::job_planish_to_end:
    case PlanishState::job_planish_to_end_wait:
      progress = 25;
      break;
    case PlanishState::job_planish_to_start:
    case PlanishState::job_planish_to_start_wait:
      progress = 60;
      break;
    case PlanishState::job_head_up:
    case PlanishState::job_head_up_wait:
      progress = 80;
      break;
    case PlanishState::job_jog_to_park:
    case PlanishState::job_jog_to_park_wait:
      progress = 90;
      break;
    default:
      return std::make_pair(0, false);
  }
  return std::make_pair(progress, true);
}

PlanishState state_machine(const PlanishState state_in) {
  switch (state_in) {
    case PlanishState::post:  // Not used anymore but could be in the future
      USB_PRINTLN("POST");
      return PlanishState::idle;

    case PlanishState::begin_homing:
      combined_print("Begin Homing", 1000);

      MOTOR_COMMAND(CARRIAGE_MOTOR.EnableRequest(false));
      is_homed = false;

      homing_disable_time = millis();
      return PlanishState::homing_wait_for_disable;

    case PlanishState::homing_wait_for_disable:
      if (millis() - homing_disable_time > MOTOR_EN_DIS_DELAY_MS) {
        MOTOR_COMMAND(CARRIAGE_MOTOR.EnableRequest(true););
        return PlanishState::wait_for_homing;
      }
      return PlanishState::homing_wait_for_disable;

    case PlanishState::wait_for_homing:
      if (!IS_MANDREL_SAFE) {
        combined_print("Mandrel latch became unsafe since homing. ESTOP", 2000);
        e_stop_handler(EstopReason::mandrel_latch);
        return PlanishState::e_stop_begin;
      }

      if (MOTOR_ASSERTED) {
        // homing done no errors
        combined_print("homing complete", 2000);
        // the position saved in the local motor instance is not neccesarily the same of the motor FW.
        // this line ensures they are set to the same thing, but when there seems to be a rounding error between the two
        // can confirm they don't desync over the course of at least ~20,000 steps
        MOTOR_COMMAND(CARRIAGE_MOTOR.PositionRefSet(0););
        is_homed = true;
        return PlanishState::idle;
      }

      if (MOTOR_HAS_ERRORS) {
        if(MOTOR_ERROR_COMMANDED_WHEN_DISABLED) {
          USB_PRINTLN("Motor was commanded to move before enabling."
                                "depending on how the code turns out this might not be problematic");
        }
        combined_print("Motor alert detected during homing HOME again to clear ESTOP", 2000);
        print_motor_alerts();
        e_stop_handler(EstopReason::motor_error);
      }
      return PlanishState::wait_for_homing;

    case PlanishState::idle:
      if (HmiAxisHomingButton.is_rising()) {
        if (IS_MANDREL_SAFE) {
          if (Head.is_fully_disengaged()) {
            return PlanishState::begin_homing;
          } else {
            combined_print("Tried to home while head engaged", 2000);
          }
        } else {
          combined_print("Tried to home without mandrel latch engaged", 2000);
        }
      }

      if (HmiCommandedPosButton.is_rising()) {
        if (IS_MANDREL_SAFE) {
          if (is_homed) {
            move_motor_auto_speed(hundreths_to_steps(mb.Hreg(HregAddr::HMI_COMMANDED_POSITION_REG_ADDR)));
            return PlanishState::manual_jog_absolute;
          } else {
            combined_print("Tried to jog to position without homed", 2000);
          }
        } else {
          combined_print("Tried to jog to position without mandrel latch engaged", 2000);
        }
      }

      if (HmiCommandedFingersUpButton.is_rising()
          || HmiCommandedFingersDownButton.is_rising())
        {
        bool new_finger_state; // < doesn't necessarily get acted on or set.
        if(HmiCommandedFingersUpButton.is_rising()) {
          new_finger_state = false;
        } else { // if (HmiIsCommandedFingersDownButton.is_rising())
          new_finger_state = true;
        }
        if (IS_MANDREL_SAFE) {
          if(Head.is_fully_disengaged() || new_finger_state) {
            // make sure the head is up and not engaged, and also not in the process of lowering.
            // OR the user is trying to engage the fingers because the head somehow was already down
            USB_PRINTLN(new_finger_state ? "Engaging fingers" : "Disengaging fingers");
            Fingers.set_commanded_state(new_finger_state);
            return PlanishState::wait_for_fingers;
          } else {
            combined_print("Tried to disengage fingers while head was down or lowering", 2000);
          }
        } else {
          combined_print("tried to engage fingers without mandrel latch engaged", 2000);
        }
      }

      if (HmiCommandedRollerUpButton.is_rising()
          || HmiCommandedRollerDownButton.is_rising())
        {
        bool new_head_state; // < doesn't necessarily get acted on or set.
        if (HmiCommandedRollerUpButton.is_rising()) {
          new_head_state = false;
        } else { // if (HmiIsCommandedRollerDownButton.is_rising())
          new_head_state = true;
        }

        if((IS_MANDREL_SAFE && Fingers.is_fully_engaged()) || !new_head_state) {
          // if safety checks for lowering head pass, or the user is trying to raise the head
          Head.set_commanded_state(new_head_state);
          return PlanishState::wait_for_head;
        } else if (!IS_MANDREL_SAFE) {
          combined_print("Tried to lower head while mandrel latch not engaged", 2000);
        } else {
          combined_print("Tried to lower head while fingers not down", 2000);
        }
        return PlanishState::idle;
      }

      if (IS_JOG_FWD_ACTIVE || IS_JOG_REV_ACTIVE) {
        if (IS_MANDREL_SAFE) {
          if (is_homed) {
            // This is weird, while there is nothing that depends on the motor's absolute position for this,
            // it needs to be enabled, and enabling is what starts the homing process.
            // in this sense, `is_homed` is just checking if the motor is enabled,
            // because otherwise it will put the motor FW in an error state
            return PlanishState::manual_jog;
          } else {
            combined_print("Home axis before jogging", 2000);
          }
        } else {
          combined_print("Secure mandrel latch before jogging", 2000);
        }
      }

      if (HmiSetJobStartButton.is_rising()) {
        if (MOTOR_ASSERTED && MOTOR_STEPS_COMPLETE) {
          mb.Hreg(HregAddr::HMI_JOB_START_POS_REG_ADDR, steps_to_hundreths(MOTOR_COMMANDED_POSITION));
          combined_print("Saved start position", 1000);
        } else {
          combined_print("Carriage still in motion, can't record start position", 2000);
        }
      }
      if (HmiSetJobEndButton.is_rising()) {
        if (MOTOR_ASSERTED && MOTOR_STEPS_COMPLETE) {
          mb.Hreg(HregAddr::HMI_JOB_END_POS_REG_ADDR, steps_to_hundreths(MOTOR_COMMANDED_POSITION));
          combined_print("Saved end position", 1000);
        } else {
          combined_print("Carriage still in motion, can't record end position", 2000);
        }
      }
      if (HmiSetJobParkButton.is_rising()) {
        if (MOTOR_ASSERTED && MOTOR_STEPS_COMPLETE) {
          mb.Hreg(HregAddr::HMI_JOB_PARK_POS_REG_ADDR, steps_to_hundreths(MOTOR_COMMANDED_POSITION));
          combined_print("Saved park position", 1000);
        } else {
          combined_print("Carriage still in motion, can't record park position", 2000);
        }
      }
      if (HmiCommitJobButton.is_rising()) {
        save_job_to_nvram();
        combined_print("Saved job to NVRAM and memory", 3000);
      }

      if (HmiStartCycleButton.is_rising()) {
        if (Fingers.is_fully_engaged() && is_homed && IS_MANDREL_SAFE) {
          return PlanishState::job_begin;
        } else if (!Fingers.is_fully_engaged()) {
          combined_print("Tried to start cycle without fingers engaged", 2000);
        } else if (!is_homed){
          combined_print("Tried to start cycle without being homed", 2000);
        } else {
          combined_print("Tried to start cycle without Mandrel latch being engaged", 2000);
        }
      }
      return PlanishState::idle;

    case PlanishState::wait_for_head:
      // if (loop_num%STATE_MACHINE_LOOPS_LOG_INTERVAL==0) {
      //   USB_PRINTLN(Head.get_commanded_state() ?
      //       "Waiting for head. Currently commanded down." :
      //       "Waiting for head. Currently commanded up.");
      // }
      if (!IS_MANDREL_SAFE && Head.get_commanded_state()) {
        // if the mandrel latch is not engaged and the head is being lowered
        combined_print("Mandrel latch disengaged while head was lowering. ESTOP", 10000);
        e_stop_handler(EstopReason::mandrel_latch);
        return PlanishState::e_stop_begin;
      }

      // if head was changed from manual control (only way to reach this state),
      // they should be able to 'undo' it without waiting for completion
      if (HmiCommandedRollerUpButton.is_rising()
          || HmiCommandedRollerDownButton.is_rising())
      {
        bool new_head_state; // < doesn't necessarily get acted on or set.
        if (HmiCommandedRollerUpButton.is_rising()) {
          new_head_state = false;
        } else { // if (HmiIsCommandedRollerDownButton.is_rising())
          new_head_state = true;
        }
        if(Fingers.is_fully_engaged() || !new_head_state) {
          // Make sure the fingers are down or the user is trying to raise the head.
          // Basically don't let them lower it without fingers engaged
          Head.set_commanded_state(new_head_state);
          return PlanishState::wait_for_head;
        } else {
          combined_print("Tried to lower head while fingers not down", 2000);
          return PlanishState::idle;
        }
      }
      if (!Head.is_mismatch()) {
        return PlanishState::idle;
      }
      return PlanishState::wait_for_head;

    case PlanishState::wait_for_fingers:
      // if (loop_num%STATE_MACHINE_LOOPS_LOG_INTERVAL==0) {
      //   USB_PRINT("Waiting for fingers. Currently commanded ");
      //   USB_PRINTLN(Fingers.get_commanded_state() ? "down." : "up.");
      // }
      if (!IS_MANDREL_SAFE) {
        combined_print("Mandrel became unsafe while fingers were moving. ESTOP", 10000);
        e_stop_handler(EstopReason::mandrel_latch);
        return PlanishState::e_stop_begin;
      }

      if (HmiCommandedFingersUpButton.is_rising()
        || HmiCommandedFingersDownButton.is_rising())
      {
        bool new_finger_state; // < doesn't necessarily get acted on or set.
        if(HmiCommandedFingersUpButton.is_rising()) {
          new_finger_state = false;
        } else { // if (HmiIsCommandedFingersDownButton.is_rising())
          new_finger_state = true;
        }
        USB_PRINTLN(new_finger_state ? "Engaging fingers" : "Disengaging fingers");
        Fingers.set_commanded_state(new_finger_state);
        return PlanishState::wait_for_fingers;
      }

      if (!Fingers.is_mismatch()) {
        return PlanishState::idle;
      }
      return PlanishState::wait_for_fingers;

    case PlanishState::manual_jog:
      if (IS_MANDREL_SAFE && is_homed) {
        if (IS_JOG_FWD_ACTIVE) {
          motor_jog(false);
          return PlanishState::manual_jog;
        }
        if (IS_JOG_REV_ACTIVE) {
          motor_jog(true);
          return PlanishState::manual_jog;
        }
        MOTOR_COMMAND(CARRIAGE_MOTOR.MoveStopDecel(););
        return PlanishState::idle;
      }
      if (!IS_MANDREL_SAFE) {
        combined_print("Mandrel unsafe while jogging! ESTOP", 2000);
        e_stop_handler(EstopReason::mandrel_latch);
        return PlanishState::e_stop_begin;
      }
      if (!is_homed) {
        combined_print("home axis before jogging", 2000);
      }
      return PlanishState::idle;

    case PlanishState::manual_jog_absolute:
      if (wait_for_motion()) {
        return PlanishState::idle;
        USB_PRINTLN("Manual jog absolute done")
      }
      return PlanishState::manual_jog_absolute;

    case PlanishState::e_stop_begin:
      estop_resume_state = secure_system(estop_last_state);
      USB_PRINT("secure_system returns");
      USB_PRINT(get_state_name(estop_resume_state));
      return PlanishState::e_stop_wait;

    case PlanishState::e_stop_wait:
      switch (estop_reason) {
        case EstopReason::NONE:
          USB_PRINTLN("E-Stop was triggered without setting the reason, changing to error");
          estop_reason = EstopReason::internal_error;
        case EstopReason::internal_error:
          return PlanishState::error;

        case EstopReason::button:
          if (!HmiMessage.is_active()) {
            combined_print("E-stopped by button. Release it to resume", 2000);
          }
          if ((ESTOP_SW.State() == ESTOP_SW_SAFE_STATE)
              && (millis() - last_estop_millis > ESTOP_COOLDOWN_MS))
          { // if the button was released and the estop has been active for over a second
            USB_PRINTLN("Emergency Stop Released, resuming");
            is_e_stop = false;
            return estop_resume_state;
          }
          return PlanishState::e_stop_wait;

        case EstopReason::mandrel_latch:
          if (!HmiMessage.is_active()) {
            combined_print("E-stopped by mandrel latch unsafe. Make sure it is secure", 2000);
          }
          if ((ESTOP_SW.State() == ESTOP_SW_SAFE_STATE)
              && (millis() - last_estop_millis > ESTOP_COOLDOWN_MS)
              && (IS_MANDREL_SAFE))
          {
            // if the latch was fixed and the button hasn't been pressed,
            // and the estop was activated more than `ESTOP_COOLDOWN_MS` ago
            combined_print("Mandrel put back, resuming", 2000);
            is_e_stop = false;
            return estop_resume_state;
          }
          return PlanishState::e_stop_wait;

        case EstopReason::motor_error:
          if (Head.is_fully_disengaged()) {
            if (!HmiMessage.is_active()) {
              combined_print("Motor error caused E-stop. Re-home motor to resume", 2000);
            }
            if ((ESTOP_SW.State() == ESTOP_SW_SAFE_STATE)
                && (millis() - last_estop_millis > ESTOP_COOLDOWN_MS)
                && (IS_MANDREL_SAFE)
                && HmiAxisHomingButton.is_rising())
            {
              // if the latch was fixed and the button hasn't been pressed,
              // and the estop was activated more than `ESTOP_COOLDOWN_MS` ago, AND the user is homing again
              USB_PRINTLN("Motor error cleared by homing, resuming");
              is_e_stop = false;
              return PlanishState::begin_homing; // this estop can only be recovered from if they press the home button
            }
            return PlanishState::e_stop_wait;
          } else if (!Head.get_commanded_state()) { // if the head is already raising
            if (!HmiMessage.is_active()) {
              combined_print("Estop caused by motor error. wait for head raise and then home", 2000);
            }
            return PlanishState::e_stop_wait;
          } else {
            // Head is fully engaged, so the motor can't start homing yet.
            if (!HmiMessage.is_active()) {
              combined_print("Estop caused by motor error. Raise head and then home", 2000);
            }
            if (HmiCommandedRollerUpButton.is_rising()) {
              Head.set_commanded_state(true);
              combined_print("Raising head", 1000);
            }
            return PlanishState::error;
          }
      }
      // should be unreachable
      return PlanishState::error;

    case PlanishState::error:
      fault_code = FaultCodes::Error;
      check_modbus(); // last words, if able
      strncpy(message_buffer, "Unrecoverable error.Restart machine.estop reason: ", 50);
      strncpy(message_buffer + 50, get_estop_reason_name(estop_reason), 14); // 14 is max length for estop reason

      mb.Coil(CoilAddr::IS_FAULT, true);
      mb.Coil(CoilAddr::SHOW_MESSAGE, true);
      mb.Coil(CoilAddr::IS_E_STOP, true);

      while (true) {
        const auto values = HmiMessage.get_message_u16();
        for (uint8_t i = 0; i < Message::MessageClass::message_u16_len_max; i++) {
          mb.Hreg(i+HregAddr::MESSAGE_START, values[i]);
        }
        if (!HmiMessage.is_active()) {
          combined_print(message_buffer, 1000);
        }
        mb.task();
        delay(100);
      } // </ while(true) >
      // this should obviously never be reached
      return PlanishState::error;

    case PlanishState::job_begin:
      COMMON_JOB_TASKS();
      Head.set_commanded_state(false); // head is supposed to already be raised, but this needs to be called
      // despite the sensor check in case it was in the proccess of lowering already
      if (Head.is_fully_disengaged()) {
        USB_PRINTLN("Job started head was already up");
        return PlanishState::job_jog_to_start;
      }
      combined_print("Job was started but head was already down, raising it to begin", 4000);
      return PlanishState::job_begin_lifting_head;

    case PlanishState::job_begin_lifting_head:
      COMMON_JOB_TASKS();
      if (!IS_MANDREL_SAFE) {
        combined_print("mandrel limit precondition failed since starting job. ESTOP", 10000);
        e_stop_handler(EstopReason::mandrel_latch);
        return PlanishState::e_stop_begin;
      }
      if (Head.is_fully_disengaged()) {
        return PlanishState::job_jog_to_start;
      }
      if (loop_num%STATE_MACHINE_LOOPS_LOG_INTERVAL==0) {
        USB_PRINTLN("Waiting for head to rise for job start");
      }
      if (HmiCancelCycleButton.is_rising()) {
        return PlanishState::job_cancel;
      }
      return PlanishState::job_begin_lifting_head;

    case PlanishState::job_jog_to_start:
      COMMON_JOB_TASKS();
      combined_print("Jog to start", 1000);
      move_motor_auto_speed(saved_job_start_pos);
      return PlanishState::job_jog_to_start_wait;

    case PlanishState::job_jog_to_start_wait:
      COMMON_JOB_TASKS();
      MOTOR_COMMAND(CARRIAGE_MOTOR.VelMax(current_jog_speed););
      if (wait_for_motion()) {
        return PlanishState::job_head_down;
      }
      if (loop_num%STATE_MACHINE_LOOPS_LOG_INTERVAL==0) {
        USB_PRINTLN("waiting for jog to start");
      }
      return PlanishState::job_jog_to_start_wait;

    case PlanishState::job_head_down:
      COMMON_JOB_TASKS();
      combined_print("Lower head", 1000);
      Head.set_commanded_state(true);
      return PlanishState::job_head_down_wait;

    case PlanishState::job_head_down_wait:
      COMMON_JOB_TASKS();
      if (!IS_MANDREL_SAFE) {
        combined_print("mandrel limit precondition failed since starting job. ESTOP", 10000);
        e_stop_handler(EstopReason::mandrel_latch);
        return PlanishState::e_stop_begin;
      }
      if (Head.is_fully_engaged()) {
        return PlanishState::job_planish_to_end;
      }
      if (loop_num%STATE_MACHINE_LOOPS_LOG_INTERVAL==0) {
        USB_PRINTLN("Waiting for head to lower to begin planish");
      }
      return PlanishState::job_head_down_wait;

    case PlanishState::job_planish_to_end:
      COMMON_JOB_TASKS();
      combined_print("Planishing to end position", 1000);
      move_motor_auto_speed(saved_job_end_pos);
      return PlanishState::job_planish_to_end_wait;

    case PlanishState::job_planish_to_end_wait:
      COMMON_JOB_TASKS();
      MOTOR_COMMAND(CARRIAGE_MOTOR.VelMax(current_planish_speed););
      if (wait_for_motion()) {
        if (mb.Coil(CoilAddr::IS_DUAL_PASS_MODE)) {
          combined_print("full cycle detected, do second pass", 2000);
          // otherwise do the other pass
          return PlanishState::job_planish_to_start;
        } else {
          combined_print("half cycle detected, raise head and go to park", 2000);
          // if only doing a half-cycle, the single pass thus far is sufficient. retract head and return to park
          return PlanishState::job_head_up;
        }
      }
      if (loop_num%STATE_MACHINE_LOOPS_LOG_INTERVAL==0) {
        USB_PRINTLN("Planishing to end position");
      }
      return PlanishState::job_planish_to_end_wait;

    case PlanishState::job_planish_to_start:
      COMMON_JOB_TASKS();
      USB_PRINTLN("Begin planishing to start position");
      move_motor_auto_speed(saved_job_start_pos);
      return PlanishState::job_planish_to_start_wait;

    case PlanishState::job_planish_to_start_wait:
      COMMON_JOB_TASKS();
      MOTOR_COMMAND(CARRIAGE_MOTOR.VelMax(current_planish_speed););
      if (wait_for_motion()) {
        return PlanishState::job_head_up;
      }
      if (loop_num%STATE_MACHINE_LOOPS_LOG_INTERVAL==0) {
        USB_PRINTLN("Planishing to start position");
      }
      return PlanishState::job_planish_to_start_wait;

    case PlanishState::job_head_up:
      COMMON_JOB_TASKS();
      USB_PRINTLN("Raising head");
      Head.set_commanded_state(false);
      return PlanishState::job_head_up_wait;

    case PlanishState::job_head_up_wait:
      COMMON_JOB_TASKS();
      if (!IS_MANDREL_SAFE) {
        combined_print("mandrel limit precondition failed since starting job. ESTOP", 10000);
        e_stop_handler(EstopReason::mandrel_latch);
        return PlanishState::e_stop_begin;
      }
      if (Head.is_fully_disengaged()) {
        return PlanishState::job_jog_to_park;
      }
      if (loop_num%STATE_MACHINE_LOOPS_LOG_INTERVAL==0) {
        USB_PRINTLN("Waiting for head to raise to park");
      }
      return PlanishState::job_head_up_wait;

    case PlanishState::job_jog_to_park:
      COMMON_JOB_TASKS();
      combined_print("Jog to park position", 2000);
      move_motor_auto_speed(saved_job_park_pos);
      return PlanishState::job_jog_to_park_wait;

    case PlanishState::job_jog_to_park_wait:
      COMMON_JOB_TASKS();
      MOTOR_COMMAND(CARRIAGE_MOTOR.VelMax(current_jog_speed););
      if (wait_for_motion()) {
        return PlanishState::idle;
      }
      if (loop_num%STATE_MACHINE_LOOPS_LOG_INTERVAL==0) {
        USB_PRINTLN("Jogging to park position");
      }
      return PlanishState::job_jog_to_park_wait;

    case PlanishState::job_paused:
      COMMON_JOB_TASKS();
      if (HmiStartCycleButton.is_rising()) {
        if (Fingers.is_fully_engaged() && IS_MANDREL_SAFE) {
          return job_resume_state;
        } else if (!Fingers.is_fully_engaged()) {
          combined_print("Tried to start cycle without fingers engaged", 2000);
        } else {
          combined_print("Tried to start cycle without mandrel safe", 2000);
        }
      }
      return PlanishState::job_paused;

    case PlanishState::job_cancel:
      COMMON_JOB_TASKS();
      combined_print("Job cancelled", 2000);
      MOTOR_COMMAND(CARRIAGE_MOTOR.MoveStopDecel(););
      return PlanishState::idle;
  }
  // this means the state machine did not already give a state
  combined_print("State machine executed without returning the next state", 10000);
  USB_PRINT("Previous state: ");
  USB_PRINTLN(get_state_name(state_in));
  e_stop_handler(EstopReason::internal_error);
  return PlanishState::e_stop_begin;
}

/**
 * Stuff that should be run on every cycle if a job is active.
 * @return (State to go to, needs attention)
 */
std::pair<PlanishState, bool> common_job_tasks() {
  if (HmiCancelCycleButton.is_rising()) {
    return {PlanishState::job_cancel, true};
  }
  if (HmiPauseCycleButton.is_rising()) {
    pause_job(machine_state);
    return {PlanishState::job_paused, true};
  }
  return {machine_state, false};
}

void pause_job(const PlanishState state_before_pause) {
  MOTOR_COMMAND(CARRIAGE_MOTOR.MoveStopDecel(););
  job_resume_state = job_pause_get_resume_state(state_before_pause);
}

PlanishState job_pause_get_resume_state(const PlanishState state_before_pause) {
  switch (state_before_pause) {
    case PlanishState::job_begin:
      return PlanishState::job_begin;

    case PlanishState::job_begin_lifting_head:
      return PlanishState::job_begin_lifting_head;

    case PlanishState::job_jog_to_start:
    case PlanishState::job_jog_to_start_wait:
      return PlanishState::job_jog_to_start;

    case PlanishState::job_head_down:
    case PlanishState::job_head_down_wait:
      return PlanishState::job_head_down;

    case PlanishState::job_planish_to_end:
    case PlanishState::job_planish_to_end_wait:
      return PlanishState::job_planish_to_end;


    case PlanishState::job_planish_to_start:
    case PlanishState::job_planish_to_start_wait:
      return PlanishState::job_planish_to_start;

    case PlanishState::job_head_up:
    case PlanishState::job_head_up_wait:
      return PlanishState::job_head_up;

    case PlanishState::job_jog_to_park:
    case PlanishState::job_jog_to_park_wait:
      return PlanishState::job_jog_to_park;

    default:
      hmi_print("Paused from invalid state. ESTOP", 10000);
      USB_PRINTLN("Invalid state before pause. Job should only be paused from a job state that isnt pause or cancel");
      e_stop_handler(EstopReason::internal_error);
      return PlanishState::e_stop_begin;
  }
}

/**
 * Configures IO, go figure
 * @return True for success
 */
bool configure_io() {

  FINGER_DOWN_LMT.Mode(Connector::INPUT_DIGITAL);
  MANDREL_LATCH_LMT.Mode(Connector::INPUT_DIGITAL);
  HEAD_UP_LMT.Mode(Connector::INPUT_DIGITAL);


  AdcMgr.AdcResolution(ADC_RES_BITS);

  CCIO1.Mode(Connector::CCIO);
  CCIO1.PortOpen();

  if (CcioMgr.LinkBroken()) {
    uint32_t lastStatusTime = Milliseconds();
    USB_PRINTLN("The CCIO-8 link is broken!");

    while (CcioMgr.LinkBroken() && Milliseconds() - lastStatusTime < CCIO_TIMEOUT_MS) {
      if (Milliseconds() - lastStatusTime > 1000) {
        USB_PRINTLN("The CCIO-8 link is still broken!");
        lastStatusTime = Milliseconds();
      }
    }
    if (CcioMgr.LinkBroken()) {
      USB_PRINTLN("Timed out waiting for CCIO");
      return false;
    }
    USB_PRINTLN("The CCIO-8 link is online again!");
  }

  if (CcioMgr.CcioCount() != EXPECTED_NUM_CCIO) {
    USB_PRINT("Expected to find exactly one CCIO connector, found ");
    USB_PRINTLN(CcioMgr.CcioCount());
    return false;
  }

  config_ccio_pin(FINGER_ACTUATION, Connector::OUTPUT_DIGITAL);
  config_ccio_pin(HEAD_ACTUATION, Connector::OUTPUT_DIGITAL);


  Fingers.set_actuator_pin(CcioMgr.PinByIndex(FINGER_ACTUATION));
  Fingers.set_sense_pin(&FINGER_DOWN_LMT);
  Fingers.set_commanded_state(false);
//  Fingers.set_commanded_state(Fingers.get_measured_state());
  // TODO: what is the desired behavior for finger and head state on boot
  Head.set_actuator_pin(CcioMgr.PinByIndex(HEAD_ACTUATION));
  Head.set_sense_pin(&HEAD_UP_LMT, true);
//  Head.set_commanded_state(Head.get_measured_state());
  Head.set_commanded_state(false);


  MOTOR_COMMAND(MotorMgr.MotorInputClocking(MotorManager::CLOCK_RATE_NORMAL););
  MOTOR_COMMAND(MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL, Connector::CPM_MODE_STEP_AND_DIR););

  MOTOR_COMMAND(CARRIAGE_MOTOR.HlfbMode(MotorDriver::HLFB_MODE_HAS_BIPOLAR_PWM););
  MOTOR_COMMAND(CARRIAGE_MOTOR.HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ););

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
    USB_PRINTLN("Motor has completed motion");
    return true;
  }
  if (MOTOR_HAS_ERRORS) {
    // motor has an error
    USB_PRINTLN("Motor alert detected.");
    print_motor_alerts();
    e_stop_handler(EstopReason::motor_error);
    return false;
  }

  if (!IS_MANDREL_SAFE) {
    combined_print("mandrel limit precondition failed while carriage moving. ESTOP", 10000);
    e_stop_handler(EstopReason::mandrel_latch);
  }
  return false;
}

/**
 * Interrupt handler gets automatically called every ms
 */
extern "C" void PeriodicInterrupt(void) {
  HmiMessage.tick();

  // Check the time since the last loop was run.
  // If it's been too long, then something is wrong.
#ifdef ENABLE_ITERATION_TIME_CHECK
  iteration_time_check();
#endif
  // Acknowledge the interrupt to clear the flag and wait for the next interrupt.
  TCC2->INTFLAG.reg = TCC_INTFLAG_MASK; // This is critical
}

/**
 * Check interval between calls.
 * This should be called regularly
 */
#ifdef ENABLE_ITERATION_TIME_CHECK
void iteration_time_check() {
  last_iteration_delta = millis()-last_iteration_time;
  if (
    machine_state == PlanishState::error ||
    machine_state == PlanishState::e_stop_begin ||
    machine_state == PlanishState::e_stop_wait
  ) {
    // error is intentionally a dead end
    return;
  }
  if (last_iteration_delta >= ITERATION_TIME_ERROR_MS && !timeout_error_since_last_loop) {
    timeout_error_since_last_loop = true;
    USB_PRINT("Last iteration of the state machine took more than `ITERATION_TIME_ERROR_MS` (");
    USB_PRINT(ITERATION_TIME_ERROR_MS);
    USB_PRINT("ms) to complete. This is likely a bug. Last iteration took ");
    USB_PRINT(last_iteration_delta);
    USB_PRINT("ms. Engaging E-Stop and stopping execution. Last state: ");
    USB_PRINTLN(get_state_name(machine_state));
    hmi_print("Watchdog error. loop likely reached dead end", 10000); // this probably will never be seen, but just in case
    fault_code = FaultCodes::SmTimeout;
    e_stop_handler(EstopReason::internal_error);
  } else if (last_iteration_delta >= ITERATION_TIME_WARNING_MS && !timeout_warning_since_last_loop) {
    timeout_warning_since_last_loop = true;
    USB_PRINT("Last iteration of the state machine took more than `ITERATION_TIME_WARNING_MS` (");
    USB_PRINT(ITERATION_TIME_WARNING_MS);
    USB_PRINT("ms) to complete. This is likely a bug. Last iteration took ");
    USB_PRINT(last_iteration_delta);
    USB_PRINT("ms. Continuing execution. Last state: ");
    USB_PRINTLN(get_state_name(machine_state));
    hmi_print("Watchdog warning. loop running dangerously slow", 10000);
  }
}
#endif
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
  USB_PRINTLN("ISR");
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
    USB_PRINTLN("Emergency Stop");
  } else {
    USB_PRINTLN("Emergency Stop Handler called when e-stop was already active");
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
  USB_PRINTLN("print_motor_alerts()");
#else
  combined_print("ClearPath Alerts present: ", 4000);
  if(CARRIAGE_MOTOR.AlertReg().bit.MotionCanceledInAlert){
    USB_PRINTLN("    MotionCanceledInAlert "); }
  if(CARRIAGE_MOTOR.AlertReg().bit.MotionCanceledPositiveLimit){
    USB_PRINTLN("    MotionCanceledPositiveLimit "); }
  if(CARRIAGE_MOTOR.AlertReg().bit.MotionCanceledNegativeLimit){
    USB_PRINTLN("    MotionCanceledNegativeLimit "); }
  if(CARRIAGE_MOTOR.AlertReg().bit.MotionCanceledSensorEStop){
    USB_PRINTLN("    MotionCanceledSensorEStop "); }
  if(CARRIAGE_MOTOR.AlertReg().bit.MotionCanceledMotorDisabled){
    USB_PRINTLN("    MotionCanceledMotorDisabled "); }
  if(CARRIAGE_MOTOR.AlertReg().bit.MotorFaulted){
    USB_PRINTLN("    MotorFaulted ");
  }
#endif
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
 */
void save_job_to_nvram() {

  const uint16_t temp_job_start_pos_hundreths = mb.Hreg(HregAddr::HMI_JOB_START_POS_REG_ADDR);
  const uint16_t temp_job_end_pos_hundreths = mb.Hreg(HregAddr::HMI_JOB_END_POS_REG_ADDR);
  const uint16_t temp_job_park_pos_hundreths = mb.Hreg(HregAddr::HMI_JOB_PARK_POS_REG_ADDR);

  saved_job_start_pos = hundreths_to_steps(temp_job_start_pos_hundreths);
  saved_job_end_pos = hundreths_to_steps(temp_job_end_pos_hundreths);
  saved_job_park_pos = hundreths_to_steps(temp_job_park_pos_hundreths);

  USB_PRINTLN("job retrieved from HMI (hundreths/raw)");
  USB_PRINTLN(temp_job_start_pos_hundreths);
  USB_PRINTLN(temp_job_end_pos_hundreths);
  USB_PRINTLN(temp_job_park_pos_hundreths);

  USB_PRINTLN("job retrieved from HMI (steps/final)");
  USB_PRINTLN(saved_job_start_pos);
  USB_PRINTLN(saved_job_end_pos);
  USB_PRINTLN(saved_job_park_pos);

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
  USB_PRINT("move_motor_with_speed(");
  USB_PRINT(position);
  USB_PRINT(", ");
  USB_PRINT(speed);
  USB_PRINT(");");
  MOTOR_COMMAND(CARRIAGE_MOTOR.VelMax(speed););
  return MOTOR_COMMAND(CARRIAGE_MOTOR.Move(position, StepGenerator::MOVE_TARGET_ABSOLUTE););
}

/**
 * Very simple helper function to avoid forgetting to call velmax, but this time, speed is calculated
 * @param position
 * @return whatever `CARRIAGE_MOTOR.Move` returns. (no one knows atm)
 */
bool move_motor_auto_speed(const int32_t position) {
  USB_PRINT("move_motor_auto_speed(");
  USB_PRINT(position);
  USB_PRINT(");");
  const int32_t speed = Head.is_fully_disengaged() ? current_jog_speed : current_planish_speed;
  MOTOR_COMMAND(CARRIAGE_MOTOR.VelMax(speed););
  return MOTOR_COMMAND(CARRIAGE_MOTOR.Move(position, StepGenerator::MOVE_TARGET_ABSOLUTE););
}

void motor_jog(const bool reverse) {
  if (Head.is_fully_disengaged()) {
#ifdef TEST_MODE_DISABLE_MOTOR
    USB_PRINT("Jog mode CARRIAGE_MOTOR.MoveVelocity(");
    USB_PRINT(reverse ? -current_jog_speed : current_jog_speed);
    USB_PRINTLN(");");
#else
    CARRIAGE_MOTOR.MoveVelocity(reverse ? -current_jog_speed : current_jog_speed);
#endif
  } else {
#ifdef TEST_MODE_DISABLE_MOTOR
    USB_PRINT("Planish Mode CARRIAGE_MOTOR.MoveVelocity(");
    USB_PRINT(reverse ? -current_planish_speed : current_planish_speed);
    USB_PRINTLN(");");
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
  mb_read_unlatch_coil(&HmiAxisHomingButton);
  mb_read_unlatch_coil(&HmiSetJobStartButton);
  mb_read_unlatch_coil(&HmiSetJobEndButton);
  mb_read_unlatch_coil(&HmiSetJobParkButton);
  mb_read_unlatch_coil(&HmiCommitJobButton);
  mb_read_unlatch_coil(&HmiCommandedFingersUpButton);
  mb_read_unlatch_coil(&HmiCommandedFingersDownButton);
  mb_read_unlatch_coil(&HmiCommandedRollerUpButton);
  mb_read_unlatch_coil(&HmiCommandedRollerDownButton);
  mb_read_unlatch_coil(&HmiCommandedPosButton);
  mb_read_unlatch_coil(&HmiStartCycleButton);
  mb_read_unlatch_coil(&HmiCancelCycleButton);
  mb_read_unlatch_coil(&HmiPauseCycleButton);
}


/**
 * Handle stopping and securing every part of the system.
 * @param last_state The state of the system before the e-stop
 * @return The state to resume to once the E-stop cause was rectified
 */
PlanishState secure_system(const PlanishState last_state) {
  USB_PRINT("secure_system called with state: ");
  USB_PRINT(get_state_name(last_state));
  switch (last_state) {
    case PlanishState::post:
      // can happen if E-stop ISR goes off erroneously on boot
      return PlanishState::post;

    case PlanishState::begin_homing:
    case PlanishState::homing_wait_for_disable:
    case PlanishState::wait_for_homing:
      // special case where telling the motor to stop won't stop it. it needs to be disabled.
      MOTOR_COMMAND(CARRIAGE_MOTOR.EnableRequest(false););
      is_homed = false;
      return PlanishState::idle;

    case PlanishState::idle:
      return PlanishState::idle;

    case PlanishState::manual_jog:
    case PlanishState::manual_jog_absolute:
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
    case PlanishState::job_paused:
      return PlanishState::job_paused;
    case PlanishState::job_cancel:
      return PlanishState::job_cancel;
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
    STATE_NAME(manual_jog_absolute);
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
    STATE_NAME(job_paused);
    STATE_NAME(job_cancel);
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

