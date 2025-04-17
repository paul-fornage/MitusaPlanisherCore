/*
  ModbusTCP Client for ClearCode Arduino wrapper

  (c)2021 Alexander Emelianov (a.m.emelianov@gmail.com)
  https://github.com/emelianov/modbus-esp8266

  This code is licensed under the BSD New License. See LICENSE.txt for more info.
*/

#include <Ethernet.h>       // Ethernet library v2 is required

#include <ModbusAPI.h>
#include <ModbusTCPTemplate.h>
#include "ClearCore.h"

#include "RegisterDefinitions.h"

// ModBus TCP stuff
class ModbusEthernet : public ModbusAPI<ModbusTCPTemplate<EthernetServer, EthernetClient>> {};
const IPAddress remote(192, 168, 0, 211);  // Address of Modbus Slave device
byte mac[] = { 0x24, 0x15, 0x10, 0xB0, 0x45, 0xA4 }; // MAC address is ignored but because of C++ types, you still need to give it garbage
IPAddress ip(192, 168, 0, 178); // The IP address will be dependent on your local network
ModbusEthernet mb;               // Declare ModbusTCP instance

bool is_HMI_comm_good = false;

bool is_mandrel_latch_closed;           // COIL R   Mandrel latch sensor reading. True for closed and safe
bool is_fingers_down;                   // COIL R   Are the workpiece holding fingers commanded down
bool is_homed;                          // COIL R   Has the axis been homed
bool is_fault;                          // COIL R   Software fault detection. Indicates an unexpected condition
bool is_ready_for_cycle;                // COIL R   Is the clearcore ready to execute its program
bool is_e_stop;                         // COIL R   Is the Emergency Stop currently active
bool is_job_active;                     // COIL R   Is the clearcore executing its program
bool is_ready_for_manual_control;       // COIL R   Is the clearcore ready for manual control commands
bool is_roller_down;                    // COIL R   Is the roller down/engaged
bool is_commanded_pos;                  // COIL R/W Has the HMI requested a new commanded position?
bool is_rth_button_latched;             // COIL R/W 'Return to home' button latch state
bool is_axis_homing_button_latched;     // COIL R/W 'Run axis homing sequence' button latch state
bool is_set_job_start_button_latched;   // COIL R/W 'Set start to current position' button latch state
bool is_set_job_end_button_latched;     // COIL R/W 'Set end to current position' button latch state
bool is_set_job_park_button_latched;    // COIL R/W 'Set park to current position' button latch state
bool commanded_fingers;                 // COIL R/W Finger state commanded by the HMI, true means engaged, false means disengaged
bool commanded_roller;                  // COIL R/W Roller state commanded by the HMI, true means engaged, false means disengaged


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


#define ESTOP_SW ConnectorA12
#define CYCLE_START_SW ConnectorA11
#define CYCLE_START_LIGHT ConnectorIO2
#define FINGER_ACTUATION ConnectorIO0
#define ROLLER_ACTUATION ConnectorIO1
#define CARRIAGE_MOTOR ConnectorM0

#define HMI_CONNECTION_TRIES_BEFORE_ERROR 5

void configure_io();
void e_stop_handler();
bool read_coils();
bool read_registers();


void setup() {
  Serial.begin(115200);
  const uint32_t timeout = 5000;
  const uint32_t startTime = millis();
  while (!Serial && millis() - startTime < timeout)
    continue;

  ESTOP_SW.InterruptHandlerSet(e_stop_handler);

  Ethernet.begin(mac, ip);

  // Make sure the physical link is up before continuing.
  while (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Waiting for Ethernet connection...");
    delay(100);
  }
  mb.client();
  while (!mb.isConnected(remote)) {
    mb.connect(remote);
    delay(500);
    Serial.println("Waiting for Modbus connection...");
  }
  is_HMI_comm_good = true;
}

bool coil = false;
uint16_t reg1 = 0;
uint16_t reg2 = 0;
uint32_t showLast = 0;
uint16_t sum = 0;

void loop() {
  if (ESTOP_SW.State()) {
    Serial.println("Emergency Stop");
  } else {
    Serial.println("Normal Operation");
  }
  if (mb.isConnected(remote)) {   // Check if connection to Modbus Slave is established
    mb.readCoil(remote, 100, &coil);  // Initiate Read Hreg from Modbus Slave
    Serial.print("coil 100: ");
    Serial.println(coil ? "True" : "False");
    mb.readHreg(remote, 16, &reg1);  // Initiate Read Hreg from Modbus Slave
    Serial.print("reg 16: ");
    Serial.println(reg1);
    mb.readHreg(remote, 17, &reg2);  // Initiate Read Hreg from Modbus Slave
    Serial.print("reg 17: ");
    Serial.println(reg2);
    sum = reg1+reg2;
    mb.writeHreg(remote, 18, &sum);  // Initiate Read Hreg from Modbus Slave

  } else {
    is_HMI_comm_good = false;
    mb.connect(remote);           // Try to connect if not connected
  }
  delay(10);                     // Pulling interval
  mb.task();                      // Common local Modbus task
}


void configure_io() {
  ESTOP_SW.Mode(Connector::INPUT_DIGITAL);
  CYCLE_START_SW.Mode(Connector::INPUT_DIGITAL);
  CYCLE_START_LIGHT.Mode(Connector::OUTPUT_DIGITAL);
  FINGER_ACTUATION.Mode(Connector::OUTPUT_DIGITAL);
  ROLLER_ACTUATION.Mode(Connector::OUTPUT_DIGITAL);
}

void e_stop_handler() {
  Serial.println("Emergency Stop");
}

/**
 * Reads all coils that can be changed by the HMI
 * @return true for success
 */
bool read_coils() {
  uint8_t remaining_tries = HMI_CONNECTION_TRIES_BEFORE_ERROR;
  while (!mb.isConnected(remote) && remaining_tries > 0) {
    mb.connect(remote);
    remaining_tries--;
  }
  if (remaining_tries == 0) {
    return false;
  }

  mb.readCoil(remote, IS_COMMANDED_POS_COIL_ADDR, &is_commanded_pos);
  mb.readCoil(remote, IS_RTH_BUTTON_LATCHED_COIL_ADDR, &is_rth_button_latched);
  mb.readCoil(remote, IS_AXIS_HOMING_BUTTON_LATCHED_COIL_ADDR, &is_axis_homing_button_latched);
  mb.readCoil(remote, IS_SET_JOB_START_BUTTON_LATCHED_COIL_ADDR, &is_set_job_start_button_latched);
  mb.readCoil(remote, IS_SET_JOB_END_BUTTON_LATCHED_COIL_ADDR, &is_set_job_end_button_latched);
  mb.readCoil(remote, IS_SET_JOB_PARK_BUTTON_LATCHED_COIL_ADDR, &is_set_job_park_button_latched);
  mb.readCoil(remote, COMMANDED_FINGERS_COIL_ADDR, &commanded_fingers);
  mb.readCoil(remote, COMMANDED_ROLLER_COIL_ADDR, &commanded_roller);

  return true;
}

/**
 * Reads all registers that can be set by HMI
 * @return True for success
 */
bool read_registers() {
  uint8_t remaining_tries = HMI_CONNECTION_TRIES_BEFORE_ERROR;
  while (!mb.isConnected(remote) && remaining_tries > 0) {
    mb.connect(remote);
    remaining_tries--;
  }
  if (remaining_tries == 0) {
    return false;
  }
  mb.readHreg(remote, COMMANDED_POSITION_REG_ADDR, &commanded_position);
  mb.readHreg(remote, JOG_SPEED_REG_ADDR, &jog_speed);
  mb.readHreg(remote, PLANISH_SPEED_REG_ADDR, &planish_speed);
  return true;
}
