//
// Created by paulw on 4/17/2025.
//

#ifndef REGISTERDEFINITIONS_H
#define REGISTERDEFINITIONS_H

#define IS_MANDREL_LATCH_CLOSED_COIL_ADDR           0     // R    mandrel latch sensor reading. True for closed and safe
#define IS_FINGERS_DOWN_COIL_ADDR                   1     // R    Are the workpiece holding fingers commanded down
#define IS_HOMED_COIL_ADDR                          2     // R    Has the axis been homed
#define IS_FAULT_COIL_ADDR                          3     // R    A fault state indicating a software issue; HMI shows a problem with the CC
#define IS_READY_FOR_CYCLE_COIL_ADDR                4     // R    Is the clearcore ready to execute its program
#define IS_E_STOP_COIL_ADDR                         5     // R    Is the E-stop currently active
#define IS_JOB_ACTIVE_COIL_ADDR                     6     // R    Is the clearcore executing its program
#define IS_READY_FOR_MANUAL_CONTROL_COIL_ADDR       7     // R    Is the clearcore ready for manual control, ensuring safety checks and no cycle in progress
#define IS_ROLLER_DOWN_COIL_ADDR                    8     // R    Is the roller down/engaged
#define IS_COMMANDED_POS_COIL_ADDR                  20    // R/W  Has the HMI requested a new commanded position. If true, reads the commanded position
#define IS_RTH_BUTTON_LATCHED_COIL_ADDR             30    // R/W  Has the 'return to home' button been pressed. Resets to 0 on completion
#define IS_AXIS_HOMING_BUTTON_LATCHED_COIL_ADDR     31    // R/W  Has the 'run axis homing sequence' button been pressed. Resets to 0 on completion
#define IS_SET_JOB_START_BUTTON_LATCHED_COIL_ADDR   32    // R/W  Has the 'Set start to current position' button been pressed. Resets to 0 on completion
#define IS_SET_JOB_END_BUTTON_LATCHED_COIL_ADDR     33    // R/W  Has the 'Set end to current position' button been pressed. Resets to 0 on completion
#define IS_SET_JOB_PARK_BUTTON_LATCHED_COIL_ADDR    34    // R/W  Has the 'Set park to current position' button been pressed. Resets to 0 on completion
#define COMMANDED_FINGERS_COIL_ADDR                 40    // R/W  Finger state commanded by the HMI, true means engaged, false means disengaged
#define COMMANDED_ROLLER_COIL_ADDR                  41    // R/W  Roller state commanded by the HMI, true means engaged, false means disengaged

// HREG

#define ACTUAL_POSITION_REG_ADDR                    2     // R    actual position of the axis measured in hundredths of an inch
#define COMMANDED_POSITION_REG_ADDR                 12    // R/W  commanded position of the axis measured in hundredths of an inch.
                                                          //      The commanded position will not be acted upon if `IS_COMMANDED_POS_COIL_ADDR` is not set.
#define JOB_PROGRESS_REG_ADDR                       16    // R    The progress of the current job if there is one. Expressed as (job_progress/65536)*100%.
#define JOB_START_POS_REG_ADDR                      19    // R    The currently saved start position of the axis measured in hundredths of an inch.
                                                          //      (While the register is read-only, the value can semantically change otherwise.)
#define JOB_END_POS_REG_ADDR                        21    // R    Same as JOB_START_POS_REG_ADDR but indicates the end position of the job.
#define JOB_PARK_POS_REG_ADDR                       23    // R    Same as JOB_START_POS_REG_ADDR but indicates the park position.
#define MIN_POS_REG_ADDR                            25    // R    Minimum absolute position of the axis measured in hundredths of an inch.
#define MAX_POS_REG_ADDR                            26    // R    Maximum absolute position of the axis measured in hundredths of an inch.
#define JOG_SPEED_REG_ADDR                          32    // R/W  target speed while jogging manually or in disengaged portion of job sequence
#define PLANISH_SPEED_REG_ADDR                      33    // R/W  target speed while in engaged portion of job sequence
#define FAULT_CODE_REG_ADDR                         40    // R    Fault code. 0 is normal operation

#endif // REGISTERDEFINITIONS_H
