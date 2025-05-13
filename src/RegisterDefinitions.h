//
// Created by paulw on 4/17/2025.
//

#ifndef REGISTERDEFINITIONS_H
#define REGISTERDEFINITIONS_H

namespace CoilAddr {
    enum CoilAddr {
        IS_MANDREL_LATCH_CLOSED         = 0,     // R    mandrel latch sensor reading. True for closed and safe
        IS_FINGERS_DOWN                 = 1,     // R    Are the workpiece holding fingers commanded down
        IS_HOMED                        = 2,     // R    Has the axis been homed
        IS_FAULT                        = 3,     // R    A fault state indicating a software issue; HMI shows a problem with the CC
        IS_READY_FOR_CYCLE              = 4,     // R    Is the clearcore ready to execute its program
        IS_E_STOP                       = 5,     // R    Is the E-stop currently active
        IS_JOB_ACTIVE                   = 6,     // R    Is the clearcore executing its program? Remains true while paused
        IS_READY_FOR_MANUAL_CONTROL     = 7,     // R    Is the clearcore ready for manual control, ensuring safety checks and no cycle in progress
        IS_ROLLER_DOWN                  = 8,     // R    Is the roller down/engaged
        CC_COMMANDED_ROLLER             = 9,     // R    Roller state commanded by the CC, true means engaged, false means disengaged
        CC_COMMANDED_FINGERS            = 10,    // R    Finger state commanded by the CC, true means engaged, false means disengaged
        IS_COMMANDED_POS_LATCHED        = 21,    // R/W  Has the HMI requested a new commanded position. If true, reads the commanded position
        IS_RTH_BUTTON_LATCHED           = 30,    // R/W  Is the "Return to Home" button latched
        IS_AXIS_HOMING_BUTTON_LATCHED   = 31,    // R/W  Has the 'run axis homing sequence' button been pressed. Resets to 0 on completion
        IS_SET_JOB_START_BUTTON_LATCHED = 32,    // R/W  Has the 'Set start to current position' button been pressed. Resets to 0 on completion
        IS_SET_JOB_END_BUTTON_LATCHED   = 33,    // R/W  Has the 'Set end to current position' button been pressed. Resets to 0 on completion
        IS_SET_JOB_PARK_BUTTON_LATCHED  = 34,    // R/W  Has the 'Set park to current position' button been pressed. Resets to 0 on completion
        IS_COMMIT_JOB_BUTTON_LATCHED    = 35,    // R/W  Has the 'Commit job' button been pressed. Resets to 0 on completion
        IS_FINGER_UP_LATCHED            = 40,    // R/W  Has the 'Finger up' button been pressed. Resets to 0 on completion
        IS_FINGER_DOWN_LATCHED          = 41,    // R/W  Has the 'Finger down' button been pressed. Resets to 0 on completion
        IS_ROLLER_UP_LATCHED            = 42,    // R/W  Has the 'Roller up' button been pressed. Resets to 0 on completion
        IS_ROLLER_DOWN_LATCHED          = 43,    // R/W  Has the 'Roller down' button been pressed. Resets to 0 on completion
        IS_JOG_POS_PRESSED              = 44,    // R    Jogging in positive direction button pressed
        IS_JOG_NEG_PRESSED              = 45,    // R    Jogging in negative direction button pressed
        IS_START_CYCLE_BUTTON_LATCHED   = 46,    // R/W  Start cycle button pressed
        IS_CANCEL_CYCLE_BUTTON_LATCHED  = 47,    // R/W  Cancel cycle button pressed
        IS_PAUSE_CYCLE_BUTTON_LATCHED   = 48,    // R/W  Pause cycle button pressed
        CAN_FINGERS_RAISE               = 49,    // R    Are fingers allowed to raise
        CAN_FINGERS_LOWER               = 50,    // R    Are fingers allowed to lower
        CAN_ROLLER_RAISE                = 51,    // R    Is the roller allowed to raise
        CAN_ROLLER_LOWER                = 52,    // R    Is the roller allowed to lower
        SHOW_MESSAGE                    = 53,    // R    Display a message on the HMI
        IS_READY_TO_HOME                = 54,    // R    Is the system ready to home
        IS_DUAL_PASS_MODE               = 55,    // W    Is the operation in dual pass mode
        IS_JOB_PAUSED                   = 56,    // R    Is the job paused
        NULL_TERM                                //      Final register (not included)
    };
}

namespace HregAddr {
    enum HregAddr {
        ACTUAL_POSITION_REG_ADDR        = 1,     // R    actual position of the axis measured in hundredths of an inch
        CC_COMMANDED_POSITION_REG_ADDR  = 2,     // R    Position of the axis commanded by the control core
        HMI_COMMANDED_POSITION_REG_ADDR = 3,     // W    Position commanded by the HMI
        JOB_PROGRESS_REG_ADDR           = 5,     // R    The progress of the current job as a percentage
        JOB_START_POS_REG_ADDR          = 6,     // R    The saved start position of the job
        JOB_END_POS_REG_ADDR            = 7,     // R    The saved end position of the job
        JOB_PARK_POS_REG_ADDR           = 8,     // R    The saved park position of the job
        MIN_POS_REG_ADDR                = 9,     // R    Minimum absolute axis position
        MAX_POS_REG_ADDR                = 10,    // R    Maximum absolute axis position
        JOG_SPEED_REG_ADDR              = 11,    // R/W  Target jogging speed
        PLANISH_SPEED_REG_ADDR          = 12,    // R/W  Target planishing speed
        FAULT_CODE_REG_ADDR             = 13,    // R    Fault code (0 means no fault)
        HEARTBEAT_IN_REG_ADDR           = 14,    // W    Heartbeat input from HMI
        HEARTBEAT_OUT_REG_ADDR          = 15,    // R    Heartbeat output to HMI
        CURRENT_STATE_REG_ADDR          = 16,    // R    Current state of the system
        HMI_JOB_START_POS_REG_ADDR      = 17,    // W    Job start position commanded by the HMI in hundredths of an inch
        HMI_JOB_END_POS_REG_ADDR        = 18,    // W    Job end position commanded by the HMI in hundredths of an inch
        HMI_JOB_PARK_POS_REG_ADDR       = 19,    // W    Job park position commanded by the HMI in hundredths of an inch
        MESSAGE_START                   = 32,    // R    32 registers of UTF-8 text displayed when `SHOW_MESSAGE` is true
        MESSAGE_END                     = 63,    //      End of message registers
        NULL_TERM                                //      Final register (not included)

    };
}

#endif // REGISTERDEFINITIONS_H