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
        IS_JOB_ACTIVE                   = 6,     // R    Is the clearcore executing its program
        IS_READY_FOR_MANUAL_CONTROL     = 7,     // R    Is the clearcore ready for manual control, ensuring safety checks and no cycle in progress
        IS_ROLLER_DOWN                  = 8,     // R    Is the roller down/engaged
        CC_COMMANDED_ROLLER             = 9,     // R    Roller state commanded by the CC, true means engaged, false means disengaged
        CC_COMMANDED_FINGERS            = 10,    // R    Finger state commanded by the CC, true means engaged, false means disengaged
        IS_COMMANDED_POS_LATCHED        = 20,    // R/W  Has the HMI requested a new commanded position. If true, reads the commanded position
        IS_RTH_BUTTON_LATCHED           = 30,    // R/W  Has the 'return to home' button been pressed. Resets to 0 on completion
        IS_AXIS_HOMING_BUTTON_LATCHED   = 31,    // R/W  Has the 'run axis homing sequence' button been pressed. Resets to 0 on completion
        IS_SET_JOB_START_BUTTON_LATCHED = 32,    // R/W  Has the 'Set start to current position' button been pressed. Resets to 0 on completion
        IS_SET_JOB_END_BUTTON_LATCHED   = 33,    // R/W  Has the 'Set end to current position' button been pressed. Resets to 0 on completion
        IS_SET_JOB_PARK_BUTTON_LATCHED  = 34,    // R/W  Has the 'Set park to current position' button been pressed. Resets to 0 on completion
        IS_COMMIT_JOB_BUTTON_LATCHED    = 35,    // R/W  Has the 'Commit job' button been pressed. Resets to 0 on completion
        HMI_COMMANDED_FINGERS           = 40,    // W    Finger state commanded by the HMI, true means engaged, false means disengaged
        IS_COMMANDED_FINGER_LATCHED     = 41,    // R/W  signals that `HMI_COMMANDED_FINGERS` has been changed and need to be read
        HMI_COMMANDED_ROLLER            = 42,    // W    Roller state commanded by the HMI, true means engaged, false means disengaged
        IS_COMMANDED_ROLLER_LATCHED     = 43,    // R/W  signals that `HMI_COMMANDED_ROLLER` has been changed and need to be read
    };
}
namespace HregAddr {
    enum HregAddr {
        // HREG
        // ACTUAL_POSITION_REG_ADDR        = 2,     // R    actual position of the axis measured in hundredths of an inch
        CC_COMMANDED_POSITION_REG_ADDR  = 12,    // R    What position of the axis the CC is actually commanding measured in hundredths of an inch.
        HMI_COMMANDED_POSITION_REG_ADDR = 13,    // W    What position of the axis the HMI is commanding measured in hundredths of an inch.
                                     //      The commanded position will not be acted upon if `IS_COMMANDED_POS_COIL_ADDR` is not set.
        JOB_PROGRESS_REG_ADDR           = 16,    // R    The progress of the current job if there is one. Expressed as (job_progress/65536)*100%.
        JOB_START_POS_REG_ADDR          = 19,    // R    The currently saved start position of the axis measured in hundredths of an inch.
                                     //      (While the register is read-only, the value can semantically change otherwise.)
        JOB_END_POS_REG_ADDR            = 21,    // R    Same as JOB_START_POS_REG_ADDR but indicates the end position of the job.
        JOB_PARK_POS_REG_ADDR           = 23,    // R    Same as JOB_START_POS_REG_ADDR but indicates the park position.
        // MIN_POS_REG_ADDR                = 25,    // R    Minimum absolute position of the axis measured in hundredths of an inch.
        // MAX_POS_REG_ADDR                = 26,    // R    Maximum absolute position of the axis measured in hundredths of an inch.
        JOG_SPEED_REG_ADDR              = 32,    // R/W  target speed while jogging manually or in disengaged portion of job sequence
        PLANISH_SPEED_REG_ADDR          = 33,    // R/W  target speed while in engaged portion of job sequence
        FAULT_CODE_REG_ADDR             = 40,    // R    Fault code. 0 is normal operation
        // HEARTBEAT_IN_REG_ADDR           = 50,    // W    Contains an arbitrary value set by the HMI
        // HEARTBEAT_OUT_REG_ADDR          = 51,    // R    This should always be equal to `(heartbeat_in*2)%65536`
    };
}



#endif // REGISTERDEFINITIONS_H