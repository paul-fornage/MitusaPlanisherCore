//
// Created by paulw on 5/8/2025.
//

#ifndef FAULTCODES_H
#define FAULTCODES_H

namespace FaultCodes {
    enum FaultCodes {
        None = 0,
        Error = 1,
        SmTimeout = 2, // Timeout of the state machine
    };
}

#endif //FAULTCODES_H
