//
// Created by paulw on 4/24/2025.
//



#ifndef INDICATOR_LIGHT_H
#define INDICATOR_LIGHT_H
#include "clearcore.h"

enum LightPattern {
    OFF,        // Always off
    ON,         // Always on
    FLASH1,     // Flash once
    FLASH2,     // Flash twice
    FLASH3,     // Flash thrice
    BLINK       // Blink at constant rate
};

class IndicatorLight {
public:
    explicit IndicatorLight(ClearCorePins* outputPin);

    // Set the light pattern and period
    void setPattern(LightPattern newPattern, uint32_t periodMs);

    // Call this function at least 16 times per period
    void tick();

    // Get current pattern
    LightPattern getPattern() const { return pattern; }

private:
    ClearCorePins* pin;
    LightPattern pattern;
    uint32_t period_ms;
    uint32_t tick_counter;
    bool output_state;

    // Helper function to reset the pattern
    void resetPattern();

    // Calculate output based on pattern and counter
    bool calculateOutput() const;
};


#endif //INDICATOR_LIGHT_H
