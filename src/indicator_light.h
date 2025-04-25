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
    explicit IndicatorLight(Connector* outputPin);

    IndicatorLight(); // Default constructor

    void setPin(Connector* outputPin) volatile; // Method to change pin


    /**
     * Set the period and pattern.
     * @param newPattern See `LightPattern` enum
     * @param periodMs The period over which to repeat the pattern.
     * This isn't technically a period because it's actually the "on" time for a blink,
     * and the full pattern repeats over a period of 8*periodMs
     */
    void setPattern(LightPattern newPattern, uint32_t periodMs);

    /**
     * Call this function every ms
     * This calculates and writes the state
     */
    void tick() volatile;

    // Get current pattern
    LightPattern getPattern() const { return pattern; }

private:
    Connector* pin;
    LightPattern pattern;
    uint32_t period_ms;
    uint32_t tick_counter;
    bool output_state;

    // Helper function to reset the pattern
    void resetPattern() volatile;

    // Calculate output based on pattern and counter
    bool calculateOutput() const volatile;
};


#endif //INDICATOR_LIGHT_H
