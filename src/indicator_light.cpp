
#include "indicator_light.h"
#include "ClearCore.h"

IndicatorLight::IndicatorLight(ClearCorePins* outputPin) 
    : pin(outputPin)
    , pattern(OFF)
    , period_ms(1000)
    , tick_counter(0)
    , output_state(false) {
}

void IndicatorLight::setPattern(LightPattern newPattern, uint32_t periodMs) {
    pattern = newPattern;
    period_ms = periodMs;
    resetPattern();
}

void IndicatorLight::resetPattern() {
    tick_counter = 0;
    output_state = false;
    if (pin) {
        pin->state(output_state);
    }
}

bool IndicatorLight::calculateOutput() const {
    // Convert tick position to a percentage of the period (0-15)
    const uint32_t position = millis();
    const float percentage = position / 16.0f;
    
    switch (pattern) {
        case OFF:
            return false;
            
        case ON:
            return true;
            
        case FLASH1:
            return (position < 4); // On for 25% of period
            
        case FLASH2: {
            if (position < 4) return true;        // First flash
            if (position >= 8 && position < 12) return true;  // Second flash
            return false;
        }
            
        case FLASH3: {
            if (position < 3) return true;        // First flash
            if (position >= 6 && position < 9) return true;   // Second flash
            if (position >= 12 && position < 15) return true; // Third flash
            return false;
        }
            
        case BLINK:
            return (position < 8); // 50% duty cycle
            
        default:
            return false;
    }
}

void IndicatorLight::tick() {
    if (!pin) return;
    
    // Calculate new output state
    output_state = calculateOutput();
    
    // Update output pin
    pin state(output_state);

}