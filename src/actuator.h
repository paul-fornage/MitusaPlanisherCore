
#ifndef ACTUATOR_H
#define ACTUATOR_H

#include "ClearCore.h"

class Actuator {
public:
    virtual ~Actuator() = default;

    virtual void set_actuator_pin(Connector* actuator_pin_in);
    virtual void set_actuator_pin(Connector* actuator_pin_in, bool inverted);

    /**
     * Sets the commanded state
     * @return True for success. Only failure mode is pins being uninitialized
     */
    virtual bool set_commanded_state(bool new_state);

    /**
     * Gets the last commanded state
     * @return the last commanded state
     */
    bool get_commanded_state() const;

    /**
     * Does the actuator need more time to get to it's commanded state.
     * For blind actuators this always returns false
     * @return Does the actuator need more time to get to it's commanded state?
     */
    virtual bool is_mismatch() const;

    virtual bool is_sensed() const;

    virtual bool is_fully_engaged() const;

    virtual bool is_fully_disengaged() const;

protected:
    Connector* actuator_pin{nullptr};
    bool actuator_inverted{false};
    bool commanded_state{false};
};


class SensedActuator final : public Actuator {
public:
    SensedActuator() = default;
    void set_sense_pin(Connector* sense_pin_in, bool inverted = false);

    /**
     * Gets the measured state. Returns false if the sense pin has not been initialized
     * @return the measured state, or false if the sense pin has not been initialized
     */
    bool get_measured_state() const;


    /**
     * checks if the commanded state does not equal the measured state
     * @return get_commanded_state() != get_measured_state()
     */
    bool is_mismatch() const override;

    bool is_sensed() const override;

    bool is_fully_engaged() const override;

    bool is_fully_disengaged() const override;

private:
    Connector* sense_pin{nullptr};
    bool sensor_inverted{false};
};

class DelayedActuator final : public Actuator {
public:
    explicit DelayedActuator(uint16_t rising_delay_ms, uint16_t falling_delay_ms);
    explicit DelayedActuator(uint16_t delay_ms = 0);
    /**
     * Gets the measured state. Returns false if the sense pin has not been initialized
     * @return the measured state, or false if the sense pin has not been initialized
     */
    bool get_measured_state() const;

    bool set_commanded_state(bool new_state) override;


    /**
     * checks if the commanded state does not equal the measured state
     * @return get_commanded_state() != get_measured_state()
     */
    bool is_mismatch() const override;

    bool is_sensed() const override;

    bool is_fully_engaged() const override;

    bool is_fully_disengaged() const override;

    void tick() volatile;

private:
    volatile uint16_t time_remaining_ms{0};
    uint16_t rising_delay_ms;
    uint16_t falling_delay_ms;
};

/**
 * Works like sensed actuator, but assumes it takes some amount of time after sensor changes to new commanded
 * position for actuator to be fully engaged or disengaged.
 *
 * Like the head sensor will reach the 'engaged' sensor position before it is fully pressurized
 */
class DelayedSensedActuator final : public Actuator {
public:
    explicit DelayedSensedActuator(uint16_t rising_delay_ms, uint16_t falling_delay_ms);
    explicit DelayedSensedActuator(uint16_t delay_ms = 0);

    void set_sense_pin(Connector* sense_pin_in, bool inverted = false);

    /**
     * Gets the measured state. Returns false if the sense pin has not been initialized
     * @return the measured state, or false if the sense pin has not been initialized
     */
    bool get_measured_state() const;

    bool set_commanded_state(bool new_state) override;


    /**
     * checks if the commanded state does not equal the measured state
     * @return get_commanded_state() != get_measured_state()
     */
    bool is_mismatch() const override;

    bool is_sensed() const override;

    bool is_fully_engaged() const override;

    bool is_fully_disengaged() const override;

    void tick() volatile;

private:
    volatile uint16_t time_remaining_ms{0};
    uint16_t rising_delay_ms;
    uint16_t falling_delay_ms;
    Connector* sense_pin{nullptr};
    bool sensor_inverted{false};
};

/**
 * Works like sensed actuator, but assumes it takes some amount of time after sensor changes to new commanded
 * position for actuator to be fully engaged or disengaged.
 *
 * Like the head sensor will reach the 'engaged' sensor position before it is fully pressurized
 */
class DelayedSensedPairActuator final : public Actuator {
public:
    explicit DelayedSensedPairActuator(uint16_t rising_delay_ms, uint16_t falling_delay_ms);
    explicit DelayedSensedPairActuator(uint16_t delay_ms = 0);

    void set_sense_pin(Connector* sense_pin_in, bool inverted = false);

    /**
     * Gets the measured state. Returns false if the sense pin has not been initialized
     * @return the measured state, or false if the sense pin has not been initialized
     */
    bool get_measured_state() const;

    bool set_commanded_state(bool new_state) override;

    void set_deactuator_pin(Connector* deactuator_pin_in);
    void set_deactuator_pin(Connector* deactuator_pin_in, bool inverted);

    /**
     * checks if the commanded state does not equal the measured state
     * @return get_commanded_state() != get_measured_state()
     */
    bool is_mismatch() const override;

    bool is_sensed() const override;

    bool is_fully_engaged() const override;

    bool is_fully_disengaged() const override;

    void tick() volatile;

private:
    Connector* deactuator_pin{nullptr};
    bool deactuator_inverted{false};
    volatile uint16_t time_remaining_ms{0};
    uint16_t rising_delay_ms;
    uint16_t falling_delay_ms;
    Connector* sense_pin{nullptr};
    bool sensor_inverted{false};
};

#endif //ACTUATOR_H