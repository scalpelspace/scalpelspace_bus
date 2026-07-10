/*******************************************************************************
 * @file mc_stepper.h
 * @brief Device handle for the ScalpelSpace MC Stepper motor controller.
 *******************************************************************************
 */

#ifndef SCALPELSPACE_BUS__MC_STEPPER_H
#define SCALPELSPACE_BUS__MC_STEPPER_H

#include "../../core/scalpel_bus.h"
#include "../../core/scalpel_bus_device.h"
#include "../../core/scalpel_bus_types.h"

extern "C" {
#include "mc_stepper_can_dbc.h"
}

/** @brief command_stepper control modes (DBC multiplexor values). */
enum McStepperControlMode {
  MC_STEPPER_MODE_NONE = 0,         // No target (enable/clear-faults only).
  MC_STEPPER_MODE_POSITION_ABS = 1, // Closed loop absolute position [rad].
  MC_STEPPER_MODE_VELOCITY = 2,     // Closed loop velocity [rad/s].
  MC_STEPPER_MODE_POSITION_REL = 3, // Closed loop relative position [rad].
  MC_STEPPER_MODE_POSITION_OL = 4,  // Open loop position [rad].
};

/** Cached telemetry (see lastUpdateMs semantics in scalpel_bus_types.h). */

struct McStepperState {
  uint8_t systemState;
  int16_t mcuTemperatureC;
  uint8_t fault;
  uint16_t tmc2209Status;
  uint32_t lastUpdateMs;
};

struct McStepperSensor {
  int16_t encoderRawCount;
  float encoderPositionRad;
  uint32_t lastUpdateMs;
};

struct McStepperStallguardEvent {
  uint16_t sgResult;
  uint8_t threshold;
  bool stalled;
  uint32_t lastUpdateMs;
};

struct McStepperControlsDiagnostic {
  float positionSetpointRad;
  float positionErrorRad;
  float velocitySetpointRadps;
  int16_t stepRateCommand;
  uint32_t lastUpdateMs;
};

struct McStepperMotionConfig {
  uint16_t stepSize;
  float maxAccelerationRadps2;
  float maxVelocityRadps;
  uint32_t lastUpdateMs;
};

struct McStepperCurrentConfig {
  uint16_t runCurrentMa;
  uint8_t currentRun;
  uint8_t currentHold;
  uint8_t currentHoldDelay;
  uint32_t lastUpdateMs;
};

struct McStepperPidConfig {
  float kP;
  float kI;
  float kD;
  uint32_t lastUpdateMs;
};

struct McStepperStallguardConfig {
  bool enabled;
  uint8_t threshold;
  uint16_t sgResult;
  bool stalled;
  uint32_t lastUpdateMs;
};

/**
 * @brief MC Stepper motor controller on a ScalpelBus.
 *
 * Commands transmit immediately. Telemetry getters return cached values,
 * refreshed by ScalpelBus::poll(); config/version getters update after the
 * corresponding request*() round trip completes.
 */
class McStepper : public ScalpelBusDevice {
public:
  explicit McStepper(ScalpelBus &bus) : ScalpelBusDevice(bus) {
    memset(&_state, 0, sizeof(_state));
    memset(&_sensor, 0, sizeof(_sensor));
    memset(&_stallguardEvent, 0, sizeof(_stallguardEvent));
    memset(&_controlsDiagnostic, 0, sizeof(_controlsDiagnostic));
    memset(&_motionConfig, 0, sizeof(_motionConfig));
    memset(&_currentConfig, 0, sizeof(_currentConfig));
    memset(&_pidConfig, 0, sizeof(_pidConfig));
    memset(&_stallguardConfig, 0, sizeof(_stallguardConfig));
    memset(&_dateTime, 0, sizeof(_dateTime));
    memset(&_version, 0, sizeof(_version));
  }

  /** Motion commands. ********************************************************/

  /** @brief Closed loop absolute position move [rad]. Enables the driver. */
  bool moveTo(float positionRad) {
    return command(MC_STEPPER_MODE_POSITION_ABS, positionRad, true, false);
  }

  /** @brief Closed loop relative position move [rad]. Enables the driver. */
  bool moveBy(float deltaRad) {
    return command(MC_STEPPER_MODE_POSITION_REL, deltaRad, true, false);
  }

  /** @brief Closed loop velocity command [rad/s]. Enables the driver. */
  bool setVelocity(float velocityRadps) {
    return command(MC_STEPPER_MODE_VELOCITY, velocityRadps, true, false);
  }

  /** @brief Open loop position move [rad]. Enables the driver. */
  bool moveOpenLoop(float positionRad) {
    return command(MC_STEPPER_MODE_POSITION_OL, positionRad, true, false);
  }

  /** @brief Enable or disable the motor driver without a new target. */
  bool setEnabled(bool enabled) {
    return command(MC_STEPPER_MODE_NONE, 0.0f, enabled, false);
  }

  /** @brief Request fault flags be cleared. */
  bool clearFaults() {
    return command(MC_STEPPER_MODE_NONE, 0.0f, false, true);
  }

  /**
   * @brief Low level command_stepper access.
   *
   * @param mode Control mode (multiplexor); selects the meaning of target.
   * @param target Target value in mode units (rad or rad/s).
   * @param enable Driver enable flag.
   * @param clearFaults Clear faults flag.
   */
  bool command(McStepperControlMode mode, float target, bool enable,
               bool clearFaults);

  /** @brief Zero the position reference at the current position. */
  bool zero();

  /** Configuration. **********************************************************/

  bool setMotionConfig(uint16_t stepSize, float maxAccelerationRadps2,
                       float maxVelocityRadps);
  bool requestMotionConfig();

  bool setCurrentConfig(uint16_t runCurrentMa, uint8_t currentRun,
                        uint8_t currentHold, uint8_t currentHoldDelay);
  bool requestCurrentConfig();

  bool setPid(float kP, float kI, float kD);
  bool requestPid();

  bool setStallguard(bool enabled, uint8_t threshold);
  bool requestStallguard();

  bool setDateTime(const ScalpelDateTime &dateTime);
  bool requestDateTime();

  bool setRgbLed(uint8_t red, uint8_t green, uint8_t blue);

  bool requestVersion();

  /** Cached telemetry. *******************************************************/

  const McStepperState &state() const { return _state; }
  const McStepperSensor &sensor() const { return _sensor; }
  const McStepperStallguardEvent &stallguardEvent() const {
    return _stallguardEvent;
  }
  const McStepperControlsDiagnostic &controlsDiagnostic() const {
    return _controlsDiagnostic;
  }
  const McStepperMotionConfig &motionConfig() const { return _motionConfig; }
  const McStepperCurrentConfig &currentConfig() const { return _currentConfig; }
  const McStepperPidConfig &pidConfig() const { return _pidConfig; }
  const McStepperStallguardConfig &stallguardConfig() const {
    return _stallguardConfig;
  }
  const ScalpelDateTime &dateTime() const { return _dateTime; }
  const ScalpelVersionInfo &version() const { return _version; }

protected:
  void handleFrame(uint16_t baseId, const uint8_t *data, uint8_t dlc) override;

private:
  McStepperState _state;
  McStepperSensor _sensor;
  McStepperStallguardEvent _stallguardEvent;
  McStepperControlsDiagnostic _controlsDiagnostic;
  McStepperMotionConfig _motionConfig;
  McStepperCurrentConfig _currentConfig;
  McStepperPidConfig _pidConfig;
  McStepperStallguardConfig _stallguardConfig;
  ScalpelDateTime _dateTime;
  ScalpelVersionInfo _version;
};

#endif
