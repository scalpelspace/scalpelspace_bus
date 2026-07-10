/*******************************************************************************
 * @file mc_brushed.h
 * @brief Device handle for the ScalpelSpace MC Brushed motor controller.
 *******************************************************************************
 */

#ifndef SCALPELSPACE_BUS__MC_BRUSHED_H
#define SCALPELSPACE_BUS__MC_BRUSHED_H

#include "../../core/scalpel_bus.h"
#include "../../core/scalpel_bus_device.h"
#include "../../core/scalpel_bus_types.h"

extern "C" {
#include "mc_brushed_can_dbc.h"
}

/** @brief command_brushed control modes (DBC multiplexor values). */
enum McBrushedControlMode {
  MC_BRUSHED_MODE_NONE = 0,         // No target (enable/clear-faults only).
  MC_BRUSHED_MODE_TORQUE = 1,       // Closed loop torque [Nm].
  MC_BRUSHED_MODE_VELOCITY = 2,     // Closed loop velocity [rad/s].
  MC_BRUSHED_MODE_POSITION_REL = 3, // Closed loop relative position [rad].
  MC_BRUSHED_MODE_HBRIDGE_OL = 4,   // Open loop H-bridge duty [-1, 1].
};

/** @brief PID controller selector (DBC multiplexor values). */
enum McBrushedController {
  MC_BRUSHED_PID_TORQUE = 0,
  MC_BRUSHED_PID_VELOCITY = 1,
  MC_BRUSHED_PID_POSITION = 2,
};

/** Cached telemetry (see lastUpdateMs semantics in scalpel_bus_types.h). */

struct McBrushedState {
  uint8_t systemState;
  uint8_t fault;
  uint8_t drv8873sFaultReg;
  uint8_t drv8873sDiagReg;
  uint32_t lastUpdateMs;
};

struct McBrushedSensor {
  uint16_t drv8873sIpropi;
  int16_t encoderRawCount;
  float encoderPositionRad;
  uint32_t lastUpdateMs;
};

struct McBrushedControlsDiagnostic {
  float torqueErrorNm;
  float velocityErrorRadps;
  float positionErrorRad;
  float dutyCommand;
  uint32_t lastUpdateMs;
};

struct McBrushedPidConfig {
  uint8_t controller; // McBrushedController the gains belong to.
  float kP;
  float kI;
  float kD;
  uint32_t lastUpdateMs;
};

/**
 * @brief MC Brushed motor controller on a ScalpelBus.
 *
 * Commands transmit immediately. Telemetry getters return cached values,
 * refreshed by ScalpelBus::poll(); config/version getters update after the
 * corresponding request*() round trip completes.
 */
class McBrushed : public ScalpelBusDevice {
public:
  explicit McBrushed(ScalpelBus &bus) : ScalpelBusDevice(bus) {
    memset(&_state, 0, sizeof(_state));
    memset(&_sensor, 0, sizeof(_sensor));
    memset(&_controlsDiagnostic, 0, sizeof(_controlsDiagnostic));
    memset(&_pidConfig, 0, sizeof(_pidConfig));
    memset(&_dateTime, 0, sizeof(_dateTime));
    memset(&_version, 0, sizeof(_version));
  }

  /** Motion commands. ********************************************************/

  /** @brief Closed loop torque command [Nm]. Enables the driver. */
  bool setTorque(float torqueNm) {
    return command(MC_BRUSHED_MODE_TORQUE, torqueNm, true, false);
  }

  /** @brief Closed loop velocity command [rad/s]. Enables the driver. */
  bool setVelocity(float velocityRadps) {
    return command(MC_BRUSHED_MODE_VELOCITY, velocityRadps, true, false);
  }

  /** @brief Closed loop relative position move [rad]. Enables the driver. */
  bool moveBy(float deltaRad) {
    return command(MC_BRUSHED_MODE_POSITION_REL, deltaRad, true, false);
  }

  /** @brief Open loop H-bridge duty [-1, 1]. Enables the driver. */
  bool setDuty(float duty) {
    return command(MC_BRUSHED_MODE_HBRIDGE_OL, duty, true, false);
  }

  /** @brief Enable or disable the motor driver without a new target. */
  bool setEnabled(bool enabled) {
    return command(MC_BRUSHED_MODE_NONE, 0.0f, enabled, false);
  }

  /** @brief Request fault flags be cleared. */
  bool clearFaults() {
    return command(MC_BRUSHED_MODE_NONE, 0.0f, false, true);
  }

  /**
   * @brief Low level command_brushed access.
   *
   * @param mode Control mode (multiplexor); selects the meaning of target.
   * @param target Target value in mode units (Nm, rad/s, rad or duty).
   * @param enable Driver enable flag.
   * @param clearFaults Clear faults flag.
   */
  bool command(McBrushedControlMode mode, float target, bool enable,
               bool clearFaults);

  /** @brief Zero the position reference at the current position. */
  bool zero();

  /** Configuration. **********************************************************/

  /** @brief Set PID gains for one controller (torque/velocity/position). */
  bool setPid(McBrushedController controller, float kP, float kI, float kD);

  /** @brief Request PID gains for one controller; see pidConfig(). */
  bool requestPid(McBrushedController controller);

  bool setDateTime(const ScalpelDateTime &dateTime);
  bool requestDateTime();

  bool setRgbLed(uint8_t red, uint8_t green, uint8_t blue);

  bool requestVersion();

  /** Cached telemetry. *******************************************************/

  const McBrushedState &state() const { return _state; }
  const McBrushedSensor &sensor() const { return _sensor; }
  const McBrushedControlsDiagnostic &controlsDiagnostic() const {
    return _controlsDiagnostic;
  }
  /** @brief Gains from the most recent controls_config_response. */
  const McBrushedPidConfig &pidConfig() const { return _pidConfig; }
  const ScalpelDateTime &dateTime() const { return _dateTime; }
  const ScalpelVersionInfo &version() const { return _version; }

protected:
  void handleFrame(uint16_t baseId, const uint8_t *data, uint8_t dlc) override;

private:
  McBrushedState _state;
  McBrushedSensor _sensor;
  McBrushedControlsDiagnostic _controlsDiagnostic;
  McBrushedPidConfig _pidConfig;
  ScalpelDateTime _dateTime;
  ScalpelVersionInfo _version;
};

#endif
