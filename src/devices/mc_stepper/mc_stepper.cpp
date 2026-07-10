/*******************************************************************************
 * @file mc_stepper.cpp
 * @brief Device handle for the ScalpelSpace MC Stepper motor controller.
 *******************************************************************************
 * Signal indices below follow the generated DBC tables in
 * mc_stepper_can_dbc.c (signal order matches the source DBC file).
 *******************************************************************************
 */

#include "mc_stepper.h"

/** Commands. *****************************************************************/

bool McStepper::command(const McStepperControlMode mode, const float target,
                        const bool enable, const bool clearFaults) {
  const can_message_t &msg =
      mc_stepper_dbc_messages[MC_STEPPER_CAN_DBC_IDX_COMMAND_STEPPER];
  uint8_t data[8] = {0};
  packSignal(msg, 0, data, nodeId());            // node_id.
  packSignal(msg, 1, data, (double)mode);        // control_mode (multiplexor).
  packSignal(msg, 2, data, enable ? 1 : 0);      // enable.
  packSignal(msg, 3, data, clearFaults ? 1 : 0); // clear_faults.
  // Multiplexed targets: signals[4 + mode] for modes 1..4.
  if (mode >= MC_STEPPER_MODE_POSITION_ABS &&
      mode <= MC_STEPPER_MODE_POSITION_OL) {
    packSignal(msg, (uint8_t)(4u + (uint8_t)mode), data, target);
  }
  return sendMessage(msg, data);
}

bool McStepper::zero() {
  const can_message_t &msg =
      mc_stepper_dbc_messages[MC_STEPPER_CAN_DBC_IDX_COMMAND_STEPPER_ZERO];
  uint8_t data[8] = {0};
  packSignal(msg, 0, data, nodeId()); // node_id.
  return sendMessage(msg, data);
}

bool McStepper::setMotionConfig(const uint16_t stepSize,
                                const float maxAccelerationRadps2,
                                const float maxVelocityRadps) {
  const can_message_t &msg =
      mc_stepper_dbc_messages[MC_STEPPER_CAN_DBC_IDX_MOTION_CONFIG_SET];
  uint8_t data[8] = {0};
  packSignal(msg, 0, data, nodeId());
  packSignal(msg, 1, data, stepSize);
  packSignal(msg, 2, data, maxAccelerationRadps2);
  packSignal(msg, 3, data, maxVelocityRadps);
  return sendMessage(msg, data);
}

bool McStepper::requestMotionConfig() {
  const can_message_t &msg =
      mc_stepper_dbc_messages[MC_STEPPER_CAN_DBC_IDX_MOTION_CONFIG_GET];
  uint8_t data[8] = {0};
  packSignal(msg, 0, data, nodeId());
  return sendMessage(msg, data);
}

bool McStepper::setCurrentConfig(const uint16_t runCurrentMa,
                                 const uint8_t currentRun,
                                 const uint8_t currentHold,
                                 const uint8_t currentHoldDelay) {
  const can_message_t &msg =
      mc_stepper_dbc_messages[MC_STEPPER_CAN_DBC_IDX_CURRENT_CONFIG_SET];
  uint8_t data[8] = {0};
  packSignal(msg, 0, data, nodeId());
  packSignal(msg, 1, data, runCurrentMa);
  packSignal(msg, 2, data, currentRun);
  packSignal(msg, 3, data, currentHold);
  packSignal(msg, 4, data, currentHoldDelay);
  return sendMessage(msg, data);
}

bool McStepper::requestCurrentConfig() {
  const can_message_t &msg =
      mc_stepper_dbc_messages[MC_STEPPER_CAN_DBC_IDX_CURRENT_CONFIG_GET];
  uint8_t data[8] = {0};
  packSignal(msg, 0, data, nodeId());
  return sendMessage(msg, data);
}

bool McStepper::setPid(const float kP, const float kI, const float kD) {
  const can_message_t &msg =
      mc_stepper_dbc_messages[MC_STEPPER_CAN_DBC_IDX_CONTROLS_CONFIG_SET];
  uint8_t data[8] = {0};
  packSignal(msg, 0, data, nodeId());
  packSignal(msg, 2, data, kP); // Index 1 is reserved0.
  packSignal(msg, 3, data, kI);
  packSignal(msg, 4, data, kD);
  return sendMessage(msg, data);
}

bool McStepper::requestPid() {
  const can_message_t &msg =
      mc_stepper_dbc_messages[MC_STEPPER_CAN_DBC_IDX_CONTROLS_CONFIG_GET];
  uint8_t data[8] = {0};
  packSignal(msg, 0, data, nodeId());
  return sendMessage(msg, data);
}

bool McStepper::setStallguard(const bool enabled, const uint8_t threshold) {
  const can_message_t &msg =
      mc_stepper_dbc_messages[MC_STEPPER_CAN_DBC_IDX_STALLGUARD_SET];
  uint8_t data[8] = {0};
  packSignal(msg, 0, data, nodeId());
  packSignal(msg, 1, data, enabled ? 1 : 0);
  packSignal(msg, 2, data, threshold);
  return sendMessage(msg, data);
}

bool McStepper::requestStallguard() {
  const can_message_t &msg =
      mc_stepper_dbc_messages[MC_STEPPER_CAN_DBC_IDX_STALLGUARD_GET];
  uint8_t data[8] = {0};
  packSignal(msg, 0, data, nodeId());
  return sendMessage(msg, data);
}

bool McStepper::setDateTime(const ScalpelDateTime &dateTime) {
  const can_message_t &msg =
      mc_stepper_dbc_messages[MC_STEPPER_CAN_DBC_IDX_DATETIME_SET];
  uint8_t data[8] = {0};
  packSignal(msg, 0, data, dateTime.year);
  packSignal(msg, 1, data, dateTime.month);
  packSignal(msg, 2, data, dateTime.date);
  packSignal(msg, 3, data, dateTime.weekday);
  packSignal(msg, 4, data, dateTime.hours);
  packSignal(msg, 5, data, dateTime.minutes);
  packSignal(msg, 6, data, dateTime.seconds);
  return sendMessage(msg, data);
}

bool McStepper::requestDateTime() {
  const can_message_t &msg =
      mc_stepper_dbc_messages[MC_STEPPER_CAN_DBC_IDX_DATETIME_GET];
  return sendMessage(msg, NULL);
}

bool McStepper::setRgbLed(const uint8_t red, const uint8_t green,
                          const uint8_t blue) {
  const can_message_t &msg =
      mc_stepper_dbc_messages[MC_STEPPER_CAN_DBC_IDX_RGB_LED_SET];
  uint8_t data[8] = {0};
  packSignal(msg, 0, data, red);
  packSignal(msg, 1, data, green);
  packSignal(msg, 2, data, blue);
  return sendMessage(msg, data);
}

bool McStepper::requestVersion() {
  const can_message_t &msg =
      mc_stepper_dbc_messages[MC_STEPPER_CAN_DBC_IDX_VERSION_GET];
  return sendMessage(msg, NULL);
}

/** RX decode. ****************************************************************/

void McStepper::handleFrame(const uint16_t baseId, const uint8_t *data,
                            const uint8_t dlc) {
  const uint32_t now = millis();

  const can_message_t &stateMsg =
      mc_stepper_dbc_messages[MC_STEPPER_CAN_DBC_IDX_STATE];
  const can_message_t &sgEventMsg =
      mc_stepper_dbc_messages[MC_STEPPER_CAN_DBC_IDX_STALLGUARD_EVENT];
  const can_message_t &sensorMsg =
      mc_stepper_dbc_messages[MC_STEPPER_CAN_DBC_IDX_SENSOR];
  const can_message_t &diagMsg =
      mc_stepper_dbc_messages[MC_STEPPER_CAN_DBC_IDX_CONTROLS_DIAGNOSTIC];
  const can_message_t &motionMsg =
      mc_stepper_dbc_messages[MC_STEPPER_CAN_DBC_IDX_MOTION_CONFIG_RESPONSE];
  const can_message_t &currentMsg =
      mc_stepper_dbc_messages[MC_STEPPER_CAN_DBC_IDX_CURRENT_CONFIG_RESPONSE];
  const can_message_t &pidMsg =
      mc_stepper_dbc_messages[MC_STEPPER_CAN_DBC_IDX_CONTROLS_CONFIG_RESPONSE];
  const can_message_t &sgMsg =
      mc_stepper_dbc_messages[MC_STEPPER_CAN_DBC_IDX_STALLGUARD_RESPONSE];
  const can_message_t &dtMsg =
      mc_stepper_dbc_messages[MC_STEPPER_CAN_DBC_IDX_DATETIME_RESPONSE];
  const can_message_t &versionMsg =
      mc_stepper_dbc_messages[MC_STEPPER_CAN_DBC_IDX_VERSION_RESPONSE];

  if (baseId == stateMsg.message_id && dlc >= stateMsg.dlc) {
    _state.systemState = (uint8_t)decodeSignal(stateMsg, 0, data);
    _state.mcuTemperatureC = (int16_t)decodeSignal(stateMsg, 1, data);
    _state.fault = (uint8_t)decodeSignal(stateMsg, 2, data);
    _state.tmc2209Status = (uint16_t)decodeSignal(stateMsg, 3, data);
    _state.lastUpdateMs = now;
  } else if (baseId == sgEventMsg.message_id && dlc >= sgEventMsg.dlc) {
    // Signal 0 is a node_id echo; the CAN ID already routed us here.
    _stallguardEvent.sgResult = (uint16_t)decodeSignal(sgEventMsg, 1, data);
    _stallguardEvent.threshold = (uint8_t)decodeSignal(sgEventMsg, 2, data);
    _stallguardEvent.stalled = decodeSignal(sgEventMsg, 3, data) != 0.0;
    _stallguardEvent.lastUpdateMs = now;
  } else if (baseId == sensorMsg.message_id && dlc >= sensorMsg.dlc) {
    _sensor.encoderRawCount = (int16_t)decodeSignal(sensorMsg, 0, data);
    _sensor.encoderPositionRad = (float)decodeSignal(sensorMsg, 1, data);
    _sensor.lastUpdateMs = now;
  } else if (baseId == diagMsg.message_id && dlc >= diagMsg.dlc) {
    _controlsDiagnostic.positionSetpointRad =
        (float)decodeSignal(diagMsg, 0, data);
    _controlsDiagnostic.positionErrorRad =
        (float)decodeSignal(diagMsg, 1, data);
    _controlsDiagnostic.velocitySetpointRadps =
        (float)decodeSignal(diagMsg, 2, data);
    _controlsDiagnostic.stepRateCommand =
        (int16_t)decodeSignal(diagMsg, 3, data);
    _controlsDiagnostic.lastUpdateMs = now;
  } else if (baseId == motionMsg.message_id && dlc >= motionMsg.dlc) {
    _motionConfig.stepSize = (uint16_t)decodeSignal(motionMsg, 0, data);
    _motionConfig.maxAccelerationRadps2 =
        (float)decodeSignal(motionMsg, 1, data);
    _motionConfig.maxVelocityRadps = (float)decodeSignal(motionMsg, 2, data);
    _motionConfig.lastUpdateMs = now;
  } else if (baseId == currentMsg.message_id && dlc >= currentMsg.dlc) {
    _currentConfig.runCurrentMa = (uint16_t)decodeSignal(currentMsg, 0, data);
    _currentConfig.currentRun = (uint8_t)decodeSignal(currentMsg, 1, data);
    _currentConfig.currentHold = (uint8_t)decodeSignal(currentMsg, 2, data);
    _currentConfig.currentHoldDelay =
        (uint8_t)decodeSignal(currentMsg, 3, data);
    _currentConfig.lastUpdateMs = now;
  } else if (baseId == pidMsg.message_id && dlc >= pidMsg.dlc) {
    _pidConfig.kP = (float)decodeSignal(pidMsg, 0, data);
    _pidConfig.kI = (float)decodeSignal(pidMsg, 1, data);
    _pidConfig.kD = (float)decodeSignal(pidMsg, 2, data);
    _pidConfig.lastUpdateMs = now;
  } else if (baseId == sgMsg.message_id && dlc >= sgMsg.dlc) {
    _stallguardConfig.enabled = decodeSignal(sgMsg, 0, data) != 0.0;
    _stallguardConfig.threshold = (uint8_t)decodeSignal(sgMsg, 1, data);
    _stallguardConfig.sgResult = (uint16_t)decodeSignal(sgMsg, 2, data);
    _stallguardConfig.stalled = decodeSignal(sgMsg, 3, data) != 0.0;
    _stallguardConfig.lastUpdateMs = now;
  } else if (baseId == dtMsg.message_id && dlc >= dtMsg.dlc) {
    _dateTime.year = (uint8_t)decodeSignal(dtMsg, 0, data);
    _dateTime.month = (uint8_t)decodeSignal(dtMsg, 1, data);
    _dateTime.date = (uint8_t)decodeSignal(dtMsg, 2, data);
    _dateTime.weekday = (uint8_t)decodeSignal(dtMsg, 3, data);
    _dateTime.hours = (uint8_t)decodeSignal(dtMsg, 4, data);
    _dateTime.minutes = (uint8_t)decodeSignal(dtMsg, 5, data);
    _dateTime.seconds = (uint8_t)decodeSignal(dtMsg, 6, data);
    _dateTime.lastUpdateMs = now;
  } else if (baseId == versionMsg.message_id && dlc >= versionMsg.dlc) {
    _version.major = (uint8_t)decodeSignal(versionMsg, 0, data);
    _version.minor = (uint8_t)decodeSignal(versionMsg, 1, data);
    _version.patch = (uint8_t)decodeSignal(versionMsg, 2, data);
    _version.identifier = (char)(uint8_t)decodeSignal(versionMsg, 3, data);
    _version.lastUpdateMs = now;
  }
}
