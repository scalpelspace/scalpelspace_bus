/*******************************************************************************
 * @file mc_brushed.cpp
 * @brief Device handle for the ScalpelSpace MC Brushed motor controller.
 *******************************************************************************
 * Signal indices below follow the generated DBC tables in
 * mc_brushed_can_dbc.c (signal order matches the source DBC file).
 *******************************************************************************
 */

#include "mc_brushed.h"

/** Commands. *****************************************************************/

bool McBrushed::command(const McBrushedControlMode mode, const float target,
                        const bool enable, const bool clearFaults) {
  const can_message_t &msg =
      mc_brushed_dbc_messages[MC_BRUSHED_CAN_DBC_IDX_COMMAND_BRUSHED];
  uint8_t data[8] = {0};
  packSignal(msg, 0, data, nodeId());            // node_id.
  packSignal(msg, 1, data, (double)mode);        // control_mode (multiplexor).
  packSignal(msg, 2, data, enable ? 1 : 0);      // enable.
  packSignal(msg, 3, data, clearFaults ? 1 : 0); // clear_faults.
  // Multiplexed targets: signals[4 + mode] for modes 1..4.
  if (mode >= MC_BRUSHED_MODE_TORQUE && mode <= MC_BRUSHED_MODE_HBRIDGE_OL) {
    packSignal(msg, (uint8_t)(4u + (uint8_t)mode), data, target);
  }
  return sendMessage(msg, data);
}

bool McBrushed::zero() {
  const can_message_t &msg =
      mc_brushed_dbc_messages[MC_BRUSHED_CAN_DBC_IDX_COMMAND_BRUSHED_ZERO];
  uint8_t data[8] = {0};
  packSignal(msg, 0, data, nodeId()); // node_id.
  return sendMessage(msg, data);
}

bool McBrushed::setPid(const McBrushedController controller, const float kP,
                       const float kI, const float kD) {
  const can_message_t &msg =
      mc_brushed_dbc_messages[MC_BRUSHED_CAN_DBC_IDX_CONTROLS_CONFIG_SET];
  uint8_t data[8] = {0};
  packSignal(msg, 0, data, nodeId());
  packSignal(msg, 1, data, (double)controller); // Multiplexor.
  // Multiplexed gains: kP at 3 + controller, kI at 6 + ..., kD at 9 + ...
  packSignal(msg, (uint8_t)(3u + (uint8_t)controller), data, kP);
  packSignal(msg, (uint8_t)(6u + (uint8_t)controller), data, kI);
  packSignal(msg, (uint8_t)(9u + (uint8_t)controller), data, kD);
  return sendMessage(msg, data);
}

bool McBrushed::requestPid(const McBrushedController controller) {
  const can_message_t &msg =
      mc_brushed_dbc_messages[MC_BRUSHED_CAN_DBC_IDX_CONTROLS_CONFIG_GET];
  uint8_t data[8] = {0};
  packSignal(msg, 0, data, nodeId());
  packSignal(msg, 1, data, (double)controller);
  return sendMessage(msg, data);
}

bool McBrushed::setDateTime(const ScalpelDateTime &dateTime) {
  const can_message_t &msg =
      mc_brushed_dbc_messages[MC_BRUSHED_CAN_DBC_IDX_DATETIME_SET];
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

bool McBrushed::requestDateTime() {
  const can_message_t &msg =
      mc_brushed_dbc_messages[MC_BRUSHED_CAN_DBC_IDX_DATETIME_GET];
  return sendMessage(msg, NULL);
}

bool McBrushed::setRgbLed(const uint8_t red, const uint8_t green,
                          const uint8_t blue) {
  const can_message_t &msg =
      mc_brushed_dbc_messages[MC_BRUSHED_CAN_DBC_IDX_RGB_LED_SET];
  uint8_t data[8] = {0};
  packSignal(msg, 0, data, red);
  packSignal(msg, 1, data, green);
  packSignal(msg, 2, data, blue);
  return sendMessage(msg, data);
}

bool McBrushed::requestVersion() {
  const can_message_t &msg =
      mc_brushed_dbc_messages[MC_BRUSHED_CAN_DBC_IDX_VERSION_GET];
  return sendMessage(msg, NULL);
}

/** RX decode. ****************************************************************/

void McBrushed::handleFrame(const uint16_t baseId, const uint8_t *data,
                            const uint8_t dlc) {
  const uint32_t now = millis();

  const can_message_t &stateMsg =
      mc_brushed_dbc_messages[MC_BRUSHED_CAN_DBC_IDX_STATE];
  const can_message_t &sensorMsg =
      mc_brushed_dbc_messages[MC_BRUSHED_CAN_DBC_IDX_SENSOR];
  const can_message_t &diagMsg =
      mc_brushed_dbc_messages[MC_BRUSHED_CAN_DBC_IDX_CONTROLS_DIAGNOSTIC];
  const can_message_t &pidMsg =
      mc_brushed_dbc_messages[MC_BRUSHED_CAN_DBC_IDX_CONTROLS_CONFIG_RESPONSE];
  const can_message_t &dtMsg =
      mc_brushed_dbc_messages[MC_BRUSHED_CAN_DBC_IDX_DATETIME_GET_RESPONSE];
  const can_message_t &versionMsg =
      mc_brushed_dbc_messages[MC_BRUSHED_CAN_DBC_IDX_VERSION_GET_RESPONSE];

  if (baseId == stateMsg.message_id && dlc >= stateMsg.dlc) {
    _state.systemState = (uint8_t)decodeSignal(stateMsg, 0, data);
    _state.fault = (uint8_t)decodeSignal(stateMsg, 1, data);
    _state.drv8873sFaultReg = (uint8_t)decodeSignal(stateMsg, 2, data);
    _state.drv8873sDiagReg = (uint8_t)decodeSignal(stateMsg, 3, data);
    _state.lastUpdateMs = now;
  } else if (baseId == sensorMsg.message_id && dlc >= sensorMsg.dlc) {
    _sensor.drv8873sIpropi = (uint16_t)decodeSignal(sensorMsg, 0, data);
    _sensor.encoderRawCount = (int16_t)decodeSignal(sensorMsg, 1, data);
    _sensor.encoderPositionRad = (float)decodeSignal(sensorMsg, 2, data);
    _sensor.lastUpdateMs = now;
  } else if (baseId == diagMsg.message_id && dlc >= diagMsg.dlc) {
    _controlsDiagnostic.torqueErrorNm = (float)decodeSignal(diagMsg, 0, data);
    _controlsDiagnostic.velocityErrorRadps =
        (float)decodeSignal(diagMsg, 1, data);
    _controlsDiagnostic.positionErrorRad =
        (float)decodeSignal(diagMsg, 2, data);
    _controlsDiagnostic.dutyCommand = (float)decodeSignal(diagMsg, 3, data);
    _controlsDiagnostic.lastUpdateMs = now;
  } else if (baseId == pidMsg.message_id && dlc >= pidMsg.dlc) {
    // Multiplexed response: the controller signal selects which gain
    // signals are present (kP at 3 + controller, kI at 6 + ..., kD at
    // 9 + ...; indices 0 and 2 are reserved bytes).
    uint32_t controller = 0;
    if (can_message_get_mux_value(&pidMsg, data, &controller) &&
        controller <= (uint32_t)MC_BRUSHED_PID_POSITION) {
      _pidConfig.controller = (uint8_t)controller;
      _pidConfig.kP =
          (float)decodeSignal(pidMsg, (uint8_t)(3u + controller), data);
      _pidConfig.kI =
          (float)decodeSignal(pidMsg, (uint8_t)(6u + controller), data);
      _pidConfig.kD =
          (float)decodeSignal(pidMsg, (uint8_t)(9u + controller), data);
      _pidConfig.lastUpdateMs = now;
    }
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
