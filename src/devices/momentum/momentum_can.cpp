/*******************************************************************************
 * @file momentum_can.cpp
 * @brief Device handle for the ScalpelSpace Momentum sensor hub over CAN.
 *******************************************************************************
 * Signal indices below follow the generated DBC tables in
 * momentum_can_dbc.c (signal order matches the source DBC file).
 *******************************************************************************
 */

#include "momentum_can.h"

/** Requests / commands. ******************************************************/

bool MomentumCan::requestDateTime() {
  const can_message_t &msg =
      momentum_dbc_messages[MOMENTUM_CAN_DBC_IDX_DATETIME_GET];
  return sendMessage(msg, NULL);
}

bool MomentumCan::requestGnssUtc() {
  const can_message_t &msg =
      momentum_dbc_messages[MOMENTUM_CAN_DBC_IDX_GNSS_UTC_GET];
  return sendMessage(msg, NULL);
}

bool MomentumCan::setRgbLed(const uint8_t red, const uint8_t green,
                            const uint8_t blue) {
  const can_message_t &msg =
      momentum_dbc_messages[MOMENTUM_CAN_DBC_IDX_RGB_LED_SET];
  uint8_t data[8] = {0};
  packSignal(msg, 0, data, red);
  packSignal(msg, 1, data, green);
  packSignal(msg, 2, data, blue);
  return sendMessage(msg, data);
}

bool MomentumCan::requestVersion() {
  const can_message_t &msg =
      momentum_dbc_messages[MOMENTUM_CAN_DBC_IDX_VERSION_GET];
  return sendMessage(msg, NULL);
}

/** RX decode. ****************************************************************/

void MomentumCan::decodeVector(const can_message_t &message,
                               const uint8_t *data, MomentumVector &out) {
  out.x = (float)decodeSignal(message, 0, data);
  out.y = (float)decodeSignal(message, 1, data);
  out.z = (float)decodeSignal(message, 2, data);
  out.lastUpdateMs = millis();
}

void MomentumCan::handleFrame(const uint16_t baseId, const uint8_t *data,
                              const uint8_t dlc) {
  const uint32_t now = millis();

  const can_message_t &stateMsg =
      momentum_dbc_messages[MOMENTUM_CAN_DBC_IDX_STATE];
  const can_message_t &baroMsg =
      momentum_dbc_messages[MOMENTUM_CAN_DBC_IDX_BAROMETRIC];
  const can_message_t &gnss1Msg =
      momentum_dbc_messages[MOMENTUM_CAN_DBC_IDX_GNSS1];
  const can_message_t &gnss2Msg =
      momentum_dbc_messages[MOMENTUM_CAN_DBC_IDX_GNSS2];
  const can_message_t &gnss3Msg =
      momentum_dbc_messages[MOMENTUM_CAN_DBC_IDX_GNSS3];
  const can_message_t &quatMsg =
      momentum_dbc_messages[MOMENTUM_CAN_DBC_IDX_QUATERNION];
  const can_message_t &gyroMsg =
      momentum_dbc_messages[MOMENTUM_CAN_DBC_IDX_GYROSCOPE];
  const can_message_t &magMsg =
      momentum_dbc_messages[MOMENTUM_CAN_DBC_IDX_MAGNETOMETER];
  const can_message_t &accelMsg =
      momentum_dbc_messages[MOMENTUM_CAN_DBC_IDX_ACCELEROMETER];
  const can_message_t &linAccelMsg =
      momentum_dbc_messages[MOMENTUM_CAN_DBC_IDX_LINEAR_ACCELEROMETER];
  const can_message_t &gravityMsg =
      momentum_dbc_messages[MOMENTUM_CAN_DBC_IDX_GRAVITY_ACCELEROMETER];
  const can_message_t &dtMsg =
      momentum_dbc_messages[MOMENTUM_CAN_DBC_IDX_DATETIME_RESPONSE];
  const can_message_t &utcMsg =
      momentum_dbc_messages[MOMENTUM_CAN_DBC_IDX_GNSS_UTC_RESPONSE];
  const can_message_t &versionMsg =
      momentum_dbc_messages[MOMENTUM_CAN_DBC_IDX_VERSION_RESPONSE];

  if (baseId == stateMsg.message_id && dlc >= stateMsg.dlc) {
    _state.systemState = (uint8_t)decodeSignal(stateMsg, 0, data);
    _state.mcuTemperatureC = (int16_t)decodeSignal(stateMsg, 1, data);
    _state.lastUpdateMs = now;
  } else if (baseId == baroMsg.message_id && dlc >= baroMsg.dlc) {
    _barometric.pressurePa = (float)decodeSignal(baroMsg, 0, data);
    _barometric.temperatureC = (float)decodeSignal(baroMsg, 1, data);
    _barometric.barometricState = (uint8_t)decodeSignal(baroMsg, 2, data);
    _barometric.lastUpdateMs = now;
  } else if (baseId == gnss1Msg.message_id && dlc >= gnss1Msg.dlc) {
    _gnssPosition.latitudeDeg = decodeSignal(gnss1Msg, 0, data);
    _gnssPosition.longitudeDeg = decodeSignal(gnss1Msg, 1, data);
    _gnssPosition.lastUpdateMs = now;
  } else if (baseId == gnss2Msg.message_id && dlc >= gnss2Msg.dlc) {
    _gnssKinematics.speedKnots = (float)decodeSignal(gnss2Msg, 0, data);
    _gnssKinematics.courseDeg = (float)decodeSignal(gnss2Msg, 1, data);
    _gnssKinematics.positionFix = (uint8_t)decodeSignal(gnss2Msg, 2, data);
    _gnssKinematics.satelliteCount = (uint8_t)decodeSignal(gnss2Msg, 3, data);
    _gnssKinematics.hdop = (float)decodeSignal(gnss2Msg, 4, data);
    _gnssKinematics.lastUpdateMs = now;
  } else if (baseId == gnss3Msg.message_id && dlc >= gnss3Msg.dlc) {
    _gnssAltitude.altitudeM = (float)decodeSignal(gnss3Msg, 0, data);
    _gnssAltitude.geoidSeparationM = (float)decodeSignal(gnss3Msg, 1, data);
    _gnssAltitude.gnssState = (uint8_t)decodeSignal(gnss3Msg, 2, data);
    _gnssAltitude.lastUpdateMs = now;
  } else if (baseId == quatMsg.message_id && dlc >= quatMsg.dlc) {
    _quaternion.x = (float)decodeSignal(quatMsg, 0, data);
    _quaternion.y = (float)decodeSignal(quatMsg, 1, data);
    _quaternion.z = (float)decodeSignal(quatMsg, 2, data);
    _quaternion.w = (float)decodeSignal(quatMsg, 3, data);
    _quaternion.lastUpdateMs = now;
  } else if (baseId == gyroMsg.message_id && dlc >= gyroMsg.dlc) {
    decodeVector(gyroMsg, data, _gyroscope);
  } else if (baseId == magMsg.message_id && dlc >= magMsg.dlc) {
    decodeVector(magMsg, data, _magnetometer);
  } else if (baseId == accelMsg.message_id && dlc >= accelMsg.dlc) {
    decodeVector(accelMsg, data, _accelerometer);
  } else if (baseId == linAccelMsg.message_id && dlc >= linAccelMsg.dlc) {
    decodeVector(linAccelMsg, data, _linearAcceleration);
  } else if (baseId == gravityMsg.message_id && dlc >= gravityMsg.dlc) {
    decodeVector(gravityMsg, data, _gravity);
  } else if (baseId == dtMsg.message_id && dlc >= dtMsg.dlc) {
    _dateTime.year = (uint8_t)decodeSignal(dtMsg, 0, data);
    _dateTime.month = (uint8_t)decodeSignal(dtMsg, 1, data);
    _dateTime.date = (uint8_t)decodeSignal(dtMsg, 2, data);
    _dateTime.weekday = (uint8_t)decodeSignal(dtMsg, 3, data);
    _dateTime.hours = (uint8_t)decodeSignal(dtMsg, 4, data);
    _dateTime.minutes = (uint8_t)decodeSignal(dtMsg, 5, data);
    _dateTime.seconds = (uint8_t)decodeSignal(dtMsg, 6, data);
    _dateTime.lastUpdateMs = now;
  } else if (baseId == utcMsg.message_id && dlc >= utcMsg.dlc) {
    _gnssUtc.year = (uint8_t)decodeSignal(utcMsg, 0, data);
    _gnssUtc.month = (uint8_t)decodeSignal(utcMsg, 1, data);
    _gnssUtc.date = (uint8_t)decodeSignal(utcMsg, 2, data);
    _gnssUtc.hours = (uint8_t)decodeSignal(utcMsg, 3, data);
    _gnssUtc.minutes = (uint8_t)decodeSignal(utcMsg, 4, data);
    _gnssUtc.seconds = (uint8_t)decodeSignal(utcMsg, 5, data);
    _gnssUtc.lastUpdateMs = now;
  } else if (baseId == versionMsg.message_id && dlc >= versionMsg.dlc) {
    _version.major = (uint8_t)decodeSignal(versionMsg, 0, data);
    _version.minor = (uint8_t)decodeSignal(versionMsg, 1, data);
    _version.patch = (uint8_t)decodeSignal(versionMsg, 2, data);
    _version.identifier = (char)(uint8_t)decodeSignal(versionMsg, 3, data);
    _version.lastUpdateMs = now;
  }
}
