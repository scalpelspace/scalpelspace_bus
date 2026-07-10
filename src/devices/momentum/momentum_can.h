/*******************************************************************************
 * @file momentum_can.h
 * @brief Device handle for the ScalpelSpace Momentum sensor hub over CAN.
 *******************************************************************************
 * Named MomentumCan (not Momentum) so this library can coexist in one sketch
 * with the scalpelspace_momentum SPI library, which defines class Momentum.
 *******************************************************************************
 */

#ifndef SCALPELSPACE_BUS__MOMENTUM_CAN_H
#define SCALPELSPACE_BUS__MOMENTUM_CAN_H

#include "../../core/scalpel_bus.h"
#include "../../core/scalpel_bus_device.h"
#include "../../core/scalpel_bus_types.h"

extern "C" {
#include "momentum_can_dbc.h"
}

/** Cached telemetry (see lastUpdateMs semantics in scalpel_bus_types.h). */

struct MomentumState {
  uint8_t systemState;
  int16_t mcuTemperatureC;
  uint32_t lastUpdateMs;
};

struct MomentumBarometric {
  float pressurePa;
  float temperatureC;
  uint8_t barometricState;
  uint32_t lastUpdateMs;
};

struct MomentumGnssPosition {
  double latitudeDeg;
  double longitudeDeg;
  uint32_t lastUpdateMs;
};

struct MomentumGnssKinematics {
  float speedKnots;
  float courseDeg;
  uint8_t positionFix;
  uint8_t satelliteCount;
  float hdop;
  uint32_t lastUpdateMs;
};

struct MomentumGnssAltitude {
  float altitudeM;
  float geoidSeparationM;
  uint8_t gnssState;
  uint32_t lastUpdateMs;
};

struct MomentumQuaternion {
  float x;
  float y;
  float z;
  float w;
  uint32_t lastUpdateMs;
};

/** @brief 3-axis vector sample (gyro, mag, accel, linear accel, gravity). */
struct MomentumVector {
  float x;
  float y;
  float z;
  uint32_t lastUpdateMs;
};

struct MomentumGnssUtc {
  uint8_t year;  // 0..99.
  uint8_t month; // 1..12.
  uint8_t date;  // 1..31.
  uint8_t hours;
  uint8_t minutes;
  uint8_t seconds;
  uint32_t lastUpdateMs;
};

/**
 * @brief Momentum sensor hub on a ScalpelBus.
 *
 * Momentum streams its telemetry periodically; getters return the cached
 * latest sample, refreshed by ScalpelBus::poll().
 */
class MomentumCan : public ScalpelBusDevice {
public:
  explicit MomentumCan(ScalpelBus &bus) : ScalpelBusDevice(bus) {
    memset(&_state, 0, sizeof(_state));
    memset(&_barometric, 0, sizeof(_barometric));
    memset(&_gnssPosition, 0, sizeof(_gnssPosition));
    memset(&_gnssKinematics, 0, sizeof(_gnssKinematics));
    memset(&_gnssAltitude, 0, sizeof(_gnssAltitude));
    memset(&_quaternion, 0, sizeof(_quaternion));
    memset(&_gyroscope, 0, sizeof(_gyroscope));
    memset(&_magnetometer, 0, sizeof(_magnetometer));
    memset(&_accelerometer, 0, sizeof(_accelerometer));
    memset(&_linearAcceleration, 0, sizeof(_linearAcceleration));
    memset(&_gravity, 0, sizeof(_gravity));
    memset(&_dateTime, 0, sizeof(_dateTime));
    memset(&_gnssUtc, 0, sizeof(_gnssUtc));
    memset(&_version, 0, sizeof(_version));
  }

  /** Requests / commands. ****************************************************/

  bool requestDateTime();
  bool requestGnssUtc();
  bool setRgbLed(uint8_t red, uint8_t green, uint8_t blue);
  bool requestVersion();

  /** Cached telemetry. *******************************************************/

  const MomentumState &state() const { return _state; }
  const MomentumBarometric &barometric() const { return _barometric; }
  const MomentumGnssPosition &gnssPosition() const { return _gnssPosition; }
  const MomentumGnssKinematics &gnssKinematics() const {
    return _gnssKinematics;
  }
  const MomentumGnssAltitude &gnssAltitude() const { return _gnssAltitude; }
  const MomentumQuaternion &quaternion() const { return _quaternion; }
  const MomentumVector &gyroscope() const { return _gyroscope; }       // deg/s.
  const MomentumVector &magnetometer() const { return _magnetometer; } // uT.
  const MomentumVector &accelerometer() const {
    return _accelerometer; // m/s^2.
  }
  const MomentumVector &linearAcceleration() const {
    return _linearAcceleration; // m/s^2.
  }
  const MomentumVector &gravity() const { return _gravity; } // m/s^2.
  const ScalpelDateTime &dateTime() const { return _dateTime; }
  const MomentumGnssUtc &gnssUtc() const { return _gnssUtc; }
  const ScalpelVersionInfo &version() const { return _version; }

protected:
  void handleFrame(uint16_t baseId, const uint8_t *data, uint8_t dlc) override;

private:
  void decodeVector(const can_message_t &message, const uint8_t *data,
                    MomentumVector &out);

  MomentumState _state;
  MomentumBarometric _barometric;
  MomentumGnssPosition _gnssPosition;
  MomentumGnssKinematics _gnssKinematics;
  MomentumGnssAltitude _gnssAltitude;
  MomentumQuaternion _quaternion;
  MomentumVector _gyroscope;
  MomentumVector _magnetometer;
  MomentumVector _accelerometer;
  MomentumVector _linearAcceleration;
  MomentumVector _gravity;
  ScalpelDateTime _dateTime;
  MomentumGnssUtc _gnssUtc;
  ScalpelVersionInfo _version;
};

#endif
