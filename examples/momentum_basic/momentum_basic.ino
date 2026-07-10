/*
 * momentum_basic.ino
 *
 * Read a Momentum sensor hub over CAN: print orientation, acceleration and GNSS
 * data streamed by the device.
 *
 * Wiring: ScalpelSpace SPI-CAN breakout (MCP2518FD) on hardware SPI,
 * chip-select on CS_PIN; one Momentum on the CAN bus.
 */

#include <scalpelspace_bus.h>

static const uint8_t CS_PIN = 10;

/* Node ID allocation. --------------------------------------------------------
 *
 * Pin the device's UID to a known node ID so the sketch always addresses this
 * exact Momentum. Run the uid_scan example to read your device's UID and
 * replace the placeholder segments below.
 */
static const uint8_t IMU_NODE_ID = 1;
static const ScalpelBusNodeIdEntry NODE_ID_TABLE[] = {
  // {uid_0, uid_1, uid_2, node_id} <- UID segments from uid_scan.
  { 0x0000, 0x0000, 0x0000, IMU_NODE_ID },
};

ScalpelBus bus(CS_PIN);
MomentumCan imu(bus);

void setup() {
  Serial.begin(115200);
  while (!Serial) {
  }

  // Run node ID allocation with the fixed UID -> node ID table, then bind the
  // handle to the node ID we chose.
  bus.useUidTableAllocation(NODE_ID_TABLE, 1);
  if (!bus.begin()) {
    Serial.print("Bus start failed, status=");
    Serial.print((int)bus.status());
    Serial.print(", transport error=0x");
    Serial.println(bus.transport().lastErrorCode(), HEX);
    while (true) {
    }
  }
  imu.attach(IMU_NODE_ID);

  // Confirm a device actually received IMU_NODE_ID.
  bool found = false;
  for (uint8_t i = 0; i < bus.nodeCount(); i++) {
    ScalpelBusNode node;
    if (bus.nodeInfo(i, node) && node.nodeId == IMU_NODE_ID) {
      found = true;
    }
  }
  if (!found) {
    Serial.println("No device matched the UID table entry.");
    Serial.println("Run the uid_scan example and update NODE_ID_TABLE.");
    while (true) {
    }
  }

  Serial.print("Momentum allocated node ID ");
  Serial.println(imu.nodeId());

  imu.setRgbLed(0, 32, 0);
}

void loop() {
  bus.poll();  // Refreshes telemetry caches.

  static uint32_t lastPrintMs = 0;
  if (millis() - lastPrintMs > 500) {
    lastPrintMs = millis();

    const MomentumQuaternion &q = imu.quaternion();
    if (q.lastUpdateMs != 0) {
      Serial.print("quat: ");
      Serial.print(q.w, 4);
      Serial.print(" ");
      Serial.print(q.x, 4);
      Serial.print(" ");
      Serial.print(q.y, 4);
      Serial.print(" ");
      Serial.println(q.z, 4);
    }

    const MomentumVector &accel = imu.accelerometer();
    if (accel.lastUpdateMs != 0) {
      Serial.print("accel [m/s^2]: ");
      Serial.print(accel.x, 2);
      Serial.print(" ");
      Serial.print(accel.y, 2);
      Serial.print(" ");
      Serial.println(accel.z, 2);
    }

    const MomentumGnssPosition &position = imu.gnssPosition();
    if (position.lastUpdateMs != 0) {
      Serial.print("gnss: ");
      Serial.print(position.latitudeDeg, 6);
      Serial.print(", ");
      Serial.println(position.longitudeDeg, 6);
    }
  }
}
