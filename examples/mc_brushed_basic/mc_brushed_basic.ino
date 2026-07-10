/*
 * mc_brushed_basic.ino
 *
 * Connect to a single MC Brushed motor controller, request and print the
 * startup PID gains of each cascade controller (torque, velocity, position),
 * then print encoder position feedback.
 *
 * SAFETY: all motion commands in this example are commented out and serve as
 * reference only, so uploading it as-is cannot move the motor. Review the
 * mechanism for safe travel before uncommenting any motion command.
 *
 * Wiring: ScalpelSpace SPI-CAN breakout (MCP2518FD) on hardware SPI,
 * chip-select on CS_PIN; one MC Brushed on the CAN bus.
 */

#include <scalpelspace_bus.h>

static const uint8_t CS_PIN = 10;

/* Node ID allocation. --------------------------------------------------------
 *
 * Motor controllers act only on commands addressed to their assigned node ID
 * (unassigned IDs are rejected by firmware), so the sketch pins the device's
 * UID to a known node ID up front. Run the uid_scan example to read your
 * device's UID and replace the placeholder segments below.
 */
static const uint8_t MOTOR_NODE_ID = 1;
static const ScalpelBusNodeIdEntry NODE_ID_TABLE[] = {
  // {uid_0, uid_1, uid_2, node_id} <- UID segments from uid_scan.
  { 0x0000, 0x0000, 0x0000, MOTOR_NODE_ID },
};

ScalpelBus bus(CS_PIN);
McBrushed motor(bus);

// Used by the commented-out velocity demo below.
// uint32_t lastCommandMs = 0;
// bool forward = true;

// Request one cascade controller's PID gains and print the response. The
// controls_config_response is multiplexed by controller, so the cache holds one
// controller's gains at a time; request and read sequentially.
static void printPid(McBrushedController controller, const char *name) {
  const uint32_t before = motor.pidConfig().lastUpdateMs;
  motor.requestPid(controller);

  const uint32_t start = millis();
  while (millis() - start < 200) {
    bus.poll();
    const McBrushedPidConfig &pid = motor.pidConfig();
    if (pid.lastUpdateMs != before && pid.controller == (uint8_t)controller) {
      Serial.print(name);
      Serial.print(" pid: kP ");
      Serial.print(pid.kP, 3);
      Serial.print(", kI ");
      Serial.print(pid.kI, 3);
      Serial.print(", kD ");
      Serial.println(pid.kD, 3);
      return;
    }
  }
  Serial.print(name);
  Serial.println(" pid: no response");
}

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
  motor.attach(MOTOR_NODE_ID);

  // Confirm a device actually received MOTOR_NODE_ID.
  bool found = false;
  for (uint8_t i = 0; i < bus.nodeCount(); i++) {
    ScalpelBusNode node;
    if (bus.nodeInfo(i, node) && node.nodeId == MOTOR_NODE_ID) {
      found = true;
    }
  }
  if (!found) {
    Serial.println("No device matched the UID table entry.");
    Serial.println("Run the uid_scan example and update NODE_ID_TABLE.");
    while (true) {
    }
  }

  Serial.print("Brushed controller allocated node ID ");
  Serial.println(motor.nodeId());

  // Request and print the startup PID gains of each cascade controller. Each
  // requestPid() sends a get message; the response arrives via bus.poll() and
  // lands in the pidConfig() cache.
  Serial.println("--- startup configuration ---");
  printPid(MC_BRUSHED_PID_TORQUE, "torque");
  printPid(MC_BRUSHED_PID_VELOCITY, "velocity");
  printPid(MC_BRUSHED_PID_POSITION, "position");
  Serial.println("-----------------------------");

  // SAFETY: controller tuning, commented out for reference. Changing PID gains
  // alters how the motor reacts to its active setpoint; only uncomment once the
  // mechanism is safe to move.
  // motor.setPid(MC_BRUSHED_PID_VELOCITY, 0.5f, 0.1f, 0.0f);
}

void loop() {
  bus.poll();  // Refreshes telemetry caches.

  // SAFETY: velocity demo alternating direction every 6 seconds, commented out
  // for reference. Uncomment (with the variables above) only once the mechanism
  // is safe to move.
  // if (millis() - lastCommandMs > 6000) {
  //   lastCommandMs = millis();
  //   forward = !forward;
  //   motor.setVelocity(forward ? 0.52360f : -0.52360f); // rad/s.
  // }

  // Print encoder feedback at 5 Hz.
  static uint32_t lastPrintMs = 0;
  if (millis() - lastPrintMs > 200) {
    lastPrintMs = millis();
    const McBrushedSensor &sensor = motor.sensor();
    if (sensor.lastUpdateMs != 0) {
      Serial.print("position [rad]: ");
      Serial.println(sensor.encoderPositionRad, 3);
    }
  }
}
