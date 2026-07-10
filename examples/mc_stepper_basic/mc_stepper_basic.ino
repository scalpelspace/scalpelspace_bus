/*
 * mc_stepper_basic.ino
 *
 * Connect to a single MC Stepper, request and print its startup configuration
 * (motion, PID, current, stallguard), then print encoder position feedback.
 *
 * SAFETY: all motion commands in this example are commented out and serve as
 * reference only, so uploading it as-is cannot move the motor. Review the
 * mechanism for safe travel before uncommenting any motion command.
 *
 * Wiring: ScalpelSpace SPI-CAN breakout (MCP2518FD) on hardware SPI,
 * chip-select on CS_PIN; one MC Stepper on the CAN bus.
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
static const uint8_t STEPPER_NODE_ID = 1;
static const ScalpelBusNodeIdEntry NODE_ID_TABLE[] = {
  // {uid_0, uid_1, uid_2, node_id} <- UID segments from uid_scan.
  { 0x0000, 0x0000, 0x0000, STEPPER_NODE_ID },
};

ScalpelBus bus(CS_PIN);
McStepper stepper(bus);

// Used by the commented-out motion sweep below.
// uint32_t lastMoveMs = 0;
// bool movedOut = false;

// Poll the bus until a cached value's lastUpdateMs changes (a response arrived)
// or the timeout expires. Call right after a request*().
static bool waitForUpdate(const uint32_t &lastUpdateMs,
                          uint16_t timeoutMs = 200) {
  const uint32_t before = lastUpdateMs;
  const uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    bus.poll();
    if (lastUpdateMs != before) {
      return true;
    }
  }
  return false;
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
  stepper.attach(STEPPER_NODE_ID);

  // Confirm a device actually received STEPPER_NODE_ID.
  bool found = false;
  for (uint8_t i = 0; i < bus.nodeCount(); i++) {
    ScalpelBusNode node;
    if (bus.nodeInfo(i, node) && node.nodeId == STEPPER_NODE_ID) {
      found = true;
    }
  }
  if (!found) {
    Serial.println("No device matched the UID table entry.");
    Serial.println("Run the uid_scan example and update NODE_ID_TABLE.");
    while (true) {
    }
  }

  Serial.print("Stepper allocated node ID ");
  Serial.println(stepper.nodeId());

  // Request and print the controller's startup configuration. Each  request*()
  // sends a get message; the response arrives via bus.poll() and lands in the
  // matching cached getter.
  Serial.println("--- startup configuration ---");

  stepper.requestMotionConfig();
  if (waitForUpdate(stepper.motionConfig().lastUpdateMs)) {
    const McStepperMotionConfig &motion = stepper.motionConfig();
    Serial.print("motion: step size ");
    Serial.print(motion.stepSize);
    Serial.print(", max accel [rad/s^2] ");
    Serial.print(motion.maxAccelerationRadps2, 1);
    Serial.print(", max vel [rad/s] ");
    Serial.println(motion.maxVelocityRadps, 3);
  } else {
    Serial.println("motion: no response");
  }

  stepper.requestPid();
  if (waitForUpdate(stepper.pidConfig().lastUpdateMs)) {
    const McStepperPidConfig &pid = stepper.pidConfig();
    Serial.print("pid: kP ");
    Serial.print(pid.kP, 3);
    Serial.print(", kI ");
    Serial.print(pid.kI, 3);
    Serial.print(", kD ");
    Serial.println(pid.kD, 3);
  } else {
    Serial.println("pid: no response");
  }

  stepper.requestCurrentConfig();
  if (waitForUpdate(stepper.currentConfig().lastUpdateMs)) {
    const McStepperCurrentConfig &current = stepper.currentConfig();
    Serial.print("current: run [mA] ");
    Serial.print(current.runCurrentMa);
    Serial.print(", irun ");
    Serial.print(current.currentRun);
    Serial.print(", ihold ");
    Serial.print(current.currentHold);
    Serial.print(", ihold delay ");
    Serial.println(current.currentHoldDelay);
  } else {
    Serial.println("current: no response");
  }

  stepper.requestStallguard();
  if (waitForUpdate(stepper.stallguardConfig().lastUpdateMs)) {
    const McStepperStallguardConfig &sg = stepper.stallguardConfig();
    Serial.print("stallguard: ");
    Serial.print(sg.enabled ? "enabled" : "disabled");
    Serial.print(", threshold ");
    Serial.println(sg.threshold);
  } else {
    Serial.println("stallguard: no response");
  }

  Serial.println("-----------------------------");

  // SAFETY: motion setup, commented out for reference.
  // stepper.setMotionConfig(16, 100.0f, 10.0f); // Microstep, accel, vel.
  // stepper.zero();
}

void loop() {
  bus.poll();  // Refreshes telemetry caches.

  // SAFETY: motion sweep between 0 and 30 degrees every 5 seconds, commented
  // out for reference. Uncomment (with the variables above) only once the
  // mechanism is safe to move.
  // if (millis() - lastMoveMs > 5000) {
  //   lastMoveMs = millis();
  //   movedOut = !movedOut;
  //   stepper.moveTo(movedOut ? 0.52360f : 0.0f);
  // }

  // Print encoder feedback at 5 Hz.
  static uint32_t lastPrintMs = 0;
  if (millis() - lastPrintMs > 200) {
    lastPrintMs = millis();
    const McStepperSensor &sensor = stepper.sensor();
    if (sensor.lastUpdateMs != 0) {
      Serial.print("position [rad]: ");
      Serial.println(sensor.encoderPositionRad, 3);
    }
  }
}
