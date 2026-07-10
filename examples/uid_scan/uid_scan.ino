/*
 * uid_scan.ino
 *
 * Scan the bus for ScalpelSpace devices and print each node's 48-bit UID hash
 * and firmware version.
 *
 * Every device reports a UID that is stable across power cycles. Copy the
 * printed UID segments into a ScalpelBusNodeIdEntry table in your sketch (see
 * the other examples) so each physical device always receives the same node ID
 * and your commands address exactly the device you intend.
 *
 * Wiring: ScalpelSpace SPI-CAN breakout (MCP2518FD) on hardware SPI,
 * chip-select on CS_PIN.
 */

#include <scalpelspace_bus.h>

static const uint8_t CS_PIN = 10;

ScalpelBus bus(CS_PIN);

void setup() {
  Serial.begin(115200);
  while (!Serial) {
  }

  Serial.println("Starting bus (discovery + node ID assignment)...");
  if (!bus.begin()) {
    Serial.print("Bus start failed, status=");
    Serial.print((int)bus.status());
    Serial.print(", transport error=0x");
    Serial.println(bus.transport().lastErrorCode(), HEX);
    while (true) {
    }
  }

  Serial.print("Nodes discovered: ");
  Serial.println(bus.nodeCount());

  for (uint8_t i = 0; i < bus.nodeCount(); i++) {
    ScalpelBusNode node;
    if (!bus.nodeInfo(i, node)) {
      continue;
    }

    Serial.print("Node ");
    Serial.print(node.nodeId);
    // Printed as a ready-to-paste table entry: {uid_0, uid_1, uid_2, id}.
    Serial.print("  table entry: { 0x");
    Serial.print(node.uid[0], HEX);
    Serial.print(", 0x");
    Serial.print(node.uid[1], HEX);
    Serial.print(", 0x");
    Serial.print(node.uid[2], HEX);
    Serial.print(", <node_id> }");

    ScalpelVersionInfo version;
    if (bus.probeVersion(node.nodeId, version)) {
      Serial.print("  fw v");
      Serial.print(version.major);
      Serial.print(".");
      Serial.print(version.minor);
      Serial.print(".");
      Serial.print(version.patch);
      Serial.print("-");
      Serial.println(version.identifier);
    } else {
      Serial.println("  (no version response)");
    }
  }
}

void loop() {
  bus.poll();
}
