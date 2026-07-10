# scalpelspace_bus

Unified Arduino library for controlling ScalpelSpace CAN devices via the
[ScalpelSpace SPI-CAN breakout](https://shop.scalpelspace.com/products/spi-can-breakout)
(MCP2518FD).

---

<details markdown="1">
  <summary>Table of Contents</summary>

<!-- TOC -->
* [scalpelspace_bus](#scalpelspace_bus)
  * [1 Overview](#1-overview)
  * [2 Installation](#2-installation)
  * [3 Hardware](#3-hardware)
  * [4 Node ID Allocation and Device Binding](#4-node-id-allocation-and-device-binding)
    * [4.1 Allocator Role](#41-allocator-role)
    * [4.2 Assignment Strategies](#42-assignment-strategies)
    * [5.3 Mixed-Device Buses](#53-mixed-device-buses)
  * [5 Library Architecture](#5-library-architecture)
    * [5.2 Custom Transports](#52-custom-transports)
  * [6 Limitations](#6-limitations)
<!-- TOC -->

</details>

---

## 1 Overview

One `ScalpelBus` object owns the CAN instance and acts as the **central
controller (allocator)**: it runs the ScalpelSpace node ID allocation protocol
and assigns node IDs to the devices on the bus. Device handles bind to allocated
nodes and expose typed command and telemetry APIs:

```cpp
#include <scalpelspace_bus.h>

ScalpelBus bus(10);      // SPI-CAN breakout chip-select pin.
McStepper stepper(bus);  // Device handles register against the bus.
MomentumCan imu(bus);

void setup() {
  bus.begin();           // Discovery + node ID assignment + binding.
  stepper.moveTo(3.14f); // rad.
}

void loop() {
  bus.poll();            // Service CAN + refresh telemetry caches.
  float heading = imu.quaternion().z;
}
```

> **Important:** This library implements the **allocator role only**. The
> ScalpelBus host must be the one and only bus owner: `begin()` always
> broadcasts discovery and assigns node IDs -- it never passively joins a bus.
> Do not connect it to a bus that already has another allocator. See
> [4.1 Allocator Role](#41-allocator-role).

Supported devices:

| Device     | Handle        | Product                           |
|------------|---------------|-----------------------------------|
| MC Stepper | `McStepper`   | Stepper motor controller          |
| MC Brushed | `McBrushed`   | Brushed motor controller          |
| Momentum   | `MomentumCan` | Sensor hub (IMU, barometer, GNSS) |

> **Note:** For a Momentum wired directly over SPI (no CAN bus), use the
> separate
> [scalpelspace_momentum](https://github.com/scalpelspace/scalpelspace_momentum)
> library. `MomentumCan` is intentionally named differently from that
> library's `Momentum` class so both can be used in one sketch.

---

## 2 Installation

Install **ScalpelSpace Bus** from the Arduino Library Manager. The
[ACAN2517](https://github.com/pierremolinaro/acan2517) driver (MCP2517FD /
MCP2518FD in classic CAN mode) is declared as a dependency and installs
alongside it.

---

## 3 Hardware

- Host: any Arduino-compatible board with hardware SPI. A 32-bit board (RP2040,
  SAMD, ESP32, Teensy, STM32) is recommended;
  see [6 Limitations](#6-limitations) for AVR notes.
- CAN interface: ScalpelSpace SPI-CAN breakout (MCP2518FD, 40 MHz crystal). Wire
  power, SPI (SCK/MISO/MOSI) and chip-select. No interrupt line is needed; the
  library services the controller from `bus.poll()`.
- Bus: 500 kbit/s classic CAN by default (`bus.begin(bitrate)` to change),
  standard 11-bit IDs, terminated per usual CAN practice.

---

## 4 Node ID Allocation and Device Binding

ScalpelSpace CAN devices boot without a node ID and wait for an allocator (this
library) to assign one. Devices act only on commands addressed to their assigned
node ID, so allocation must complete before any device can be commanded.
`bus.begin()`:

1. Broadcasts `DISCOVER` and collects device `ADVERTISE` messages (each carries
   a unique, power-cycle-stable 48-bit UID hash) during the discovery window.
2. Assigns node IDs using the selected strategy and waits for each node to
   `ACK`.
3. Binds registered device handles positionally in discovery order (unless
   attached explicitly, see below).

Devices must be powered and listening before `begin()` runs; call
`bus.rediscover()` to re-run allocation if devices boot late. Inspect the result
with `bus.nodeCount()`, `bus.nodeInfo()` and `bus.probeVersion()` (see the
`uid_scan` example).

Each allocation run carries an incrementing session ID so frames from an older
session are discarded. The upstream counter restarts at 0 on MCU reset, so the
library additionally randomizes the starting session once per boot; without
this, a run interrupted by a host reset could leak same-session frames into the
next run.

### 4.1 Allocator Role

This library implements **only the allocator** side of the protocol, and it
assumes it owns the bus's node ID space:

- `begin()` and `rediscover()` always broadcast `DISCOVER` and assign node IDs
  to every unassigned device that responds. There is no passive join mode: a
  ScalpelBus host never attaches to a bus as a mere observer or device.
- Exactly one allocator may exist per bus. Never connect a ScalpelBus host to a
  live bus that already has a owner (another Arduino running this library, or
  any other allocator implementation).

Connecting a second allocator to a live bus fails subtly rather than loudly:
devices that already hold node IDs ignore the new discovery, so the bus looks
undisturbed -- but any device that missed the original owner's discovery window
will answer the intruder and take an ID from the second owner's numbering,
which can collide with an ID the original owner already assigned. Two devices
then transmit under the same CAN IDs. Concurrent discovery windows from two
owners also corrupt each other's allocation sessions.

The **allocatee (device/node) role is intentionally not provided** and must be
custom implemented. An allocatee is conceptually a device on someone else's bus,
with its own message ID space and DBC definition, exactly like the ScalpelSpace
boards themselves. To build a custom device node, implement the allocatee half
of [can_driver](https://github.com/scalpelspace/can_driver)
(`can_id_allocatee.c`/`.h`) in that device's firmware, the same way the
ScalpelSpace device firmware does.

### 4.2 Assignment Strategies

Select before `begin()`:

| Strategy       | Method                                    | Behaviour                                                                                    |
|----------------|-------------------------------------------|----------------------------------------------------------------------------------------------|
| FIFO (default) | `bus.useFifoAllocation()`                 | Node IDs 1..N in advertise-arrival order. Simple, but not deterministic across power cycles. |
| UID ascending  | `bus.useUidAscendingAllocation()`         | Node IDs 1..N in ascending UID order. Deterministic for a fixed set of hardware.             |
| UID table      | `bus.useUidTableAllocation(table, count)` | Each known UID gets a fixed node ID; unknown devices fall back to the lowest free node IDs.  |

The **UID table** is the recommended pattern whenever a sketch addresses a
device by node ID (all motor control use cases):

```cpp
static const ScalpelBusNodeIdEntry NODE_ID_TABLE[] = {
    // {uid_0, uid_1, uid_2, node_id} <- UIDs from the uid_scan example.
    {0x1A2B, 0x3C4D, 0x5E6F, 1}, // MC Stepper.
    {0x7788, 0x99AA, 0xBBCC, 2}, // Momentum.
};

bus.useUidTableAllocation(NODE_ID_TABLE, 2);
bus.begin();          // Allocation runs here.
stepper.attach(1);    // Bind handles to the node IDs you reserved.
imu.attach(2);
```

Run the `uid_scan` example once to read the UIDs of the devices on your bus; it
prints ready-to-paste table entries.

### 5.3 Mixed-Device Buses

Positional binding is deterministic only when the binding order cannot be
confused (single device, or all devices the same product). With different
products on one bus, use the UID table strategy plus explicit `attach()` as
above; the `mixed_bus` example shows the full pattern. Nodes can also be
identified at runtime by product: `bus.probeVersion(nodeId, v)` returns the
firmware version and a product identifier character.

---

## 5 Library Architecture

```
src/
  scalpelspace_bus.h    Umbrella header.
  can_driver/           Vendored ScalpelSpace CAN protocol core
                        (frame pack/unpack, CAN ID scheme, allocator).
  core/                 ScalpelBus + device base class.
  hal/                  Transport interface + MCP2518FD (ACAN2517)
                        implementation.
  devices/
    .../                Generated Vendored DBC tables + handlers.
```

Device layers are independent: handles you never instantiate are discarded by
linker garbage collection, so unused devices cost near-zero flash.

The vendored sources (`src/can_driver/` and the `*_can_dbc.*` files under
`src/devices/`) are copies from their upstream repos, refreshed internal CI.

### 5.2 Custom Transports

The protocol stack talks to the wire through the `ScalpelCanTransport`
interface. To use a different classic CAN controller, implement the interface
and hand it to the bus:

```cpp
MyTransport transport;
ScalpelBus bus(transport);
```

---

## 6 Limitations

- **One `ScalpelBus` instance per sketch.** The vendored allocator state
  machine keeps static state; a second bus instance would corrupt it.
- **Allocator role only, single owner.** The host always assigns node IDs on
  `begin()`; it cannot passively join a bus owned by another allocator, and it
  cannot act as an allocatee (see [4.1 Allocator Role](#41-allocator-role)).
- **Polled operation.** Call `bus.poll()` frequently (every loop). The MCP2518FD
  interrupt pin is not used due to conflicts in some developer boards'
  hardware/software architectures.
- **AVR (Uno/Nano) RAM.** DBC signal tables live in RAM on AVR. On 2 KB SRAM
  parts (Uno, classic Nano) only bus-level usage fits (discovery, uid_scan, raw
  frames); a single device profile needs ~4-6 KB, so use a Mega (one device,
  tight for two) or preferably a 32-bit board.
- **Telemetry is cached, not evented.** Getters return the latest received
  values with a `lastUpdateMs` timestamp (0 = never received).
