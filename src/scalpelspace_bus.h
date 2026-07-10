/*******************************************************************************
 * @file scalpelspace_bus.h
 * @brief Unified Arduino library for ScalpelSpace CAN bus devices.
 *******************************************************************************
 * Include this single header to use the bus and any device handle:
 *
 *   #include <scalpelspace_bus.h>
 *
 *   ScalpelBus bus(10);        // CS pin of the SPI-CAN breakout.
 *   McStepper stepper(bus);    // Device handles bind during bus.begin().
 *   MomentumCan imu(bus);
 *
 *   void setup() { bus.begin(); }
 *   void loop() { bus.poll(); }
 *
 * Unused device handles cost near-zero flash: their code and DBC tables are
 * discarded by linker garbage collection when never instantiated.
 *******************************************************************************
 */

#ifndef SCALPELSPACE_BUS_H
#define SCALPELSPACE_BUS_H

#include "core/scalpel_bus.h"
#include "core/scalpel_bus_device.h"
#include "core/scalpel_bus_types.h"

#include "devices/mc_brushed/mc_brushed.h"
#include "devices/mc_stepper/mc_stepper.h"
#include "devices/momentum/momentum_can.h"

#endif
