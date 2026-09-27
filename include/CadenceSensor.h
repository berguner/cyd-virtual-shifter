#pragma once
#include <Arduino.h>
#include <NimBLEDevice.h>

namespace Sensor {

bool connectTo(const NimBLEAddress& addr);
bool connected();
void dropIfStale();
void disconnect();

}  // namespace Sensor
