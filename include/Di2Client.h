#pragma once
#include <Arduino.h>
#include <NimBLEDevice.h>

namespace Di2 {

enum class Press { Short, Long, Double };

// channel is 1-based, matching the numbering shown in E-TUBE PROJECT.
using Handler = void (*)(int channel, Press press);

void setHandler(Handler handler);
bool connectTo(const NimBLEAddress& addr);
bool connected();
void dropIfStale();
void disconnect();

}  // namespace Di2
