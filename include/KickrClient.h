#pragma once
#include <Arduino.h>
#include <NimBLEDevice.h>

namespace Kickr {

bool connectTo(const NimBLEAddress& addr);
bool connected();
void dropIfStale();
void disconnect();

// Control commands. Call these from a task, never from a BLE callback.
bool unlock();
bool setSimMode(float weightKg, float crr, float cw);
bool setSimGrade(float gradePercent);
bool setWheelCircumference(float mm);
bool setErgMode(uint16_t watts);

// True once the trainer has answered the latest write of this command with
// success - or with anything, for unlock, which the v5 answers 02 and obeys.
// A write accepted at the BLE level can still go unanswered.
bool acknowledged(uint8_t op);

// Both directions of the trainer link as raw values, for the debug screen.
struct Debug {
  // The latest Cycling Power Measurement, field by field.
  uint16_t flags = 0;
  int16_t  power = 0;
  uint16_t torque = 0;     // accumulated, 1/32 N·m
  uint32_t wheelRevs = 0;
  uint16_t wheelTime = 0;  // 1/2048 s
  uint16_t crankRevs = 0;
  uint16_t crankTime = 0;  // 1/1024 s
  uint32_t packets = 0;
  uint32_t lastPacketMs = 0;

  // The last value written with each command (NAN / -1 until one is), and the
  // status the trainer answered it with (0 until it answers, 0x01 = success).
  float    simKg = NAN;
  float    simCrr = NAN;
  float    simCw = NAN;
  float    grade = NAN;
  float    wheelMm = NAN;
  int32_t  ergW = -1;
  uint8_t  mode = 0;       // kWahooSetSimMode or kWahooSetErgMode, whichever went last
  uint8_t  ackUnlock = 0;
  uint8_t  ackSim = 0;
  uint8_t  ackGrade = 0;
  uint8_t  ackWheel = 0;
  uint8_t  ackErg = 0;
};

Debug debug();

}  // namespace Kickr
