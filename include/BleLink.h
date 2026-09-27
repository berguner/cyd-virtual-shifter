#pragma once
#include <Arduino.h>
#include <NimBLEDevice.h>

// NimBLEClient::disconnect() only asks the stack to terminate the link: it sets
// the status to DISCONNECTING and returns, and isConnected() reports false from
// that moment even though the connection is still up. Connecting to the same
// peer in that window is rejected with BLE_HS_EDONE, which is what breaks
// probing one device against several roles in turn.
//
// The only reliable "it is really gone" signal is the disconnect callback, so
// arm the gate before hanging up and wait on it afterwards.
class DisconnectGate {
 public:
  void arm() { m_pending = true; }
  void signal() { m_pending = false; }

  void wait(uint32_t timeoutMs = 2000) {
    const uint32_t start = millis();
    while (m_pending && (millis() - start) < timeoutMs) delay(10);
    m_pending = false;
  }

 private:
  volatile bool m_pending = false;
};
