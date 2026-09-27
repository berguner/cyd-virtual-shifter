#pragma once
#include <Arduino.h>

#include "State.h"

namespace Ui {

enum class Screen { Devices, Ride, Debug, Profile };

enum class Action {
  None,
  ShiftUp,
  ShiftDown,
  ToggleCadenceSource,
  ToggleGearCount,
  OpenDevices,
  OpenDebug,
  OpenProfile,
  CloseProfile,  // back to the ride screen, keeping the edits in profileDraft()
  StartRide,     // to the ride screen, from either the picker or the debug screen
  ScrollUp,
  ScrollDown,
  SelectDevice,
  ForgetDevice,
};

struct Touch {
  Action action = Action::None;
  int index = -1;  // list row, for SelectDevice
};

void begin();
void setScreen(Screen screen);
Screen screen();

// Redraw whatever changed. Cheap to call often.
void update();

// Shift buttons debounce and auto-repeat on a hold; everything else fires once
// per press. The profile screen's -/+ buttons edit profileDraft() directly.
Touch pollTouch();

// What the profile screen shows and edits, loaded from the settings each time
// the screen opens. Nothing is applied until the screen closes.
const Profile& profileDraft();

}  // namespace Ui
