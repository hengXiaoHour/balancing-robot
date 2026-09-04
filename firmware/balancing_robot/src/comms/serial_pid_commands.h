#ifndef SERIAL_PID_COMMANDS_H
#define SERIAL_PID_COMMANDS_H

#include <Arduino.h>

// PID-tuning sub-commands (set/load/reset/cascade). Returns true when the
// command was handled. Called from handleSerialCommand() (see serial_commands.cpp).
bool handlePIDCommand(const String& command);

#endif
