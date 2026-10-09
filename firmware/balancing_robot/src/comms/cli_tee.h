#ifndef CLI_TEE_H
#define CLI_TEE_H
// Wireless CLI reply routing (UI SERIAL CONSOLE input -> {"cli":"..."}).
//
// How it works: a CLI TU (serial_commands.cpp, serial_pid_commands.cpp, ...)
// includes this header AFTER all other includes, then does
//     #undef Serial
//     #define Serial cliTee
// Every Serial.print in that TU then writes to USB as before AND, while a
// wireless line is being served, forwards a copy as WS {"console"} frames.
// Zero existing lines change; USB behavior is byte-identical when idle.
//
// Forwarding targets the real Serial device (CDC or UART0, whichever the
// build selected): the references below resolve BEFORE the per-TU #define,
// so they always bind the core Serial macro, never the tee itself.
#include <Arduino.h>

// WS console sink, defined in websocket_handler.cpp (sanitizes + broadcasts).
void cliConsoleOut(const String& line);

struct CliTee : public Print {
  bool active = false;
  String buf;
  size_t write(uint8_t c) override {
    size_t n = Serial.write(c);
    if (active) {
      if (buf.length() < 2048) buf += (char)c;
      if (c == '\n') teeFlush();
    }
    return n;
  }
  size_t write(const uint8_t *b, size_t s) override {
    size_t n = Serial.write(b, s);
    if (active) {
      for (size_t i = 0; i < s && buf.length() < 2048; i++) {
        buf += (char)b[i];
        if (b[i] == '\n') teeFlush();
      }
    }
    return n;
  }
  // Serial-compatible surface used by the CLI reader.
  int available() { return Serial.available(); }
  int peek() { return Serial.peek(); }
  int read() { return Serial.read(); }
  void flush() { Serial.flush(); }
  String readStringUntil(char c) { return Serial.readStringUntil(c); }
  // Send any remainder without a trailing newline as a final line.
  void teeFlush() {
    if (buf.length()) {
      String line = buf;
      buf = "";
      cliConsoleOut(line);
    }
  }
};

extern CliTee cliTee;

#endif
