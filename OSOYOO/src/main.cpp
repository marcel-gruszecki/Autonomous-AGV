#include <Arduino.h>
#include "drive.cpp"
#include "connection.hpp"

AVG robot;
long lastDoneId = 0;   // id of the last executed command (protects against duplicates)

void setup() {
  Serial.begin(115200);

  initWiFi();
}

void execute(const String& msg) {
  
  // "<COMMAND> [number of lines]", e.g. "FORWARD_AFTER_LINE 2" = pass one intersection without stopping
  int sp = msg.indexOf(' ');
  String cmd = sp > 0 ? msg.substring(0, sp) : msg;
  int lines = sp > 0 ? max(1L, msg.substring(sp + 1).toInt()) : 1;

  if (cmd == "FORWARD_AFTER_LINE") {
    robot.follow_line(true, lines);

  } else if (cmd == "FORWARD") {
    robot.follow_line(false, lines);
  
  } else if (cmd == "TURN_LEFT") {
    robot.turn_left();
    
  } else if (cmd == "TURN_RIGHT") {
    robot.turn_right();
    
  } else if (cmd == "RETURN_RIGHT") {
    robot.return_back_right();

  } else if (cmd == "RETURN_LEFT" || cmd == "RETURN") {
    robot.return_back_left();

  } else {
    robot.drive(0, 0);
  }
}

void loop() {
  String msg = fetchCommand();

  //Serial.printf("L = %d, M = %d, R = %d\n", robot.onLine(32), robot.onLine(35), robot.onLine(34));

  int sp = msg.indexOf(' ');
  long id = sp > 0 ? msg.substring(0, sp).toInt() : 0;
  String cmd = sp > 0 ? msg.substring(sp + 1) : msg;

  if (id == 0) {
    // nothing to do or no connection
    robot.drive(0, 0);
  } else {
    if (id != lastDoneId) {
      Serial.printf("#%ld %s\n", id, cmd.c_str());
      execute(cmd);
      lastDoneId = id;
    }
    sendDone(id);
  }

  delay(100);
}
