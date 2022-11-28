#include "vex.h"

extern MotorVector vec;

// Motors
extern motor m1;
extern motor m2;
extern motor m3;
extern motor m4;

// Necessary Variables
extern int controllerJoystickMinActivation;
extern int controllerJoystickMaxBeforeFullPower;

// RECORDING FLAG
const bool recordingEnabledForThisBuild = false;

// For recording
task recordTask;
task replayTask;
extern task processInput;
extern task displayInput;

bool recording = false;
bool recordingAvail = false;

double recordingSampleInterval = 10;

bool replaying = false;

std::vector<double> m1Buf, m2Buf, m3Buf, m4Buf;

int record() {
  displayInput.suspend();
  m1Buf.clear();
  m2Buf.clear();
  m3Buf.clear();
  m4Buf.clear();

  while (true) {
    m1Buf.push_back(m1.velocity(rpm));
    m2Buf.push_back(m2.velocity(rpm));
    m3Buf.push_back(m3.velocity(rpm));
    m4Buf.push_back(m4.velocity(rpm));

    wait(recordingSampleInterval, msec);
  }
  displayInput.resume();
  return 0;
}

int replay() {
  processInput.suspend();
  int m1s, m2s, m3s, m4s;
  m1s = m1Buf.size();
  m2s = m2Buf.size();
  m3s = m3Buf.size();
  m4s = m4Buf.size();

  printf("Test debug");

  if (m1s == m2s && m3s == m4s) {
    while (m1Buf.size()) {
      m1.spin(reverse, m1Buf.back(), rpm);
      m2.spin(reverse, m1Buf.back(), rpm);
      m3.spin(reverse, m1Buf.back(), rpm);
      m4.spin(reverse, m1Buf.back(), rpm);

      // PrimaryController.Screen.clearLine();
      // PrimaryController.Screen.setCursor(1, 1);
      // PrimaryController.Screen.print("%.0f, %.0f, %.0f, %.0f", m1Buf.back(), m2Buf.back(), m3Buf.back(), m4Buf.back());

      m1Buf.pop_back();
      m2Buf.pop_back();
      m3Buf.pop_back();
      m4Buf.pop_back();
      
      wait(recordingSampleInterval, msec);
    }
  }
  else {
    PrimaryController.Screen.clearLine();
    PrimaryController.Screen.setCursor(1, 1);
    PrimaryController.Screen.print("%d, %d, %d, %d", m1Buf.size(), m2Buf.size(), m3Buf.size(), m4Buf.size());
  }

  replaying = false;
  PrimaryController.Screen.clearLine();
  PrimaryController.Screen.setCursor(1, 1);
  PrimaryController.Screen.print("Ready to Record");

  processInput.resume();
  return 0;
}

void recordHandling() {
  PrimaryController.Screen.clearLine();
  PrimaryController.Screen.setCursor(1, 1);
  if (!recording) {
    recording = true;
    PrimaryController.Screen.print("Recording");
    recordTask = task(record);
  }
  else {
    recordTask.suspend();
    displayInput.resume();
    recording = false;
    PrimaryController.Screen.print("Ready to replay");
    recordingAvail = true;
  }
}

void replayHandling() {
  PrimaryController.Screen.clearLine();
  PrimaryController.Screen.setCursor(1, 1);
  if (recordingAvail) {
    replaying = true;
    PrimaryController.Screen.print("Replaying");
    replayTask = task(replay);
  }
  else {
    if (replaying == true) {
      PrimaryController.Screen.print("Ready to Record");
    }
    else {
      PrimaryController.Screen.print("Was not replaying");
    }
  }
}

int processControllerInput() {
  if (recordingEnabledForThisBuild) {
    PrimaryController.Screen.print("Ready to Record");
    PrimaryController.ButtonUp.pressed(recordHandling);
    PrimaryController.ButtonDown.pressed(replayHandling);
  }
  else {
    PrimaryController.Screen.print("Recording Unavailable");
  }

  while (true) {
    vec.inputX = PrimaryController.Axis4.position();
    vec.inputY = PrimaryController.Axis3.position();
    vec.solve();
    vec.calculateMotorPower();

    m1.spin(forward, vec.m1Power, percent);
    m2.spin(forward, vec.m2Power, percent);
    m3.spin(forward, vec.m3Power, percent);
    m4.spin(forward, vec.m4Power, percent);

    wait(20, msec);
  }
  return 0;
}

int displayControllerInput() {
  const int updateInterval = 150;
  Brain.Screen.setCursor(12, 1);
  Brain.Screen.print("Updated every %d msec", updateInterval);

  while (true) {
    int lineNumber = 7;
    Brain.Screen.setCursor(lineNumber, 1);
    Brain.Screen.clearLine();
    
    Brain.Screen.print("X: %d", PrimaryController.Axis4.position());
    Brain.Screen.setCursor(lineNumber, 10);
    // Brain.Screen.print("X': %d", augmentControllerValues(PrimaryController.Axis4.position()));
    // Brain.Screen.setCursor(lineNumber, 30);

    Brain.Screen.print("Angle: %d", vec.angleFromOrigin);

    lineNumber++;
    Brain.Screen.setCursor(lineNumber, 1);
    Brain.Screen.clearLine();

    Brain.Screen.print("Y: %d", PrimaryController.Axis3.position());
    Brain.Screen.setCursor(lineNumber, 10);
    // Brain.Screen.print("Y': %d", augmentControllerValues(PrimaryController.Axis3.position()));
    // Brain.Screen.setCursor(lineNumber, 30);
    Brain.Screen.print("%Power: %d", vec.magnitude);

    lineNumber++;
    Brain.Screen.setCursor(lineNumber, 1);
    Brain.Screen.clearLine();

    Brain.Screen.print("R: %d", PrimaryController.Axis1.position());
    Brain.Screen.setCursor(lineNumber, 10);

    // Find max temp for all motors
    double max1 = std::max(m1.temperature(celsius), m2.temperature(celsius));
    double max2 = std::max(m3.temperature(celsius), m4.temperature(celsius));
    double maxTemp = std::max(max1, max2);

    Brain.Screen.print("MTemp': %.1fC", maxTemp);


    Brain.Screen.setCursor(7, 35);
    // Brain.Screen.print("M1: %.0f", vec.m1Power);
    Brain.Screen.print("M1: %.0f", m1.velocity(rpm));

    Brain.Screen.setCursor(9, 40);
    // Brain.Screen.print("M2: %.0f", vec.m2Power);
    Brain.Screen.print("M2: %.0f", m2.velocity(rpm));

    Brain.Screen.setCursor(11, 35);
    Brain.Screen.clearLine();
    Brain.Screen.setCursor(11, 35);
    // Brain.Screen.print("M3: %.0f", vec.m3Power);
    Brain.Screen.print("M3: %.0f", m3.velocity(rpm));

    Brain.Screen.setCursor(9, 30);
    // Brain.Screen.print("M4: %.0f", vec.m4Power);
    Brain.Screen.print("M4: %.0f", m4.velocity(rpm));

    wait(updateInterval, msec);
  }

  return 0;
}

int augmentControllerValues(int x) {
  // Using equation: y = 0.01x^2
  int negative = x < 0;

  if (abs(x) < controllerJoystickMinActivation) return 0;
  else if (abs(x) > controllerJoystickMaxBeforeFullPower) return (pow(-1, negative) * 100);
  else {
    double y = 0.01 * pow(x, 2);

    return (int)(pow(-1, negative) * round(y));
  }
}

// Callback function for displaying system statistics
int displaySystemStats() {
  while (true) {
    // Brain.Screen.clearScreen(); // Clear the brain's screen
    int lineNumber = 1;
    Brain.Screen.setCursor(lineNumber, 1); // Set the cursor position for printing, THE TOP LEFT CORNER IS (1, 1), not (0, 0).
    Brain.Screen.clearLine();

    // Self-explanatory
    Brain.Screen.print("Battery Remaining: %d%%", Brain.Battery.capacity());
    lineNumber++;
    Brain.Screen.setCursor(lineNumber, 1);
    Brain.Screen.clearLine();

    Brain.Screen.print("Battery Current: %.3fA", Brain.Battery.current());
    lineNumber++;
    Brain.Screen.setCursor(lineNumber, 1);
    Brain.Screen.clearLine();

    Brain.Screen.print("Battery Temperature: %.1fC", Brain.Battery.temperature(celsius));

    wait(1000, msec); // Only update every 1 second
  }
  return 0;
}