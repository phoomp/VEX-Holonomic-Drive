#include "vex.h"


MotorVector::MotorVector(int angleDisc, int axisSum) {
  controllerAngleofDiscrepancyStartDelta = angleDisc;
  controllerAxisSum = axisSum;
}

void MotorVector::solve() {
  // First, find the resulting angles of the two vectors
  // tan(theta) = x / y
  // theta = tanh(x / y)

  double angleRadians = atan((double)inputX / (double)inputY);
  int refAngle = (int)round(((double)(angleRadians * (double)180) / (double)M_PI));

  // Now we have to figure out the absolute angle
  if (inputX >= 0 && inputY >= 0) angleFromOrigin = refAngle;
  else if (inputX >= 0 && inputY < 0) angleFromOrigin = 180 + refAngle;
  else if (inputX < 0 && inputY < 0) angleFromOrigin = 180 + refAngle;
  else angleFromOrigin = 360 + refAngle;

  // Find the magnitude
  // magnitude = (int)round(sqrt(pow(inputX, 2) + pow(inputY, 2)));

  // From angle, deduce magnitude
  // Case 1: Angle ≤ discrepancy from 0, 90, 270 or 360
  bool discrepancy = false;

  int absRefAngle = abs(refAngle);
  if (absRefAngle < controllerAngleofDiscrepancyStartDelta || 90 - absRefAngle < controllerAngleofDiscrepancyStartDelta) {
    discrepancy = true;
  }

  if (discrepancy) {
    // Use 100 as reference, magnitude = (maximum / 100) * 100
    int maximum = std::max(abs(inputX), abs(inputY));
    magnitude = maximum;
  }
  else {
    int xySum = abs(inputX) + abs(inputY);
    magnitude = (int)round(((double)xySum / (double)controllerAxisSum) * 100);
    magnitude = std::min(100, magnitude);
  }
}

void MotorVector::calculateMotorPower() {
  // Think about the "+" sign
  // At the top of the sign, horizontal, is M1;
  // At the right of the sign, vertical, is M2;
  // At the bottom of the sign, horzontal, is M3;
  // At the left of the sign, vertical, is M4;

  // If only "vertical" movement, then M2 and M4 spin only
  // If only "horizontal" movement, then M1 and M3 spin only

  // If the robot needs to travel at 45 degrees, then all motors must be activated in the "same" direction.

  // M1 and M3 depends on cos(theta)
  // M2 and M4 depends on sin(theta)

  int refAngle = angleFromOrigin;

  bool xNeg = false;
  bool yNeg = false;

  if (refAngle > 270) {
    refAngle = 360 - refAngle;
    xNeg = true;
  }

  else if (refAngle > 180) {
    refAngle -= 180;
    xNeg = true;
    yNeg = true;
  }

  else if (refAngle > 90) {
    refAngle = 180 - refAngle;
    yNeg = true;
  }

  double thetaRad = ((double)refAngle * M_PI) / (double)180;
  
  double cosTheta = cos(thetaRad);
  double sinTheta = sin(thetaRad);
  
  double xComponent = sinTheta;
  double yComponent = cosTheta;

  m1Power = xComponent;
  m3Power = xComponent;

  m2Power = yComponent;
  m4Power = yComponent;

  // Now we scale it back up
  double maximum = std::max(xComponent, yComponent);
  double ratio = magnitude / maximum;

  m1Power *= round(ratio) * pow(-1, xNeg);
  m2Power *= round(ratio) * pow(-1, yNeg);
  m3Power *= round(ratio) * pow(-1, xNeg);
  m4Power *= round(ratio) * pow(-1, yNeg);
}
