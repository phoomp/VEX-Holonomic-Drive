// MotorVector Class
class MotorVector {
  int controllerAngleofDiscrepancyStartDelta;
  int controllerAxisSum;

  public:
    MotorVector(int angleDisc, int axisSum);

    int inputX;
    int inputY;
    int rotation;
    
    int magnitude;
    int angleFromOrigin;

    double m1Power, m2Power, m3Power, m4Power;

    void solve();
    void calculateMotorPower();
};

extern MotorVector vec;