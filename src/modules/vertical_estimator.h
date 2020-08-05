#ifndef vertical_estimator_h
#define vertical_estimator_h

#include "mbed.h"
// #include "src/utils/parameters.h"
// #include "src/drivers/vl53l1x.h"
#include "crazyflie.h"

// Vertical estimator class
class VerticalEstimator
{
  public:
    // Class constructor
    VerticalEstimator();
    // Initialize class
    void init();
    // Predict vertical position and velocity from model
    void predict(float f_t);
    // Correct vertical position and velocity with measurement
    void correct(float phi, float theta);
    // Vertical position (m) and velocity (m/s) estimations
    float z, w;
    float z_m;
  private:
    // Range sensor object
    VL53L0X range;
};

#endif
