#include "mbed.h"
#include "vertical_estimator.h"

// Class constructor
VerticalEstimator::VerticalEstimator() : range(PB_7,PB_6)
{
    z = 0.0;
    w = 0.0;
}

// Initialize class 
void VerticalEstimator::init()
{
    range.init();
}

// Predict vertical position and velocity from model
void VerticalEstimator::predict(float f_t)
{
    z = z + w*dt;
    if (z > 0)
    {
        w = w + (f_t/m-g)*dt;
    }
}

// Correct vertical position and velocity with measurement
void VerticalEstimator::correct(float phi, float theta)
{
    range.read();
    if (range.d < 2.0)
    {
        float z_m = range.d*cos(phi)*cos(theta);
        w = w + (ld_ver*dt_range)*(z_m-z);
        z = z + (lp_ver*dt_range)*(z_m-z);
    }
}


