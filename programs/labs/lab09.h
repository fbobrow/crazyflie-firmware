#include "mbed.h"
#include "crazyflie.h"
#include "USBSerial.h"

// Define serial object
USBSerial serial;

// Crazyflie controller objects
AttitudeEstimator att_est;
VerticalEstimator ver_est;

// Ticker objects
Ticker tic, tic_range;

// Interrupt flag and counter variables
bool flag, flag_range;

// Callback functions
void callback()
{
    flag = true;
}
void callback_range()
{
    flag_range = true;
}

// Log data
float data[2][250][2];
int i = 0;
int j;

// Main program
int main()
{
    // Initialize estimators objects
    att_est.init();
    ver_est.init();
    // Initialize interrupts and timers
    tic.attach(&callback, dt);
    tic_range.attach(&callback_range, dt_range);
    while(true) 
    {
        if (flag) 
        {
            flag = false;
            att_est.estimate();
            ver_est.predict(0.0); 
            if (flag_range)
            {
                flag_range = false;  
                ver_est.correct(att_est.phi,att_est.theta); 
                serial.printf("z [m]:%6.2f | w [m/s]:%6.2f \n", ver_est.z, ver_est.w);
                // data[0][i%250][(int)i/250] = ver_est.z_m;
                // data[1][i%250][(int)i/250] = ver_est.z;
                // i++;
                // if(i==500)
                // {
                //     i = 0;
                //     for (j = 0; j< 500; j++)
                //     {
                //         serial.printf("%6.4f\t%6.4f\n", data[0][j%250][(int)j/250], data[1][j%250][(int)j/250]);
                //     }
                //     serial.printf("\n");
                // }
            }
        }
    }
}