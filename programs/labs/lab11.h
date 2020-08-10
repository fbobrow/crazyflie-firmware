#include "mbed.h"
#include "crazyflie.h"
#include "USBSerial.h"

// Define serial object
USBSerial serial;

// Crazyflie controller objects
AttitudeEstimator att_est;
VerticalEstimator ver_est;
HorizontalEstimator hor_est;

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
float data[3][250][10];
int i = 0;
int j;

// Main program
int main()
{
    // Initialize estimators objects
    att_est.init();
    ver_est.init();
    hor_est.init();
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
                serial.printf("%10.8f\t%10.8f\n", hor_est.u_m, hor_est.u); 
            }
            hor_est.predict(0.0,0.0);
            if (ver_est.z >= 0.05)
            {
                hor_est.correct(att_est.phi,att_est.theta,att_est.p,att_est.q,ver_est.z);
            }
            // data[0][i%250][(int)i/250] = hor_est.u_m;
            // data[1][i%250][(int)i/250] = hor_est.u;
            // data[2][i%250][(int)i/250] = hor_est.x;
            // i++;
            // if(i==2500)
            // {
            //     i = 0;
            //     for (j = 0; j< 2500; j++)
            //     {
            //         serial.printf("%10.8f\t%10.8f\t%10.8f\n", data[0][j%250][(int)j/250], data[1][j%250][(int)j/250],data[2][j%250][(int)j/250]);
            //     }
            //     serial.printf("\n");
            // }
        }
    }
}