/*
    Calculated wake using equation 1 in the paper
    Time consuming
    For tuning purpose
*/
#include <iostream>
#include "influencedpoint.h"
#include "influencingpoint.h"
#include "shipcourse.h"
#include <time.h>


using namespace std;
float speed =2;
float k0 =  9.8 / (speed * speed);
float d = 0.15;
float b = 0.75;
float l = 1.55;
float integrand(float x, float y, float theta)
{
    float modual1 = 1 - exp(-k0 * d * pow(1 / cos(theta), 2));
    float phase = k0 * pow(1 / cos(theta), 2) * (x * cos(theta) + y * sin(theta));
    return modual1 * sin(phase);
}

float integrate(float x, float y)
{
    float dTheta = 0.005;
    float z = 0;
    for (float theta = -pi / 2; theta < pi / 2; theta += dTheta)
    {
        z += integrand(x, y, theta) * dTheta;
    }
    return 4 * b * z / (pi * k0 * l);
}


int main()
{
    float dx = 0.02;
    float dy = 0.02;
    string filename = "/scrpit/cal2m.csv";
    ofstream ofile;
    ofile.open(filename.c_str());
    for (float x=0;x<20;x+=dx)
    {
        for (float y=-0.36*x;y<0.36*x;y+=dy)
        {
            float ele = integrate(x,y);
            ofile<<x<<","<<y<<","<<ele<<"\n";
        }
    }
    return 0;
}
