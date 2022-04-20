/* Unused */

#ifndef INFLUENCEDPOINT_H
#define INFLUENCEDPOINT_H
#include <math.h>
#include <cstdlib>

/*Constants*/
const float gAccelation = 9.8;
const float pi = 3.1415926;
const float density = 1;
const float k_0 = -gAccelation/(pow(2,3.5)*pi*density);//negative
const float modual = (pow(2,4.5)*k_0*pow(pi,0.5))/pow(gAccelation,0.5);//negative

const float criticalPoint = 0.8165;//sqrt(6)/3 0.8164965 cosine value

class influencedPoint
{
public:
    influencedPoint();
    influencedPoint(
            float Cosangle,
            float diameter,
            float speed,
            float cstmod,
            float xcoordinate,
            float ycoordinate,
            float resolution);
    float getangle();
    float getdistance();
    float getdiameter();
    float getwaveElevation();



    /*no use in v2 */
    //void update(float aTimer);

    /*
    float xCorTransformation();
    float yCorTransformation();
    */
    float getPhase();
    float getCST();
    float geta();
    float getspeed();
    int getsign();

    float getX();
    float getY();
    int getGridX() const;
    int getGridY() const;
    float Cosangle;
    int sign;
    float distance;
    float diameter;
    float a;
    float speed;
    float cst;
    float angleModual;
    bool isBoundary;

    //its coordinates in the frame refering to the influencing point
    float xcoordinate;
    float ycoordinate;
    int GridX;
    int GridY;
    float resolution;
};

#endif // INFLUENCEDPOINT_H
