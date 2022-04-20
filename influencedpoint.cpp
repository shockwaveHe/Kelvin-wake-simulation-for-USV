#include "influencedpoint.h"
#include <iostream>
influencedPoint::influencedPoint(){
    distance = 0;
    Cosangle = 0;
    sign = 1;
    diameter = 0;
    a = 0;
    cst = 0;
    isBoundary = false;
}
influencedPoint::influencedPoint(
        float Cosangle,
        float diameter,
        float speed,
        float cstmod,
        float xcoordinate,
        float ycoordinate,
        float resolution){
    this->Cosangle = Cosangle;
    this->diameter = diameter;
    this->distance = this->Cosangle*this->diameter;
    this->a = 2*diameter/Cosangle;
    this->speed = speed;

    if(Cosangle<=criticalPoint-0.1){
        this->angleModual = (pow(1/Cosangle,3))/fabs(3*pow(Cosangle,2)-2);
        this->cst = cstmod*angleModual;
        this->isBoundary = false;
        sign = -1;
    }
    else if(Cosangle>=criticalPoint+0.1){
        this->angleModual = (pow(1/Cosangle,3))/fabs(3*pow(Cosangle,2)-2);
        this->cst = cstmod*angleModual;
        this->isBoundary = false;
        sign = 1;
    }
    /*Points on the boundary*/
    else {
        this->angleModual = 1/(pow((1-pow(Cosangle,2))*pow(Cosangle,10),1/3));
        this->cst = 16*0.827*k_0/pow(speed,10/3);
        this->cst = cst*angleModual;
        this->isBoundary = true;
        sign = 1;
    }

    this->xcoordinate = xcoordinate;
    this->ycoordinate = ycoordinate;
    this->resolution = resolution;
    GridX = int(xcoordinate/resolution);
    GridY = int(ycoordinate/resolution);
}

float influencedPoint::getangle(){
    return acos(Cosangle);// in pi
}
float influencedPoint::getdistance(){
    return distance;
}
float influencedPoint::getdiameter(){
    return diameter;
}
float influencedPoint::getwaveElevation(){
    float waveelevation=0;
    if (distance<1.5) return waveelevation;
    if (Cosangle<0.5735) return waveelevation;
    if(isBoundary){
        waveelevation = cst*sin(gAccelation*a/(2*speed*speed));
        waveelevation = waveelevation/pow(a,1/3);
    }
    else{
        if (sign == 1){
            waveelevation = cst*sin(gAccelation*a/(2*speed*speed+pi/4));
        }
        else{
            waveelevation = cst*sin(gAccelation*a/(2*speed*speed-pi/4));
        }
        waveelevation = waveelevation/sqrt(a);
    }
    return waveelevation;
}

float influencedPoint::getPhase(){
    if (isBoundary) return sin(gAccelation*a/(2*speed*speed));
    else {
        if (sign == 1){
            return sin(gAccelation*a/(2*speed*speed+pi/4));
        }
        else return sin(gAccelation*a/(2*speed*speed-pi/4));
    }
}
float influencedPoint::getCST(){
    return cst;
}
float influencedPoint::geta(){
    return a;
}
float influencedPoint::getspeed(){
    return speed;
}
int influencedPoint::getsign(){
    return sign;
}
float influencedPoint::getX(){
    return xcoordinate;
}
float influencedPoint::getY(){
    return ycoordinate;
}
int influencedPoint::getGridX() const{
    return GridX;
}

int influencedPoint::getGridY() const{
    return GridY;
}
