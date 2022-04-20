#ifndef SURFACEGRID_H
#define SURFACEGRID_H


#include <string>
#include <iostream>
#include <fstream>
#include "vector"
#include <sstream>

class surfaceGrid
{
public:
    surfaceGrid();
    surfaceGrid(float minX,float maxX,float minY,float maxY,float resolution);
    ~surfaceGrid();
    void initElevation(float x,float z,float elevation,float Xc,float Yc,float r);
    void refreshElevation(float x,float z,float elevation);
    void refreshEleByGrid(int x,int z,float elevation);
    /*getters*/
    float getX(int posX) const;
    float getY(int posY) const;
    float getElevation(float x,float z) const;
    float getminX() const;
    float getminY() const;
    float getmaxX() const;
    float getmaxY() const;
    float getResolution() const;
    float getSizeX() const;
    float getSizeY() const;


    void processData(std::string filename);
    void clear();
    void outputData(std::string filename);

private:
    float resolution;// 10e-x,jumping step

    float minX;
    float maxX;
    float minY;
    float maxY;

    int sizeX;
    int sizeY;
    float** mygrid;//3d grid pointer to a array of pointer to arrays
};

#endif // SURFACEGRID_H
