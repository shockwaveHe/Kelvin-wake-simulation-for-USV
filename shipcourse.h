#ifndef SHIPCOURSE_H
#define SHIPCOURSE_H
#include "influencingpoint.h"
#include <string>
#include <fstream>
#include <sstream>
#include <queue>
#include <vector>
using namespace std;

class shipCourse
{
public:
    shipCourse();

    /* 
    * Construct ship course for each ship in the environment
    * Input: ship parameters: draft, half beam, half length
    * loadfile: loading the tuning factors from hand-measured data.
    *                   Default is none, using default tuning factors. 
     */
    shipCourse(float draft,float halfBeam,float halfLength,std::string loadfile="");
    ~shipCourse();

    /*
    * Expand the ship course at each time stamp
    * Input: ship world position (x,y), current time, gird resolution
    */
    void expand(float x,float y,float currentTime,float resolution);
    /*Update course at each timestamp. Called in expand()*/
    void updateCourse(float currentTime,float x,float y);

    /*
    * Output wake data to grid
    */
    void outputDataToGrid(surfaceGrid & grid);
    //void outputData(std::string filename);

    deque<InfluencingPoint*> chain;

    std::vector<float> tunefactor_v_list;
    std::vector<float> tunefactor_list;

private:
    float lastPosX;
    float lastPosY;
    float lastTime;
    bool  isStart;
    float d;//draft
    float b;//half bean
    float l;//half length

};


#endif // SHIPCOURSE_H
