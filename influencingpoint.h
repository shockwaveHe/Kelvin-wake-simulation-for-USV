#ifndef INFLUENCINGPOINT_H
#define INFLUENCINGPOINT_H

#include <math.h>
#include <fstream>
#include <string>
#include "surfacegrid.h"

/*Constants*/
const float gAccelation = 9.8;
const float pi = 3.1415926;
const float density = 1;
const float k_0 = -gAccelation/(pow(2,3.5)*pi*density);//negative
const float modual = (pow(2,4.5)*k_0*pow(pi,0.5))/pow(gAccelation,0.5);//negative

const float criticalPoint = 0.8165;//sqrt(6)/3 0.8164965 cosine value

/* Default  tunning factors*/
const float myCrest[] =
    {150,37.947, 26.668, 5.54987, 1.74725, -1.08316, -0.51, -0.283359, -0.166954, -0.103131,
     -0.0680337, -0.0464794, -0.0328417};

const float calculatedCrest[] =
    {0.0239524, 0.0660981, 0.112372, 0.209575, 0.305178, 0.377062,
     0.443533, 0.521793, 0.590507, 0.696453, 0.727327, 0.820361, 0.877303};
const float sampleSpeed[] =
    {0.5, 0.75, 1, 1.5, 2, 2.5, 3, 3.5, 4, 4.5, 5, 5.5, 6};

/*
    * Influencing points. 
    * Stored in ship course.
    * Do not call it explicitly.
*/
class InfluencingPoint
{
public:
    /*
    * Constructor
    * Input: 
    *               resolution: Grid resolution
    *               v_list, t_list: tuning factor lists
    *               posX, posY: current ship position
    *               time: generation time
    *               draft, halfBeam, halflength: ship parameters
    */
    InfluencingPoint(
        float resolution,
        std::vector<float> *v_list,
        std::vector<float> *t_list,
        float posX = 0,
        float posY = 0,
        float time = 0,
        float draft = 0.15,
        float halfBeam = 0.75,
        float halflength = 1.55);
    ~InfluencingPoint();
    
    /*parameter getters*/
    float getposX();
    float getposY();
    float getvelX();
    float getvelY();
    float getvelDir();
    float getspeed();

    /*
     * Called after the second time stamp after the influencing point is initialized.
     * Initialize key parameters including:
     *  velocity, speed, k0, module, tuning factor
     * */
    void setEndPoint(float nextX, float nextY, float t);

    /* Check whether a position will be affected by this influencing point */
    bool isInsideActiveRegionByframe(float xInframe, float yInframe);
    bool isInsideActiveRegion(float x, float y); //&x,&y ordinaray x,y

    /*
     * Coordinate computer
     * Input: (x,y) in world frame
     * output: (x,y) in influencing point frame
     * */
    float XintheSameFrame(float x, float y);
    float YintheSameFrame(float x, float y);
    /*
     * Coordinate computer
     * Input: (x,y) in influencing point  frame
     * output: (x,y) in world frame
     * */
    float XintheWorldFrame(float xInFrame, float yInFrame);
    float YintheWorldFrame(float xInFrame, float yInFrame);

    /*Compute the boundary of inflencing area*/
    float worldBoundary_maxX();
    float worldBoundary_minX();
    float worldBoundary_maxY();
    float worldBoundary_minY();

    /*update the parameters in each time stamp*/
    void update(float t, float x, float y);
    /*get the elevation contributed by this influencing point*/
    float getElevation(float x, float y);
    /*Output data to surface grid*/
    void outputDataToGrid(surfaceGrid &grid);

    float maxEle;

    // private:
    float posX;    // X position
    float posY;    // Y position
    float velX;    // X velocity
    float velY;    // Y velocity
    float genTime; // generated time
    float velDir;  // velocity direction represent by arctan velX/velY
    float speed;

    float currentTime;
    float endX;
    float endY;
    float timeStep;
    float effectiveLength;
    float virtualHead;

    float resolution;

    float modualWithSpeed; // constant infront the equation other than boundary

    bool isInit;

    float tuningFactor;
    float d;  // draft
    float b;  // half bean
    float l;  // half length
    float k0; // 9.8/v^2
    std::vector<float> *tunefactor_list;
    std::vector<float> *tunefactor_v_list;

    float computeTuningFactor_simple();

    /*under develop*/
    // float computeTuningFactor();
    // float integrand(float x, float y, float theta);
    // float integrate(float x, float y);
    // float getMinZonX(float x, float step2);
    // float getMaxZonX(float x, float step2);
    // float computeApproCrestH(int index, float v);
};

float min(float a, float b);
float max(float a, float b);

/*
class TuningFactorNode
{
public:
    TuningFactorNode();
    TuningFactorNode(float speed, float tunning);
    bool isInside(float v);
    float getfactor() const;

private:
    float speed;
    float intervalMin;
    float intervalMax;
    float factor;
};
class FactorNodeList
{ // hash map is better
public:
    FactorNodeList();
    FactorNodeList(int s);
    ~FactorNodeList();
    void push(float v, float tuning);
    bool locate(float v);
    float getfactor(float v);

private:
    int size;
    int capacity;
    TuningFactorNode *list;
    void expand();
};
*/

#endif // INFLUENCINGPOINT_H
