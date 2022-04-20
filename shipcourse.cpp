#include "shipcourse.h"
#include <iostream>

shipCourse::shipCourse()
{
    lastPosX = 0;
    lastPosY = 0;
    lastTime = 0;
    isStart = false;
}
shipCourse::shipCourse(float draft, float halfBeam, float halfLength, std::string loadfile)
{
    lastPosX = 0;
    lastPosY = 0;
    lastTime = 0;
    isStart = false;
    d = draft;
    b = halfBeam;
    l = halfLength;
    if (loadfile.length() == 0)
    {
        // load from default parameters
        tunefactor_v_list.push_back(0.5);
        tunefactor_list.push_back(0.0239524 / 150);

        tunefactor_v_list.push_back(0.75);
        tunefactor_list.push_back(0.0660981/37.947);

        tunefactor_v_list.push_back(1);
        tunefactor_list.push_back(0.112372 / 26.668);

        tunefactor_v_list.push_back(1.5);
        tunefactor_list.push_back(0.209575 / 5.54987);

        tunefactor_v_list.push_back(2);
        tunefactor_list.push_back(0.305178 / 1.74725);

        tunefactor_v_list.push_back(2.5);
        tunefactor_list.push_back(0.377062 / 1.08316);

        tunefactor_v_list.push_back(3);
        tunefactor_list.push_back(0.443533 / 0.51);

        tunefactor_v_list.push_back(3.5);
        tunefactor_list.push_back(0.521793 / 0.283359);

        tunefactor_v_list.push_back(4);
        tunefactor_list.push_back(0.590507 / 0.166954);

        tunefactor_v_list.push_back(4.5);
        tunefactor_list.push_back(0.696453 / 0.103131);

        tunefactor_v_list.push_back(5);
        tunefactor_list.push_back(0.727327 / 0.0680337);

        tunefactor_v_list.push_back(5.5);
        tunefactor_list.push_back(0.820361 / 0.0464794);

        tunefactor_v_list.push_back(6);
        tunefactor_list.push_back(0.877303 / 0.0328417);
    }
    // FactorNodeList newList = FactorNodeList(0);
    // myList = new FactorNodeList();
    // *myList = newList;
}
shipCourse::~shipCourse()
{
}

void shipCourse::expand(float x, float y, float currentTime, float resolution)
{
    if (!isStart)
    {
        lastPosX = x;
        lastPosY = y;
        lastTime = currentTime;
        isStart = true;
        std::cout << "end initialize" << std::endl;
        return;
    }
    InfluencingPoint *pt = new InfluencingPoint(resolution, &tunefactor_v_list, &tunefactor_list, x, y, currentTime);
    chain.push_back(pt);
    lastPosX = x;
    lastPosY = y;
    lastTime = currentTime;
    updateCourse(currentTime, x, y);
}

void shipCourse::updateCourse(float currentTime, float x, float y)
{

    if (chain.size() <= 1)
    {
        return;
    }
    for (int i = 0; i < chain.size() - 1; i++)
    {
        chain[i]->update(currentTime, x, y);
    }
    while (chain.size()>1000)
    {
        InfluencingPoint* pt = chain.front();
        delete pt;
        chain.pop_front();
    }
}

float shipCourse::getElevation(float x, float y)
{
    float elevation = 0;
    for (auto i : chain)
    {
        elevation += i->getElevation(x,y);
    }
    return elevation;
}

void shipCourse::outputDataToGrid(surfaceGrid &grid)
{
    for (auto i : chain)
    {
        i->outputDataToGrid(grid);
    }
}
