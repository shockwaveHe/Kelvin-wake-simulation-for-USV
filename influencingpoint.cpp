#include "influencingpoint.h"
#include <string>
#include <iostream>

InfluencingPoint::InfluencingPoint(
    float resolution,
    std::vector<float> *v_list,
    std::vector<float> *t_list,
    float posX,
    float posY,
    float time,
    float draft,
    float halfBeam,
    float halflength)
{
    this->posX = posX;
    this->posY = posY;
    this->genTime = time;
    this->currentTime = time;
    this->resolution = resolution;
    tunefactor_v_list = v_list;
    tunefactor_list = t_list;
    isInit = false;
    d = draft;
    b = halfBeam;
    l = halflength;
    k0 = 0;
    tuningFactor = 1;
    maxEle = 0;
    speed = 0;
}
InfluencingPoint::~InfluencingPoint()
{
}
float InfluencingPoint::getposX()
{
    return posX;
}
float InfluencingPoint::getposY()
{
    return posY;
}
float InfluencingPoint::getvelX()
{
    return velX;
}
float InfluencingPoint::getvelY()
{
    return velY;
}
float InfluencingPoint::getvelDir()
{
    return velDir;
}
float InfluencingPoint::getspeed()
{
    return speed;
}

void InfluencingPoint::setEndPoint(float nextX, float nextY, float t)
{
    endX = nextX;
    endY = nextY;
    effectiveLength = sqrt((nextX - posX) * (nextX - posX) + (nextY - posY) * (nextY - posY));
    // std::cout<<effectiveLength<<std::endl;
    timeStep = t - genTime;
    currentTime = t;
    velX = (nextX - posX) / timeStep;
    velY = (nextY - posY) / timeStep;

    if (velX >= 0 && velY >= 0)
        velDir = atan(velY / velX);
    if (velX >= 0 && velY < 0)
        velDir = atan(velY / velX);
    if (velX < 0 && velY >= 0)
        velDir = pi + atan(velY / velX);
    if (velX < 0 && velY < 0)
        velDir = pi + atan(velY / velX);

    speed = sqrt(velX * velX + velY * velY);

    k0 = 9.8 / (speed * speed);

    if (fabs(speed) < 0.2)
    {
        modualWithSpeed = 0;
    }
    else
        modualWithSpeed = modual / pow(speed, 3); // negative
    if (speed < 0.5)
    {
        tuningFactor = 0;
    }
    else
    {
        tuningFactor = computeTuningFactor_simple();
    }
    std::cout << "tunefactor"<<tuningFactor << std::endl;
    isInit = true;
}
/* computing tuning factor part */
float InfluencingPoint::computeTuningFactor_simple()
{
    if (speed < tunefactor_v_list->at(0))
    {
        return 0;
    }
    for (int i = 0; i < tunefactor_v_list->size() - 1; i++)
    {
        if (speed >= tunefactor_v_list->at(i))
        {
            if (speed < tunefactor_v_list->at(i + 1))
            {   
                std::cout<<"locate v="<<tunefactor_v_list->at(i)<<std::endl;
                float k = (tunefactor_list->at(i + 1) - tunefactor_list->at(i)) / (tunefactor_v_list->at(i + 1) - tunefactor_v_list->at(i));
                float z = tunefactor_list->at(i) - k * tunefactor_v_list->at(i);
                return k * speed + z;
            }
        }
    }
    int idx = tunefactor_list->size() - 2;
    float k = (tunefactor_list->at(idx + 1) - tunefactor_list->at(idx)) / (tunefactor_v_list->at(idx + 1) - tunefactor_v_list->at(idx));
    float z = tunefactor_list->at(idx) - k * tunefactor_v_list->at(idx);
    return k * speed + z;
    /*
    return 1;
    if (speed == 2)
        return 10;
    if (speed == 4)
        return 2.618;
        */
}
// verified
float InfluencingPoint::XintheSameFrame(float x, float y)
{
    return (x - posX) * cos(velDir) + (y - posY) * sin(velDir);
}
// verified
float InfluencingPoint::YintheSameFrame(float x, float y)
{
    return -(x - posX) * sin(velDir) + (y - posY) * cos(velDir);
}
// verified
float InfluencingPoint::XintheWorldFrame(float xInFrame, float yInFrame)
{
    return xInFrame * cos(velDir) - yInFrame * sin(velDir) + posX;
}
// verified
float InfluencingPoint::YintheWorldFrame(float xInFrame, float yInFrame)
{
    return xInFrame * sin(velDir) + yInFrame * cos(velDir) + posY;
}

float InfluencingPoint::worldBoundary_maxX()
{
    float m = 1 / (2 * sqrt(2));
    float x1 = XintheWorldFrame(0, m * virtualHead);
    float x2 = XintheWorldFrame(0, -m * virtualHead);
    float x = effectiveLength + 0.5 * speed * (currentTime - genTime - timeStep);
    float x3 = XintheWorldFrame(x, -m * x + m * virtualHead);
    float x4 = XintheWorldFrame(x, m * x - m * virtualHead);
    float maxX = x1;
    if (x2 > maxX)
        maxX = x2;
    if (x3 > maxX)
        maxX = x3;
    if (x4 > maxX)
        maxX = x4;
    return maxX;
}

float InfluencingPoint::worldBoundary_minX()
{
    float m = 1 / (2 * sqrt(2));
    float x1 = XintheWorldFrame(0, m * virtualHead);
    float x2 = XintheWorldFrame(0, -m * virtualHead);
    float x = effectiveLength + 0.5 * speed * (currentTime - genTime - timeStep);
    float x3 = XintheWorldFrame(x, -m * x + m * virtualHead);
    float x4 = XintheWorldFrame(x, m * x - m * virtualHead);
    float minX = x1;
    if (x2 < minX)
        minX = x2;
    if (x3 < minX)
        minX = x3;
    if (x4 < minX)
        minX = x4;
    return minX;
}
float InfluencingPoint::worldBoundary_maxY()
{
    float m = 1 / (2 * sqrt(2));
    float y1 = YintheWorldFrame(0, m * virtualHead);
    float y2 = YintheWorldFrame(0, -m * virtualHead);
    float x = effectiveLength + 0.5 * speed * (currentTime - genTime - timeStep);
    float y3 = YintheWorldFrame(x, -m * x + m * virtualHead);
    float y4 = YintheWorldFrame(x, m * x - m * virtualHead);
    float maxY = y1;
    if (y2 > maxY)
        maxY = y2;
    if (y3 > maxY)
        maxY = y3;
    if (y4 > maxY)
        maxY = y4;
    return maxY;
}
float InfluencingPoint::worldBoundary_minY()
{
    float m = 1 / (2 * sqrt(2));
    float y1 = YintheWorldFrame(0, m * virtualHead);
    float y2 = YintheWorldFrame(0, -m * virtualHead);
    float x = effectiveLength + 0.5 * speed * (currentTime - genTime - timeStep);
    float y3 = YintheWorldFrame(x, -m * x + m * virtualHead);
    float y4 = YintheWorldFrame(x, m * x - m * virtualHead);
    float minY = y1;
    if (y2 < minY)
        minY = y2;
    if (y3 < minY)
        minY = y3;
    if (y4 < minY)
        minY = y4;
    return minY;
}
bool InfluencingPoint::isInsideActiveRegionByframe(float xInframe, float yInframe)
{
    if (xInframe >= 0 - 0.5 * resolution && xInframe <= effectiveLength + 0.5 * speed * (currentTime - genTime - timeStep) + 0.5 * resolution)
    {
        float c0 = 1 / (2 * 1.414213562);
        float yUpperBound = -c0 * xInframe + c0 * speed * virtualHead;
        float yLowerBound = c0 * xInframe - c0 * speed * virtualHead;
        if (yInframe <= yUpperBound && yInframe >= yLowerBound)
        {
            return true;
        }
        else
            return false;
    }
    else
        return false;
}
bool InfluencingPoint::isInsideActiveRegion(float x, float y)
{
    if (!isInit)
        return false;
    float xInframe = XintheSameFrame(x, y);
    float yInframe = YintheSameFrame(x, y);
    // effective region checking
    return isInsideActiveRegionByframe(xInframe, yInframe);
}
void InfluencingPoint::update(float t, float x, float y)
{
    if (!isInit)
    {
        setEndPoint(x, y, t);
    }
    currentTime = t;
    virtualHead = speed * (t - genTime);
}

/* key part: get elevation by the coordinate*/
float InfluencingPoint::getElevation(float x, float y)
{
    if (speed < 0.5)
        return 0;
    if ((currentTime - genTime) * speed < 0.1)
        return 0;
    float xInframe = XintheSameFrame(x, y);
    float yInframe = YintheSameFrame(x, y);
    if (!isInsideActiveRegionByframe(xInframe, yInframe))
        return 0;
    // calculate x, y in the frame
    float x1 = (xInframe + virtualHead) / 2;
    float y1 = yInframe / 2;
    /*center of circle*/
    float x0 = (x1 + xInframe) / 2;
    float y0 = (y1 + yInframe) / 2;
    float sqr_r = (xInframe - x0) * (xInframe - x0) + (yInframe - y0) * (yInframe - y0); // r^2
    if (sqr_r < y0 * y0)
        return 0;
    float intersection1 = x0 + sqrt(sqr_r - y0 * y0);
    float intersection2 = x0 - sqrt(sqr_r - y0 * y0);
    float Q1 = virtualHead - 2 * (virtualHead - intersection1); // center 1
    float Q2 = virtualHead - 2 * (virtualHead - intersection2); // center 2
    float elevation = 0;
    float diameter = 0.5 * speed * (currentTime - genTime);

    if (Q1 > 0 && Q1 < effectiveLength)
    {
        float bound1 = 1.05; // 1.05
        float bound2 = 1.570796327;
        float angle1 = atan(yInframe / (xInframe - Q1));
        // if (fabs(angle1)>1.05 && diameter<1.5*speed) elevation +=0;
        if (fabs(angle1) > bound2)
            elevation += 0;
        // if (distance1<0) elevation +=0;
        else
        {
            // float angle1 = atan(yInframe/(xInframe-Q1));
            // if (cos(angle1)>0.8 && cos(angle1)<0.83){
            if (fabs(cos(angle1) - criticalPoint) < 0.04)
            {
                return 0;
            }
            else
            {

                float a = speed * (currentTime - genTime) / cos(angle1);
                float angleModual = pow(1 / cos(angle1), 3) / sqrt(fabs(1 - 3 * pow(sin(angle1), 2)));
                angleModual = angleModual / sqrt(a);
                angleModual = modualWithSpeed * angleModual;
                // if (fabs(angle1)>1.05) angleModual = angleModual*(1/(1-bound)*fabs(angle1)+(1-1/(1-bound)));
                if (fabs(angle1) > bound1)
                    angleModual = angleModual * (1 / ((bound1 - bound2) * (bound1 - bound2) * (bound1 - bound2)) * (fabs(angle1) - bound2) * (fabs(angle1) - bound2) * (fabs(angle1) - bound2));
                // if (fabs(angle1)>bound1) angleModual = angleModual*(pow((fabs(angle1)-bound2),4)/pow((bound1-bound2),4));
                float sinphase = 0;
                if (cos(angle1) < criticalPoint)
                {
                    sinphase = sin(gAccelation * a / (2 * pow(speed, 2)) - pi / 4);
                }
                else
                {
                    sinphase = sin(gAccelation * a / (2 * pow(speed, 2)) + pi / 4);
                }
                elevation += angleModual * sinphase;
            }
        }
    }

    // if (Q2>0-0.5*resolution && Q2< effectiveLength-0.5*resolution){
    if (Q2 > 0 && Q2 < effectiveLength)
    {
        float bound1 = 1.05; // 1.05
        float bound2 = 1.570796327;
        float angle2 = atan(yInframe / (xInframe - Q2));
        // if (fabs(angle2)>1.05 && diameter<1.5*speed) elevation +=0;
        if (fabs(angle2) > bound2)
            elevation += 0;
        // if (distance2<0.3*diameter) elevation +=0;
        else
        {
            if (fabs(cos(angle2) - criticalPoint) < 0.04)
            { // 0.005
                return 0;
            }
            else
            {

                float a = speed * (currentTime - genTime) / cos(angle2);
                float angleModual = pow(1 / cos(angle2), 3) / sqrt(fabs(1 - 3 * pow(sin(angle2), 2)));
                angleModual = angleModual / sqrt(a);
                angleModual = modualWithSpeed * angleModual;
                // if (fabs(angle2)>1.05) angleModual = angleModual*(1/(1-bound)*fabs(angle2)+(1-1/(1-bound)));
                if (fabs(angle2) > bound1)
                    angleModual = angleModual * (1 / ((bound1 - bound2) * (bound1 - bound2) * (bound1 - bound2)) * (fabs(angle2) - bound2) * (fabs(angle2) - bound2) * (fabs(angle2) - bound2));
                // if (fabs(angle2)>bound1) angleModual = angleModual*(pow((fabs(angle2)-bound2),4)/pow((bound1-bound2),4));
                float sinphase = 0;
                if (cos(angle2) < criticalPoint)
                {
                    sinphase = sin(gAccelation * a / (2 * pow(speed, 2)) - pi / 4);
                }
                else
                {
                    sinphase = sin(gAccelation * a / (2 * pow(speed, 2)) + pi / 4);
                }
                elevation += angleModual * sinphase;
            }
        }
    }
    if (elevation > maxEle)
        maxEle = elevation;
    if (elevation == NAN)
        return 0;
    // std::cout<<elevation<<std::endl;
    return elevation * tuningFactor;
}

void InfluencingPoint::outputDataToGrid(surfaceGrid &grid)
{
    float minX = max(worldBoundary_minX(), grid.getminX());
    float maxX = min(worldBoundary_maxX(), grid.getmaxX());
    float minY = max(worldBoundary_minY(), grid.getminY());
    float maxY = min(worldBoundary_maxY(), grid.getmaxY());
    int minXInGrid = (minX - grid.getminX()) / resolution;
    int maxXInGrid = (maxX - grid.getminX()) / resolution;
    int minYInGrid = (minY - grid.getminY()) / resolution;
    int maxYInGrid = (maxY - grid.getminY()) / resolution;

    // std::cout << "---------" << std::endl;
    // std::cout << "X:" << minXInGrid << "," << maxXInGrid << std::endl;
    // std::cout << "Y:" << minYInGrid << "," << maxYInGrid << std::endl;
    // std::cout << resolution << std::endl;
    // std::cout << genTime << std::endl;
    // std::cout << "v:" << speed << std::endl;
    // std::cout << tuningFactor << std::endl;
    for (int i = minXInGrid; i < maxXInGrid; i++)
    {
        for (int j = minYInGrid; j < maxYInGrid; j++)
        {
            float x = i * resolution + grid.getminX();
            float y = j * resolution + grid.getminY();
            float elevation = getElevation(x, y);
            grid.refreshEleByGrid(i, j, elevation);
        }
    }
}
float min(float a, float b)
{
    return a < b ? a : b;
}
float max(float a, float b)
{
    return a > b ? a : b;
}
/*
TuningFactorNode::TuningFactorNode()
{
    speed = 0;
    intervalMin = 0;
    intervalMax = 0;
    factor = 0;
}
TuningFactorNode::TuningFactorNode(float speed, float tunning)
{
    this->speed = speed;
    this->intervalMax = speed + 0.1;
    this->intervalMin = speed - 0.1;
    factor = tunning;
}

bool TuningFactorNode::isInside(float v)
{
    // std::cout<<intervalMax<<","<<intervalMin<<std::endl;
    if (v > intervalMin && v <= intervalMax)
        return true;
    else
        return false;
}
float TuningFactorNode::getfactor() const
{
    return factor;
}
FactorNodeList::FactorNodeList()
{
    size = 0;
    capacity = 10;
    list = new TuningFactorNode[capacity];
}
FactorNodeList::FactorNodeList(int s)
{
    size = s;
    capacity = 10;
    list = new TuningFactorNode[capacity];
}
FactorNodeList::~FactorNodeList()
{
    delete[] list;
}
void FactorNodeList::push(float v, float tuning)
{
    TuningFactorNode newNode = TuningFactorNode(v, tuning);
    // std::cout<<"!!!!"<<std::endl;
    list[size] = newNode;
    // std::cout<<"!!!!"<<std::endl;
    size++;
    // std::cout<<"!!!!"<<std::endl;
    if (size > capacity)
        expand();
    // std::cout<<"!!!!"<<std::endl;
}
bool FactorNodeList::locate(float v)
{
    std::cout << "size:" << size << std::endl;
    if (size == 0)
        return false;
    for (int i = 0; i < size; i++)
    {
        if (list[i].isInside(v))
            return true;
    }
    return false;
}
float FactorNodeList::getfactor(float v)
{
    for (int i = 0; i < size; i++)
    {
        if (list[i].isInside(v))
        {
            return list[i].getfactor();
        }
    }
    return 0;
}
void FactorNodeList::expand()
{
    TuningFactorNode *oldList = list;
    capacity = 2 * (capacity);
    list = new TuningFactorNode[capacity];
    for (int i = 0; i < size; i++)
    {
        list[i] = oldList[i];
    }
    delete[] oldList;
}

*/

/* computing tuning factor part
    ! This function is under development for its computation efficiency
    Currently it computes the tuning factor from the
    max hand measured maxcrest, maxHeight list
    These lists only work for ship  with draft = 0.15, halfbeam= 0.75, halflength=1.55
float InfluencingPoint::computeTuningFactor()
{
    if (speed == 2)
        return 10;
    if (speed == 4)
        return 2.618;
    if (speed > 5)
    {
        printf("Velocity out of scope! \n");
        return 1;
    }
    if (mylist->locate(speed))
    {
        std::cout << "located:" << mylist->getfactor(speed) << std::endl;
        return mylist->getfactor(speed);
    }
    float step1 = 0.05;
    float step2 = 0.05;
    float maxz = -100000000;
    float minz = 100000000;
    bool locateMin = false;
    float XofminZ = 0;
    int count = 0;
    float startX = 0.5;
    while (!locateMin)
    {
        for (float x = startX; x < startX + 10; x += step1)
        {
            if (count > 20)
            {
                locateMin = true;
                break;
            }
            float mz = getMinZonX(x, step2);
            if (mz < minz)
            {
                minz = mz;
                XofminZ = x;
                count = 0;
            }
            else
                count++;
        }
        if (!locateMin)
            startX += 10;
    }
    bool locateMax = false;
    float XofmaxZ = XofminZ;
    startX = XofminZ;
    count = 0;
    step1 = 0.02;
    step2 = 0.02;
    while (!locateMax)
    {
        for (float x = startX; x < startX + 10; x += step1)
        {
            if (count > 40)
            {
                locateMax = true;
                break;
            }
            float mz = getMaxZonX(x, step2);
            if (mz > maxz)
            {
                maxz = mz;
                XofmaxZ = x;
                count = 0;
            }
            else
                count++;
        }
        if (!locateMax)
            startX += 10;
    }
    int index = (speed - 0.5) / 0.25;
    float tuning = 0;
    if (index > 18 || index < 0)
        return 0;
    float approCrestH = computeApproCrestH(index, speed);
    // std::cout<<mylist->locate(speed)<<std::endl;
    tuning = maxz / approCrestH;
    std::cout << "tuning" << tuning << std::endl;
    std::cout << maxz << std::endl;
    mylist->push(speed, tuning);

    return tuning;
}

float InfluencingPoint::integrand(float x, float y, float theta)
{
    float modual1 = 1 - exp(-k0 * d * pow(1 / cos(theta), 2));
    float phase = k0 * pow(1 / cos(theta), 2) * (x * cos(theta) + y * sin(theta));
    return modual1 * sin(phase);
}
float InfluencingPoint::integrate(float x, float y)
{
    float dTheta = 0.005;
    float z = 0;
    for (float theta = -pi / 2; theta < pi / 2; theta += dTheta)
    {
        z += integrand(x, y, theta) * dTheta;
    }
    return 4 * b * z / (pi * k0 * l);
}
float InfluencingPoint::getMinZonX(float x, float step2)
{
    float upperbound = 0.4 * x;
    float lowerbound = 0;
    float minZonX = 100000000;
    for (float y = lowerbound; y < upperbound; y += step2)
    {
        float z = integrate(x, y);
        if (z < minZonX)
        {
            minZonX = z;
        }
    }
    return minZonX;
}
float InfluencingPoint::getMaxZonX(float x, float step2)
{
    float upperbound = 0.4 * x;
    float lowerbound = 0;
    float maxZonX = -100000000;
    for (float y = lowerbound; y < upperbound; y += step2)
    {
        float z = integrate(x, y);
        if (z > maxZonX)
        {
            maxZonX = z;
        }
    }
    return maxZonX;
}
float InfluencingPoint::computeApproCrestH(int index, float v)
{
    float v1 = sampleSpeed[index];
    float v2 = sampleSpeed[index + 1];
    float h1 = maxCrest[index];
    float h2 = maxCrest[index + 1];
    float k = (h2 - h1) / (v2 - v1);
    float b = (v2 * h1 - v1 * h2) / (v2 - v1);
    return k * v + b;
}
*/