#include "surfacegrid.h"
surfaceGrid::surfaceGrid()
{
    resolution  = 0.1;
    minX = 0;
    minY = 0;
    maxX = 1;
    maxY = 1;
    sizeX = (maxX-minX)/resolution;
    sizeY = (maxY-minY)/resolution;

    mygrid = new float*[sizeX];
    for (int i = 0;i<sizeX;i++){
        mygrid[i] = new float[sizeY];
        for (int j=0;j<sizeY;j++){
            mygrid[i][j] = 0;
        }
    }
}
surfaceGrid::~surfaceGrid(){
    for (int i = 0;i<sizeX;i++){
        delete [] mygrid[i];
    }
    delete [] mygrid;
}

surfaceGrid::surfaceGrid(float minX,float maxX,float minY,float maxY,float resolution){
    this->resolution = resolution;
    this->minX = minX;
    this->maxX = maxX;
    this->minY = minY;
    this->maxY = maxY;
    sizeX = (maxX-minX)/resolution;
    sizeY = (maxY-minY)/resolution;
    std::cout<<sizeX<<std::endl;
    std::cout<<sizeY<<std::endl;
    mygrid = new float*[sizeX];
    for (int i = 0;i<sizeX;i++){
        mygrid[i] = new float[sizeY];
        for (int j=0;j<sizeY;j++){
            mygrid[i][j] = 0;
        }
    }
}

void surfaceGrid::initElevation(float x, float z, float elevation, float Xcenter, float Ycenter, float r){
    if (x< minX || z <minY) return;
    if (x>=maxX || z >=maxY) return;

    int posX = (x-minX)/resolution;
    int posZ = (z-minY)/resolution;
    if (posX<0||posX>=sizeX) return;
    if (posZ<0||posZ>=sizeY) return;
    mygrid[posX][posZ] = elevation;


}
void surfaceGrid::refreshElevation(float x, float z,float elevation){
    if (x< minX || z <minY) return;
    if (x>=maxX || z >=maxY) return;

    int posX = (x-minX)/resolution;
    int posZ = (z-minY)/resolution;

    if (posX<0||posX>=sizeX) return;
    if (posZ<0||posZ>=sizeY) return;

    mygrid[posX][posZ] += elevation;
}

void surfaceGrid::refreshEleByGrid(int x, int z, float elevation){
    if (x<0||x>=sizeX) return;
    if (z<0||z>=sizeY) return;

    mygrid[x][z] +=elevation;
}

float surfaceGrid::getElevation(float x, float z) const{
    if (x< minX || z <minY) return 0;
    if (x>=maxX || z >=maxY) return 0;

    int posX = (x-minX)/resolution;
    int posZ = (z-minY)/resolution;

    return mygrid[posX][posZ];
}

float surfaceGrid::getX(int posX) const{
    if (posX<0 ||posX>=sizeX) return 0;
    return float(posX) * resolution + minX;
}

float surfaceGrid::getY(int posY) const{
    if (posY<0 ||posY>=sizeY) return 0;
    return float(posY) * resolution + minY;
}
float surfaceGrid::getminX() const{
    return minX;
}
float surfaceGrid::getmaxX() const{
    return maxX;
}
float surfaceGrid::getminY() const{
    return minY;
}
float surfaceGrid::getmaxY() const{
    return maxY;
}
float surfaceGrid::getResolution() const{
    return resolution;
}
float surfaceGrid::getSizeX() const{
    return sizeX;
}
float surfaceGrid::getSizeY() const{
    return sizeY;
}

void surfaceGrid::processData(std::string filename){
    std::ifstream infile;
    infile.open(filename.c_str());
    std::string line;
    std::string value;
    while(getline(infile,line)){
        std::stringstream ss(line);
        std::vector<float> shipData;
        while (getline(ss,value,',')){
            shipData.push_back(atof(value.c_str()));
        }
        std::cout<<shipData[0]<<","<<shipData[1]<<","<<shipData[2]<<std::endl;
        refreshElevation(shipData[0],shipData[1],shipData[2]);
    }
    std::cout<<"Finish Loading Data"<<std::endl;
    infile.close();
}
/*output data to csv file */
void surfaceGrid::outputData(std::string filename){
    std::ofstream ofile;
    int deltaX = 1;
    int deltaY = 1;
    int count = 0;
    ofile.open(filename.c_str());
    for (int i = 0;i<sizeX;i+=deltaX){
        for(int j = 0;j<sizeY;j+=deltaY){
            if (mygrid[i][j]==0) continue;
            float x = float(i)*resolution+minX;
            float y = float(j)*resolution+minY;
            ofile<<x<<","<<y<<","<<mygrid[i][j]<<"\n";
            count++;
        }
    }
    ofile.close();
    std::cout<<count<<std::endl;
}
void surfaceGrid::clear(){
    for (int i = 0;i<sizeX;i++){
        for (int j = 0;j<sizeX;j++){
            mygrid[i][j] = 0;
        }
    }
}
