#include <iostream>
#include "influencingpoint.h"
#include "shipcourse.h"
#include <time.h>

using namespace std;

/* Example of generating from kelvin wake
 * Using constant velocity
 */
int main()
{
    clock_t start, end;
    /*Set the time step of each sample*/
    float timestep = 0.05;
    /*Constant velocity */
    float vel = 2;

    /* generate the ship course member of each ship
     * Draft: 0.15 m
     * Halfbeam: 0.75 m
     * Halflength:1.55 m
     */
    shipCourse course = shipCourse(0.15, 0.75, 1.55);
    /* Generate surface grid to store wake data
     * xRange: [-0.02 m,40 m]
     * yRange: [-25 m, 25 m]
     * resolution: 0.05
     */
    surfaceGrid grid(-0.02000000, 40.00000, -25.000000, 25.000000, 0.0500000);

    /* grow the ship course for 20 seconds */
    for (float time = 0; time < 20; time += timestep)
    {
        /* ship position */
        float posX = vel * time;
        float posY = 0;
        /* expand the course, add more influencing point */
        course.expand(posX, posY, time, grid.getResolution());
    }

    std::cout << "output" << std::endl;
    cout << "Influencing point number:" << course.chain.size() << endl;
    start = clock();
    /* output data to surface grid */
    course.outputDataToGrid(grid);
    end = clock();
    cout << "time:" << (double)(end - start) / CLOCKS_PER_SEC << endl;
    /*Store the data to csv file*/
    /* Use Matlab script to visualize the data */
    grid.outputData("/script/2m.csv");
    cout << "Finish importing data" << endl;
    return 0;
}
