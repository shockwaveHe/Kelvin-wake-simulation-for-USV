#
**Kelvin wake simulation for USV**
#
author: Yao He

##
**Paper link and citation**
##

Link: https://ieeexplore.ieee.org/document/9517533

To cite this 
```
@inproceedings{HeSQJQ21,
  title = {Computational Efficient Simulation of Kelvin Wake for Unmanned Surface Vehicles},
  author = {Yao He and Qinbo Sun and Weimin Qi and Xiaoqiang Ji and Huihuan Qian},
  year = {2021},
  doi = {10.1109/RCAR52367.2021.9517533},
  url = {https://doi.org/10.1109/RCAR52367.2021.9517533},
  pages = {1070-1075},
  booktitle = {IEEE International Conference on Real-time Computing and Robotics, RCAR 2021, Xining, China, July 15-19, 2021},
  publisher = {IEEE},
  isbn = {978-1-6654-3678-6}
}
```
##
**Dependencies**
##
C++ 11 and above

No specific third party repositories

##
**Usage**
##

Include the file to your project file folders then use CMake to build it.

###
Specific steps
###
1. Use the *shipCourse* class to generate the course for each ship
2. Generate a global water surface grid using *surfaceGrid* class
3. Update each ship course at each time stamp using 
```
shipCourse::expand()
```
Every ship position is in the frame of your generated surface gird.

4. Output wake data to surface grid using 
```
shipCourse::outputDataToGrid(surfaceGrid)
```
Every point (x,y) is in the frame of your generated surface gird.

5. Store wake data to .csv file
```
surfaceGrid::outputData(filename)
```
6. Get wake elevation at point (x, y) in the world call
```
shipCourse::getElevation(x,y)
```
If there are multiple ships, you should itter all the ship courses. 
Every point (x,y) is in the frame of your generated surface gird.

##
**Running the example**
##

We provide an example *KW_example* for you to run. 

###
Steps:
###
1. Build the porject using CMake
 ```
 $ cmake .
 $ make
 ```
 2. Run the example 
  ```
 $ ./KW_example
 ```
 3. For comparison, we provide a *calculated_example* using equation (1) in the paper.
   ```
 $ ./calculated_example
 ```
 4. The generated wake data will be store in .csv files. Use Matlab script to visualize the results. We provide the visualization scripts in the script folder. You should specify the .csv file name in the script.
 
 ##
 **Sample results**
 ##
 Ship speed 2m/s
 
 <img src="https://github.com/shockwaveHe/Kelvin-wake-simulation-for-USV/blob/main/figures/2m-1.png" width="300">
 
 
 Ship speed varies
 
  <img src="https://github.com/shockwaveHe/Kelvin-wake-simulation-for-USV/blob/main/figures/figure_varied-1.png" width="300">
  
 See more in figures folder.
