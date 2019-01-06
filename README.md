# kidnapped vehicle Localization using Particle Filter Project
Self-Driving Car Engineer Nanodegree Program (Term-2 Project-3)

![](https://github.com/emilkaram/kidnapped-vehicle-Localization-using-Particle-Filter-Project-T2-P3/blob/master/img/0.jpg)


## Project Introduction
A Car/Robot has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.

In this project I implemented a 2 dimensional particle filter in C++. 
The particle filter is be given a map and some initial localization information (analogous to what a GPS would provide). 
At each time step the filter also gets observation and control data.

## Running the Code
This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

The project code is c++ and uses a Simulator to  demonstrate the particle filter.

I used Linux for coding and run the simulator on Windows 10 using a port forward from the VM (Oracle VM virtualBox). 
 
To compile and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./particle_filter

## Project description:

Particle filters are the realisation of Bayes filters or Markov localisation filter. Particle filters concepts are mainly used to solve the localisation problems

In the picture below, red dots are the discrete guesses of where the car/robot might be. 
Each red dot has x coordinates, y coordinates, and orientation. 
Particle filter is the set of several thousand such guesses comprise approximate representation of the posterior of the robot. 
In the beginning, particles are uniformly spread, but filter make them survive in proportion to how consistent is the particles with sensor measurements and this called the particle weight.

![](https://github.com/emilkaram/kidnapped-vehicle-Localization-using-Particle-Filter-Project-T2-P3/blob/master/img/3.gif) 

# The input data:
INPUT: values provided by the simulator to the c++ program

// sense noisy position data from the simulator

["sense_x"]

["sense_y"]

["sense_theta"]

// get the previous velocity and yaw rate to predict the particle's transitioned state

["previous_velocity"]

["previous_yawrate"]

// receive noisy observation data from the simulator, in a respective list of x/y values

["sense_observations_x"]

["sense_observations_y"]


# The output data:
OUTPUT: values provided by the c++ program to the simulator

// best particle values used for calculating the error evaluation

["best_particle_x"]

["best_particle_y"]

["best_particle_theta"]

//Optional message data used for debugging particle's sensing and associations

// for respective (x,y) sensed positions ID label

["best_particle_associations"]

// for respective (x,y) sensed positions

["best_particle_sense_x"] <= list of sensed x positions

["best_particle_sense_y"] <= list of sensed y positions


I built out the methods in `particle_filter.cpp` until the simulator output says:

```
Success! Your particle filter passed!
```
![](https://github.com/emilkaram/kidnapped-vehicle-Localization-using-Particle-Filter-Project-T2-P3/blob/master/img/2.png)

# Implementing the Particle Filter
The directory structure of this repository is as follows:

```
root
|   build.sh
|   clean.sh
|   CMakeLists.txt
|   README.md
|   run.sh
|
|___data
|   |   
|   |   map_data.txt
|   
|   
|___src
    |   helper_functions.h
    |   main.cpp
    |   map.h
    |   particle_filter.cpp
    |   particle_filter.h
```

## Inputs to the Particle Filter
You can find the inputs to the particle filter in the `data` directory.

#### The Map*
`map_data.txt` includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns
1. x position
2. y position
3. landmark id

### All other data the simulator provides, such as observations and controls.

> * Map data provided by 3D Mapping Solutions GmbH.


# Steps to implement particle filter

1- Initialisation step: 
In the initialization step I estimated the car/robot position from GPS input. 

![](https://github.com/emilkaram/kidnapped-vehicle-Localization-using-Particle-Filter-Project-T2-P3/blob/master/img/4.png)


2-Prediction step: 
During the prediction step we add the control input (yaw rate & velocity) for all particles.

![](https://github.com/emilkaram/kidnapped-vehicle-Localization-using-Particle-Filter-Project-T2-P3/blob/master/img/5.png)

3-Update step: 
During the update step, we update our particle weights using map landmark positions and feature measurements.
Update step has three sub-steps:
a- Transformation

![](https://github.com/emilkaram/kidnapped-vehicle-Localization-using-Particle-Filter-Project-T2-P3/blob/master/img/7.png)

![](https://github.com/emilkaram/kidnapped-vehicle-Localization-using-Particle-Filter-Project-T2-P3/blob/master/img/8.png)
 


b - Association

![](https://github.com/emilkaram/kidnapped-vehicle-Localization-using-Particle-Filter-Project-T2-P3/blob/master/img/11.png)

![](https://github.com/emilkaram/kidnapped-vehicle-Localization-using-Particle-Filter-Project-T2-P3/blob/master/img/13.png)


c - Update Weights

![](https://github.com/emilkaram/kidnapped-vehicle-Localization-using-Particle-Filter-Project-T2-P3/blob/master/img/9.png)


4-Resample step: 
During resampling we will resample M times (M is range of 0 to length_of_particleArray) drawing a particle i (i is the particle index) proportional to its weight . Resampling wheel is used at this step.

![](https://github.com/emilkaram/kidnapped-vehicle-Localization-using-Particle-Filter-Project-T2-P3/blob/master/img/6.png)
 
 
 
 
 ![](https://github.com/emilkaram/kidnapped-vehicle-Localization-using-Particle-Filter-Project-T2-P3/blob/master/img/10.png)
 
 

 
 
 
 

# Conclusion:
1. **Accuracy**: my particle filter successfully localized the vehicle position and yaw to within the values specified in the parameters `max_translation_error` and `max_yaw_error` in `src/main.cpp`.

2. **Performance**: My particle filter completed execution within the time of 100 seconds.

![](https://github.com/emilkaram/kidnapped-vehicle-Localization-using-Particle-Filter-Project-T2-P3/blob/master/img/2.png)

 
