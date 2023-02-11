### *** Archieved: As this repo is and will not updated anymore, it is archieved. ***

# Particle Filter Project Code

## Self-Driving Car Engineer Nanodegree Program

System: Desktop, Windows 10 Bash on Ubuntu (Linux Subsystem)

##The goals / steps of this project are the following:

Your robot has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.

In this project I will implement a 2 dimensional particle filter in C++. The particle filter will be given a map and some initial localization information (analogous to what a GPS would provide). At each time step, the filter will also get observation and control data. 

## Running the Code
This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

Here is the main protcol that main.cpp uses for uWebSocketIO in communicating with the simulator.

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


OUTPUT: values provided by the c++ program to the simulator

// best particle values used for calculating the error evaluation

["best_particle_x"]

["best_particle_y"]

["best_particle_theta"] 

1. ParticleFilter::init:

Our car has no idea where it is, so let it get some noisy data from GPS to initialize the particles and set all the weights to 1.

2. ParticleFilter::prediction

Add velocity and yaw_rate measurements from simulator to C++ code to predict particles locations with noise.

3. ParticleFilter::dataAssociation

I prefered not the use this method. Instead used a for loop in updateWeights method where I looked for measurement-landmark relation.

4. ParticleFilter::updateWeights

Choose, for each particle -> 
			for each observation -> 
						which of them are in sensor range 
								with the closest distance between measurement-landmark.

Then calculate new weights with those chosen landmarks.


The things the grading code is looking for are:

1. **Accuracy**: your particle filter should localize vehicle position and yaw to within the values specified in the parameters `max_translation_error` and `max_yaw_error` in `src/main.cpp`.

2. **Performance**: your particle filter should complete execution within the time of 100 seconds.
