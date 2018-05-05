/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

default_random_engine gen;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

	num_particles = 100;


	if(!is_initialized){
		normal_distribution<double> dist_x(x,std[0]);
		normal_distribution<double> dist_y(y,std[1]);
		normal_distribution<double> dist_theta(theta,std[2]);

		for (int i = 0; i < num_particles; i++)
		{
			Particle particle;
			particle.id = i;
			particle.x = dist_x(gen);
			particle.y = dist_y(gen);
			particle.theta = dist_theta(gen);
			particle.weight = 1;

			particles.push_back(particle);
			weights.push_back(1); //particle.weight?
		}

		is_initialized = true;
		//cout << "Initialized!"<<endl;
	}
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/


	for (int i = 0; i < num_particles; i++)
	{
		//Particle particle = particles[i];

		double new_x;
		double new_y;
		double new_theta;

		if(fabs(yaw_rate) >= 0.0001){	
		new_x = particles[i].x + velocity/yaw_rate*(sin(particles[i].theta+yaw_rate*delta_t)-sin(particles[i].theta));
		new_y = particles[i].y +velocity/yaw_rate*(cos(particles[i].theta)-cos(particles[i].theta+yaw_rate*delta_t));
		new_theta = particles[i].theta + yaw_rate*delta_t;
	}
	else{
		new_x = particles[i].x + velocity*cos(particles[i].theta)*delta_t;
		new_y = particles[i].y + velocity*sin(particles[i].theta)*delta_t;			
	}
	

	normal_distribution<double> dist_x(new_x,std_pos[0]);
	normal_distribution<double> dist_y(new_y,std_pos[1]);
	normal_distribution<double> dist_theta(new_theta,std_pos[2]);

	particles[i].x = dist_x(gen);
	particles[i].y = dist_y(gen);
	particles[i].theta = dist_theta(gen);
}	

}
void ParticleFilter::dataAssociation (std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations){
// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	for (int i = 0; i < observations.size(); i++)
	{
		//LandmarkObs observation = observations[i];
		double distance;
		double min_dist = INFINITY;
		//int min_dist_id = -1;

		for (int j = 0; j < predicted.size(); j++)
		{
			//LandmarkObs landmark_from_map = predicted[j];
			//distance = dist(observation.x,observation.y,landmark_from_map.x,landmark_from_map.y);
			distance = dist(observations[i].x,observations[i].y,predicted[j].x,predicted[j].y);

			if (distance<min_dist)
			{
				min_dist = distance;
			 	//min_dist_id = j;
				observations[i].id = predicted[j].id;
			 	//cout << "particle " <<j<< " distance " <<min_dist << endl;
			} 
		}

	}
}
void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
	const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html
	
	//double x_trans;
	//double y_trans;
	double theta_i;	

	double sigma_x = std_landmark[0]; //Used at weight calculations
	double sigma_y = std_landmark[1]; //Used at weight calculations
	double normalizer = 1.0 / (2*M_PI*sigma_x*sigma_y); //Used at weight calculations

	double distance = 0.0;
	double min_dist = INFINITY;

	int associated_landmark_id = -1;

	for (int p = 0; p < num_particles; p++)
	{
		vector<int> associations;
		vector<double> sense_x;
		vector<double> sense_y;

		Particle particle = particles[p];
		theta_i = particle.theta;

		vector<LandmarkObs> transformed_observations;		

		LandmarkObs obs;

		//Transform observations associated with the given particle from vehcile to map coordinate
		for (unsigned int i = 0; i < observations.size(); i++)
		{
			LandmarkObs trans_obs;
			obs = observations[i];

			trans_obs.x = particle.x + obs.x*cos(theta_i) - obs.y*sin(theta_i);
			trans_obs.y = particle.y + obs.x*sin(theta_i) + obs.y*cos(theta_i);
			trans_obs.id = obs.id;
			transformed_observations.push_back(trans_obs);
		}

		particles[p].weight = 1.0;

		//Closest distanced landmark-particle
		for (int i = 0; i < transformed_observations.size(); i++)
		{
			for (int j = 0; j < map_landmarks.landmark_list.size(); j++)
			{
				distance = dist(transformed_observations[i].x,transformed_observations[i].y,map_landmarks.landmark_list[j].x_f,map_landmarks.landmark_list[j].y_f);
				if (distance < min_dist)
				{
					min_dist = distance;
					associated_landmark_id = j;
				}
			}

		//Check if associated Landmark exists
			if (associated_landmark_id >= 0)
			{
				double x_mean = map_landmarks.landmark_list[associated_landmark_id].x_f;
				double y_mean = map_landmarks.landmark_list[associated_landmark_id].y_f;

				double p_x = pow((transformed_observations[i].x-x_mean),2)/(2*pow(sigma_x,2));
				double p_y = pow((transformed_observations[i].y-y_mean),2)/(2*pow(sigma_y,2));

				double probability = normalizer * exp(-1*(p_x+p_y));
				if (probability>0)
				{
					particles[p].weight *= probability;
				}
			}

			associations.push_back(associated_landmark_id+1);
			sense_x.push_back(transformed_observations[i].x);
			sense_y.push_back(transformed_observations[i].y);
		}

		particles[p] = SetAssociations(particles[p], associations, sense_x,sense_y);
		weights[p] = particles[p].weight;
	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	
	discrete_distribution<int> distribution(weights.begin(), weights.end());

	vector<Particle> resample_particles;
	for (int i = 0; i < num_particles; i++)
	{
		resample_particles.push_back(particles[distribution(gen)]);
	}
	particles = resample_particles;

}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
	const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations= associations;
	particle.sense_x = sense_x;
	particle.sense_y = sense_y;
	return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
	copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
	string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
	copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
	string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
	copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
	string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
