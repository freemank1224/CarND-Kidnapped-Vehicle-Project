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

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	default_random_engine gen;

	//num_particles = 1000;  DEFINED in particle_filter.h

	double std_x, std_y, std_theta;
	std_x = std[0];
	std_y = std[1];
	std_theta = std[2];


	normal_distribution<double> dist_x(x, std_x);
	normal_distribution<double> dist_y(y, std_y);
	normal_distribution<double> dist_theta(theta, std_theta);

	for(int i=0; i<num_particles; i++){



		particles[i].id = i;
		particles[i].x = dist_x(gen);
		particles[i].y = dist_y(gen);
		particles[i].theta = dist_theta(gen);

		particles[i].weight = 1.0;
	}

	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	default_random_engine gen;

	double std_x, std_y, std_theta;
	std_x = std_pos[0];
	std_y = std_pos[1];
	std_theta = std_pos[2];

	// Add noise
	normal_distribution<double> dist_x(0, std_x);
	normal_distribution<double> dist_y(0, std_y);
	normal_distribution<double> dist_theta(0, std_theta);


	if(yaw_rate != 0){
		for(int i=0; i<num_particles ;i++){
			particles[i].x += velocity * (sin(particles[i].theta + yaw_rate*delta_t) - sin(particles[i].theta)) / yaw_rate;
			particles[i].y += velocity * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate*delta_t)) / yaw_rate;
			particles[i].theta += yaw_rate * delta_t;

			particles[i].x = dist_x(gen);
			particles[i].y = dist_y(gen);
			particles[i].theta = dist_theta(gen);
		}	
	}else{
		for(int i=0; i<num_particles ;i++){
			particles[i].x += velocity * delta_t * cos(particles[i].theta) + dist_x(gen);
			particles[i].y += velocity * delta_t * sin(particles[i].theta) + dist_y(gen);
			particles[i].theta += dist_theta(gen);
		}

	}

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

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


	// Each particle see many observations and choose the closest observed landmark
	for (int i = 0; i < num_particles; ++i)
	{
		
		// Define the initial range between the adjacent observed and map landmarks
		double min_sensor_dist = 2 * sensor_range;

		double weight_temp = 1.0;

		// For each observation landmark
		for (int j = 0; j < observations.size(); ++j)
		{

			int temp_id_weight;

			// Transform the observation landmarks from car observation coordinates to map coordinates
			double x_m = particles[i].x + cos(particles[i].theta) * observations[j].x - sin(particles[i].theta) * observations[j].y;
			double y_m = particles[i].y + sin(particles[i].theta) * observations[j].x - cos(particles[i].theta) * observations[j].y;


			// 给定粒子与观测物的距离 Calculate the distance between observed landmark from a given particle
			double observation_dist = dist(x_m, y_m, particles[i].x, particles[i].y);

			for (int k = 0; k < map_landmarks.landmark_list.size(); ++k)
			{

				// 地图观测物与粒子的距离 For a given particle, calculate the max view distance which defined by the sensor_range variable
				double map_mark_dist = dist(particles[i].x, particles[i].y, map_landmarks.landmark_list[k].x_f, map_landmarks.landmark_list[k].y_f);

				// Within the view distance
				if ((map_landmarks.landmark_list[k].x_f - particles[i].x < sensor_range) && (map_landmarks.landmark_list[k].y_f - particles[i].y < sensor_range))
				{
					// 特定观测物与地图标志的距离 Obtain the distance between map landmarks and observation landmark
					double near_dist = dist(x_m, y_m, map_landmarks.landmark_list[k].x_f, map_landmarks.landmark_list[k].y_f);

					if (near_dist < min_sensor_dist)
					{
						particles[i].id = k;
						min_sensor_dist = near_dist;

					}

				}else{
					
				}

				cout << "-------- Current Association ---------" << endl;
				cout << "Particle" << i << "'s association:" << particles[i].id << endl;

				// *********** Find the nearest matched one !!! ************
				temp_id_weight = k;
			}

			
			
			/*weight_temp = exp(-(pow((x_m - map_landmarks.landmark_list[temp_id_weight].x_f),2)/(2*pow(std_landmark[0],2)) + pow((y_m - map_landmarks.landmark_list[temp_id_weight].y_f),2)/(2*pow(std_landmark[1],2)))) 
								/ (2*M_PI*std_landmark[0],std_landmark[1]);	
			*/

			particles[i].weight *= 1/(2*M_PI*std_landmark[0]*std_landmark[1]) * exp(-(pow(x_m - map_landmarks.landmark_list[particles[i].id].x_f, 2) 
                                                                                      + pow(y_m - map_landmarks.landmark_list[particles[i].id].y_f, 2))
									/(2 * M_PI *std_landmark[0]*std_landmark[1]));

		}


		weights[i] = particles[i].weight;


	}

}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	default_random_engine gen;

	discrete_distribution<int> dist_d(weights.begin(), weights.end());

	for (int i = 0; i < num_particles; ++i)
	{
		particles_re[i] = particles[dist_d(gen)];
	}

	particles = particles_re;

}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
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
