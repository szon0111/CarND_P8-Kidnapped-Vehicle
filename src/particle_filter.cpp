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
// set number of particles
const int NUM_PARTICLES = 100;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	
	// engine for generation of particles
	default_random_engine gen;

	// create normal distribution
	normal_distribution<double> N_x_init(x, std[0]);
	normal_distribution<double> N_y_init(y, std[1]);
	normal_distribution<double> N_theta_init(theta, std[2]);

	// initialize particles
	for (int i = 0; i < NUM_PARTICLES; i++) {
		Particle particle;
		particle.id = i;
		particle.x = N_x_init(gen);
		particle.y = N_y_init(gen);
		particle.theta = N_theta_init(gen);
		particle.weight = 1.0;

		// add to list
		particles.push_back(particle);
		weights.push_back(particle.weight);

	}

	// set as initialized
	is_initialized = true;

	cout << "initialized particles with uniform weights" <<endl;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	
	// engine for generation of particles
	default_random_engine gen;

	// create normal distribution for sensor noise
	normal_distribution<double> N_x(0, std_pos[0]);
	normal_distribution<double> N_y(0, std_pos[1]);
	normal_distribution<double> N_theta(0, std_pos[2]);

	// predict new state depending on yaw_rate
	for (int i = 0; i < NUM_PARTICLES; i++) {

		double theta = particles[i].theta;
		
		if (fabs(yaw_rate) < 0.00001) {
			particles[i].x += velocity * delta_t * cos(particles[i].theta);
			particles[i].y += velocity * delta_t * sin(particles[i].theta);
		}
		else {
			particles[i].x += velocity / yaw_rate * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
			particles[i].y += velocity / yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t));
			particles[i].theta += yaw_rate * delta_t;
		}

		// add noise to particles
		particles[i].x += N_x(gen);
		particles[i].y += N_y(gen);
		particles[i].theta += N_theta(gen);

	}
	cout << "prediction complete" << endl;

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {

	/**
	Implemented in updateWeights function
	**/

	// double min_dist = 100000;
	// int association = 0;

	// for (int i = 0; i < observations.size(); i++) {

	// 	min_dist = 100000;
	// 	for (int j = 0; j < predicted.size(); j++) {
			
	// 		double d = dist(predicted[j].x, predicted[j].y, observations[i].x, observations[i].y);
	// 		if (d < min_dist) {
	// 			association = predicted[j].id;
	// 			min_dist = d;
	// 		}
	// 	}
	// observations[i].id = association;
	// }
	// cout << "data association complete" << endl;

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {

	double std_x = std_landmark[0];
	double std_y = std_landmark[1];
	double weights_sum = 0;	
	
	for (int i = 0; i < NUM_PARTICLES; i++) {
		// predict measurements to all map landmarks
		Particle& particle = particles[i];
		double w = 1.0;
		
		for (int j = 0; j < observations.size(); j++) {
			// transform observation to map coordinates
			LandmarkObs obs = observations[j];
			
			// predict landmark x, y
			double predicted_x = obs.x * cos(particle.theta) - obs.y * sin(particle.theta) + particle.x;
			double predicted_y = obs.x * sin(particle.theta) + obs.y * cos(particle.theta) + particle.y;

			Map::single_landmark_s nearest_landmark;
			double min_distance = sensor_range;
			double distance = 0;

			// associate landmark in range to landmark obs 
			for (int k = 0; k < map_landmarks.landmark_list.size(); k++) {

				Map::single_landmark_s landmark = map_landmarks.landmark_list[k];

				// calculate distance between landmark and transformed observations
				distance = fabs(predicted_x - landmark.x_f) + fabs(predicted_y - landmark.y_f);

				// update nearest landmark to obs
				if (distance < min_distance) {
					min_distance = distance;
					nearest_landmark = landmark;
				}
			}

			// calculate new weight of each particle
			double x_diff = predicted_x - nearest_landmark.x_f;
			double y_diff = predicted_y - nearest_landmark.y_f;
			double num = exp(-0.5 * ((x_diff * x_diff)/(std_x * std_x) + (y_diff * y_diff)/(std_y * std_y)));
			double denom = 2 * M_PI * std_x * std_y;
			
			w *= num / denom;

		}

		cout << "weight: " << w << endl;

		// update particle weight 
		particle.weight = w;
		weights[i] = w;

	}

	cout << "weight update complete" << endl;
}

void ParticleFilter::resample() {

	default_random_engine gen;
	discrete_distribution<int> distr (weights.begin(),weights.end());
	vector<Particle> resampled_particles;

	for (int i = 0; i < NUM_PARTICLES; i++) {
		resampled_particles.push_back(particles[distr(gen)]);
	}
	particles = resampled_particles;

	cout << "resampling complete" << endl;

}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
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
