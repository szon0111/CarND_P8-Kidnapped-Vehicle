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
	normal_distribution<double> N_x(x, std[0]);
	normal_distribution<double> N_y(y, std[1]);
	normal_distribution<double> N_theta(theta, std[2]);

	// initialize particles
	for (int i = 0; i < NUM_PARTICLES; i++) {
		Particle particle;
		particle.id = i;
		particle.x = N_x(gen);
		particle.y = N_y(gen);
		particle.theta = N_theta(gen);
		particle.weight = 1;

		// add to list
		particles.push_back(particle);
		weights.push_back(1);
	}

	// set as initialized
	is_initialized = true;

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
		
		if (yaw_rate ==  0) {
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

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

	double big_num = 1.0e99;

	for (int i = 0; i < observations.size(); i++) {
		int current_j;
		double current_smallest_error = big_num;

		for (int j = 0; j < predicted.size(); j++) {
			double d_x = predicted[j].x - observations[i].x;
			double d_y = predicted[j].y - observations[i].y;
			double error = d_x * d_x + d_y * d_y;

			if (error < current_smallest_error) {
				current_j = j;
				current_smallest_error = error;
			}
		}
		observations[i].id = current_j;
	}

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {

	double std_x = std_landmark[0];
	double std_y = std_landmark[1];
	double n_a = 0.5 / (std_x * std_x);
	double n_b = 0.5 / (std_y * std_y);

	for (int i = 0; i < NUM_PARTICLES; i++) {

		double p_x = particles[i].x;
		double p_y = particles[i].y;
		double p_theta = particles[i].theta;

		vector<LandmarkObs> landmarks_in_range;
		vector<LandmarkObs> map_observations;

		// transform each observation to map coordinate system
		for (int j = 0; j < observations.size(); j++) {
			
			int obs_id = observations[j].id;
			double obs_x = observations[j].x;
			double obs_y = observations[j].y;

			double tran_x = p_x + obs_x * cos(p_theta) - obs_y * sin(p_theta);
			double tran_y = p_y + obs_y * cos(p_theta) + obs_x * sin(p_theta);

			LandmarkObs observation;
			observation.id = obs_id;
			observation.x = tran_x;
			observation.y = tran_y;

			map_observations.push_back(observation);
			
		}

		// find map landmarks within sensor range
		for (int j = 0; j < map_landmarks.landmark_list.size(); j++) {

			int map_id = map_landmarks.landmark_list[j].id_i;
			double map_x = map_landmarks.landmark_list[j].id_i;
			double map_y = map_landmarks.landmark_list[j].id_i;

			double d_x = map_x - p_x;
			double d_y = map_y - p_y;
			double error = sqrt(d_x * d_x + d_y * d_y);

			if (error < sensor_range) {

				LandmarkObs landmark_in_range;
				landmark_in_range.id = map_id;
				landmark_in_range.x = map_x;
				landmark_in_range.y = map_y;

				landmarks_in_range.push_back(landmark_in_range);	
			}
		}

		// associate landmark in range to landmark observations
		dataAssociation(landmarks_in_range, map_observations);

		// update weight based on comparison of observations
		int w = 1;

		for (int j = 0; j < map_observations.size(); j++) {
			int obs_id = map_observations[j].id;
			int obs_x = map_observations[j].x;
			int obs_y = map_observations[j].y;

			double pred_x = landmarks_in_range[obs_id].x;
			double pred_y = landmarks_in_range[obs_id].y;

			double d_x = obs_x - pred_x;
			double d_y = obs_x - pred_y;

			double a = n_a * d_x * d_x;
			double b = n_b * d_y * d_y;
			double eq = (exp(-(a + b))) / (sqrt( 2.0 * M_PI * std_x * std_y));

			w *= eq;
		
		}

		// update
		particles[i].weight = w;
		weights[i] = w;

	}
}

void ParticleFilter::resample() {

	default_random_engine gen;
	discrete_distribution<int> distr (weights.begin(),weights.end());
	vector<Particle> resampled_particles;

	for (int i = 0; i < NUM_PARTICLES; i++) {
		resampled_particles.push_back(particles[distr(gen)]);
	}
	particles = resampled_particles;

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
