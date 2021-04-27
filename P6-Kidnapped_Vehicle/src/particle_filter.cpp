/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using namespace std;
using std::string;
using std::vector;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  
  default_random_engine gen; // random engine
  
  num_particles = 80;  // TODO: Set the number of particles
  
  // Standard deviations for x, y, and theta
//   double std_x, std_y, std_theta;  
//   std_x = std[0];
//   std_y = std[1];
//   std_theta = std[2]; 

  // This line creates a normal (Gaussian) distribution for x, y and theta
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);
  
 
   for (int i = 0; i <num_particles ; ++i) 
   {
     Particle particle_obj;
     particle_obj.id = i;
     particle_obj.x = dist_x(gen);
     particle_obj.y = dist_y(gen);
     particle_obj.theta = dist_theta(gen);
     particles.push_back(particle_obj);
     particle_obj.weight = 1.0;
     weights.push_back(particle_obj.weight);
   }
  	is_initialized = true;
    std::cout << "All the particles are initialized." << std::endl;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) 
{
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */
  default_random_engine gen;
  
  for (unsigned int i = 0; i < num_particles; ++i)
	{
    	double theta_to_add = particles[i].theta + yaw_rate*delta_t;
    
    	if(abs(yaw_rate)>0.0001)
        {
            particles[i].x = particles[i].x + (velocity/yaw_rate)*(sin(theta_to_add) - sin(particles[i].theta));
            particles[i].y  = particles[i].y + (velocity/yaw_rate) * (cos(particles[i].theta) - cos(theta_to_add));
            particles[i].theta = theta_to_add;
        }
    	else // For straight line. Without this at some point, the accumulated error goes really high.
        {
         	particles[i].x = particles[i].x + velocity*cos(particles[i].theta)*delta_t;
          	particles[i].y = particles[i].y + velocity*sin(particles[i].theta)*delta_t;
        }
		
		// noisy normal distributions around the new predictions and update the particles.
		normal_distribution<double> dist_x(particles[i].x, std_pos[0]);
		normal_distribution<double> dist_y(particles[i].y, std_pos[1]);
		normal_distribution<double> dist_theta(particles[i].theta, std_pos[2]);
		
		particles[i].x = dist_x(gen);
		particles[i].y = dist_y(gen);
		particles[i].theta = dist_theta(gen);
	}
	std::cout << "Velocity and Yaw rate incorporated." << std::endl;
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */
  	for (unsigned int i = 0; i < observations.size(); ++i) 
    {
	    LandmarkObs current_observation = observations[i];

	    // Initialize minimum distance to max value
	    double dist_min = numeric_limits<double>::max();
	    
      // Calculate distance between current observation and all the predicted landmarks and find the one 
      // with minimum distance and set that prediction id to observation id.
	    for (unsigned int j = 0; j < predicted.size(); ++j) 
        {
	      LandmarkObs predicted_observation = predicted[j];
	      double temp_distance = dist(current_observation.x, current_observation.y, predicted_observation.x, predicted_observation.y);
          
	      // Set id of the nearest landmark
	      if (temp_distance < dist_min)
          {
	        dist_min = temp_distance;
	        observations[i].id = predicted_observation.id;
	      }
	    }
	}
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) 
{
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian 
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */
  double std_x = std_landmark[0];
  double std_y = std_landmark[1];
  
  for (unsigned int i = 0; i < particles.size(); ++i)
	{
		double x_particle = particles[i].x;
		double y_particle = particles[i].y;
		double theta_particle = particles[i].theta;
		
    	// Transform observations from car to particle coordinates
		std::vector<LandmarkObs> transformed_observations_map;
		LandmarkObs transformed_obs;

		for (unsigned int j = 0; j < observations.size(); ++j)
		{		
			transformed_obs.id = observations[j].id; // used for checking if a landmark has been found or not
			transformed_obs.x = observations[j].x*cos(theta_particle) - observations[j].y*sin(theta_particle) + x_particle;
			transformed_obs.y = observations[j].x*sin(theta_particle) + observations[j].y*cos(theta_particle) + y_particle;
			transformed_observations_map.push_back(transformed_obs);
		}
		
    	// Figure out all the particles within given sensor range.
	    vector<LandmarkObs> predicted_landmarks;
    	LandmarkObs pred_landmark;
    
	    for (unsigned int j = 0; j < map_landmarks.landmark_list.size(); ++j) 
        {
	      pred_landmark.x = map_landmarks.landmark_list[j].x_f;
	      pred_landmark.y = map_landmarks.landmark_list[j].y_f;
	      pred_landmark.id = map_landmarks.landmark_list[j].id_i;
	      
          double temp_distance = dist(pred_landmark.x, pred_landmark.y, x_particle, y_particle);
	      if (temp_distance <= sensor_range) 
          {
	        predicted_landmarks.push_back(pred_landmark);
	      }
	    }

	    // DataAssociation
	    dataAssociation(predicted_landmarks, transformed_observations_map);

	    // Update weights based on particle observations and actual observations
		particles[i].weight = 1.0;
	    for (unsigned int j = 0; j < transformed_observations_map.size(); ++j) 
        {
          	// Find the nearest landmark cordinates.
		    int landmark_id = transformed_observations_map[j].id;
		    double x_landmark;
          	double y_landmark;

			for (unsigned int k = 0; k < predicted_landmarks.size(); ++k) 
            {
        		if (predicted_landmarks[k].id == landmark_id) 
                {
		        	x_landmark = predicted_landmarks[k].x;
		        	y_landmark = predicted_landmarks[k].y;
		        }
			}
		    
		    // Observed coordinates
			double x_observed = transformed_observations_map[j].x;
		    double y_observed = transformed_observations_map[j].y;

		    // Multivariate Gaussian logic
          	double normalizer = (2*M_PI*std_x*std_y);
		    double probability = exp(-( pow(x_landmark-x_observed,2)/(2*pow(std_x, 2)) + (pow(y_landmark-y_observed,2)/(2*pow(std_y, 2)))));
            double updated_weight = probability/normalizer;

		    particles[i].weight = particles[i].weight * updated_weight;
        }
	    weights[i] = particles[i].weight;
  	}
	std::cout << "Prediction Step done." << std::endl;
}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
  	default_random_engine gen;
	discrete_distribution<int> weighted_dist(weights.begin(), weights.end());
	
	vector<Particle> resampled_particles;
	
	// Resample particles based on weights
	for (unsigned int i = 0; i < num_particles; ++i)
	{
      int temp_index = weighted_dist(gen);
	  resampled_particles.push_back(particles[temp_index]);
	}
  
	particles = resampled_particles;
	std::cout << "Particles resampled." << std::endl;
}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) 
{
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}
