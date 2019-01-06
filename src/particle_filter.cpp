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
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"
#include "helper_functions.h"

using namespace std;


//Initialize random engine
static default_random_engine gen;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of
	//   x, y, theta and their uncertainties from GPS) and all weights to 1.
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).


	//Emil code start-1

	/**
	 * init Initializes particle filter by initializing particles to Gaussian
	 *   distribution around first position and all the weights to 1.
	 * @param x Initial x position [m] (simulated estimate from GPS)
	 * @param y Initial y position [m]
	 * @param theta Initial orientation [rad]
	 * @param std[] Array of dimension 3 [standard deviation of x [m], standard deviation of y [m]
	 *   standard deviation of yaw [rad]]
	 */


	if(is_initialized){
        return;
	}


	//Set the number of particles
	num_particles = 100;


	//Set standard deviations for x, y, and theta.
	 double std_x = std[0];
	 double std_y = std[1];
	 double std_theta = std[2];


	// Create normal (Gaussian) distribution for x,y,theta.
	normal_distribution<double> dist_x(x, std_x);
	normal_distribution<double> dist_y(y, std_y);
  normal_distribution<double> dist_theta(theta, std_theta);


	for (int i = 0; i < num_particles; i++) {

		//create an object for Particle calss
		Particle p;

		//Create random particles with normal distrubtions from GPS readings mean and std:
		p.id = i;
		p.x = dist_x(gen);
		p.y = dist_y(gen);
		p.theta = dist_theta(gen);
		// where "gen" is the random engine initialized earlier.

		//Initialize all particles with equal weight =1;
		p.weight = 1.0;

		//Create the particles vector
		particles.push_back(p);

	}

	is_initialized = true;

}

	//Emil code end-1




void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/


	//Emil code start-2

	/**
	 * prediction Predicts the state for the next time step
	 *   using the process model.
	 * @param delta_t Time between time step t and t+1 in measurements [s]
	 * @param std_pos[] Array of dimension 3 [standard deviation of x [m], standard deviation of y [m]
	 *   standard deviation of yaw [rad]]
	 * @param velocity Velocity of car from t to t+1 [m/s]
	 * @param yaw_rate Yaw rate of car from t to t+1 [rad/s]
	 */

	 //Normal distribution for sensor noise with zero mean and std of the pos
	normal_distribution<double> dist_x(0,std_pos[0]);
	normal_distribution<double> dist_y(0,std_pos[1]);
	normal_distribution<double> dist_theta(0,std_pos[2]);


	for (int i = 0 ; i < num_particles ; i++) {
        //check if the yaw_rate is not changing
        if(fabs(yaw_rate)<0.00001){

          particles[i].x += velocity * delta_t * cos(particles[i].theta);
          particles[i].y += velocity * delta_t * sin(particles[i].theta);
        }
        //yaw_rate is changing
        else {
        particles[i].x += velocity / yaw_rate * (sin(particles[i].theta + yaw_rate*delta_t) - sin(particles[i].theta));
		    particles[i].y += velocity / yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate*delta_t));
        particles[i].theta += yaw_rate * delta_t;

        }

        //Add Noise

        particles[i].x += dist_x(gen);
        particles[i].y += dist_y(gen);
        particles[i].theta += dist_theta(gen);


	}


	//Emil code end-2

}



void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
	//   implement this method and use it as a helper during the updateWeights phase.


	//Emil code start-3
	/**
	 * dataAssociation Finds which observations correspond to which landmarks (likely by using
	 *   a nearest-neighbors data association).
	 * @param predicted Vector of predicted landmark observations
	 * @param observations Vector of landmark observations
	 */


    // Iterate through each transformed observation to associate to a landmark
    for (unsigned int i = 0; i < observations.size() ; i++) {
        
        double min_dist = numeric_limits<double>::max();
        
        int closest_landmark = -1;
        
        double curr_dist;
        
        // Iterate through all landmarks to check which is closest
        for (unsigned int j = 0; j < predicted.size() ; ++j) {
            // Calculate Euclidean distance
            curr_dist = sqrt(pow(observations[i].x - predicted[j].x, 2)
                             + pow(observations[i].y - predicted[j].y, 2));
            
            // Compare to min_dist and find the predicted landmark nearest the current observed landmark
            if (curr_dist < min_dist) {
                min_dist = curr_dist;
                closest_landmark = predicted[j].id;
            }
            
        }
        // set the observation id to the nearest predicted landmark id
        observations[i].id = closest_landmark;
        
     }
  }

	//Emil code end-3



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

    //Emil code start-4
	/**
	 * updateWeights Updates the weights for each particle based on the likelihood of the
	 *   observed measurements.
	 * @param sensor_range Range [m] of sensor
	 * @param std_landmark[] Array of dimension 2 [Landmark measurement uncertainty [x [m], y [m]]]
	 * @param observations Vector of landmark observations
	 * @param map Map class containing map landmarks
	 */

   //for each particle
   for (int i = 0 ; i< num_particles ; i++){

        //Read particls vector x , y , theta
        double Particle_x = particles[i].x;
        double Particle_y = particles[i].y;
        double Particle_theta = particles[i].theta;

        // create a vector for the map landmark locations predicted and are within sensor range of the particle
        vector<LandmarkObs> predictions;


        //for each map landmark
        for (unsigned int j = 0 ; j <map_landmarks.landmark_list.size() ; j++){
            //Read landmark id and x,y coordinates
             float lm_x = map_landmarks.landmark_list[j].x_f;
             float lm_y = map_landmarks.landmark_list[j].y_f;
             int lm_id = map_landmarks.landmark_list[j].id_i;

             if (fabs(lm_x - Particle_x) <= sensor_range && fabs(lm_y - Particle_y) <= sensor_range) {
                //Add the prediction to vector
                predictions.push_back (LandmarkObs{lm_id, lm_x, lm_y });
             }

        }

       //Transform observations from vehicle coordinates to map coordinates

       vector<LandmarkObs> trans_os;

      for(unsigned int j = 0; j < observations.size(); j++) {
			double t_x = cos(Particle_theta)*observations[j].x - sin(Particle_theta)*observations[j].y + Particle_x;
			double t_y = sin(Particle_theta)*observations[j].x + cos(Particle_theta)*observations[j].y + Particle_y;
			trans_os.push_back(LandmarkObs{observations[j].id, t_x, t_y });
		}
  
   

        //Data association for the predictions and transformed observations on current particle
		dataAssociation(predictions, trans_os);
		particles[i].weight = 1.0;
		for(unsigned int j = 0; j < trans_os.size(); j++) {
			double o_x, o_y, pr_x, pr_y;
			o_x = trans_os[j].x;
			o_y = trans_os[j].y;
			int asso_prediction = trans_os[j].id;

			//x,y coordinates of the prediction associated with the current observation
			for(unsigned int k = 0; k < predictions.size(); k++) {
				if(predictions[k].id == asso_prediction) {
					pr_x = predictions[k].x;
					pr_y = predictions[k].y;
				}
			}

         //Weight for this observation with multivariate Gaussian
			double s_x = std_landmark[0];
			double s_y = std_landmark[1];
			double obs_w = ( 1/(2*M_PI*s_x*s_y)) * exp( -( pow(pr_x-o_x,2)/(2*pow(s_x, 2)) + (pow(pr_y-o_y,2)/(2*pow(s_y, 2))) ) );
         //Product of this obersvation weight with total observations weight
			particles[i].weight *= obs_w;
		}
	}


    }


	 //Emil code end-4





void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight.
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution


//Emil code start-5
	/**
	 * resample Resamples from the updated set of particles to form
	 *   the new set of particles.
	 */

	//create resampled particles vector
	vector<Particle> new_particles;
  
  //get the current weights
  vector<double> weights;
  for (int i = 0; i < num_particles; i++) {
    weights.push_back(particles[i].weight);
  }
  
  //generate random starting index for resampling wheel
  uniform_int_distribution<int> distInt(0, num_particles - 1);
  int index = distInt(gen);
  
  //get max weight
  double max_weight = *max_element(weights.begin(), weights.end());
  
  //uniform random distribution [0.0, max_weight)
  uniform_real_distribution<double> unirealdist(0.0, max_weight);
  
  double beta = 0.0;
  
  //resample wheel
  for (int i = 0; i < num_particles; i++) {
    beta += unirealdist(gen) * 2.0;
    while (beta > weights[index]) {
      beta -= weights[index];
      index = (index + 1) % num_particles;
    }
    new_particles.push_back(particles[index]);
  }
  
  particles = new_particles; 


	 //Emil code end-5

}

void ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations,
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
