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
  num_particles = 100;  // TODO: Set the number of particles

  // Random engine for normal distribution initialization
  std::default_random_engine gen;

  // Create normal distributions for x, y and theta
  std::normal_distribution<double> dist_x(x, std[0]);
  std::normal_distribution<double> dist_y(y, std[1]);
  std::normal_distribution<double> dist_theta(theta, std[2]);

  // Initialize all particles to first position (based on estimates of x, y, theta 
  // and their uncertainties from GPS) and all weights to 1.
  for (int i = 0; i < num_particles; i++) {
    Particle particle;
    particle.id = i;
    particle.x = dist_x(gen);
    particle.y = dist_y(gen);
    particle.theta = dist_theta(gen);
    particle.weight = 1;

    particles.push_back(particle);
    weights.push_back(1);
  }

  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */

  /**
   * See Lesson 5 Session 21 (Explanation of Project)
   * 
   * Using the measurements passed in as arguments, update each particle's 
   * position estimates and account for sensor noise by adding Gaussian noise. 
   * Add Gaussian noise by sampling from a Gaussian distribution with mean 
   * equal to the updated particle position and standard deviation equal to the
   * standard deviation of the measurements. 
   *
   * Equations in Lesson 5 Session 9 (Calculate Prediction Step)
   *
   */
  std::default_random_engine gen;

  for (int i = 0; i < num_particles; i++) {
    double predicted_x;
    double predicted_y;
    double predicted_theta;

    // Lesson 3 Session 4
    if (yaw_rate == 0) {
      predicted_x = particles[i].x + velocity * delta_t * cos(particles[i].theta);
      predicted_y = particles[i].y + velocity * delta_t * sin(particles[i].theta);
      predicted_theta = particles[i].theta;
    } else {
      predicted_x = particles[i].x + velocity / yaw_rate * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
      predicted_y = particles[i].y + velocity / yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t));
      predicted_theta = particles[i].theta + yaw_rate * delta_t;
    }

    // Create normal distributions for the new values
    std::normal_distribution<double> dist_x(predicted_x, std_pos[0]);
    std::normal_distribution<double> dist_y(predicted_y, std_pos[1]);
    std::normal_distribution<double> dist_theta(predicted_theta, std_pos[2]);

    particles[i].x = dist_x(gen);
    particles[i].y = dist_y(gen);
    particles[i].theta = dist_theta(gen);
  }
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
  


  // TODO delete
  // See https://knowledge.udacity.com/questions/689706
  //
  // Taking Lesson 5 Section 16 as reference, "predicted" vector contains the 
  // list of landmarks within a single particle sensors range. This function 
  // gets called for each particle. 
  // The "observations" vector contains the car observations in the car coordinate system. 
  // Car observations are its sensor measurements, taken within sensor range. 
  // 
  //
  // The function should update the Observations vector elements with the
  // transformed observations in map coordinates (associated with the landmarks).

  // Lesson 5 Session 21 - Explanation of Project Code
  // The "predicted" vector contains the prediction measurements between one
  // particular particle and all of the map landmarks within sensor range (so
  // the function needs to be called for each particle). 
  // The "observations"vector is the actual landmark measurements gathered from
  // the LIDAR.
  // This function will perform nearest neighbor data association and assign
  // each sensor observation the map landmark ID associated with it. 


  //DELETE
  // Print the content of observations and predictions to inspect them.
  //std::cout << "\nVector of observations has these objects: ";
  //std::cout << observations.size() << std::endl;

  //std::cout << "Vector of predictions has these objects: ";
  //std::cout << predicted.size() << std::endl;
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
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


  // Goal: Update each particle's weight based on Multivariate Gaussian distribution (Lesson 5 Session 20)
  // 
  // Steps:
  // 1. Loop through the particles. 
  // 2. For each particle (remember that a poarticle is a possible position of the car in the map) save a list
  // of possible landmarks that could be reached within sensor range if the car were in that particle's position. 
  // 3. Transform each car observation received as argument in map coordinates (Transformations: Lesson 5 Session 17).
  // 4. Loop through the observations and associate its ID  to the ID of the nearest landmark from the list 
  // (vector) saved at point 2.
  // 5. Compute the Multivariate-Gaussian probability density for each observation, taking as x, y those of the 
  // observation and as mean those of the corresponding landmark (they have corresponding IDs)
  // 6. The particle's final weight is the product of the single weights. 



  // Lesson 5 Session 20
  for (int i = 0; i < num_particles; i++) {
    
    // Step 1: Predict measurements to all the map landmarks within sensor range for each particle. 
    // 
    // Explanation: If you inspect the "map_data.txt" file, you can see 42 landmarks in it with
    // their positions in meter. Argument "map_landmarks" is an object of type Map that contains 
    // a vector of landmarks (all 42 of them were read). 
    // Insert in a vector of "predictions" the landmarks that are within sensor range for this particle.
    // 
    // Vector of landmarks locations predicted to be within "sensor range" of the particle. 
    vector<LandmarkObs> predictions; 

    // https://knowledge.udacity.com/questions/690641 and https://knowledge.udacity.com/questions/181044
    for (int l = 0; l < map_landmarks.landmark_list.size(); l++) {
      // Get single_landmark structure members and assign them to a new variable for convenience
      float landmark_x = map_landmarks.landmark_list[l].x_f;
      float landmark_y = map_landmarks.landmark_list[l].y_f;
      int landmark_id = map_landmarks.landmark_list[l].id_i;

      // Insert landmarks that are within sensor range of a particle in the particle predictions vector
      if (abs(landmark_x - particles[i].x) <= sensor_range && abs(landmark_y - particles[i].y <= sensor_range)) {
        predictions.push_back(LandmarkObs{ landmark_id, landmark_x, landmark_y });
      }
    }

    // Create a copy of observations to be passed to the "dataAssociation" function. 
    vector<LandmarkObs> sensor_observations = observations;
    dataAssociation(predictions, sensor_observations);    

    // TODO
    // Find the nearest landmark for each observation
    double mu_x = 0;
    double mu_y = 0;

    // TODO
    // Convert the observations from the vehicle coordinate's system to the map coordinate's system

    double x_obs = observations[i].x;
    double y_obs = observations[i].y;
    particles[i].weight = multiv_prob(std_landmark[0], std_landmark[1], x_obs, y_obs, mu_x, mu_y);
  }
}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */

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

string ParticleFilter::getSenseCoord(Particle best, string coord) {
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