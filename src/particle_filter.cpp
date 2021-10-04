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
#include <limits>

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

  // Lesson 5 Session 21 - Explanation of Project Code
  // The "predicted" vector contains the prediction measurements between one
  // particular particle and all of the map landmarks within sensor range (so
  // the function needs to be called for each particle). 
  // The "observations"vector is the actual landmark measurements gathered from
  // the LIDAR.
  // This function will perform nearest neighbor data association and assign
  // each sensor observation the map landmark ID associated with it. 

  double x_obs;
  double y_obs;

  for (int i = 0; i < observations.size(); i++) {
    x_obs = observations[i].x;
    y_obs = observations[i].y;

    // Store the id of the particle with the lower distance as the loop goes 
    // over the predicted landmarks. 
    int nearest_id;
    // Store the temporary distance at each loop iteration.
    double temp;
    double smallest = std::numeric_limits<double>::max();

    for (int j = 0; j < predicted.size(); j++) {
      temp = abs(dist(predicted[j].x, predicted[j].y, x_obs, y_obs));
      if (temp < smallest) {
        smallest = temp;
        nearest_id = predicted[j].id;
      }
    }
    
    // Assign the nearest landmark id to this measurement
    observations[i].id = nearest_id;
  }
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
  // 2. For each particle (remember that a particle is a possible position of the car in the map) save a list
  //    of possible landmarks that could be reached within sensor range if the car were in that particle's position. 
  //    Create a vector to store that list and call it "predictions".  
  // 3. Transform in map coordinates each car observation that was received as argument (Transformations: Lesson 5 Session 17).
  //    To execute this step, take the elements of the "observations" vector and apply a homogeneous transformation. 
  // 4. Loop through the observations and associate its IDs to the ID of the nearest landmark from the list (vector) 
  //    saved at point 2.
  // 5. Compute the Multivariate-Gaussian probability density for each transformed observation, taking as x, y those of the 
  //    observation and as mean those of the corresponding landmark (they have corresponding IDs).
  // 6. The particle's final weight is the product of the single weights computed at the step before for all observations. 


  // EXPLANATION: If you inspect the "map_data.txt" file, you can see 42 landmarks in it with their positions in meter. 
  // Argument "map_landmarks" is an object of type Map that contains a vector of those 42 landmarks, each with an ID, 
  // x position and y position. Those are the ground truth reference values. 
  //
  // As the simulator runs, the car moves through the map. The simulator makes noisy observations of the environment at 
  // time intervals of dt=0.1 sec and sends those noisy observations to the application. Those measurements are made 
  // with the car sensors, so at each time step the application only receives those that are in range of the car sensors. 
  // This means that at each time step the number of observations can change and that it is less than the total number 
  // of landmarks, as the car sensor cannot cover the entire map and there can be landmarks far away from sensor reach. 
  // We also have to predict the particle observations, given the sensor range. Thus for each particle, only the landmarks 
  // within the particle position plus the sensor range would be observable. 
  // These observable landmarks can be thought of as predictions for the particle and inserted into a predictions vector.
  //
  // Each time the application receives a message with new measurements (dt=0.1 sec), it runs this function "update_weights" 
  // that has the vector of new observations from the car, taken in car coordinates. 
  // This function's purpose is to update the weight of each particle, meaning the weight of each possible car position in 
  // the map. For each particle, the weight is the product of the weights of each observation. 
  //
  // As the observations are in car coordinates system, we need to transform them in map coordinates through a homogeneous
  // transformation matrix. 
  // Observations in the car coordinate system can be transformed into map coordinates (x_map and y_map) by passing car
  // observation coordinates (x_observation and y_observation), map particle coordinates (x_particle and y_particle) and 
  // the particle rotation angle (theta) to the homogeneous transformation matrix. 
  //
  // So for each particle we have to perform a homogeneous transformation for all the observations received. 
  // 
  // When all the observations have been transformed into the map's coordinate system, the next step is to associate each 
  // transformed observation with a landmark identifier. 
  // How do we do this? We use a nearest neighbour function to calculate the nearest landmark for each transformed 
  // observation. The nearest landmark is taken from the "predictions" vector. As we associate them, each transformed observation 
  // takes the nearest landmark ID. 
  //
  // That ID will help us solve the last step: compute the weight for each transformed observation, solving a multivariate 
  // Gaussian distribution evaluated at each x and y observation coordinate, taking as x_mean and y_mean the coordinates
  // of the landmark with matching IDs.
  // The final particle weight is the multiplication of these values.

  // Store the sum of all the weights. We will need this value to normalize weights before resampling.
  double weights_sum = 0;

  // Step 1
  // Lesson 5 Session 20
  for (int i = 0; i < num_particles; i++) {
    
    // Step 2
    // Predict measurements to all the map landmarks within sensor range for each particle. 

    // Vector of landmarks locations predicted to be within "sensor range" of the particle. 
    vector<LandmarkObs> predictions; 
    // Vector of observation weights. All these values will make for the particle's final weight. 
    vector<double> observations_weights; 

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

    // Step 3
    // Transform each car observation received as argument in map coordinates.

    // Create a vector of transformed observations.
    vector<LandmarkObs> transformed_observations;
    // Define coordinates and theta
    double x_map, y_map, x_obs, y_obs;
    double x_part = particles[i].x;
    double y_part = particles[i].y;
    double theta = particles[i].theta;

    for (int j = 0; j < observations.size(); j++) {
      x_obs = observations[j].x;
      y_obs = observations[j].y;
      // Transform to map x coordinate.
      x_map = x_part + (cos(theta) * x_obs) - (sin(theta) * y_obs);
      // Transform to y map coordinate.
      y_map = y_part + (sin(theta) * x_obs) + (cos(theta) * y_obs);
      // Add to vector of transformed observations.
      transformed_observations.push_back(LandmarkObs{ j, x_map, y_map });
    }

    // Step 4
    // Associate each transformed observation with a landmark identifier from predictions.
    dataAssociation(predictions, transformed_observations);    

    // Step 5
    // Compute the Multivariate-Gaussian probability density for each transformed observation. 

    // Find the nearest landmark for each observation
    double x_tobs, y_tobs, mu_x, mu_y, observation_weight;
    double sig_x = std_landmark[0];
    double sig_y = std_landmark[1];

    for (int m = 0; m < transformed_observations.size(); m++) {
      x_tobs = transformed_observations[m].x;
      y_tobs = transformed_observations[m].y;

      // mu_x and mu_y are the coordinates of the nearest landmark to this observation
      for (int n = 0; n < predictions.size(); n++) {
        if (predictions[n].id == transformed_observations[m].id) {
          mu_x = predictions[n].x;
          mu_y = predictions[n].y;
        } 
      }

      // calculate normalization term
      double gauss_norm = 1 / (2 * M_PI * sig_x * sig_y);

      // Compute the multivariate Gaussian probability for this observation.
      observation_weight = multiv_prob(gauss_norm, sig_x, sig_y, x_tobs, y_tobs, mu_x, mu_y);

      if (observation_weight) {
        observations_weights.push_back(observation_weight);
      }
    }

    // Step 6
    // Final weight
    double final_weight = 1;

    // Loop through the observations vector until it is empty
    while (!observations_weights.empty()) {
      // Multiply the last element
      final_weight*=observations_weights.back();
      // Remove the last element
      observations_weights.pop_back();
    }

    // Update the particle's weight.
    particles[i].weight = final_weight;
    // Update the weight value in the vector of weights. 
    weights[i] = final_weight;
    // Update the weights sum
    weights_sum += final_weight;
  }
  
  // Normalize the particles weights in the weights vector.
  for (int k = 0; k < weights.size(); k++) {
    weights[k] = weights[k] / weights_sum;    
  }  
}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */

  // Set up a random generator
  std::random_device rd;
  std::mt19937 generator(rd());

  // Create a discrete distribution with those weights
  std::discrete_distribution<> distribution(weights.begin(), weights.end());
  // Create a vector to host the replaced particles
  vector<Particle> replaced_particles;

  for (int n = 0; n < num_particles; n++) {
    // Get a new particle from the distribution
    Particle replaced_particle = particles[distribution(generator)];
    replaced_particles.push_back(replaced_particle);
  }
  // Replace the old particles vector with the new one
  particles = replaced_particles;
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