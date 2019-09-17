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

void ParticleFilter::Init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1.
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method
   *   (and others in this file).
   */
  num_particles = 500;  // TODO: Set the number of particles
  is_initialized = false;
  particles_.resize(num_particles);
  for (unsigned int i = 0; i < num_particles; i++) {
    auto& p = particles_[i];
    p.id = i;
    const auto& sampled = GaussianSample({x, y, theta}, {std[0], std[1], std[2]});
    p.x = sampled[0];
    p.y = sampled[1];
    p.theta = sampled[2];
    p.weight = 1;
  }
  is_initialized = true;
}

void ParticleFilter::Prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */
  for (unsigned int i = 0; i < num_particles; i++) {
    auto& p = particles_[i];
    const auto& new_state = BicyclePredict({p.x, p.y, p.theta}, delta_t, velocity, yaw_rate);
    const auto& sampled = GaussianSample(new_state, {std_pos[0], std_pos[1], std_pos[2]});
    p.x = sampled[0];
    p.y = sampled[1];
    p.theta = sampled[2];
  }
}

void ParticleFilter::DataAssociate(vector<int> predicted_obs_id, const Map& map, vector<Landmark>* observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each
   *   observed measurement and assign the observed measurement to this
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will
   *   probably find it useful to implement this method and use it as a helper
   *   during the UpdateWeights phase.
   */
}

void ParticleFilter::UpdateWeights(double sensor_range, double std_landmark[], vector<Landmark>& observations,
                                   const Map& map) {
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
  double total_weight = 0;
  for (unsigned int i = 0; i < particles_.size(); i++) {
    auto& p = particles_[i];
    TransformLandmarksFromLocalToGlobal(p, &observations);
    const auto& predicted_obs_id = QueryMapLandmarks(p, sensor_range, map);
    DataAssociate(predicted_obs_id, map, &observations);
    double weight = 1;
    for (const auto& obs : observations) {
      const auto& ground_truth = map.landmarks[obs.id];
      const auto& w = multiv_prob(std_landmark[0], std_landmark[1], obs.x, obs.y, ground_truth.x, ground_truth.y);
      weight *= w;
    }
    p.weight = weight;
    total_weight += p.weight;
  }
  for (unsigned int i = 0; i < particles_.size; i++) {
    auto& p = particles_[i];
    p.weight = p.weight / total_weight;
  }
}

void ParticleFilter::Resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional
   *   to their weight.
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
}

void ParticleFilter::SetAssociations(Particle& particle, const vector<int>& associations, const vector<double>& sense_x,
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association,
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations = associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::GetAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length() - 1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::GetSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length() - 1);  // get rid of the trailing space
  return s;
}