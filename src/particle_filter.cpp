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
  num_particles_ = 50;  // TODO: Set the number of particles
  is_initialized_ = false;
  particles_.resize(num_particles_);
  for (unsigned int i = 0; i < num_particles_; i++) {
    auto& p = particles_[i];
    p.id = i;
    const auto& sampled = GaussianSample({x, y, theta}, {std[0], std[1], std[2]});
    p.x = sampled[0];
    p.y = sampled[1];
    p.theta = sampled[2];
    p.weight = 1;
  }
  is_initialized_ = true;
}

void ParticleFilter::Prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */
  for (unsigned int i = 0; i < num_particles_; i++) {
    auto& p = particles_[i];
    const auto& new_state = BicyclePredict({p.x, p.y, p.theta}, delta_t, velocity, yaw_rate);
    const auto& sampled = GaussianSample(new_state, {std_pos[0] * 0.8, std_pos[1] * 0.8, std_pos[2] * 0.8});
    p.x = sampled[0];
    p.y = sampled[1];
    p.theta = sampled[2];
  }
}

void ParticleFilter::DataAssociate(const std::vector<int>& predicted_obs_id, Map& map,
                                   std::vector<Landmark>* observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each
   *   observed measurement and assign the observed measurement to this
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will
   *   probably find it useful to implement this method and use it as a helper
   *   during the UpdateWeights phase.
   */
  for (auto& obs : *observations) {
    double min_dis = std::numeric_limits<double>::max();
    int min_id = -1;
    for (const auto& pred_obs_id : predicted_obs_id) {
      const auto& lm = map.landmarks[pred_obs_id];
      double dis = EuclideanDistance({lm.x, lm.y}, {obs.x, obs.y});
      if (dis < min_dis) {
        min_dis = dis;
        min_id = lm.id;
      }
    }
    obs.id = min_id;
  }
}

void ParticleFilter::UpdateWeights(double sensor_range, double std_landmark[], const vector<Landmark>& observations,
                                   Map& map) {
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
  double total_weight = 0.0;
  for (unsigned int i = 0; i < particles_.size(); i++) {
    auto& p = particles_[i];
    auto obs_copy = observations;
    TransformLandmarksFromLocalToGlobal(p, &obs_copy);
    const auto& predicted_obs_id = QueryMapLandmarks(p, sensor_range, map);
    DataAssociate(predicted_obs_id, map, &obs_copy);
    double weight = 1.0;
    std::vector<int> p_associations;
    std::vector<double> obs_x;
    std::vector<double> obs_y;
    for (const auto& obs : obs_copy) {
      const auto& map_lm = map.landmarks[obs.id];
      const auto& w = MultivariableProb(std_landmark[0], std_landmark[1], obs.x, obs.y, map_lm.x, map_lm.y);
      weight *= w;
      p_associations.push_back(obs.id);
      obs_x.push_back(obs.x);
      obs_y.push_back(obs.y);
    }
    SetAssociations(p, p_associations, obs_x, obs_y);
    p.weight = weight;
    total_weight += p.weight;
  }
  latest_max_weight_ = 0.0;
  for (unsigned int i = 0; i < particles_.size(); i++) {
    auto& p = particles_[i];
    p.weight = p.weight / (total_weight);
    if (p.weight > latest_max_weight_) {
      latest_max_weight_ = p.weight;
    }
  }
}

void ParticleFilter::Resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional
   *   to their weight.
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
  std::vector<Particle> sampled_particles;
  double r = Random(0.0, 1.0);
  int index = static_cast<int>(r * static_cast<double>(num_particles_));
  double beta = 0.0;
  for (unsigned int i = 0; i < num_particles_; i++) {
    beta += Random(0.0, 2 * latest_max_weight_);
    while (beta > particles_[index].weight) {
      beta -= particles_[index].weight;
      index = (index + 1) % num_particles_;
    }
    sampled_particles.push_back(particles_[index]);
  }
  particles_ = sampled_particles;
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