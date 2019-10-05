/**
 * helper_functions.h
 * Some helper functions for the 2D particle filter.
 *
 * Created on: Dec 13, 2016
 * Author: Tiffany Huang
 */

#ifndef HELPER_FUNCTIONS_H_
#define HELPER_FUNCTIONS_H_

#include <math.h>
#include <cassert>
#include <fstream>
#include <iostream>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

// for portability of M_PI (Vis Studio, MinGW, etc.)
#ifndef M_PI
const double M_PI = 3.14159265358979323846;
#endif

/**
 * Struct representing one position/control measurement.
 */
struct ControlS {
  double velocity;  // Velocity [m/s]
  double yawrate;   // Yaw rate [rad/s]
};

/**
 * Struct representing one ground truth position.
 */
struct GroundTruth {
  double x;      // Global vehicle x position [m]
  double y;      // Global vehicle y position
  double theta;  // Global vehicle yaw [rad]
};

/**
 * Struct representing one landmark observation measurement.
 */
struct Landmark {
  int id;    // Id of matching landmark in the map.
  double x;  // Local (vehicle coords) x position of landmark observation [m]
  double y;  // Local (vehicle coords) y position of landmark observation [m]
};

struct Map {
  std::unordered_map<int, Landmark> landmarks;
};

struct Particle {
  int id;
  double x;
  double y;
  double theta;
  double weight;
  std::vector<int> associations;
  std::vector<double> sense_x;
  std::vector<double> sense_y;
};

/**
 * Computes the error between ground truth and particle filter data.
 * @param (gt_x, gt_y, gt_theta) x, y and theta of ground truth
 * @param (pf_x, pf_y, pf_theta) x, y and theta of particle filter
 * @output Error between ground truth and particle filter data.
 */
inline double* GetError(double gt_x, double gt_y, double gt_theta, double pf_x, double pf_y, double pf_theta) {
  static double error[3];
  error[0] = fabs(pf_x - gt_x);
  error[1] = fabs(pf_y - gt_y);
  error[2] = fabs(pf_theta - gt_theta);
  error[2] = fmod(error[2], 2.0 * M_PI);
  if (error[2] > M_PI) {
    error[2] = 2.0 * M_PI - error[2];
  }
  return error;
}

/**
 * Reads map data from a file.
 * @param filename Name of file containing map data.
 * @output True if opening and reading file was successful
 */
inline bool ReadMapData(std::string filename, Map& map) {
  // Get file of map
  std::ifstream in_file_map(filename.c_str(), std::ifstream::in);
  // Return if we can't open the file
  if (!in_file_map) {
    return false;
  }

  // Declare single line of map file
  std::string line_map;

  // Run over each single line
  while (getline(in_file_map, line_map)) {
    std::istringstream iss_map(line_map);

    // Declare landmark values and ID
    float landmark_x_f, landmark_y_f;
    int id_i;

    // Read data from current line to values
    iss_map >> landmark_x_f;
    iss_map >> landmark_y_f;
    iss_map >> id_i;

    // Declare single_landmark
    Landmark single_landmark_temp;

    // Set values
    single_landmark_temp.id = id_i;
    single_landmark_temp.x = landmark_x_f;
    single_landmark_temp.y = landmark_y_f;
    std::cout << "map lm:"
              << "(" << landmark_x_f << "," << landmark_y_f << ")" << std::endl;
    map.landmarks[single_landmark_temp.id] = single_landmark_temp;
  }
  std::cout << "map lm size:" << map.landmarks.size() << std::endl;
  return true;
}

/**
 * Reads control data from a file.
 * @param filename Name of file containing control measurements.
 * @output True if opening and reading file was successful
 */
inline bool ReadControlData(std::string filename, std::vector<ControlS>& position_meas) {
  // Get file of position measurements
  std::ifstream in_file_pos(filename.c_str(), std::ifstream::in);
  // Return if we can't open the file
  if (!in_file_pos) {
    return false;
  }

  // Declare single line of position measurement file:
  std::string line_pos;

  // Run over each single line:
  while (getline(in_file_pos, line_pos)) {
    std::istringstream iss_pos(line_pos);

    // Declare position values:
    double velocity, yawrate;

    // Declare single control measurement:
    ControlS meas;

    // read data from line to values:
    iss_pos >> velocity;
    iss_pos >> yawrate;

    // Set values
    meas.velocity = velocity;
    meas.yawrate = yawrate;

    // Add to list of control measurements:
    position_meas.push_back(meas);
  }
  return true;
}

/**
 * Reads ground truth data from a file.
 * @param filename Name of file containing ground truth.
 * @output True if opening and reading file was successful
 */
inline bool ReadGroundTruthData(std::string filename, std::vector<GroundTruth>& gt) {
  // Get file of position measurements
  std::ifstream in_file_pos(filename.c_str(), std::ifstream::in);
  // Return if we can't open the file
  if (!in_file_pos) {
    return false;
  }

  // Declare single line of position measurement file
  std::string line_pos;

  // Run over each single line
  while (getline(in_file_pos, line_pos)) {
    std::istringstream iss_pos(line_pos);

    // Declare position values
    double x, y, azimuth;

    // Declare single ground truth
    GroundTruth single_gt;

    // read data from line to values
    iss_pos >> x;
    iss_pos >> y;
    iss_pos >> azimuth;

    // Set values
    single_gt.x = x;
    single_gt.y = y;
    single_gt.theta = azimuth;

    // Add to list of control measurements and ground truth
    gt.push_back(single_gt);
  }
  return true;
}

/**
 * Reads landmark observation data from a file.
 * @param filename Name of file containing landmark observation measurements.
 * @output True if opening and reading file was successful
 */
inline bool ReadLandmarkData(std::string filename, std::vector<Landmark>& observations) {
  // Get file of landmark measurements
  std::ifstream in_file_obs(filename.c_str(), std::ifstream::in);
  // Return if we can't open the file
  if (!in_file_obs) {
    return false;
  }

  // Declare single line of landmark measurement file
  std::string line_obs;

  // Run over each single line
  while (getline(in_file_obs, line_obs)) {
    std::istringstream iss_obs(line_obs);

    // Declare position values
    double local_x, local_y;

    // read data from line to values
    iss_obs >> local_x;
    iss_obs >> local_y;

    // Declare single landmark measurement
    Landmark meas;

    // Set values
    meas.x = local_x;
    meas.y = local_y;

    // Add to list of control measurements
    observations.push_back(meas);
  }
  return true;
}

static std::mt19937 G_random_engine{1024};

static double Random(double start, double end) {
  std::uniform_real_distribution<double> dist(start, end);
  return dist(G_random_engine);
}

static std::vector<double> GaussianSample(std::vector<double> mean, std::vector<double> std) {
  assert(mean.size() == std.size());
  std::vector<std::normal_distribution<double>> dists;
  for (unsigned int i = 0; i < mean.size(); i++) {
    dists.push_back(std::normal_distribution<double>(mean[i], std[i]));
  }
  std::vector<double> sampled;
  sampled.resize(dists.size());
  for (unsigned int i = 0; i < dists.size(); i++) {
    sampled[i] = dists[i](G_random_engine);
  }
  return sampled;
}
static std::vector<double> BicyclePredict(std::vector<double> state0, double delta_t, double velocity,
                                          double yaw_rate) {
  double x0 = state0[0];
  double y0 = state0[1];
  double theta0 = state0[2];

  double x1 = x0 + (velocity / yaw_rate) * (std::sin(theta0 + yaw_rate * delta_t) - std::sin(theta0));
  double y1 = y0 + (velocity / yaw_rate) * (std::cos(theta0) - std::cos(theta0 + yaw_rate * delta_t));
  double theta1 = theta0 + yaw_rate * delta_t;
  return {x1, y1, theta0};
}

static void TransformLandmarksFromLocalToGlobal(const Particle& global_pose, std::vector<Landmark>* local_obs) {
  for (auto& obs : *local_obs) {
    // std::cout << "local obs:(" << obs.x << "," << obs.y << ")" << std::endl;
    double x = global_pose.x + std::cos(global_pose.theta) * obs.x - std::sin(global_pose.theta) * obs.y;
    double y = global_pose.y + std::sin(global_pose.theta) * obs.x + std::cos(global_pose.theta) * obs.y;
    obs.x = x;
    obs.y = y;
    // std::cout << "global obs:(" << obs.x << "," << obs.y << ")" << std::endl;
  }
}

static double EuclideanDistance(const std::vector<double>& p1, const std::vector<double>& p2) {
  assert(p1.size() == p2.size());
  double result = 0.0;
  for (unsigned int i = 0; i < p1.size(); i++) {
    result += (p1[i] - p2[i]) * (p1[i] - p2[i]);
  }
  return std::sqrt(result);
}

static std::vector<int> QueryMapLandmarks(const Particle& center, double radius, const Map& map) {
  std::vector<int> result;
  std::cout << "center:(" << center.x << "," << center.y << ")"
            << " radius:" << radius << std::endl;
  for (const auto& map_lm : map.landmarks) {
    if (EuclideanDistance({map_lm.second.x, map_lm.second.y}, {center.x, center.y}) < radius) {
      // std::cout << "target map lm:" << map_lm.first << "(" << map_lm.second.x << "," << map_lm.second.y << ")"
      //           << std::endl;
      result.push_back(map_lm.first);
    }
  }
  return result;
}

static double MultivariableProb(double sig_x, double sig_y, double x_obs, double y_obs, double mu_x, double mu_y) {
  // calculate normalization term
  double gauss_norm = 1.0 / (2.0 * M_PI * sig_x * sig_y);
  // calculate exponent
  double exponent =
      (pow(x_obs - mu_x, 2.0) / (2.0 * pow(sig_x, 2.0))) + (pow(y_obs - mu_y, 2.0) / (2.0 * pow(sig_y, 2.0)));
  // calculate weight using normalization terms and exponent
  double weight = gauss_norm * exp(-exponent);

  return weight;
}

#endif  // HELPER_FUNCTIONS_H_
