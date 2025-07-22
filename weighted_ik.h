#ifndef WEIGHTED_IK_H
#define WEIGHTED_IK_H

#include <array>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <limits>
#include "Eigen/Dense"
#include "geofik.h"

using namespace std;
using namespace std::chrono;

// Structure to hold the result of weighted IK optimization
struct WeightedIKResult {
    bool success;                           // Whether a valid solution was found
    std::array<double, 7> joint_angles;     // Optimal joint angles in radians
    double q7_optimal;                      // Optimal q7 value
    double score;                           // Final optimization score
    double manipulability;                  // Manipulability measure
    double neutral_distance;                // Distance from neutral pose
    double current_distance;                // Distance from current pose
    int solution_index;                     // Which solution index was selected
    std::array<std::array<double, 6>, 7> jacobian; // Jacobian matrix
    
    // Statistics
    int total_solutions_found;              // Total solutions across all q7 values
    int valid_solutions_count;              // Number of valid solutions
    int q7_values_tested;                   // Number of q7 values tested
    long duration_microseconds;             // Computation time in microseconds
};

// Main function for weighted IK optimization
WeightedIKResult weighted_ik_q7(
    const std::array<double, 3>& target_position,
    const std::array<double, 9>& target_orientation,
    const std::array<double, 7>& neutral_pose,
    const std::array<double, 7>& current_pose,
    double q7_start,
    double q7_end,
    double step_size,
    double weight_manip,
    double weight_neutral,
    double weight_current,
    bool verbose = true
);

// Helper function to calculate manipulability from Jacobian
double calculate_manipulability_weighted(const std::array<std::array<double, 6>, 7>& J);

// Function to print detailed results
void print_weighted_ik_results(const WeightedIKResult& result);

#endif // WEIGHTED_IK_H