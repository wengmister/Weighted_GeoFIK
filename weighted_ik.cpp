#include "weighted_ik.h"

double calculate_manipulability_weighted(const std::array<std::array<double, 6>, 7>& J) {
    // Convert array to Eigen matrix
    Eigen::MatrixXd jacobian(6, 7);
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 7; j++) {
            jacobian(i, j) = J[j][i]; // Note: transposed because J[joint][dof]
        }
    }
    
    // Calculate manipulability as sqrt(det(J * J^T))
    Eigen::MatrixXd JJT = jacobian * jacobian.transpose();
    double det = JJT.determinant();
    
    // Return sqrt of determinant (manipulability measure)
    return (det >= 0) ? sqrt(det) : 0.0;
}

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
    bool verbose
) {
    WeightedIKResult result;
    result.success = false;
    result.score = -std::numeric_limits<double>::infinity();
    result.total_solutions_found = 0;
    result.valid_solutions_count = 0;
    result.q7_values_tested = (int)((q7_end - q7_start) / step_size) + 1;
    
    // Variables for IK solving
    unsigned int nsols = 0;
    bool joint_angles = true;
    std::array<std::array<double, 7>, 8> qsols;
    std::array<std::array<std::array<double, 6>, 7>, 8> Jsols;
    
    if (verbose) {
        cout << endl << "=======================================================" << endl;
        cout << "Weighted IK Q7 Optimization" << endl;
        cout << "=======================================================" << endl;
        cout << "Target position: [" << target_position[0] << ", " << target_position[1] << ", " << target_position[2] << "]" << endl;
        cout << "Q7 range: " << q7_start << " to " << q7_end << " rad (step: " << step_size << ")" << endl;
        cout << "Weights - Manipulability: " << weight_manip << ", Neutral: " << weight_neutral << ", Current: " << weight_current << endl;
        cout << endl;
    }
    
    auto start = high_resolution_clock::now();
    
    // Sweep through q7 values
    for (double q7_sweep = q7_start; q7_sweep <= q7_end; q7_sweep += step_size) {
        nsols = franka_J_ik_q7(target_position, target_orientation, q7_sweep, Jsols, qsols, joint_angles);
        result.total_solutions_found += nsols;
        
        // Check each solution for this q7 value
        for (int i = 0; i < nsols; i++) {
            // Check if solution is valid (all joints within limits)
            bool valid_solution = true;
            for (int j = 0; j < 7; j++) {
                if (isnan(qsols[i][j])) {
                    valid_solution = false;
                    break;
                }
            }
            
            if (valid_solution) {
                result.valid_solutions_count++;
                
                // Calculate manipulability
                double manipulability = calculate_manipulability_weighted(Jsols[i]);
                
                // Calculate distance from neutral pose
                double neutral_distance = 0.0;
                for (int j = 0; j < 7; j++) {
                    double diff = qsols[i][j] - neutral_pose[j];
                    neutral_distance += diff * diff;
                }
                neutral_distance = sqrt(neutral_distance);
                
                // Calculate distance from current pose
                double current_distance = 0.0;
                for (int j = 0; j < 7; j++) {
                    double diff = qsols[i][j] - current_pose[j];
                    current_distance += diff * diff;
                }
                current_distance = sqrt(current_distance);
                
                // Calculate weighted score (higher is better)
                // Normalize distances by dividing by typical joint range (~6.28 rad for most joints)
                double normalized_neutral_dist = neutral_distance / (7 * 6.28);
                double normalized_current_dist = current_distance / (7 * 6.28);
                
                double score = weight_manip * manipulability 
                             - weight_neutral * normalized_neutral_dist 
                             - weight_current * normalized_current_dist;
                
                // Update best solution if this one is better
                if (score > result.score) {
                    result.success = true;
                    result.score = score;
                    result.manipulability = manipulability;
                    result.neutral_distance = neutral_distance;
                    result.current_distance = current_distance;
                    result.q7_optimal = q7_sweep;
                    result.joint_angles = qsols[i];
                    result.jacobian = Jsols[i];
                    result.solution_index = i;
                }
            }
        }
    }
    
    auto end = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(end - start);
    result.duration_microseconds = duration.count();
    
    if (verbose) {
        print_weighted_ik_results(result);
    }
    
    return result;
}

void print_weighted_ik_results(const WeightedIKResult& result) {
    cout << "Optimization completed!" << endl;
    cout << "Q7 values tested: " << result.q7_values_tested << endl;
    cout << "Total solutions found: " << result.total_solutions_found << endl;
    cout << "Valid solutions: " << result.valid_solutions_count << endl;
    cout << "Duration: " << result.duration_microseconds << " microseconds (" 
         << result.duration_microseconds / 1000.0 << " milliseconds)" << endl;
    cout << endl;
    
    if (result.success) {
        cout << "OPTIMAL SOLUTION FOUND:" << endl;
        cout << "Optimal q7: " << result.q7_optimal << " rad" << endl;
        cout << "Overall score: " << std::setprecision(8) << result.score << endl;
        cout << "Solution index: " << result.solution_index + 1 << endl;
        cout << endl;
        
        cout << "Solution metrics:" << endl;
        cout << "  Manipulability: " << std::setprecision(6) << result.manipulability << endl;
        cout << "  Distance from neutral: " << std::setprecision(6) << result.neutral_distance << " rad" << endl;
        cout << "  Distance from current: " << std::setprecision(6) << result.current_distance << " rad" << endl;
        cout << endl;
        
        cout << "Joint angles (radians):" << endl;
        for (int j = 0; j < 7; j++) {
            cout << "q_" << j + 1 << " = " << std::setprecision(6) << result.joint_angles[j] << endl;
        }
        cout << endl;
        
        cout << "Joint angles (degrees):" << endl;
        for (int j = 0; j < 7; j++) {
            cout << "q_" << j + 1 << " = " << std::setprecision(6) << result.joint_angles[j] * 180 / PI << endl;
        }
        cout << endl;
        
        // Forward kinematics verification
        Eigen::Matrix4d T_best = franka_fk(result.joint_angles);
        cout << "Forward kinematics verification:" << endl;
        cout << T_best << endl;
        
    } else {
        cout << "No valid solutions found in the specified q7 range!" << endl;
    }
}