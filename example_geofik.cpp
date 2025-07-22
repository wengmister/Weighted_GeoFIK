#include <iostream>
#include <array> 
#include <vector>
#include <chrono>
#include "Eigen/Dense"
#include <iomanip>
#include <limits>
using namespace std;
using namespace std::chrono;

#include "geofik.h"

// compile with: g++ -I/usr/include/eigen3 example_geofik.cpp geofik.cpp -O3 -o example_geofik.exe

void print_results(const array<array<double, 7>, 8>& sols, const bool swivel = false, const double theta = 0.0);
void print_results_J(const array<array<array<double, 6>, 7>, 8>& Jsols, const array<array<double, 7>, 8>& qsols, const bool joint_angles, const bool swivel = false, const double theta = 0.0);

// Function to calculate manipulability from Jacobian
double calculate_manipulability(const array<array<double, 6>, 7>& J) {
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

int main() {

    unsigned int nsols = 0; 
    array<double, 9> ROE; // rotation matrix of frame E with respect to frame O
    array<double, 3> r; // position vector of frame E with respect to frame O
    double q7, q6, q4, theta; // free variables
    bool joint_angles;
    array<array<double, 7>, 8> qsols;
    array<array<array<double, 6>, 7>, 8> Jsols; 
    high_resolution_clock::time_point start = high_resolution_clock::now();
    high_resolution_clock::time_point end = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(end - start);

    // TEST franka_ik_q7()
    // cout << endl << "=======================================================" << endl;
    // cout << "franka_ik_q7()" << endl;
    // cout << "=======================================================" << endl;
    // ROE = {0.6688331000000003,0.3170534383478098,0.6724130000000006,-0.6398146000000005,-0.2150772409286401,0.7378204999999999,0.3785492999999998,-0.9236984268883508,0.05900459999999996};
    // r = {0.61674948,0.32278029,0.56790512};
    // q7 = -0.37218362471412003 ;
    // start = high_resolution_clock::now();
    // nsols = franka_ik_q7(r, ROE, q7, qsols);
    // end = high_resolution_clock::now();
    // duration = duration_cast<microseconds>(end - start);
    // print_results(qsols);
    // cout << "number of solutions found:" << nsols;
    // cout << endl << "duration (only q):" << duration.count() << endl;
    // // second run
    // start = high_resolution_clock::now();
    // nsols = franka_ik_q7(r, ROE, q7, qsols);
    // end = high_resolution_clock::now();
    // duration = duration_cast<microseconds>(end - start);
    // cout << endl << "duration in second run:" << duration.count() << endl;


    // TEST franka_ik_q6()
    /*
    cout << endl << "=======================================================" << endl;
    cout << "franka_ik_q6()" << endl;
    cout << "=======================================================" << endl;
    //test parallel case:
    ROE = {-0.6575819999999999,0.22565347159914026,0.7187951000000004,0.5218621000000001,-0.5516899774048302,0.6506136000000007,0.5433652999999999,0.8029437126709107,0.24502140000000006};
    r = {0.65543571,0.5899264,0.59988412};
    q6 = 3.141592653589793 ;
    //test non-parallel case:
    //ROE = { -0.1192774000000002,0.8774473359489797,0.4646064,0.47138319999999956,0.4618910375721001,-0.7513018999999999,-0.8738254000000001,0.12939431432541965,-0.4687071000000001 };
    //r = { 0.43749227,-0.73488144,0.4951873 };
    //q6 = 2.6179938779914944;
    start = high_resolution_clock::now();
    nsols = franka_ik_q6(r, ROE, q6, qsols);
    end = high_resolution_clock::now();
    duration = duration_cast<microseconds>(end - start);
    print_results(qsols);
    cout << "number of solutions found:" << nsols;
    cout << endl << "duration:" << duration.count() << endl;
    // second run
    start = high_resolution_clock::now();
    nsols = franka_ik_q6(r, ROE, q6, qsols);
    end = high_resolution_clock::now();
    duration = duration_cast<microseconds>(end - start);
    cout << endl << "duration in second run:" << duration.count() << endl;
    */

    // TEST franka_ik_q4()
    /*
    cout << endl << "=======================================================" << endl;
    cout << "franka_ik_q4()" << endl;
    cout << "=======================================================" << endl;
    ROE = { 0.3522750000000002,0.8334980991082203,0.4256560000000001,-0.6332361000000003,-0.12262271641620001,0.764183,0.6891402000000002,-0.5387433117066003,0.48460420000000004 };
    r = { -0.26617644,0.31984145,0.76295964 };
    q4 = -2.3401949461109055;
    start = high_resolution_clock::now();
    nsols = franka_ik_q4(r, ROE, q4, qsols);
    end = high_resolution_clock::now();
    duration = duration_cast<microseconds>(end - start);
    print_results(qsols);
    cout << "number of solutions found:" << nsols;
    cout << endl << "duration:" << duration.count() << endl;
    //second run
    start = high_resolution_clock::now();
    nsols = franka_ik_q4(r, ROE, q4, qsols);
    end = high_resolution_clock::now();
    duration = duration_cast<microseconds>(end - start);
    cout << endl << "duration in second run:" << duration.count() << endl;
    */

    // TEST franka_ik_swivel()
    /*
    cout << endl << "=======================================================" << endl;
    cout << "franka_ik_swivel()" << endl;
    cout << "=======================================================" << endl;
    ROE = { -0.5878745000000002,-0.7990239702980599,-0.12635000000000007,-0.16720689999999983,0.2728366411870001,-0.9474186000000001,0.7914831,-0.5358366439507003,-0.293996 };
    r = { -0.15116089,-0.397596,0.47386124 };
    theta = -0.3490658503988659;
    start = high_resolution_clock::now();
    nsols = franka_ik_swivel(r, ROE, theta, qsols);
    end = high_resolution_clock::now();
    print_results(qsols, true, theta);
    duration = duration_cast<microseconds>(end - start);
    cout << "Number of solutions found: " << nsols;
    cout << endl << "duration:" << duration.count() << endl;
    */

    // TEST franka_J_ik_q7()
    /*
    cout << endl << "=======================================================" << endl;
    cout << "franka_J_ik_q7()" << endl;
    cout << "=======================================================" << endl;
    // ROE = {0.6688331000000003,0.3170534383478098,0.6724130000000006,-0.6398146000000005,-0.2150772409286401,0.7378204999999999,0.3785492999999998,-0.9236984268883508,0.05900459999999996};
    // r = {0.61674948,0.32278029,0.56790512};
    // q7 = -0.37218362471412003 ;
    ROE = { -0.189536, 0.0420467, -0.980973,
             0.404078, -0.907217, -0.116958,
            -0.894873, -0.418557, 0.15496 };
    r = { 0.23189, -0.0815989, 0.607269 };
    q7 = 0.771925;
    joint_angles = true;
    start = high_resolution_clock::now();
    nsols = franka_J_ik_q7(r, ROE, q7, Jsols, qsols, joint_angles);
    end = high_resolution_clock::now();
    duration = duration_cast<microseconds>(end - start);
    print_results_J(Jsols, qsols, joint_angles);
    cout << "number of solutions found: " << nsols;
    cout << endl<<"duration in microsecond:" << duration.count() << endl;
    */

    // TEST franka_J_ik_q6()
    /*
    cout << endl << "=======================================================" << endl;
    cout << "franka_J_ik_q6()" << endl;
    cout << "=======================================================" << endl;
    //test parallel case:
    //ROE = { -0.6575819999999999,0.22565347159914026,0.7187951000000004,0.5218621000000001,-0.5516899774048302,0.6506136000000007,0.5433652999999999,0.8029437126709107,0.24502140000000006 };
    //r = { 0.65543571,0.5899264,0.59988412 };
    //q6 = 3.141592653589793;
    //test non-parallel case:
    ROE = {-0.1192774000000002,0.8774473359489797,0.4646064,0.47138319999999956,0.4618910375721001,-0.7513018999999999,-0.8738254000000001,0.12939431432541965,-0.4687071000000001};
    r = {0.43749227,-0.73488144,0.4951873};
    q6 = 2.6179938779914944 ;
    joint_angles = true;
    start = high_resolution_clock::now();
    nsols = franka_J_ik_q6(r, ROE, q6, Jsols, qsols, joint_angles, '8');
    end = high_resolution_clock::now();
    duration = duration_cast<microseconds>(end - start);
    print_results_J(Jsols, qsols, joint_angles);
    cout << "number of solutions found: " << nsols;
    cout << endl << "duration:" << duration.count() << endl;
    */

    // TEST franka_J_ik_q4()
    /*
    cout << endl << "=======================================================" << endl;
    cout << "franka_J_ik_q4()" << endl;
    cout << "=======================================================" << endl;
    ROE = { 0.3522750000000002,0.8334980991082203,0.4256560000000001,-0.6332361000000003,-0.12262271641620001,0.764183,0.6891402000000002,-0.5387433117066003,0.48460420000000004 };
    r = { -0.26617644,0.31984145,0.76295964 };
    q4 = -2.3401949461109055;
    joint_angles = true;
    start = high_resolution_clock::now();
    nsols = franka_J_ik_q4(r, ROE, q4, Jsols, qsols, joint_angles);
    end = high_resolution_clock::now();
    duration = duration_cast<microseconds>(end - start);
    print_results_J(Jsols, qsols, joint_angles);
    cout << "number of solutions found: " << nsols;
    cout << endl << "duration:" << duration.count() << endl;
    */

    // TEST franka_J_ik_swivel()
    /*
    cout << endl << "=======================================================" << endl;
    cout << "franka_J_ik_swivel()" << endl;
    cout << "=======================================================" << endl;
    ROE = { -0.5878745000000002,-0.7990239702980599,-0.12635000000000007,-0.16720689999999983,0.2728366411870001,-0.9474186000000001,0.7914831,-0.5358366439507003,-0.293996 };
    r = { -0.15116089,-0.397596,0.47386124 };
    theta = -0.3490658503988659;
    joint_angles = true;
    start = high_resolution_clock::now();
    nsols = franka_J_ik_swivel(r, ROE, theta, Jsols, qsols, joint_angles, 'F');
    end = high_resolution_clock::now();
    duration = duration_cast<microseconds>(end - start);
    print_results_J(Jsols, qsols, joint_angles);
    cout << "number of solutions found: " << nsols;
    cout << endl << "duration:" << duration.count() << endl;
    */

    // TEST franka_J_ik_q7() - Sweep through q7 values
    cout << endl << "=======================================================" << endl;
    cout << "franka_J_ik_q7() - Q7 Sweep for Best IK Solution" << endl;
    cout << "=======================================================" << endl;
    
    ROE = { -0.189536, 0.0420467, -0.980973,
             0.404078, -0.907217, -0.116958,
            -0.894873, -0.418557, 0.15496 };
    r = { 0.23189, -0.0815989, 0.607269 };
    joint_angles = true;
    
    // Reference poses
    array<double, 7> neutral_pose = {0.0, 0.0, 0.0, -1.5, 0.0, 1.86, 0.0};
    array<double, 7> current_pose = {-1.5, 0.5, 1.5, -1.5, 0.5, 0.5, 1.5};
    
    // Weights for multi-objective optimization
    double w_manipulability = 1.0;    // Higher is better
    double w_neutral_distance = 0.5;  // Lower is better
    double w_current_distance = 2.0;  // Lower is better
    
    // Variables for tracking best solution
    double best_score = -std::numeric_limits<double>::infinity();  // Changed from -1.0
    double best_manipulability = -1.0;
    double best_neutral_distance = 1e6;
    double best_current_distance = 1e6;
    double best_q7 = 0.0;
    array<double, 7> best_qsol;
    array<array<double, 6>, 7> best_Jsol;
    int best_solution_index = -1;
    
    // Sweep parameters
    double q7_start = 0.3;
    double q7_end = 1.9;
    double q7_step = 0.01;
    int total_solutions_found = 0;
    int valid_solutions_count = 0;
    
    start = high_resolution_clock::now();
    
    // Sweep through q7 values
    for (double q7_sweep = q7_start; q7_sweep <= q7_end; q7_sweep += q7_step) {
        nsols = franka_J_ik_q7(r, ROE, q7_sweep, Jsols, qsols, joint_angles);
        total_solutions_found += nsols;
        
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
                valid_solutions_count++;
                
                // Calculate manipulability
                double manipulability = calculate_manipulability(Jsols[i]);
                
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
                
                double score = w_manipulability * manipulability 
                             - w_neutral_distance * normalized_neutral_dist 
                             - w_current_distance * normalized_current_dist;
                
                // Update best solution if this one is better
                if (score > best_score) {
                    best_score = score;
                    best_manipulability = manipulability;
                    best_neutral_distance = neutral_distance;
                    best_current_distance = current_distance;
                    best_q7 = q7_sweep;
                    best_qsol = qsols[i];
                    best_Jsol = Jsols[i];
                    best_solution_index = i;
                }
            }
        }
    }
    
    end = high_resolution_clock::now();
    duration = duration_cast<microseconds>(end - start);
    
    // Print results
    cout << "Sweep completed!" << endl;
    cout << "Q7 range: " << q7_start << " to " << q7_end << " rad (step: " << q7_step << ")" << endl;
    cout << "Total q7 values tested: " << (int)((q7_end - q7_start) / q7_step) + 1 << endl;
    cout << "Total solutions found: " << total_solutions_found << endl;
    cout << "Valid solutions: " << valid_solutions_count << endl;
    cout << "Duration: " << duration.count() << " microseconds (" << duration.count() / 1000.0 << " milliseconds)" << endl;
    cout << endl;
    
    cout << "Optimization weights:" << endl;
    cout << "  Manipulability weight: " << w_manipulability << endl;
    cout << "  Neutral distance weight: " << w_neutral_distance << endl;
    cout << "  Current distance weight: " << w_current_distance << endl;
    cout << endl;
    
    if (best_score > -std::numeric_limits<double>::infinity()) {  // Changed from best_score > -1.0
        cout << "BEST SOLUTION:" << endl;
        cout << "Best q7: " << best_q7 << " rad" << endl;
        cout << "Best overall score: " << std::setprecision(8) << best_score << endl;
        cout << "Solution index: " << best_solution_index + 1 << endl;
        cout << endl;
        
        cout << "Solution metrics:" << endl;
        cout << "  Manipulability: " << std::setprecision(6) << best_manipulability << endl;
        cout << "  Distance from neutral: " << std::setprecision(6) << best_neutral_distance << " rad" << endl;
        cout << "  Distance from current: " << std::setprecision(6) << best_current_distance << " rad" << endl;
        cout << endl;
        
        cout << "Joint angles (radians):" << endl;
        for (int j = 0; j < 7; j++) {
            cout << "q_" << j + 1 << " = " << std::setprecision(6) << best_qsol[j] << endl;
        }
        cout << endl;
        
        cout << "Joint angles (degrees):" << endl;
        for (int j = 0; j < 7; j++) {
            cout << "q_" << j + 1 << " = " << std::setprecision(6) << best_qsol[j] * 180 / PI << endl;
        }
        cout << endl;
        
        cout << "Comparison with reference poses:" << endl;
        cout << "Neutral pose [0, 0, 0, -1.5, 0, 1.86, 0]:" << endl;
        for (int j = 0; j < 7; j++) {
            cout << "  Joint " << j + 1 << ": " << std::setprecision(4) 
                 << "best=" << best_qsol[j] << ", neutral=" << neutral_pose[j] 
                 << ", diff=" << (best_qsol[j] - neutral_pose[j]) << endl;
        }
        cout << endl;
        
        cout << "Current pose [-1.5, 0.5, 1.5, -1.5, 0.5, 0.5, 1.5]:" << endl;
        for (int j = 0; j < 7; j++) {
            cout << "  Joint " << j + 1 << ": " << std::setprecision(4) 
                 << "best=" << best_qsol[j] << ", current=" << current_pose[j] 
                 << ", diff=" << (best_qsol[j] - current_pose[j]) << endl;
        }
        cout << endl;
        
        // cout << "Jacobian:" << endl;
        // for (int j = 0; j < 7; j++) {
        //     for (int k = 0; k < 6; k++) {
        //         cout << std::setw(12) << std::setprecision(6) << best_Jsol[j][k];
        //     }
        //     cout << endl;
        // }
        // cout << endl;
        
        // Forward kinematics verification
        Eigen::Matrix4d T_best = franka_fk(best_qsol);
        cout << "Forward kinematics verification:" << endl;
        cout << T_best << endl;
        
    } else {
        cout << "No valid solutions found in the q7 sweep range!" << endl;
    }
    
    return 0;
}


// functions to print results

void print_results(const array<array<double, 7>, 8>& sols, const bool swivel, const double theta) {
    cout << "SOLUTIONS ...................." << endl;
    Eigen::Matrix4d T;
    double theta_res;
    bool sol_in_lims;
    for (int i = 0; i < sols.size(); i++) {
        sol_in_lims = true;
        cout << endl << "solution " << i + 1 << endl;
        for (int j = 0; j < 7; j++) {
            cout << "q_" << j + 1 << " = " << sols[i][j] * 180 / PI << endl;
            if (isnan(sols[i][j]))
                sol_in_lims = false;
        }
        // only print retrieved T if all joints are within limits:
        if (sol_in_lims) {
            T = franka_fk(sols[i]);
            cout << "Forward kinematics for this solution:" << endl;
            cout << T;
            cout << endl;
            if (swivel) {
                theta_res = franka_swivel(sols[i]);
                cout << "theta = " << theta_res << "rad or " << theta_res * 180 / PI << "degs" << endl;
                cout << "error = " << theta - theta_res << "degs" <<endl;
            }
        }
    }
}

void print_results_J(const array<array<array<double, 6>, 7>, 8>& Jsols, const array<array<double, 7>, 8>& qsols, const bool joint_angles, const bool swivel, const double theta) {
    cout << "SOLUTIONS ...................." << endl;
    double theta_res;
    std::cout << std::fixed << std::setprecision(5);
    bool sol_in_lims;
    Eigen::Matrix4d T;
    for (int i = 0; i < 8; i++) {
        cout << endl << "solution " << i + 1 << endl;
        cout << "Jacobian:" << endl;
        for (int j = 0; j < 7; j++) {
            for (int k = 0; k < 6; k++) {
                cout << std::setw(10) << Jsols[i][j][k];
            }
            cout << endl;
        }
        
        // Calculate and display manipulability
        double manipulability = calculate_manipulability(Jsols[i]);
        cout << "Manipulability: " << std::setprecision(6) << manipulability << endl;
        
        if (joint_angles) {
            sol_in_lims = true;
            cout << "joint angles" << endl;
            for (int j = 0; j < 7; j++) {
                cout << "q_" << j + 1 << " = " << qsols[i][j] * 180 / PI << endl;
                if (isnan(qsols[i][j]))
                    sol_in_lims = false;
            }
            if (sol_in_lims) {
                T = franka_fk(qsols[i]);
                cout << "Forward kinematics for this solution:" << endl;
                cout << T;
                cout << endl;
                if (swivel) {
                    theta_res = franka_swivel(qsols[i]);
                    cout << "theta = " << theta_res << "rad or " << theta_res * 180 / PI << "degs" << endl;
                    cout << "error = " << theta - theta_res << endl;
                }
            }
        }
        cout << endl;
    }
}