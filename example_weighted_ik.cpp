#include "weighted_ik.h"

int main() {
    // Target pose
    std::array<double, 3> target_position = {0.23189, -0.0815989, 0.607269};
    std::array<double, 9> target_orientation = {
        -0.189536, 0.0420467, -0.980973,
         0.404078, -0.907217, -0.116958,
        -0.894873, -0.418557, 0.15496
    };
    
    // Reference poses
    std::array<double, 7> neutral_pose = {0.0, 0.0, 0.0, -1.5, 0.0, 1.86, 0.0};
    std::array<double, 7> current_pose = {-1.5, 0.5, 1.5, -1.5, 0.5, 0.5, 1.5};
    
    // Call weighted IK optimization
    WeightedIKResult result = weighted_ik_q7(
        target_position,
        target_orientation,
        neutral_pose,
        current_pose,
        0.3,    // q7_start
        0.5,    // q7_end
        0.01,   // step_size
        1.0,    // weight_manip
        0.5,    // weight_neutral
        2.0,    // weight_current
        false    // verbose
    );
    
    if (result.success) {
        cout << "Success! Optimal joint angles found." << endl;
        // Use result.joint_angles for your application
    } else {
        cout << "Failed to find valid solution." << endl;
    }
    
    return 0;
}