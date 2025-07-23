#include "weighted_ik.h"

int main() {
    // Create solver once with robot-specific parameters
    std::array<double, 7> neutral_pose = {0.0, 0.0, 0.0, -1.5, 0.0, 1.86, 0.0};
    
    WeightedIKSolver solver(neutral_pose, 1.0, 0.5, 2.0, true);
    
    // Multiple target poses to demonstrate efficiency
    std::vector<std::array<double, 3>> targets = {
        {0.23189, -0.0815989, 0.607269}
    };
    
    std::array<double, 9> orientation = {
        -0.189536, 0.0420467, -0.980973,
         0.404078, -0.907217, -0.116958,
        -0.894873, -0.418557, 0.15496
    };
    
    // Simulate robot motion - current pose changes with each movement
    std::array<double, 7> current_pose = {-1.5, 0.5, 1.5, -1.5, 0.5, 0.5, 1.5};
    
    cout << "=== Weighted IK with 1D Optimization ===" << endl;
    cout << "Solving " << targets.size() << " target poses efficiently..." << endl << endl;
    
    auto total_start = std::chrono::high_resolution_clock::now();
    
    // Solve multiple targets efficiently using optimization
    for (int target_idx = 0; target_idx < targets.size(); target_idx++) {
        cout << "--- Target " << (target_idx + 1) << " ---" << endl;
        
        // Use wider search range to demonstrate optimization power
        WeightedIKResult result = solver.solve_q7_optimized(
            targets[target_idx], 
            orientation, 
            current_pose, 
            -1.0,  // Wider search range
            2.0,   // to show optimization benefits
            1e-6,  // High precision
            50     // Max iterations
        );
        
        if (result.success) {
            cout << "   Target solved successfully!" << endl;
            cout << "   Optimization converged in " << result.optimization_iterations << " iterations" << endl;
            cout << "   Optimal q7: " << std::fixed << std::setprecision(6) << result.q7_optimal << " rad" << endl;
            cout << "   Final score: " << result.score << endl;
            
            // Update current pose for next iteration (robot moves to new position)
            current_pose = result.joint_angles;
            cout << "   Robot moved to new configuration." << endl;
        } else {
            cout << "   Failed to find solution for target " << (target_idx + 1) << endl;
        }
        cout << endl;
    }
    
    auto total_end = std::chrono::high_resolution_clock::now();
    auto total_duration = std::chrono::duration_cast<std::chrono::microseconds>(total_end - total_start);
    
    cout << "=== Summary ===" << endl;
    cout << "Total time for " << targets.size() << " targets: " 
         << total_duration.count() << " μs (" 
         << total_duration.count() / 1000.0 << " ms)" << endl;
    cout << "Average time per target: " 
         << total_duration.count() / targets.size() << " μs" << endl;
    
    return 0;
}