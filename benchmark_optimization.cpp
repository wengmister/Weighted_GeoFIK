#include "weighted_ik.h"

void run_benchmark(const std::string& test_name, double q7_min, double q7_max, double step_size) {
    // Test parameters
    std::array<double, 7> neutral_pose = {0.0, 0.0, 0.0, -1.5, 0.0, 1.86, 0.0};
    std::array<double, 7> current_pose = {-1.5, 0.5, 1.5, -1.5, 0.5, 0.5, 1.5};
    
    std::array<double, 3> target = {0.23189, -0.0815989, 0.607269};
    std::array<double, 9> orientation = {
        -0.189536, 0.0420467, -0.980973,
         0.404078, -0.907217, -0.116958,
        -0.894873, -0.418557, 0.15496
    };
    
    WeightedIKSolver solver(neutral_pose, 1.0, 0.5, 2.0, false); // verbose = false for benchmarking
    
    cout << "=== " << test_name << " ===" << endl;
    cout << "Range: [" << q7_min << ", " << q7_max << "] rad, Grid step: " << step_size << endl;
    int expected_grid_points = (int)((q7_max - q7_min) / step_size) + 1;
    cout << "Expected grid points: " << expected_grid_points << endl << endl;
    
    // Grid search
    auto start1 = std::chrono::high_resolution_clock::now();
    WeightedIKResult result_grid = solver.solve_q7(target, orientation, current_pose, q7_min, q7_max, step_size);
    auto end1 = std::chrono::high_resolution_clock::now();
    auto duration1 = std::chrono::duration_cast<std::chrono::microseconds>(end1 - start1);
    
    // Optimization
    auto start2 = std::chrono::high_resolution_clock::now();
    WeightedIKResult result_opt = solver.solve_q7_optimized(target, orientation, current_pose, q7_min, q7_max);
    auto end2 = std::chrono::high_resolution_clock::now();
    auto duration2 = std::chrono::duration_cast<std::chrono::microseconds>(end2 - start2);
    
    // Results
    double speedup = (double)duration1.count() / duration2.count();
    double efficiency = (double)result_grid.q7_values_tested / result_opt.optimization_iterations;
    double score_improvement = result_opt.score - result_grid.score;
    
    cout << "Grid Search:    " << std::setw(6) << duration1.count() << " μs, " 
         << std::setw(3) << result_grid.q7_values_tested << " evals, score: " 
         << std::fixed << std::setprecision(6) << result_grid.score << endl;
    
    cout << "Optimization:   " << std::setw(6) << duration2.count() << " μs, "
         << std::setw(3) << result_opt.optimization_iterations << " evals, score: "
         << std::fixed << std::setprecision(6) << result_opt.score << endl;
    
    cout << "Speedup: " << std::fixed << std::setprecision(2) << speedup << "x, "
         << "Efficiency: " << std::setprecision(1) << efficiency << "x, "
         << "Score improvement: " << std::scientific << std::setprecision(2) << score_improvement << endl;
    
    if (result_opt.success && result_grid.success) {
        cout << "Both methods successful";
        if (score_improvement > 1e-6) cout << " (optimization better)";
        else if (score_improvement < -1e-6) cout << " (grid better)";  
        else cout << " (equivalent)";
    } else {
        cout << "Success mismatch: Grid=" << result_grid.success << ", Opt=" << result_opt.success;
    }
    cout << endl << endl;
}

int main() {
    cout << "=== COMPREHENSIVE OPTIMIZATION BENCHMARK ===" << endl << endl;
    
    // Test 1: Small fine-grained search (like current usage)
    run_benchmark("Small Fine Search", 0.3, 0.5, 0.001);
    
    // Test 2: Medium range with coarse grid
    run_benchmark("Medium Coarse Search", 0.0, 1.0, 0.01);
    
    // Test 3: Large range with fine grid (where optimization should excel)
    run_benchmark("Large Fine Search", -1.0, 2.0, 0.001);
    
    // Test 4: Very large range (full joint limits)
    run_benchmark("Full Range Search", -2.8, 2.8, 0.005);
    
    // Test 5: Narrow range with very fine grid
    run_benchmark("Ultra-Fine Search", 0.4, 0.6, 0.0001);
    
    cout << "=== SUMMARY ===" << endl;
    cout << "The optimization method should show:" << endl;
    cout << "- Higher speedup for larger search ranges" << endl;
    cout << "- Better or equivalent solutions" << endl;
    cout << "- Consistent low iteration count (~10-30)" << endl;
    cout << "- Higher precision than grid step size" << endl;
    
    return 0;
}