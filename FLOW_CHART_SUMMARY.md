# Weighted IK Summary

The `weighted_ik` module provides an intelligent inverse kinematics solver that optimizes robot configurations based on multiple criteria using efficient 1D optimization instead of exhaustive search.

## System Architecture

```mermaid
graph TD
    A[WeightedIKSolver Constructor<br/>Neutral Pose + Weights] --> B[solve_q7_optimized Method Call]
    
    B1[Target Position xyz] --> B
    B2[Target Orientation 3x3] --> B
    B3[Current Pose 7 joints] --> B
    B4[Q7 Range min max] --> B
    B5[Tolerance + Max Iterations] --> B
    
    B --> C[Initialize Brent Optimization]
    
    C --> D[Golden Section Search<br/>Find Initial Bracket]
    D --> E[Bracket Q7 Range<br/>ax bx cx points]
    
    E --> F[Brent Method Loop<br/>Parabolic + Golden Section]
    
    F --> G[Evaluate Q7 Cost Function]
    G --> H[Analytical IK Solver<br/>franka_J_ik_q7 at current q7]
    
    H --> I[Multiple Solutions<br/>Up to 8 per q7]
    
    I --> J{Valid Solution?<br/>Joint limits + NaN check}
    J -->|No| K[Discard Solution]
    J -->|Yes| L[Calculate Metrics]
    
    L --> M[Manipulability<br/>sqrt det J*JT]
    L --> N[Neutral Distance<br/>norm q - q_neutral]
    L --> O[Current Distance<br/>norm q - q_current]
    
    M --> P[Weighted Score<br/>w1*M - w2*Dn - w3*Dc]
    N --> P
    O --> P
    
    P --> Q[Return Best Score<br/>for this q7]
    
    K --> R[Return -infinity<br/>if no valid solutions]
    Q --> S{Convergence Check}
    R --> S
    
    S -->|Not Converged| T[Update Q7 Estimate<br/>Parabolic Interpolation]
    S -->|Converged| U[Final Q7 Found]
    
    T --> F
    
    U --> V[Evaluate Final Solution<br/>at optimal q7]
    V --> W[Return WeightedIKResult<br/>Joint angles + Metrics + Stats]
    
    style A fill:#e1f5fe
    style B fill:#e8f5e8
    style C fill:#fff3e0
    style W fill:#c8e6c9
    style P fill:#fce4ec
    style M fill:#f3e5f5
    style N fill:#f3e5f5
    style O fill:#f3e5f5
    style F fill:#ffecb3
```
## Grid Search Flowchart

```mermaid
graph TD
    A[Input: Target Pose<br/>Position + Orientation] --> B[Q7 Sweep Loop<br/>q7_start to q7_end]
    
    B --> C[Analytical IK Solver<br/>franka_J_ik_q7]
    C --> D[Multiple Solutions<br/>Up to 8 per q7]
    
    D --> E{Valid Solution?<br/>Joint limits + NaN check}
    E -->|No| F[Discard Solution]
    E -->|Yes| G[Calculate Metrics]
    
    G --> H[Manipulability<br/>sqrt det J*JT]
    G --> I[Neutral Distance<br/>norm q - q_neutral]
    G --> J[Current Distance<br/>norm q - q_current]
    
    H --> K[Weighted Score<br/>w1*M - w2*Dn - w3*Dc]
    I --> K
    J --> K
    
    K --> L{Best Score?}
    L -->|Yes| M[Update Best Solution]
    L -->|No| N[Continue Search]
    
    F --> O[Next Solution]
    N --> O
    M --> O
    O --> P{More q7 values?}
    
    P -->|Yes| B
    P -->|No| Q[Return Best Solution<br/>Joint angles + Metrics]
    
    style A fill:#e1f5fe
    style Q fill:#c8e6c9
    style K fill:#fff3e0
    style H fill:#f3e5f5
    style I fill:#f3e5f5
    style J fill:#f3e5f5
```

## Key Features

### Multi-Objective Optimization
- **Manipulability**: Maximizes robot dexterity (avoids singular configurations)
- **Neutral Distance**: Minimizes deviation from robot's neutral/home pose  
- **Current Distance**: Minimizes joint movement from current configuration

### Optimization-Based Solving
1. **Primary Method** (`solve_q7_optimized`): Uses Brent's method for efficient 1D optimization
2. **Fallback Method** (`solve_q7`): Grid search available for comparison/debugging

### Algorithms
- **Brent's Method**: Combines parabolic interpolation with golden section search
- **Automatic Bracketing**: Finds optimal search interval automatically
- **Convergence Control**: Configurable tolerance and iteration limits
- **Robust Handling**: Gracefully handles discontinuous cost functions

## Optimization Process

```mermaid
graph LR
    A[Initialize Search] --> B[Golden Section<br/>Find Bracket]
    B --> C[Parabolic<br/>Interpolation]
    C --> D{Good Fit?}
    D -->|Yes| E[Use Parabolic Step]
    D -->|No| F[Use Golden Section Step]
    E --> G[Evaluate Cost]
    F --> G
    G --> H{Converged?}
    H -->|No| C
    H -->|Yes| I[Return Optimal Q7]
    
    style A fill:#e3f2fd
    style I fill:#c8e6c9
    style G fill:#fff3e0
```

## Performance Advantages

```mermaid
graph LR
    A[Optimization Method] --> A1[Targeted Search<br/>~10-20 iterations]
    A --> A2[~80-160 solutions evaluated]
    A --> A3[Near-optimal solution]
    A --> A4[Fast execution ~1-5ms]
    A --> A5[Smooth convergence]
    
    B[Grid Search Comparison] --> B1[Exhaustive Search<br/>~158 q7 values]
    B --> B2[~1000+ solutions evaluated]
    B --> B3[Guaranteed global optimum]
    B --> B4[Slower execution ~10-50ms]
    B --> B5[Complete coverage]
    
    style A fill:#e8f5e8
    style B fill:#ffebee
```