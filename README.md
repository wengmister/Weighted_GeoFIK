# Weighted GeoFIK

Adding secondary objectives to original GeoFIK implementation.
- Manipulability
- Distance from Neutral Pose
- Distance from Current Pose

~~Discretizing sweep over q7 to generate optimized solution.~~
Now uses Brent's method to optimize over the 1D cost function constructed by the IK results, massively sped up solution optimization.