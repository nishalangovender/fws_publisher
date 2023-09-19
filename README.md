# fws_publisher

Converts Twist messages to steering and wheel commands for park_bot https://github.com/nishalangovender/park_bot.

# Kinematics

## Assumptions

Steering: The steering angle (δ) is constrained within the range of -δ_max to δ_max.

Side-Slip Angle: Side-slip angle (β) is defined as β = arctan(1/2) * (tan(δ_A) + tan(δ_B)).

Vehicle Velocity: Vehicle velocity (V) is calculated as V = (v_A * cos(δ_A) + v_B * cos(δ_B)) / (2 * cos(β)).

Angular Velocity: Angular velocity (ψ_dot) is determined by ψ_dot = V * cos(β) * (tan(δ_A) + tan(δ_B)) / L.

Heading Angle: The heading angle (θ) is given by θ = ψ + β.

## Manouevre 1 (Zero Side-Slip)

Assumptions
For this maneuver, we assume:

Side-Slip: β = 0.
Steering Angles: δ = δ_A = δ_B.
Wheel Velocities: V = v_A = v_B.

Final Equations
The following equations describe the vehicle's behavior during this maneuver:

Steering Angle: δ = arctan(1/2) * (ψ_dot * L) / x_dot.
Wheel Velocity: V = x_dot / cos(δ_A).

## Manouevre 2 (Parallel Steering)

Assumptions
For this maneuver, we assume:

Steering Angles: β = δ_A = δ_B.
Wheel Velocities: V = v_A = v_B.

Final Equations
During this maneuver, the following equations govern the vehicle's dynamics:

Relationship: x_dot = (ψ_dot * L) / tan(β).
Wheel Velocity: V = (ψ_dot * L) / (2 * sin(β)).