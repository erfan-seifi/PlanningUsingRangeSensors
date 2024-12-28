# Planning Using Range Sensors

This project demonstrates the mathematical and algorithmic principles behind egocentric occupancy maps, motion planning, and trajectory generation using range sensor data. 

## Mathematical Foundations

### 1. **Occupancy Maps**
Occupancy maps represent the environment as a grid, where each cell contains a probability value indicating the likelihood of it being occupied. Mathematically, the probability of a cell being occupied is updated using sensor measurements via:

P(O | Z) = (P(Z | O) * P(O)) / P(Z)
Where:

P(O): Prior probability of the cell being occupied.

P(Z | O): Probability of the sensor measurement given the cell is occupied.

P(Z): Normalizing constant.

This project uses a binary occupancy map, with the occupancy value \( O \) thresholded for visualization.

### 2. **LIDAR Simulation**
The project uses a simulated LIDAR sensor for range measurements. The sensor provides:
- Ranges (r): Representing the distance to obstacles.
- Angles (θ): Representing the direction of the measurement.
   ```bash
   p_world = T_sensor * p_sensor
   ```
The transformation from sensor coordinates to world coordinates is achieved using homogeneous transformations:
   ```bash
   T_sensor =
   [ cos(θ)  -sin(θ)   x ]
   [ sin(θ)   cos(θ)   y ]
   [  0         0      1 ]
   ```


### 3. **Trajectory Planning**
The robot's trajectory is planned using a Dubins path planner, which minimizes the turning radius while connecting start and goal points. The Dubins path enforces:
   ```bash
   κ ≥ κ_min
   ```
Where:
   ```bash
   κ: Curvature of the path.
   κ_min: Minimum allowable turning radius.
   ```

### 4. **Real-Time Map Updates**
As the robot moves, sensor data is used to update the local egocentric map. Ray tracing is performed for each LIDAR measurement to update cells along the beam:
   ```bash
   Occupancy Update = logit(P(O | Z))
   ```
The logit transformation is used for efficient Bayesian updates.

## Implementation Workflow

1. **Map Generation**: A binary occupancy map is generated using predefined data.
2. **Trajectory Planning**: The Dubins path planner computes a trajectory from the start to the goal state.
3. **Simulation**:
   - At each timestep, the robot's position is updated.
   - LIDAR measurements are simulated and incorporated into the egocentric map.
4. **Visualization**: The world map, local egocentric map, and robot trajectory are visualized in real-time.

## Key Equations

1. **Distance Between Waypoints**:
   ```bash
   d = sqrt((x2 - x1)^2 + (y2 - y1)^2)
   ```

2. **Angle Transformation**:
   ```bash
   θ = atan2(y2 - y1, x2 - x1)
   ```

3. **Ray Tracing Update**:
For each ray (r), the cells along the beam are updated using the sensor model.

## Requirements

- MATLAB R2022b or later
- Navigation Toolbox
- Robotics Toolbox

## Usage

1. Clone the repository:
   ```bash
   git clone https://github.com/erfan-seifi/PlanningUsingRangeSensors.git
   cd PlanningUsingRangeSensors
   ```
2. Open the project in MATLAB.
3. Run the script PlanningUsingRangeSensors.m to execute the simulation.

## REFERENCES
MATLAB Navigation Toolbox Documentation: Create Egocentric Occupancy Maps Using Range Sensors
