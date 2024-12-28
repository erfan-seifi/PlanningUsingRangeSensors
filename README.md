# Planning Using Range Sensors

This project demonstrates the mathematical and algorithmic principles behind egocentric occupancy maps, motion planning, and trajectory generation using range sensor data. The implementation is inspired by the MATLAB example ["Create Egocentric Occupancy Maps Using Range Sensors"](https://www.mathworks.com/help/nav/ug/create-egocentric-occupancy-maps-using-range-sensors.html).

## Mathematical Foundations

### 1. **Occupancy Maps**
Occupancy maps represent the environment as a grid, where each cell contains a probability value indicating the likelihood of it being occupied. Mathematically, the probability of a cell being occupied is updated using sensor measurements via:
\[
P(O | Z) = \frac{P(Z | O) P(O)}{P(Z)}
\]
where:
- \( P(O) \): Prior probability of the cell being occupied.
- \( P(Z | O) \): Probability of the sensor measurement given the cell is occupied.
- \( P(Z) \): Normalizing constant.

This project uses a binary occupancy map, with the occupancy value \( O \) thresholded for visualization.

### 2. **LIDAR Simulation**
The project uses a simulated LIDAR sensor for range measurements. The sensor provides:
- Ranges \( r \), representing the distance to obstacles.
- Angles \( \theta \), representing the direction of the measurement.

The transformation from sensor coordinates to world coordinates is achieved using homogeneous transformations:
\[
\mathbf{p}_{\text{world}} = \mathbf{T}_{\text{sensor}} \mathbf{p}_{\text{sensor}}
\]
where:
\[
\mathbf{T}_{\text{sensor}} =
\begin{bmatrix}
\cos\theta & -\sin\theta & x \\
\sin\theta & \cos\theta & y \\
0 & 0 & 1
\end{bmatrix}
\]

### 3. **Trajectory Planning**
The robot's trajectory is planned using a Dubins path planner, which minimizes the turning radius while connecting start and goal points. The Dubins path enforces:
\[
\kappa \geq \kappa_{\text{min}}
\]
where \( \kappa \) is the curvature and \( \kappa_{\text{min}} \) is the minimum allowable turning radius.

The planner interpolates the path states to generate a smooth trajectory.

### 4. **Real-Time Map Updates**
As the robot moves, sensor data is used to update the local egocentric map. Ray tracing is performed for each LIDAR measurement to update cells along the beam:
\[
\text{Occupancy Update} = \text{logit}\left(P(O | Z)\right)
\]
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
\[
d = \sqrt{(x_2 - x_1)^2 + (y_2 - y_1)^2}
\]

2. **Angle Transformation**:
\[
\theta = \text{atan2}(y_2 - y_1, x_2 - x_1)
\]

3. **Ray Tracing Update**:
For each ray \( r \), the cells along the beam are updated using the sensor model.

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
