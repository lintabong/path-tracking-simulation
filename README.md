# Robot Path Tracking Simulation

A Python-based robot path tracking simulation with environmental disturbances and multiple control algorithms. This application provides a visual interface for simulating robot navigation through predefined paths while accounting for wind effects and tracking accuracy.

## Features

- **Visual Path Tracking**: Real-time visualization of robot movement along predefined paths
- **Multiple Control Modes**: 
  - Simple control
  - Fuzzy logic control
  - Model Predictive Control (MPC)
- **Environmental Simulation**: Wind disturbance effects on robot movement
- **Accuracy Monitoring**: Real-time calculation and display of tracking accuracy
- **Data Logging**: Complete logging of robot states and reference points
- **Excel Export**: Export simulation data for analysis

## Requirements

### Python Dependencies

```bash
pip install tkinter openpyxl
```

### System Requirements
- Python 3.6 or higher
- Windows OS (for DPI awareness feature)
- Display resolution: Minimum 800x600

## Installation

1. Clone or download the repository
2. Install required dependencies:
   ```bash
   pip install openpyxl
   ```
   Note: `tkinter` comes pre-installed with most Python distributions

3. Run the simulation:
   ```bash
   python robot_simulation.py
   ```

## Usage

### Interface Overview

The application consists of two main sections:

#### Left Panel - Control Interface
- **Accuracy Display**: Shows real-time tracking accuracy percentage
- **Control Mode Selection**: Choose between Simple, Fuzzy, or MPC control
- **Wind Settings**: Adjust environmental wind parameters (X and Y components)
- **Control Buttons**: Start, Stop, Reset simulation
- **Export Function**: Save simulation data to Excel

#### Right Panel - Simulation Canvas
- **Path Visualization**: Black lines showing the predefined robot path
- **Robot Representation**: Blue circle representing the robot position
- **Wind Indicator**: Red arrow showing wind direction and magnitude

### Running a Simulation

1. **Select Control Mode**: Choose your desired control algorithm
2. **Set Wind Parameters**: Adjust wind X and Y values (default: 0.0)
3. **Start Simulation**: Click "Start" to begin robot movement
4. **Monitor Progress**: Watch real-time accuracy updates
5. **Control Simulation**: Use Stop/Reset buttons as needed
6. **Export Data**: Click "Export to Excel" to save results

### Control Modes

#### Simple Control
- Basic proportional control for heading correction
- Gain factor: k_omega = 2.0

#### Fuzzy Logic Control
- Currently implements same logic as simple control
- Framework ready for fuzzy rule implementation

#### Model Predictive Control (MPC)
- Currently implements same logic as simple control
- Framework ready for MPC algorithm implementation

## Technical Details

### Robot Model
- **Position**: (x, y) coordinates in pixels
- **Orientation**: θ (theta) in radians
- **Velocity**: Fixed at 2.0 m/s
- **Angular Velocity**: ω (omega) - controlled parameter

### Path Definition
The default path consists of 8 waypoints:
```python
path = [(50,200),(150,200),(250,100),(350,200),(450,200),(450,300),(350,300),(250,200)]
```

### Error Calculations
- **Lateral Error (ey)**: Cross-track error with added sensor noise
- **Heading Error (etheta)**: Angular difference from desired heading
- **Accuracy**: Calculated as (100 - RMSE_percentage)%

### Environmental Effects
- **Wind Disturbance**: Added to robot velocity components
- **Sensor Noise**: Gaussian noise added to position and heading measurements

## Data Export

The simulation logs the following data points:
- Robot position (x, y)
- Reference position (x_ref, y_ref)
- Lateral error (ey)
- Heading error (etheta)
- Distance to next waypoint (dist)

Data is exported to `robot_log.xlsx` in the same directory.

## Customization

### Modifying the Path
Edit the `path` list in the code:
```python
path = [(x1,y1), (x2,y2), ..., (xn,yn)]
```

### Adjusting Robot Parameters
Modify these values in the `RobotSim` class:
- `self.v`: Robot velocity
- `k_omega`: Control gain
- Noise parameters in `random.gauss()`

### Adding Control Algorithms
Implement new control logic in the `step()` method under the respective control mode conditions.

## Known Issues

- Control algorithms (Fuzzy and MPC) currently use placeholder implementations
- DPI awareness is Windows-specific
- Canvas size is fixed (may need adjustment for different screen sizes)

## Future Enhancements

- Implement full fuzzy logic controller
- Add complete MPC algorithm
- Dynamic path generation
- Obstacle avoidance
- Multiple robot support
- Real-time parameter tuning

## License

This project is provided as-is for educational and research purposes.

## Contributing

Feel free to contribute by:
- Implementing missing control algorithms
- Adding new features
- Improving the user interface
- Optimizing performance
- Adding unit tests

---

For questions or support, please refer to the code comments or create an issue in the repository.