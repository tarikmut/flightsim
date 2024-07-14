# Flight Simulation with JSBSim and PID Controllers

This project simulates the flight of a Cessna 172P aircraft using the JSBSim flight dynamics model. The simulation includes real-time plotting of various flight parameters and the implementation of a PID controller for altitude and speed control.

## Features

- **JSBSim Integration**: Uses JSBSim for flight dynamics simulation.
- **PID Controllers**: Implements a cascade PID controller for altitude and pitch control, and additional PID controllers for speed and roll control.
- **Real-time Plotting**: Visualizes flight parameters such as altitude, speed, roll angle, and control inputs in real-time.

## Requirements

- Python 3.x
- JSBSim
- Matplotlib

## Installation

1. Install JSBSim:
    ```bash
    pip install jsbsim
    ```

2. Install Matplotlib:
    ```bash
    pip install matplotlib
    ```

## Usage

1. Clone the repository:
    ```bash
    git clone https://github.com/tarikmut/flightsim.git
    cd flightsim
    ```

2. Run the simulation script:
    ```bash
    python flight_simulation.py
    ```

3. You will be prompted to choose whether to update the graphics in real-time. Enter `yes` or `no`.

## PID Controllers

The simulation uses the following PID controllers:

- **Outer Altitude PID**: Controls the altitude.
- **Inner Pitch PID**: Controls the pitch angle.
- **Speed PID**: Controls the speed.
- **Roll PID**: Controls the roll angle.

## Visualization

The simulation visualizes the following parameters:

- **Altitude Over Time**
- **Speed Over Time**
- **Roll Angle Over Time**
- **Elevator Command Over Time**
- **Throttle Command Over Time**
- **Aileron Command Over Time**
- **3D Flight Path**

## License

This project is licensed under the MIT License.

## Acknowledgments

- [JSBSim](https://github.com/JSBSim-Team/jsbsim) for the flight dynamics model.
- [Matplotlib](https://matplotlib.org/) for plotting and visualization.
