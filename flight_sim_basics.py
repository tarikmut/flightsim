import jsbsim
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
import csv
from matplotlib.gridspec import GridSpec

# Start JSBSim simulator
sim = jsbsim.FGFDMExec(root_dir='C:\\Users\\tarik\\AppData\\Local\\JSBSim')
# Load aircraft model
sim.load_model('c172p')

# Set time step
sim.set_dt(0.01)

# Set initial conditions
sim['ic/h-sl-ft'] = 1250  # Initial altitude (feet)
sim['ic/vg-kts'] = 100  # Initial speed (knots)
sim['ic/psi-true-deg'] = 0  # Initial heading (degrees)
sim['ic/gamma-deg'] = 0  # Initial climb angle (degrees)
sim['ic/lat-gc-deg'] = 0  # Initial latitude (degrees)
sim['ic/long-gc-deg'] = 0  # Initial longitude (degrees)
sim['propulsion/engine[0]/set-running'] = 1

# Start simulation
sim.run_ic()

# Fuel and engine setup
sim['fcs/throttle-cmd-norm'] = 0.50
sim['fcs/mixture-cmd-norm'] = 1.0
sim['propulsion/magneto_cmd'] = 3
sim['propulsion/starter_cmd'] = 1

# Check initial speed
initial_speed = sim['velocities/vc-kts']
print(f"Initial speed: {initial_speed} knots")

class PID:
    def __init__(self, kp, ki, kd, setpoint, output_limits=(None, None)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.prev_error = 0
        self.integral = 0
        self.output_limits = output_limits  # Output clamping limits for anti-windup

    def compute(self, measured_value, dt):
        error = self.setpoint - measured_value
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        # Clamping (anti-windup)
        if self.output_limits[0] is not None and output < self.output_limits[0]:
            output = self.output_limits[0]
            self.integral -= error * dt  # Adjust integral to avoid windup
        elif self.output_limits[1] is not None and output > self.output_limits[1]:
            output = self.output_limits[1]
            self.integral -= error * dt  # Adjust integral to avoid windup

        self.prev_error = error
        return output

class CascadePID:
    def __init__(self, outer_pid, inner_pid):
        self.outer_pid = outer_pid
        self.inner_pid = inner_pid

    def compute(self, outer_measured, inner_measured, dt):
        # Outer loop (e.g., altitude control)
        outer_output = self.outer_pid.compute(outer_measured, dt)
        # Inner loop (e.g., pitch control)
        self.inner_pid.setpoint = outer_output
        inner_output = self.inner_pid.compute(inner_measured, dt)
        return inner_output

# Define PID controllers
outer_altitude_pid = PID(kp=0.026, ki=0.0004, kd=0.0005, setpoint=1600, output_limits=(-10, 10))  # Target altitude: 1500 feet
inner_pitch_pid = PID(kp=0.1, ki=0.0002, kd=0.0005, setpoint=0, output_limits=(-1, 1))  # Initial target pitch angle: 0 degrees
cascade_pid = CascadePID(outer_altitude_pid, inner_pitch_pid)

speed_pid = PID(kp=0.1, ki=0.005, kd=0.1, setpoint=100, output_limits=(0, 1))  # Target speed: 100 knots
roll_pid = PID(kp=0.05, ki=0.001, kd=0.0005, setpoint=10, output_limits=(-1, 1))  # Target roll angle: 0 degrees (level flight)

# Simulation duration and time step
duration = 600  # Simulation duration (seconds)
dt = 0.01  # Time step (seconds)

# Lists to store aircraft position data
times = []
altitudes = []
elevator_commands = []    
speeds = []
throttle_commands = []
thrusts = []
roll_angles = []
aileron_commands = []
yaw_angles = []
pitch_angles = []
total_fuels = []

positions = []

# Create log file and write headers
with open('flight_log.csv', mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['Time', 'Altitude', 'Speed', 'Roll Angle', 'Elevator Command', 'Throttle Command', 'Aileron Command'])

# Get user input for real-time graphics update
update_graphics = input("Would you like to update the graphics in real-time? (yes/no): ").strip().lower()

# Real-time graphics update settings with matplotlib
fig = plt.figure(figsize=(18, 10))
fig.suptitle('Flight Simulation Data', fontsize=16)

# Layout settings with GridSpec
gs = GridSpec(2, 4, figure=fig)
ax1 = fig.add_subplot(gs[0, 0])
ax2 = fig.add_subplot(gs[0, 2])
ax3 = fig.add_subplot(gs[0, 1])
ax4 = fig.add_subplot(gs[1, 0])
ax5 = fig.add_subplot(gs[1, 2])
ax6 = fig.add_subplot(gs[1, 1])
ax7 = fig.add_subplot(gs[1, 3], projection='3d')

# Initial plot settings
ax1.set_xlim(0, duration)
ax1.set_ylim(0, 3000)
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Altitude (ft)')
ax1.set_title('Altitude Over Time')
line1, = ax1.plot([], [], label='Altitude')

ax2.set_xlim(0, duration)
ax2.set_ylim(0, 150)
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Speed (kts)')
ax2.set_title('Speed Over Time')
line2, = ax2.plot([], [], label='Speed')

ax3.set_xlim(0, duration)
ax3.set_ylim(-30, 30)
ax3.set_xlabel('Time (s)')
ax3.set_ylabel('Roll Angle (deg)')
ax3.set_title('Roll Angle Over Time')
line3, = ax3.plot([], [], label='Roll Angle')

ax4.set_xlim(0, duration)
ax4.set_ylim(-1, 1)
ax4.set_xlabel('Time (s)')
ax4.set_ylabel('Elevator Command')
ax4.set_title('Elevator Command Over Time')
line4, = ax4.plot([], [], label='Elevator Command')

ax5.set_xlim(0, duration)
ax5.set_ylim(0, 1)
ax5.set_xlabel('Time (s)')
ax5.set_ylabel('Throttle Command')
ax5.set_title('Throttle Command Over Time')
line5, = ax5.plot([], [], label='Throttle Command')

ax6.set_xlim(0, duration)
ax6.set_ylim(-1, 1)
ax6.set_xlabel('Time (s)')
ax6.set_ylabel('Aileron Command')
ax6.set_title('Aileron Command Over Time')
line6, = ax6.plot([], [], label='Aileron Command')

ax7.set_xlim(-0.01, 0.04)
ax7.set_ylim(-0.03, 0.03)
ax7.set_zlim(1200, 2000)
ax7.set_xlabel('Longitude')
ax7.set_ylabel('Latitude')
ax7.set_zlabel('Altitude')
ax7.set_title('3D Flight Path', pad=20)
line7, = ax7.plot([], [], [], label='Flight Path')

# Add text box
text_box = fig.text(0.75, 0.85, '', transform=fig.transFigure, fontsize=12, verticalalignment='top')

plt.legend()

# Update function
def update(frame):
    sim.run()  # Run one time step
    sim_time = sim['simulation/sim-time-sec']

    # Get current altitude, speed, and roll angle
    altitude = sim['position/h-sl-ft']
    speed = sim['velocities/vc-kts']
    roll_angle = sim['attitude/phi-deg']
    yaw_angle = sim['attitude/psi-deg']  # Get yaw angle
    pitch_angle = sim['attitude/theta-deg']  # Get pitch angle

    # Get current latitude and longitude
    latitude = sim['position/lat-gc-deg']
    longitude = sim['position/long-gc-deg']

    # Calculate elevator command using cascade PID controller
    elevator_command = -cascade_pid.compute(altitude, pitch_angle, dt)
    throttle_command = speed_pid.compute(speed, dt)
    aileron_command = roll_pid.compute(roll_angle, dt)

    # Send commands to the simulator
    sim['fcs/elevator-cmd-norm'] = elevator_command
    sim['fcs/throttle-cmd-norm'] = throttle_command
    sim['fcs/aileron-cmd-norm'] = aileron_command

    # Print current state information to console
    thrust = sim['propulsion/engine/thrust-lbs']
    total_fuel = sim['propulsion/total-fuel-lbs']
    print(f'{sim_time:.2f}s-> Altitude: {altitude:.2f} ft, Elevator Command: {elevator_command:.2f}, Airspeed: {speed:.2f} kts, Throttle Command: {throttle_command:.2f}, Thrust: {thrust:.0f}, Roll Angle: {roll_angle:.2f}, Aileron Command: {aileron_command:.3f}, Yaw Angle: {yaw_angle:.1f}, Pitch Angle: {pitch_angle:.3f}, Total Fuel: {total_fuel:.1f}')

    # Append simulation data to lists
    times.append(frame * dt)
    altitudes.append(altitude)
    elevator_commands.append(elevator_command)
    speeds.append(speed)
    throttle_commands.append(throttle_command)
    thrusts.append(thrust)
    roll_angles.append(roll_angle)
    aileron_commands.append(aileron_command)
    yaw_angles.append(yaw_angle)
    pitch_angles.append(pitch_angle)
    total_fuels.append(total_fuel)
    positions.append((latitude, longitude, altitude))

    # Write to log file
    with open('flight_log.csv', mode='a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow([frame * dt, altitude, speed, roll_angle, elevator_command, throttle_command, aileron_command])

    # Update plot data
    if update_graphics == 'yes':
        line1.set_data(times, altitudes)
        line2.set_data(times, speeds)
        line3.set_data(times, roll_angles)
        line4.set_data(times, elevator_commands)
        line5.set_data(times, throttle_commands)
        line6.set_data(times, aileron_commands)

        # Update 3D flight path
        lats, longs, alts = zip(*positions)
        line7.set_data(longs, lats)
        line7.set_3d_properties(alts)

        # Update text box
        text_box.set_text(f'Simulation Time: {sim_time:.2f}s\nAltitude: {altitude:.2f} ft Elevator Command: {elevator_command:.2f}\nPitch Angle: {pitch_angle:.3f}\nRoll Angle: {roll_angle:.2f} Aileron Command: {aileron_command:.3f}\nYaw Angle: {yaw_angle:.1f}\nAirspeed: {speed:.2f} kts, Throttle Command: {throttle_command:.2f}\nTotal Fuel: {total_fuel:.1f} Thrust: {thrust:.0f}')

    return line1, line2, line3, line4, line5, line6, line7, text_box

if update_graphics == 'yes':
    # Start animation
    ani = animation.FuncAnimation(fig, update, frames=int(duration / dt), interval=10, repeat=False)

    # Get window manager
    mng = plt.get_current_fig_manager()

    # Set window title
    mng.set_window_title("Flight Simulation")

    # Set minimum window size
    mng.window.minsize(800, 600)

    # Maximize window
    mng.window.state('zoomed')

    plt.tight_layout()
    plt.show()
else:
    # Populate plots at the end of the simulation
    for frame in range(int(duration / dt)):
        update(frame)

    # Get window manager
    mng = plt.get_current_fig_manager()

    # Set window title
    mng.set_window_title("Flight Simulation")
    # Set minimum window size
    mng.window.minsize(800, 600)

    # Maximize window
    mng.window.state('zoomed')

    plt.tight_layout()
    
    # Update plot data
    line1.set_data(times, altitudes)
    line2.set_data(times, speeds)
    line3.set_data(times, roll_angles)
    line4.set_data(times, elevator_commands)
    line5.set_data(times, throttle_commands)
    line6.set_data(times, aileron_commands)

    # Update 3D flight path
    lats, longs, alts = zip(*positions)
    line7.set_data(longs, lats)
    line7.set_3d_properties(alts)

    # Update text box
    text_box.set_text(f'Simulation Time: {times[-1]:.2f}s\nAltitude: {altitudes[-1]:.2f} ft Elevator Command: {elevator_commands[-1]:.2f}\nPitch Angle: {pitch_angles[-1]:.3f}\nRoll Angle: {roll_angles[-1]:.2f} Aileron Command: {aileron_commands[-1]:.3f}\nYaw Angle: {yaw_angles[-1]:.1f}\nAirspeed: {speeds[-1]:.2f} kts, Throttle Command: {throttle_commands[-1]:.2f}\nTotal Fuel: {total_fuels[-1]:.1f} Thrust: {thrusts[-1]:.0f}')

    plt.show()
