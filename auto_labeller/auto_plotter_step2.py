import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import matplotlib.cm as cm
import matplotlib.colors as mcolors
import datetime
import numpy as np

# Load data
file_path = "data/clean_data/labelled/ws_hareesh_labeled.csv"
data = pd.read_csv(file_path)

# Ensure trajectory_id is an integer
data['trajectory_id'] = data['trajectory_id'].fillna(-1).astype(int)

# Get unique trajectory IDs
trajectory_ids = sorted(data['trajectory_id'].unique())

# Initialize figure and axis
fig, ax = plt.subplots()
plt.subplots_adjust(bottom=0.2)  # Leave space for buttons

# Plot settings
current_index = 0  # Start with the first trajectory

# Helper function to convert timestamp to hh:mm:ss
def timestamp_to_time(ts):
    return datetime.datetime.fromtimestamp(int(ts / 10**9)).strftime('%Y-%m-%d %H:%M:%S')

# Function to plot a trajectory
def plot_trajectory(index):
    global current_index
    current_index = index

    # Clear previous plot
    ax.clear()

    # Get trajectory data
    traj_id = trajectory_ids[index]
    trajectory_data = data[data['trajectory_id'] == traj_id]

    # Normalize time for coloring
    norm = mcolors.Normalize(vmin=trajectory_data['timestamp'].min(), vmax=trajectory_data['timestamp'].max())
    sm = cm.ScalarMappable(cmap='Blues', norm=norm)

    # Plot trajectory
    ax.scatter(trajectory_data['y'], -trajectory_data['x'], c=trajectory_data['timestamp'], marker='o', cmap='Blues', norm=norm)

    # Plot start and goal workstations
    start_workstation = trajectory_data.iloc[0]['start']
    goal_workstation = trajectory_data.iloc[0]['goal']

    if not np.isnan(start_workstation):
        ax.scatter(trajectory_data.iloc[0]['y'], -trajectory_data.iloc[0]['x'], color='white', label=f"Start: WS {int(start_workstation)}", s=100, edgecolors='black')
        # plt.text(trajectory_data.iloc[0]['y'], -trajectory_data.iloc[0]['x'], f"Start: WS {int(start_workstation)}", fontsize=100)

    if not np.isnan(goal_workstation):
        ax.scatter(trajectory_data.iloc[-1]['y'], -trajectory_data.iloc[-1]['x'], color='blue', label=f"Goal: WS {int(goal_workstation)}", s=100, edgecolors='black')
        # plt.text(trajectory_data.iloc[-1]['y'], -trajectory_data.iloc[-1]['x'], f"Goal: WS {int(goal_workstation)}", fontsize=100)

    # # Add colorbar
    # cbar = plt.colorbar(sm, ax=ax, orientation='vertical', pad=0.05)
    # cbar.set_label("Timestamp")

    # Title and labels
    first_time = timestamp_to_time(trajectory_data['timestamp'].iloc[0])
    last_time = timestamp_to_time(trajectory_data['timestamp'].iloc[-1])
    ax.set_title(f"Trajectory {traj_id} ({index + 1}/{len(trajectory_ids)})")#\nStart: WS {int(start_workstation) if not np.isnan(start_workstation) else 'N/A'}, Goal: WS {int(goal_workstation) if not np.isnan(goal_workstation) else 'N/A'}\nFirst Time: {first_time}, Last Time: {last_time}")
    ax.set_xlabel('Y Position')
    ax.set_ylabel('Flipped X Position')

    # Set fixed axis limits
    ax.set_xlim(-12.5, 12.5)
    ax.set_ylim(-4, 9)  # Adjusted range for flipped X
    # Ensure equal scaling for both axes
    ax.set_aspect('equal', adjustable='box')

    # Add legend
    ax.legend()

    # Redraw plot
    plt.draw()

# Button callback functions
def next_trajectory(event):
    global current_index
    if current_index < len(trajectory_ids) - 1:
        plot_trajectory(current_index + 1)

def previous_trajectory(event):
    global current_index
    if current_index > 0:
        plot_trajectory(current_index - 1)

# Add buttons
ax_prev = plt.axes([0.1, 0.05, 0.2, 0.075])
ax_next = plt.axes([0.7, 0.05, 0.2, 0.075])

btn_prev = Button(ax_prev, 'Previous')
btn_next = Button(ax_next, 'Next')

btn_prev.on_clicked(previous_trajectory)
btn_next.on_clicked(next_trajectory)

# Initial plot
plot_trajectory(current_index)

# Show plot
plt.show()
