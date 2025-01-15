import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import matplotlib.cm as cm
import matplotlib.colors as mcolors
import datetime
import math

# Configuration
filename = "data/clean_data/ws_even_labeled.csv"  # Set your CSV file name here

# Load data
data = pd.read_csv(filename)

# Filter data for id_prefix 4
data = data[data['id_prefix'] == 4.0]

# Ensure timestamps are valid
def validate_timestamps(df):
    try:
        df['timestamp'] = pd.to_numeric(df['timestamp'], errors='coerce')
        df = df.dropna(subset=['timestamp'])  # Remove rows with invalid timestamps
        df = df[df['timestamp'] >= 0]  # Keep only non-negative timestamps
        return df
    except KeyError:
        raise ValueError("The dataset must include a 'timestamp' column.")

data = validate_timestamps(data)

# Get unique trajectory IDs
trajectory_ids = data['trajectory_id'].dropna().unique()
current_idx = 0

# Helper function to convert timestamp to hh:mm:ss
def timestamp_to_time(ts):
    # try:
    print(math.floor(ts/10**9))
    return datetime.datetime.fromtimestamp(math.floor(ts/10**9)).strftime('%Y-%m-%d %H:%M:%S')
    # except (OSError, OverflowError, ValueError):
    #     return "Invalid Timestamp"

# Function to plot a single trajectory for one ID
def plot_trajectory(idx):
    ax.clear()
    trajectory_id = trajectory_ids[idx]
    trajectory_data = data[data['trajectory_id'] == trajectory_id]
    
    # Select the first ID in the trajectory to plot
    unique_ids = trajectory_data['id'].unique()
    current_id = unique_ids[0]
    id_data = trajectory_data[trajectory_data['id'] == current_id]

    # Sort data by timestamp
    # id_data = id_data.sort_values('timestamp')

    # Normalize time for coloring
    norm = mcolors.Normalize(vmin=id_data['timestamp'].min(), vmax=id_data['timestamp'].max())
    sm = cm.ScalarMappable(cmap='RdBu', norm=norm)

    # Plot points with gradient coloring
    scatter = ax.scatter(id_data['y'], -id_data['x'], c=id_data['timestamp'], cmap='RdBu', norm=norm)
    
    # Add colorbar
    # cbar = plt.colorbar(sm, ax=ax, orientation='vertical', pad=0.05)
    # cbar.set_label("Timestamp")

    # Title and labels
    first_time = timestamp_to_time(id_data['timestamp'].iloc[0])
    last_time = timestamp_to_time(id_data['timestamp'].iloc[-1])
    ax.set_title(f"Trajectory ID: {trajectory_id}, ID: {current_id}\nFirst Time: {first_time}, Last Time: {last_time}")
    ax.set_xlabel('Y Position')
    ax.set_ylabel('Flipped X Position')

    # Set fixed axis limits
    ax.set_xlim(-12.5, 12.5)
    ax.set_ylim(-4, 9)  # Adjusted range for flipped X

    # Ensure equal scaling for both axes
    ax.set_aspect('equal', adjustable='box')

    # Refresh the plot
    fig.canvas.draw_idle()

# Navigation functions
def next_trajectory(event):
    global current_idx
    if current_idx < len(trajectory_ids) - 1:
        current_idx += 1
        plot_trajectory(current_idx)

def previous_trajectory(event):
    global current_idx
    if current_idx > 0:
        current_idx -= 1
        plot_trajectory(current_idx)

# Set up the plot
fig, ax = plt.subplots(figsize=(8, 8))  # Fixed size for equal increments

# Add buttons for navigation
ax_prev = plt.axes([0.7, 0.02, 0.1, 0.05])  # Position for "Previous" button
ax_next = plt.axes([0.81, 0.02, 0.1, 0.05])  # Position for "Next" button

btn_prev = Button(ax_prev, 'Previous')
btn_next = Button(ax_next, 'Next')

btn_prev.on_clicked(previous_trajectory)
btn_next.on_clicked(next_trajectory)

# Initial plot
plot_trajectory(current_idx)
plt.show()
