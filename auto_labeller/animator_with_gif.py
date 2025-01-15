import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import datetime
import seaborn as sns
from matplotlib.patches import Patch

# Configuration
person = "tuan"
filename = "data/clean_data/done_" + person + "_labeled.csv"  # Set your CSV file name here
time_speedup = 3/4  # Speed up the time by 4x
fade_time = 20  # Points disappear after 10 seconds of simulated time

# Load and prepare the data
data = pd.read_csv(filename)
data = data[data['id_prefix'] == 4.0]  # Filter for id_prefix 4
data = data.sort_values('timestamp')  # Sort by timestamp

# Normalize timestamps to start at 0
data['timestamp'] = data['timestamp'] - data['timestamp'].min()

# Convert timestamps to seconds for animation
data['time'] = data['timestamp'] / 1_000_000_000  # If timestamp is in nanoseconds
data['time'] = data['time'].astype(float)

# Assign distinct colors for each unique ID
unique_ids = data['id'].unique()
color_palette = sns.color_palette("husl", len(unique_ids))  # Generate a color palette
id_to_color = {id_: color_palette[i] for i, id_ in enumerate(unique_ids)}  # Map IDs to colors

# Initialize the plot
fig, ax = plt.subplots(figsize=(8, 8))
ax.set_xlim(-12.5, 12.5)  # Fixed Y Position range
ax.set_ylim(-4, 9)        # Fixed flipped X Position range
ax.set_aspect('equal', adjustable='box')
ax.set_title("Animated Trajectory Plot")
ax.set_xlabel('Y Position')
ax.set_ylabel('Flipped X Position')

# Create a legend for the IDs
# legend_patches = [Patch(color=color, label=f'ID {id_}') for id_, color in id_to_color.items()]
# ax.legend(handles=legend_patches, title="Legend", bbox_to_anchor=(1.05, 1), loc='upper left')

# Prepare scatter plot
sc = ax.scatter([], [], c=[], alpha=1, cmap='bwr')

# Function to update the plot
def update(frame):
    current_time = frame / time_speedup  # Simulated time at speedup
    active_data = data[data['time'] <= current_time]  # Points that have appeared
    alpha_values = np.clip(1 - (current_time - active_data['time']) / fade_time, 0, 1)

    if len(alpha_values[alpha_values > 0]) > 0:
        # Only include points that haven't completely faded
        visible_data = active_data[alpha_values > 0]
        alphas = alpha_values[alpha_values > 0]
        
        # Assign colors based on ID
        colors = visible_data['id'].map(id_to_color)

        # Update scatter plot
        sc.set_offsets(np.c_[visible_data['y'], -visible_data['x']])  # Flipped X values
        # sc.set_alpha(alphas)
        sc.set_array(visible_data['timestamp'])  # Use local time for coloring
        sc.set_alpha(alphas)  # Gradually fade out points

        ax.set_title(f"Time: {datetime.timedelta(seconds=int(current_time))}")
    return sc,

# Create animation
total_time = data['time'].max() + fade_time
frames = int(total_time * time_speedup)  # Frames for speedup
ani = animation.FuncAnimation(fig, update, frames=frames, interval=25, blit=False)

# Save animation as a GIF
output_gif = "data/img/" + person + "_animation.gif"
ani.save(output_gif, writer="pillow", fps=10)  # Adjust FPS if needed

print(f"Animation saved as: {output_gif}")

# Display the plot
plt.show()
