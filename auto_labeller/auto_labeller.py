import pandas as pd
import numpy as np

# Define bounding box coordinates for workstations (example format: [(x1, y1, x2, y2), ...])
workstations_com = [
    (-4, 12.1),
    (-7.5, 9.5),
    (-8.5, 2.3),
    (3.4, 3.0),
    (2.1, -10.6),
    (-0.4, -12)
]

workstations = []

for workstaion_com in workstations_com:
    workstations.append((workstaion_com[0]-0.05, workstaion_com[1]-0.05,workstaion_com[0]+0.05, workstaion_com[1]+0.05))

def is_in_bounding_box(x, y, bbox):
    """Check if a point is inside a bounding box."""
    x1, y1, x2, y2 = bbox
    return x1 <= x <= x2 and y1 <= y <= y2

# Pad nanoseconds to ensure correct timestamp format
def pad_timestamp(timestamp):
    """Pad nanoseconds in the timestamp to ensure 9 digits."""
    sec = timestamp[:10]  # First 10 digits are seconds
    nanosec = timestamp[10:].ljust(9, '0')  # Rest are nanoseconds, padded to 9 digits
    return f"{sec}{nanosec}"

# Label each row with the workstation index (0 for not in any bounding box)
def label_workstation(row, workstations):
    for i, bbox in enumerate(workstations, start=1):
        if is_in_bounding_box(row['x'], row['y'], bbox):
            return i
    return 0

def process_trajectory_data(input_csv, output_csv):
    """Process the trajectory data to label each point with trajectory properties."""
    # Read the input CSV
    df = pd.read_csv(input_csv)

    # Fix timestamp formatting
    df['timestamp'] = df['timestamp'].astype(str).apply(lambda x: pad_timestamp(x))

    # Add a column for workstation labels
    df['workstation'] = df.apply(label_workstation, workstations=workstations, axis=1)

    df = df[df['id_prefix'] == 4.0].copy()
    df = df[df['velocity_scalar'] > 0.1].copy()

    # Initialize new columns for trajectory ID, start, and goal
    df['trajectory_id'] = np.nan
    df['start'] = np.nan
    df['goal'] = np.nan

    current_trajectory_id = 0
    start_station = None
    
    indices_to_drop = []
    
    for i, row in df.iterrows():
        # if row['id_prefix'] != 4.0:
        #     df.drop(i)
        #     continue
        
        if is_in_bounding_box(row['x'], row['y'], (0.5, -12, 2, -11.45)):
            indices_to_drop.append(i)
            continue

        current_station = row['workstation']

        if start_station is None and current_station > 0:
            # Entering a workstation for the first time
            start_station = current_station

        elif start_station is not None and current_station != start_station and current_station > 0:
            # Transition between workstations
            df.loc[i, 'trajectory_id'] = current_trajectory_id
            df.loc[i, 'start'] = start_station
            df.loc[i, 'goal'] = current_station

            # Update goal for all points in the trajectory
            trajectory_indices = df[(df['trajectory_id'] == current_trajectory_id)].index
            df.loc[trajectory_indices, 'goal'] = current_station

            current_trajectory_id += 1
            start_station = current_station

        elif current_station == 0 and start_station is not None:
            # Inside a trajectory between workstations
            df.loc[i, 'trajectory_id'] = current_trajectory_id
            df.loc[i, 'start'] = start_station

        elif current_station > 0 and start_station is None:
            # Just entered a workstation
            start_station = current_station


    if len(indices_to_drop) > 0:
        print(indices_to_drop)

        print(df['x'][indices_to_drop[0]])

        df = df.drop(indices_to_drop)

    df = df[df['workstation'] == 0].copy()

    # Save the labeled data to a new CSV
    df.to_csv(output_csv, index=False)

# Example usage
base_path = "data/"
filename = "even"
process_trajectory_data(base_path + filename + '.csv', base_path + filename + '_labeled.csv')
