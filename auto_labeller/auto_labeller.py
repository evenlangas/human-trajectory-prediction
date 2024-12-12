import pandas as pd
import numpy as np

# Define bounding box coordinates for workstations (example format: [(x1, y1, x2, y2), ...])
workstations = [
    (10, 10, 20, 20),
    (30, 30, 40, 40),
    (50, 50, 60, 60),
    (70, 70, 80, 80),
    (90, 90, 100, 100),
    (110, 110, 120, 120)
]

def is_in_bounding_box(x, y, bbox):
    """Check if a point is inside a bounding box."""
    x1, y1, x2, y2 = bbox
    return x1 <= x <= x2 and y1 <= y <= y2

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

    # Add a column for workstation labels
    df['workstation'] = df.apply(label_workstation, workstations=workstations, axis=1)

    # Initialize new columns for trajectory ID, start, and goal
    df['trajectory_id'] = np.nan
    df['start'] = np.nan
    df['goal'] = np.nan

    current_trajectory_id = 0
    start_station = None

    for i, row in df.iterrows():
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

    # Save the labeled data to a new CSV
    df.to_csv(output_csv, index=False)

# Example usage
process_trajectory_data('input.csv', 'output_labeled.csv')
