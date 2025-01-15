import pandas as pd
import numpy as np

# Define bounding box coordinates for workstations (example format: [(x1, y1, x2, y2), ...])
workstations_com = [
    (-4, 12.1),
    (-7.3, 9.5),
    (-8.5, 2.3),
    (3.4, 3.0),
    (2.1, -10.6),
    (-0.4, -12)
]

workstations = []

for workstaion_com in workstations_com:
    workstations.append((workstaion_com[0]-1.4, workstaion_com[1]-1.4,workstaion_com[0]+1.4, workstaion_com[1]+1.4))

# Split trajectory if the person stands still for more than 2 seconds within a workstation
stand_still_threshold = 1 * 10**9  # 2 seconds in nanoseconds
position_tolerance = 0.1

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

def is_standing_still(start_idx, df):
    if start_idx < 0:
        return False
    start_time = int(df.loc[start_idx, 'timestamp'])
    end_time = start_time
    prev_end_time = start_time
    idx = start_idx
    # print(start_time) 
    # print(stand_still_threshold)
    while (end_time - start_time) < stand_still_threshold:
        idx += 1
        try:
            end_time = int(df.loc[idx, 'timestamp'])
            if (idx - start_idx == 2) and (end_time - prev_end_time) > stand_still_threshold:
                return True
            prev_end_time = end_time
            x_diff = abs(df.loc[idx, 'x'] - df.loc[start_idx, 'x'])
            y_diff = abs(df.loc[idx, 'y'] - df.loc[start_idx, 'y'])
            if x_diff >= position_tolerance or y_diff >= position_tolerance:
                return False
        except:
            return False
    return True

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
    
    # Drop duplicate positions appearing sequentially as it is a bug.
    df = df.loc[(df[['x', 'y']].shift(1) != df[['x', 'y']]).any(axis=1)].copy()
    df = df.reset_index(drop=True)

    # Initialize new columns for trajectory ID, start, and goal
    df['trajectory_id'] = np.nan
    df['start'] = np.nan
    df['goal'] = np.nan
    current_trajectory_id = 0

    # Initialize the previous orientation and workstation
    previous_orientation = None
    previous_workstation = None

    

    for i, row in df.iterrows():
        current_orientation = row['orientation']
        current_workstation = row['workstation']

        if previous_orientation is not None:
            # Calculate the absolute change in orientation
            orientation_change = abs(current_orientation - previous_orientation)
            # Adjust for circular orientation (0-360 degrees)
            if orientation_change > 180:
                orientation_change = 360 - orientation_change

            if orientation_change > 80 and (current_workstation != 0 or previous_workstation != 0):
                # Increment trajectory ID on orientation change > 20 degrees
                current_trajectory_id += 1

            elif is_standing_still(i-2, df) and (current_workstation != 0 and previous_workstation != 0):
                current_trajectory_id += 1

        # Assign the trajectory ID to the row
        df.loc[i, 'trajectory_id'] = current_trajectory_id

        # Update previous orientation and workstation
        previous_orientation = current_orientation
        previous_workstation = current_workstation

    # # Apply trajectory splitting to each trajectory group
    # for traj_id, group in df.groupby('trajectory_id'):
    #     split_trajectory(group, df, traj_id)

    # Set start and goal workstations for each trajectory
    trajectory_groups = df.groupby('trajectory_id')
    for traj_id, group in trajectory_groups:
        if not group.empty:
            start_station = group.iloc[0]['workstation']
            if start_station == 0:
                # Use the goal of the previous trajectory if starting in 0
                previous_traj = df[df['trajectory_id'] == traj_id - 1]
                if not previous_traj.empty:
                    start_station = previous_traj.iloc[-1]['goal']

            goal_station = group.iloc[-1]['workstation']
            if goal_station == 0:
                # Use the start of the next trajectory if goal is 0
                next_traj = df[df['trajectory_id'] == traj_id + 1]
                if not next_traj.empty:
                    goal_station = next_traj.iloc[0]['start']

            df.loc[group.index, 'start'] = start_station
            df.loc[group.index, 'goal'] = goal_station

    # Do it again to make sure it gets right goal workstation when goal workstation is 0
    trajectory_groups = df.groupby('trajectory_id')
    for traj_id, group in trajectory_groups:
        if not group.empty:
            start_station = group.iloc[0]['workstation']
            if start_station == 0:
                # Use the goal of the previous trajectory if starting in 0
                previous_traj = df[df['trajectory_id'] == traj_id - 1]
                if not previous_traj.empty:
                    start_station = previous_traj.iloc[-1]['goal']

            goal_station = group.iloc[-1]['workstation']
            if goal_station == 0:
                # Use the start of the next trajectory if goal is 0
                next_traj = df[df['trajectory_id'] == traj_id + 1]
                if not next_traj.empty:
                    goal_station = next_traj.iloc[0]['start']

            df.loc[group.index, 'start'] = start_station
            df.loc[group.index, 'goal'] = goal_station

    # # Remove trajectories that only include points inside the same workstation
    # to_remove = []
    # for traj_id, group in trajectory_groups:
    #     if group['workstation'].nunique() == 1 and group['workstation'].iloc[0] != 0:
    #         to_remove.append(traj_id)

    # df = df[~df['trajectory_id'].isin(to_remove)]

    # # Remove trajectories with less than 5 rows
    # trajectory_counts = df['trajectory_id'].value_counts()
    # valid_trajectories = trajectory_counts[trajectory_counts >= 5].index
    # df = df[df['trajectory_id'].isin(valid_trajectories)]

    # # Remove trajectories with consecutive points having the same position
    # valid_trajectories = []
    # for traj_id, group in trajectory_groups:
    #     unique_positions = group[['x', 'y']].drop_duplicates()
    #     if len(unique_positions) > 1:
    #         valid_trajectories.append(traj_id)
    
    # df = df[df['trajectory_id'].isin(valid_trajectories)]

    # # Ensure trajectory IDs are consecutive
    # unique_trajectory_ids = sorted(df['trajectory_id'].dropna().unique())
    # id_mapping = {old_id: new_id for new_id, old_id in enumerate(unique_trajectory_ids)}
    # df['trajectory_id'] = df['trajectory_id'].map(id_mapping)
    
    # Merge trajectories if start and goal match next trajectory
    merged_trajectories = [99999]
    while len(merged_trajectories) != 0:
        merged_trajectories = []
        # Ensure trajectory IDs are consecutive
        unique_trajectory_ids = sorted(df['trajectory_id'].dropna().unique())
        id_mapping = {old_id: new_id for new_id, old_id in enumerate(unique_trajectory_ids)}
        df['trajectory_id'] = df['trajectory_id'].map(id_mapping)

        trajectory_groups = df.groupby('trajectory_id')
        for traj_id, group in trajectory_groups:
            if traj_id in merged_trajectories:
                continue

            next_traj = df[df['trajectory_id'] == traj_id + 1]
            if not next_traj.empty:
                current_goal = group.iloc[-1]['goal']
                next_start = next_traj.iloc[0]['start']

                if group.iloc[0]['start'] == current_goal == next_start:
                    # Merge current and next trajectory
                    df.loc[next_traj.index, 'start'] = group.iloc[0]['start']
                    df.loc[group.index, 'goal'] = next_traj.iloc[-1]['goal']
                    # if traj_id == 10.0:
                    #     print(len(df.loc[next_traj.index, 'y']))
                    df.loc[next_traj.index, 'trajectory_id'] = traj_id
                    merged_trajectories.append(traj_id + 1)

    # print(merged_trajectories)
    
    # Ensure trajectory IDs are consecutive
    unique_trajectory_ids = sorted(df['trajectory_id'].dropna().unique())
    id_mapping = {old_id: new_id for new_id, old_id in enumerate(unique_trajectory_ids)}
    df['trajectory_id'] = df['trajectory_id'].map(id_mapping)

    # Save the labeled data to a new CSV
    df.to_csv(output_csv, index=False)

# Example usage
base_path = "data/clean_data"
person = "tuan"
process_trajectory_data(base_path + "/done_" + person + '_labeled.csv', base_path + "/ws_" + person + '_labeled.csv')

