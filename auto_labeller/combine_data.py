import os
import pandas as pd

# Function to gather and combine CSV files while ensuring unique trajectory IDs
def combine_csv_files(input_folder, output_file):
    combined_df = pd.DataFrame()
    max_trajectory_id = 0  # To track the largest trajectory_id encountered so far

    # Iterate through all CSV files in the folder
    for filename in os.listdir(input_folder):
        if filename.endswith('.csv'):
            filepath = os.path.join(input_folder, filename)

            # Read the current CSV file
            df = pd.read_csv(filepath)

            # Adjust trajectory_id to ensure uniqueness
            if 'trajectory_id' in df.columns:
                df['trajectory_id'] = df['trajectory_id'] + max_trajectory_id
                max_trajectory_id = df['trajectory_id'].max() + 1

            # Append the data to the combined dataframe
            combined_df = pd.concat([combined_df, df], ignore_index=True)

    # Save the combined dataframe to the output file
    combined_df.to_csv(output_file, index=False)
    print(f"Combined data saved to {output_file}")

# Example usage
input_folder = 'data/clean_data/labelled'  # Replace with the path to your folder containing CSV files
output_file = 'data/clean_data/data.csv'  # Replace with the desired output file path
combine_csv_files(input_folder, output_file)