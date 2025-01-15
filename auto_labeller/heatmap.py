import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def plot_heatmap_from_csv_folder(folder_path, output_file="heatmap.png", bins=100, max_color=1000):
    """
    Reads all CSV files in the folder, extracts x and y coordinates, and plots a heatmap.
    
    Parameters:
    folder_path (str): Path to the folder containing CSV files.
    output_file (str): Path to save the heatmap image.
    bins (int): Number of bins for the heatmap.
    max_color (int): Maximum number of points per bin for the color scale.
    """
    x_coords = []
    y_coords = []
    
    # Loop through all files in the folder
    for file_name in os.listdir(folder_path):
        if file_name.endswith('.csv'):  # Process only CSV files
            file_path = os.path.join(folder_path, file_name)
            print(f"Processing: {file_path}")
            
            try:
                # Read the CSV file
                df = pd.read_csv(file_path)
                
                # Check if x and y columns exist
                if 'x' in df.columns and 'y' in df.columns:
                    x_coords.extend(df['y'].values)
                    y_coords.extend(-df['x'].values)
                else:
                    print(f"Skipped {file_name}: Missing 'x' or 'y' columns.")
            except Exception as e:
                print(f"Error processing {file_name}: {e}")
    
    # Plot the heatmap
    plt.figure(figsize=(15, 6))
    plt.xlim(-12.5, 13)  # Fixed Y Position range
    plt.ylim(-4, 9)        # Fixed flipped X Position range
    plt.gca().set_aspect('equal')
    
    # ax.set_xlim(-12, -11)  # Fixed Y Position range
    # ax.set_ylim(-2, -0.5)        # Fixed flipped X Position range

    # plt.aspect('equal', adjustable='box')
    # hb = plt.scatter(x_coords, y_coords, c="blue", alpha=0.008, marker='s')
    # H, yedges, xedges = np.histogram2d(y_coords, x_coords, bins=75)
    # hb = plt.pcolormesh(xedges, yedges, H, cmap='rainbow', vmax=120)
    hb = plt.hexbin(
        x_coords, 
        y_coords, 
        gridsize=bins, 
        cmap='Blues', 
        mincnt=1, 
        vmax=max_color
    )
    plt.grid(color='white', linestyle='-', linewidth=2, alpha=0.2)
    plt.colorbar(label='Number of Points')
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.title(f'Heatmap of X and Y Coordinates (Max Color at {max_color})')
    
    # Save and show the heatmap
    plt.savefig(output_file, dpi=300)
    plt.show()
    print(f"Heatmap saved to: {output_file}")

# Example usage
folder_path = "data/clean_data"  # Replace with your folder path
output_file = "heatmap.png"
plot_heatmap_from_csv_folder(folder_path, output_file, bins=75, max_color=120)
