import pandas as pd
import numpy as np
import os

# Configuration: Set to True for 2011–2023, False for 2023 only
USE_FULL_RANGE = True

# Define the 13 parameters
parameters = [
    'A',        # Front End Length (Front hood length)
    #'B'        # Rear End Lenght (no longer using B, this variable is not stable)
    'C',        # Window Height
    'D',        # (Door - Window) Height
    'E',        # Roof Width
    'F',        # Front Overhang
    'G',        # Rear Overhang
    'OH ',       # Overal Height
    'OL',       # Overal Length
    'OW',       # Overal Width
    'TWF',      # Track Width Front
    'TWR '      # Track Width Rear
]

# Load CVS dataset
if USE_FULL_RANGE:
    _HERE = os.path.dirname(os.path.abspath(__file__))
    csv_files = [os.path.join(_HERE, "cars_database", f"{year}_en.csv") for year in range(2011, 2024)]
    dfs = []
    for csv_file in csv_files:
        if os.path.exists(csv_file):
            try:
                df = pd.read_csv(csv_file)
                dfs.append(df)
            except UnicodeDecodeError:
                # Fall back to latin1 if UTF-8 fails
                try:
                    df = pd.read_csv(csv_file, encoding='latin1')
                    print(f"Loaded {csv_file} with latin1 encoding due to UTF-8 failure.")
                    dfs.append(df)
                except Exception as e:
                    print(f"Error loading {csv_file}: {e}")
            except Exception as e:
                print(f"Error loading {csv_file}: {e}")
        else:
            print(f"Warning: {csv_file} not found.")
    if not dfs:
        print("Error: No CSV files found for 2011–2023.")
        exit(1)
    df = pd.concat(dfs, ignore_index=True)
else:
    csv_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "cars_database", "2023_en.csv")
    try:
        df = pd.read_csv(csv_file)
    except FileNotFoundError:
        print("Error: CSV file not found. Please check the file path.")
        exit(1)

data = df[parameters].dropna()

# Convert to numeric, handle missing values
for col in parameters:
    if col in df.columns:
        df[col] = pd.to_numeric(df[col], errors='coerce')

# Compute mean values
mean_values = {}
for col in parameters:
    if col in df.columns:
        mean_values[col] = round(df[col].mean(), 1)

# Save mean car shape to CSV
output_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "mean_car_shape_2011_2023.csv" if USE_FULL_RANGE else "mean_car_shape_2023.csv")
mean_car_shape = pd.DataFrame([mean_values], index=['Mean Car Shape'])
mean_car_shape.to_csv(output_file, index=True)
print(f"Mean car shape saved to {output_file}")
