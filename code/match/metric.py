import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

from sklearn.metrics import mean_absolute_error, mean_squared_error


# Load data from Excel file
model_name = r"Raw_Incomplete_Trees"
excel_file = rf"E:\Result\LLC_02022022\Row13\{model_name}\Primary\TreeQSM\QSM\3DGAC_TreeQSM_Matched_Branch.xlsx"
dfs = pd.read_excel(excel_file, sheet_name=None)

# Initialize lists to store results
results_diameter = []
results_angle = []

# Iterate through each worksheet
for sheet_name, df in dfs.items():
    # Group data by 'Filename' (tree identifier)
    grouped = df.groupby('Filename')

    # Compute metrics for each group (tree)
    for tree_name, group in grouped:
        # Remove NaNs
        group = group.dropna()
        # Compute MAE for branch diameter
        mae_diameter = mean_absolute_error(group['Manual_Branch_Diameter-mm'], group['Primary_Branch_Diameter-mm_df2'])
        # Compute MAPE for branch diameter
        mape_diameter = np.mean(np.abs((group['Manual_Branch_Diameter-mm'] - group['Primary_Branch_Diameter-mm_df2']) / group['Manual_Branch_Diameter-mm'])) * 100
        # Compute RMSE for branch diameter
        rmse_diameter = np.sqrt(mean_squared_error(group['Manual_Branch_Diameter-mm'], group['Primary_Branch_Diameter-mm_df2']))

        # Compute MAE for branch angle
        mae_angle = mean_absolute_error(group['Manual_Vertical_Crotch_Angle-Degree'], group['Vertical_Croth_Angle-Degree_df2'])
        # Compute MAPE for branch angle
        mape_angle = np.mean(np.abs((group['Manual_Vertical_Crotch_Angle-Degree'] - group['Vertical_Croth_Angle-Degree_df2']) / group['Manual_Vertical_Crotch_Angle-Degree'])) * 100
        # Compute RMSE for branch angle
        rmse_angle = np.sqrt(mean_squared_error(group['Manual_Vertical_Crotch_Angle-Degree'], group['Vertical_Croth_Angle-Degree_df2']))

        # Append results to lists
        results_diameter.append({'Tree': tree_name, 'MAE_Diameter': mae_diameter, 'MAPE_Diameter': mape_diameter, 'RMSE_Diameter': rmse_diameter})
        results_angle.append({'Tree': tree_name, 'MAE_Angle': mae_angle, 'MAPE_Angle': mape_angle, 'RMSE_Angle': rmse_angle})

# Convert lists to DataFrames
results_diameter_df = pd.DataFrame(results_diameter)
results_angle_df = pd.DataFrame(results_angle)

result_filepath = rf"E:\Result\LLC_02022022\Row13\{model_name}\Primary\TreeQSM\QSM\Evaluation.xlsx"
# Create an ExcelWriter object
with pd.ExcelWriter(result_filepath) as writer:
    results_diameter_df.to_excel(writer, sheet_name='Diameter', index=False)
    results_angle_df.to_excel(writer, sheet_name='Angle', index=False)

# Compute tree-wise mean MAE, MAPE, and RMSE
mean_results_diameter = results_diameter_df.groupby('Tree').mean()
mean_results_angle = results_angle_df.groupby('Tree').mean()

print(mean_results_diameter)

# Define the number of rows and columns for the subplot grid
num_rows = 2
num_cols = 5

# Create a figure and axis objects
fig, axes = plt.subplots(num_rows, num_cols, figsize=(15, 8))

# Flatten the axes array for easy iteration
axes = axes.flatten()

# Plot histograms for MAE, MAPE, and RMSE for each tree
for i, (tree_name, group) in enumerate(results_diameter_df.groupby('Tree')):
    # Plot histogram for MAE
    axes[i].hist(group['MAE_Diameter'], bins=10, alpha=0.7, color='blue', label='MAE')
    axes[i].set_title(f'Tree: {tree_name}')
    axes[i].set_xlabel('MAE')
    axes[i].set_ylabel('Frequency')
    axes[i].legend()
# Add spacing between subplots
plt.tight_layout()
result_filepath = rf"E:\Result\LLC_02022022\Row13\{model_name}\Primary\TreeQSM\QSM\Branch_Diameter_Hist.png"
plt.savefig(result_filepath, dpi=300)
