"""
This scrip aims to use Hungarian algorithm for the assignment of branches from 3DGAC to AdQSM (1-to-1 guaranteed)
"""
import pandas as pd
from scipy.optimize import linear_sum_assignment

model_name = r"Generator2-AdaPoinTr-Skeleton-GAN_LTB81-v4_CDL1_SkelLoss-Supervised-0.01_Finetune"

# Load data from local CSV files
df1 = pd.read_csv(r"D:\Code\Apple_Crop_Potential_Prediction\data\row13\AdQSM\3DGAC_Result.csv")
dfs = pd.read_excel(rf"E:\Result\LLC_02022022\Row13\{model_name}\Primary\TreeQSM\QSM\TreeQSM_Result.xlsx", sheet_name=None)

# Extract the features for comparison
features = ['Vertical_Croth_Angle-Degree', 'Branch_Height-cm']

counter = 1
matched_filepath = rf"E:\Result\LLC_02022022\Row13\{model_name}\Primary\TreeQSM\QSM\3DGAC_TreeQSM_Matched_Branch.xlsx"
# Create an ExcelWriter object
with pd.ExcelWriter(matched_filepath) as writer:
    for sheet_name, df2 in dfs.items():

        # Initialize an empty DataFrame to store the results
        result_df = pd.DataFrame()

        # Iterate over unique tree names
        for tree_name in pd.concat([df1['Filename'], df2['Filename']]).unique():
            # Filter rows for the current tree from both dataframes
            tree_df1 = df1[df1['Filename'] == tree_name]
            tree_df2 = df2[df2['Filename'] == tree_name]
            tree_df2 = tree_df2.reset_index(drop=True)

            tree_df1 = tree_df1.dropna()
            
            # Check if there are rows for the current tree in both dataframes
            if not tree_df1.empty and not tree_df2.empty:
                # Create a cost matrix based on the absolute difference in both features
                cost_matrix = abs(tree_df1[features].values[:, None, :] - tree_df2[features].values)
                cost_matrix = cost_matrix.sum(axis=2)  # Sum the absolute differences across features

                # Solve the assignment problem using the Hungarian algorithm
                row_indices, col_indices = linear_sum_assignment(cost_matrix)

                # Create new columns in tree_df1 and fill in values from tree_df2 based on the optimal assignment
                for column in tree_df2.columns[:-1]:
                    # # TODO - Handle when len(tree_df1) > len(tree_df2)
                    # tree_df1[f'{column}_df2'] = tree_df2.loc[col_indices, column].values

                    # Ensure lengths match before assignment
                    if len(col_indices) == len(tree_df1):
                        tree_df1[f'{column}_df2'] = tree_df2.loc[col_indices, column].values
                    else:
                        # Handle case where lengths don't match (e.g., skip assignment or fill missing values)
                        # Here, we're filling missing values with NaNs
                        tree_df1[f'{column}_df2'] = [tree_df2.loc[idx, column] if idx in col_indices else float('nan') for idx in range(len(tree_df1))]

                # Append the results for the current tree to the final result DataFrame
                result_df = pd.concat([result_df, tree_df1])


        # Write each DataFrame to a separate worksheet
        result_df.to_excel(writer, sheet_name=f'Sheet{counter}', index=False)
        counter = counter + 1