"""
This scrip aims to use Hungarian algorithm for the assignment of branches from 3DGAC to AdQSM (1-to-1 guaranteed).
It relies on the manually produced ground-truth data to automatically map branches.
"""

import pandas as pd
pd.options.mode.copy_on_write = True
from scipy.optimize import linear_sum_assignment


# Load data from local CSV files
df1 = pd.read_csv(r"D:\Data\Apple_Orchard\Lailiang_Cheng\Branch_GT.csv")
model_names = [r"AdaPoinTr_LT2000_FT", "AdaPoinTr_LT2000-04132024_FT"]

for model_name in model_names:
    df2 = pd.read_csv(
        rf"E:\Result\LLC_02022022\Row13_Tree\{model_name}\AppleQSM\Characterization\Branch_Trait.csv"
    )
    # Extract the features for comparison
    # features = ["Vertical_Croth_Angle-Degree", "Branch_Height-cm"]
    features = ["Branch_Height-cm"]

    matched_filepath = rf"E:\Result\LLC_02022022\Row13_Tree\{model_name}\AppleQSM\Characterization\3DGAC_Matched_Branch_Trait_Height.csv"

    # Initialize an empty DataFrame to store the results
    result_df = pd.DataFrame()

    # Iterate over unique tree names
    for tree_name in pd.concat([df1["Filename"], df2["Filename"]]).unique():
        # Filter rows for the current tree from both dataframes
        tree_df1 = df1[df1["Filename"] == tree_name]
        tree_df2 = df2[df2["Filename"] == tree_name]

        tree_df2 = tree_df2.reset_index(drop=True)

        # Check if there are rows for the current tree in both dataframes
        if not tree_df1.empty and not tree_df2.empty:
            # Create a cost matrix based on the absolute difference in both features
            cost_matrix = abs(
                tree_df1[features].values[:, None, :] - tree_df2[features].values
            )
            cost_matrix = cost_matrix.sum(
                axis=2
            )  # Sum the absolute differences across features

            # Solve the assignment problem using the Hungarian algorithm
            row_indices, col_indices = linear_sum_assignment(cost_matrix)

            ### Attention ###
            ### might need to change columns
            # Create new columns in tree_df1 and fill in values from tree_df2 based on the optimal assignment
            for column in tree_df2.columns[1:]:

                # Ensure lengths match before assignment
                if len(col_indices) == len(tree_df1):
                    tree_df1[f"{column}_df2"] = tree_df2.loc[col_indices, column].values
                else:
                    # Handle case where lengths don't match (e.g., skip assignment or fill missing values)
                    # Here, we're filling missing values with NaNs
                    tree_df1[f"{column}_df2"] = [
                        (
                            tree_df2.loc[idx, column]
                            if idx in col_indices
                            else float("nan")
                        )
                        for idx in range(len(tree_df1))
                    ]

            # Append the results for the current tree to the final result DataFrame
            result_df = pd.concat([result_df, tree_df1])

    result_df.to_csv(matched_filepath, index=False)
