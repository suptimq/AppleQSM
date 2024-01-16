"""
This script aims to format the raw output from AdQSM and produce a nice CSV file
"""
import pandas as pd


def extract_info(tree_data, col=2):
    # Split the lines and extract diameter for order 1 branches
    lines = tree_data.split('\n')
    trait_list = [float(line.split()[col]) for line in lines[2:] if line.strip() and int(line.split()[0]) == 1]
    return trait_list

### AdQSM
with open(r"D:\Code\Apple_Crop_Potential_Prediction\data\row13\AdQSM\CleanBranchStructure.txt", 'r') as file:
    all_tree_data = file.read()

# Split the data into individual trees
tree_data_list = all_tree_data.split('DataName: ')[1:]

COLS = ['Filename', 'Branch ID', 'Vertical_Croth_Angle-Degree', 'Primary_Branch_Diameter-mm', 'Branch_Height-cm', 'Branch_Length-cm']
df = pd.DataFrame(columns=COLS)

# For each individual tree
for tree_data in tree_data_list:
    # Extract information
    diameter_list = [x * 1000 for x in extract_info(tree_data, col=2)]
    length_list = [x * 100 for x in extract_info(tree_data, col=5)]
    height_list = [x * 100 for x in extract_info(tree_data, col=6)]
    angle_list = [x for x in extract_info(tree_data, col=7)]

    # Get the filename (DataName)
    filename = tree_data.splitlines()[0].strip()

    # Get the branch IDs
    branch_ids = list(range(1, len(diameter_list) + 1))

    # Create a DataFrame for the current tree
    tree_df = pd.DataFrame({
        'Filename': [filename] * len(branch_ids),
        'Branch ID': branch_ids,
        'Vertical_Croth_Angle-Degree': angle_list,
        'Primary_Branch_Radius-mm': diameter_list,
        'Branch_Height-cm': height_list,
        'Branch_Length-cm': length_list
    })

    # Append the current tree DataFrame to the main DataFrame
    df = pd.concat([df, tree_df], ignore_index=True)

df.to_csv('AdQSM_Result.csv', index=False)