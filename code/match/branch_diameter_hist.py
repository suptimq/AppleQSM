"""
This script plots a primary branch diameter distribution figure containing the overlaid of 3DGAC, AdQSM, and GT results
"""
import pandas as pd
import matplotlib.pyplot as plt


def extract_diameter(tree_data):
    # Split the lines and extract diameter for order 1 branches
    lines = tree_data.split('\n')
    diameters = [float(line.split()[2]) for line in lines[2:] if line.strip() and int(line.split()[0]) == 1]
    return diameters


file_path = r"D:\Code\Apple_Crop_Potential_Prediction\data\row13\AdQSM\3DGAC_Result.csv"
df = pd.read_csv(file_path)

column_name = 'Primary_Branch_Radius-mm' # 'Primary_Branch_Radius-mm' or 'Diameter (mm)'

# Plotting tree-wise histogram
num_rows = 2
num_cols = 5
fig, axes = plt.subplots(num_rows, num_cols, figsize=(15, 6))

# Plotting tree-wise histogram
i = 1
for (tree, group), ax in zip(df.groupby('Filename'), axes.flatten()):

    ax.hist(group[column_name]*2, bins=10, alpha=0.7, label='Ours')

    i = i + 1

### GT
file_path = r"D:\Data\Apple_Orchard\Lailiang_Cheng\Branch_GT.xlsx"
df = pd.read_excel(file_path)

column_name = 'Diameter (mm)' # 'Primary_Branch_Radius-mm' or 'Diameter (mm)'

i = 1
for (tree, group), ax in zip(df.groupby('Filename'), axes.flatten()):

    ax.hist(group[column_name], bins=10, alpha=0.5, label='GT')

    i = i + 1


### AdQSM
with open(r"D:\Code\Apple_Crop_Potential_Prediction\data\row13\AdQSM\CleanBranchStructure.txt", 'r') as file:
    all_tree_data = file.read()

# Split the data into individual trees
tree_data_list = all_tree_data.split('DataName: ')[1:]

for i, (tree_data, ax) in enumerate(zip(tree_data_list, axes.flatten()), start=1):
    diameter_list = [x*1000 for x in extract_diameter(tree_data)]
    print(f"Tree {i} - Diameters of Order 1 Branches:", diameter_list)

    # Plot the distribution for each tree
    ax.hist(diameter_list, bins=10, alpha=0.5, label='AdQSM')
    ax.set_title(f"Tree {i}", fontsize=14, fontweight='bold')

    # Set common labels
    ax.set_xlabel("Diameter (mm)", fontsize=14)
    ax.set_ylabel("Frequency", fontsize=14)


axes[0, 0].legend()

plt.tight_layout()
plt.savefig('test3.png', dpi=300)