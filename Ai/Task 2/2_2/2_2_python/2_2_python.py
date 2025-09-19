import pandas as pd
import matplotlib.pyplot as plt
import numpy.linalg as LA
import helpers
import os
import csv
import time

start = time.perf_counter()
#------------------
#Loading file
#------------------
script_dir = os.path.dirname(os.path.abspath(__file__))
path = os.path.join(script_dir, "CARS_1.csv")
df = pd.read_csv(path, on_bad_lines='skip').replace(r'"', '', regex=True)
df = pd.concat([df]*100000, ignore_index=True)



# Simple test (increase data rows to ~5000 manually)


#------------------
#Cleaning
#------------------
df.columns = df.columns.str.strip()                                   #remove spaces from columns names

#dropping non tech columns
df.drop(["reviews_count", "fuel_type", "seating_capacity", "rating","fuel_tank_capacity"], axis=1, inplace=True)

#drop columns with more than 80% empty values before anything
bad_col_percent = df.apply(helpers.empty_percentge)
bad_cols_to_drop = bad_col_percent[bad_col_percent > 0.75].index     #increase threshold to be more strict
df.drop(columns=bad_cols_to_drop, inplace=True)

#remove nulls (numericals) per row with more than "threshold" missing values
numerial_cols = ["engine_displacement", "no_cylinder","starting_price","ending_price"
                           ,"max_torque_nm" , "max_torque_rpm", "max_power_bhp", "max_power_rp"]
threshold  = 4              #threshold to decide removing or imputating
df = df[df.isnull().sum(axis=1) < threshold]    

#replace empty numericals with mean value of their col 
for col in numerial_cols :
    mean_value = df[col].mean()
    df[col] = df[col].fillna(mean_value)     

#Remove rows with missing texts
text_cols = df.select_dtypes(include=['object']).columns 
for col in text_cols:
    df[col] = df[col].str.lower().str.strip()  #lowercasing all characters
df = df[df[text_cols].notnull().all(axis=1)]

#remove dupliactes exceluding the id -automatically generated-
df = df.drop_duplicates(subset=df.columns.difference(['resultId'])) 

#reset index
df = df.reset_index(drop=True)

#Saving cleaned data


#------------------
#Standardization using pure python 
#------------------
raw_data = df[numerial_cols].values.tolist()  # list of lists
data_matrix = helpers.Matrix(raw_data).standardize()

# Z-score filtering
z_filtered_matrix, keep_indices = data_matrix.zscore_filter(threshold=3)  # new function with tracking

# Apply filtering to original DataFrame to align labels
df = df.iloc[keep_indices].reset_index(drop=True)

# Continue with PCA
cov_matrix = z_filtered_matrix.covariance_matrix()

#---------------
#Eigen decomposition using numpy
#---------------

eigenvalues, eigenvectors = LA.eig(cov_matrix.tolist())  #input as list of lists
#print(eigenvalues,eigenvectors)

#---------------
#Sorting eigenvalue/vectors
#---------------
eig_pairs = [(eigenvalues[i], [row[i] for row in eigenvectors]) for i in range(len(eigenvalues))]
eig_pairs.sort(key=lambda x: x[0], reverse=True) #sort pairs decending 

# Unzip sorted pairs
eigenvalues_sorted = [pair[0] for pair in eig_pairs]
eigenvectors_sorted = [pair[1] for pair in eig_pairs]

#---------------
#Scree plot
#---------------
plt.figure(figsize=(8, 5))
plt.plot(range(1, len(eigenvalues_sorted) + 1), eigenvalues_sorted, marker='o')
plt.title('Scree Plot by python')
plt.xlabel('Principal Component')
plt.ylabel('Eigenvalue')
plt.xticks(range(1, len(eigenvalues_sorted) + 1))
plt.grid(True)
# plt.show()

#---------------
#Data projection using Matrix class
#---------------
n_dims = 2
top_components = [list(col) for col in zip(*eigenvectors_sorted[:n_dims])]
data_pca = z_filtered_matrix.project(top_components, n_dims).tolist()


#------------------
#PCA DataFrame
#------------------
pca_df = pd.DataFrame(data_pca, columns=[f'PC{i+1}' for i in range(n_dims)])
pca_df["Class"] = df["body_type"].values  # align with PCA points
pca_df.to_csv("PCA.csv" , index= False)

#------------------
#Visualization
#------------------

#rotating it -_- i've spent so much time before relizing its just flipped and it doesn't matter
pca_df["PC1"] *= -1
pca_df["PC2"] *= -1
plt.figure(figsize=(14, 8))  # Wider figure to fit legend

# Plot each class with a different color
for label in pca_df["Class"].unique():
    subset = pca_df[pca_df["Class"] == label]
    plt.scatter(subset["PC1"], subset["PC2"], label=label, alpha=0.5)

plt.xlabel("PC1")
plt.ylabel("PC2")
plt.title("PCA of Cars (By python)")

# Add legend outside the plot on the right
plt.legend(title="Car Class", loc='center left', bbox_to_anchor=(1, 0.5))
plt.subplots_adjust(right=0.75)
plt.grid(True)
plt.tight_layout()
# plt.show()

end = time.perf_counter()
print ("\ntime by python = : " , end  - start)