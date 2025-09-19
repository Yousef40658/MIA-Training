import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from sklearn.preprocessing import StandardScaler
import helpers
import os
import time
import csv

start = time.perf_counter()

#------------------
#Loading file
#------------------
script_dir = os.path.dirname(os.path.abspath(__file__))
path = os.path.join(script_dir, "CARS_1.csv")
df = pd.read_csv(path, on_bad_lines='skip').replace(r'"', '', regex=True)
df = pd.concat([df]*50000, ignore_index=True)


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
df=df[df.isnull().sum(axis=1) < threshold]    

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
# df.to_csv("cleaned_results.csv", index=False, quoting=csv.QUOTE_MINIMAL)

####
#checking data before standardizing 
# df[numerial_cols].hist(bins=30, figsize=(12, 10))
# plt.tight_layout()
# plt.show()
####

#------------------
#Standardization 
#------------------

scaler = StandardScaler()
df[numerial_cols] = scaler.fit_transform(df[numerial_cols])

#removing outliers
z_scores = np.abs((df[numerial_cols] - df[numerial_cols].mean()) / df[numerial_cols].std())
z_threshold = 3 

df = df[(z_scores < z_threshold).all(axis=1)] #99.7% values in case of normal distribution

# df.to_csv("standardized.csv", index=False, quoting=csv.QUOTE_MINIMAL)

#------------------
#Covariance Matrix
#------------------
cov_matrix = np.cov(df[numerial_cols].values , rowvar=False) #false -> each col is a feature

eigenvalues,eigenvectors = np.linalg.eig(cov_matrix)
#print(eigenvalues,eigenvectors)

sorted_indices = np.argsort(eigenvalues)[::-1] #np.argsort returns the index of each value should be to sort ascending
eigenvalues_sorted = eigenvalues[sorted_indices]
eigenvectors_sorted = eigenvectors[:, sorted_indices]
# print(eigenvalues_sorted,eigenvectors_sorted)

#according to -https://communities.sas.com/t5/SAS-Communities-Library/How-many-principal-components-should-I-keep-Part-1-common/ta-p/948949#:~:text=If%20we%20want%20to%20save,of%202.9%20would%20be%20kept.-
# Scree plot of eigenvalues
plt.figure(figsize=(8, 5))
plt.plot(range(1, len(eigenvalues_sorted) + 1), eigenvalues_sorted, marker='o')
plt.title('Scree Plots by numpy')
plt.xlabel('Principal Component')
plt.ylabel('Eigenvalue')
plt.xticks(range(1, len(eigenvalues_sorted) + 1))
plt.grid(True)
# plt.show()
#3 dim for more accuracy , 2 for dimensionality reduction 
#more than 3 feels an overkill

n_dims = 2 
#-------------
#data projection
#-------------

projection_matrix = eigenvectors_sorted[:,:n_dims]
data_standardized = df[numerial_cols].values
data_pca = np.dot(data_standardized, projection_matrix) #linaer transformation

pca_df = pd.DataFrame(data_pca, columns=[f'PC{i+1}' for i in range(n_dims)])
pca_df["Class"] = df["body_type"].values  # align with PCA points
# pca_df.to_csv("PCA.csv" , index= False)


#visualize 
plt.figure(figsize=(14, 8))  # Wider figure to fit legend

# Plot each class with a different color
for label in pca_df["Class"].unique():
    subset = pca_df[pca_df["Class"] == label]
    plt.scatter(subset["PC1"], subset["PC2"], label=label, alpha=0.7)

plt.xlabel("PC1")
plt.ylabel("PC2")
plt.title("PCA of Cars (by numpy)")

# Add legend outside the plot on the right
plt.legend(title="Car Class", loc='center left', bbox_to_anchor=(1, 0.5))

# Adjust layout to make space for legend
plt.subplots_adjust(right=0.75)  # ← this creates extra space on the right

plt.grid(True)
plt.tight_layout()
# plt.show()
# plt.savefig("pca_plot_with_classes.png", dpi=300, bbox_inches="tight")
end = time.perf_counter()

print ("\ntime by numpy = : " , end  - start)