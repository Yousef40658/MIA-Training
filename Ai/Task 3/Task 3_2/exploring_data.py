# import pandas as pd
# import numpy as np
# import matplotlib.pyplot as plt
# import seaborn as sns
# import scipy.stats as stats
# from scipy.stats import skew
# import os 
from preprocessing import *
from preprocessing import df_id



# df = load_data("train.csv")

# # print(df.head())
# # print(df.describe()) #some outliers  in height specially 

# data = df["Calories"]


# #Plot histogram
# # plt.figure(figsize=(8,5))
# # sns.histplot(data, kde=True, bins=30)
# # plt.title("Histogram with KDE")



# #Q-Q Plot to check skewness
# # plt.figure(figsize=(6,6))
# # stats.probplot(data, dist="norm", plot=plt)
# # plt.title("Q-Q Plot")
# # # plt.show()
# # print("Skewness:", skew(data))  #positive skew 

# #Now that we need data is skewed we'll fix it in preprocessing

# # #Corelations
# df = df.drop(["Sex", "id"], axis=1)
# # correlations = df.corr()

# # # Correlation of all features with Calories
# # calories_corr = correlations['Calories'].sort_values(ascending=False)

# # print(calories_corr)

# #height , age , weight are barely effecting

# """
# lets check if they together contribute
# """

# # Create new feature 'person_body' as the product of Age, Weight, and Height
# df["person_body"] =  df["Weight"] * df["Height"] #still low , we can skip it
# df["density?"] = df["Weight"] / df["Height"]

# correlations = df.corr()

# calories_corr = correlations['Calories'].sort_values(ascending=False)

# print(calories_corr)

# #check all corelations
# print(correlations,"\n\n")     #duration,body_temp are highly corelated with calories and heartrate
                        

# #---------------
# #OUTLIERS
# #--------------
# import pandas as pd

# def detect_outliers_iqr(df, cols):
#     outlier_indices = set()
#     for col in cols:
#         Q1 = df[col].quantile(0.25)
#         Q3 = df[col].quantile(0.75)
#         IQR = Q3 - Q1
#         lower_bound = Q1 - 1.5 * IQR
#         upper_bound = Q3 + 1.5 * IQR
#         outliers_col = df[(df[col] < lower_bound) | (df[col] > upper_bound)].index
#         print(f"{col}: {len(outliers_col)} outliers")
#         outlier_indices.update(outliers_col)
#     return list(outlier_indices)

# # numeric_cols = df.select_dtypes(include=["number"]).columns.drop("Calories")  # exclude target

# # outliers = detect_outliers_iqr(df, numeric_cols)
# # print(f"Total unique outliers across features: {len(outliers)}")              #body_temp has most outliers

# # # Summary stats for Body_Temp
# # print(df["Body_Temp"].describe())

# # # Histogram to visualize distribution                           #all temp values are close together , no need to cut outliers
# # plt.figure(figsize=(10, 5))
# # sns.histplot(df["Body_Temp"], bins=100, kde=True)
# # plt.title("Distribution of Body_Temp")
# # plt.xlabel("Body_Temp")
# # plt.ylabel("Frequency")
# # plt.show()

# df = load_data("train.csv")
# # print(df.describe() , "\n\n")

# df = cleaning(df)

# # helpers.plot_all_distributions(df)

# df_Scaled,_,_ = normalization(df)

# # helpers.plot_all_distributions(df_Scaled)

# print (df_Scaled.head(15))
# print (df_Scaled.describe())

df = pd.read_csv("test.csv")
df = cleaning(df)

print (df_id)