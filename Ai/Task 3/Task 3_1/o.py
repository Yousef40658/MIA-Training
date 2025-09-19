import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import scipy.stats as stats
from scipy.stats import skew
import os 
from preprocessing import *



df = load_data("train.csv")

# print(df.head())
# print(df.describe()) #some outliers  in height specially 

data = df["is_spam"]


#Plot histogram
plt.figure(figsize=(8,5))
sns.histplot(data, kde=True, bins=30)
plt.title("Histogram with KDE")



#Q-Q Plot to check skewness
plt.figure(figsize=(6,6))
stats.probplot(data, dist="norm", plot=plt)
plt.title("Q-Q Plot")
# plt.show()
print("Skewness:", skew(data))  #positive skew 

#Now that we need data is skewed we'll fix it in preprocessing

# #Corelations
df = df.drop(["message_id"], axis=1)
# correlations = df.corr()

# # Correlation of all features with Calories
# calories_corr = correlations['Calories'].sort_values(ascending=False)

# print(calories_corr)

#height , age , weight are barely effecting

"""
lets check if they together contribute
"""

# Create new feature 'person_body' as the product of Age, Weight, and Height
#df["person_body"] =  df["Weight"] * df["Height"] #still low , we can skip it
df["links-offer"] = df["num_links"] * df["has_offer"]
correlations = df.corr()
calories_corr = correlations['is_spam'].sort_values(ascending=False)

print(calories_corr)

#check all corelations
print(correlations)     #duration,body_temp are highly corelated with calories and heartrate
                        #
