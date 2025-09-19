import pandas as pd
import os
from sklearn.preprocessing import RobustScaler
import helpers
from scipy.stats import skew,boxcox
import numpy as np

def load_data(file) :
    folder_path = os.path.dirname(os.path.abspath(__file__))
    file_path   = os.path.join(folder_path, file)
    return pd.read_csv(file_path , on_bad_lines="skip").replace(r'"' , regex=True)

def cleaning(df : pd.DataFrame , digital_cols : list , corelation_cols : list , numerical_cols : list , skewed_cols : list , has_label = True) :
    #removing duplicates
    df.drop_duplicates()
    
    #dropping low corelation cols 
    df.drop(corelation_cols , inplace = True , axis = 1)

    #replacing rows with much missing info
    threshold  = 2 
    df = df[df.isnull().sum(axis=1) < threshold]
    
    #replacing numerical cols with their mean
    for col in numerical_cols :
        df[col] = df[col].fillna(df[col].mean())
    
    #replacing digital cols with digital random values 
    for col in digital_cols :
        missing_mask = df[col].isna()
        #this may affect bias in small data sets
        df.loc[missing_mask, col] = np.random.choice(df[col].dropna().values, size=missing_mask.sum(), replace=True)

    # fix skewness
    for col in skewed_cols:
        if col not in df.columns:
            continue

        technique = None
        minimum_skewness = skew(df[col])

        # log and sqrt are safe for zeros
        skew_list = {"log": skew(np.log1p(df[col])), "sqrt": skew(np.sqrt(df[col]))}

        # Box-Cox requires strictly positive and non-constant values
        if (df[col] > 0).all() and df[col].nunique() > 1:
            try:
                transformed, _ = boxcox(df[col])
                skew_list["boxcox"] = skew(transformed)
            except ValueError:
                pass  # skip Box-Cox if it fails for any reason

        # choose the technique with minimum skewness
        for tech, value in skew_list.items():
            if abs(value) < abs(minimum_skewness):
                minimum_skewness = value
                technique = tech

        # apply the best technique
        if technique == 'log':
            df[col] = np.log1p(df[col])  # log1p handles zeros
        elif technique == 'sqrt':
            df[col] = np.sqrt(df[col])
        elif technique == 'boxcox' and (df[col] > 0).all() and df[col].nunique() > 1:
            df[col], _ = boxcox(df[col])


    #removing outliers
    for col in numerical_cols :
        lower = df[col].quantile(0.005)
        upper = df[col].quantile(0.995)
    

    return df


def normalization (df: pd.DataFrame ,numerical_cols : list, digital_cols: list , target_col = None , scaler = None) :
    
    
    #applying scaler
    if target_col :
        y = df[target_col]
        X = df.drop(columns=[target_col])
    
    #setting scaler
    else :
        y = None
        X = df 
    
    #
    if scaler is None :
        scaler = RobustScaler()
        X_Scaled =  scaler.fit_transform(X[numerical_cols])
    else :
        X_Scaled = scaler.transform(X[numerical_cols])

    #rebuilding 
    X_Scaled_df = pd.DataFrame(X_Scaled , columns= numerical_cols , index= X.index)
    X_Scaled_df[digital_cols] = X[digital_cols].values

    #adding label back
    if target_col :
        X_Scaled_df[target_col] = y.values.ravel() #turning it to 1D

    return X_Scaled , y , scaler
        


    

