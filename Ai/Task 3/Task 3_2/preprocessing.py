import pandas as pd 
import os
from sklearn.preprocessing import RobustScaler
import helpers
from scipy.stats import skew,boxcox
import numpy as np
#---------------
#Helpers
#---------------
operation_on_target = ""
df_id = None
#loading
def load_data(file) :
    folder_path = os.path.dirname(os.path.abspath(__file__))
    file_path = os.path.join(folder_path,file)
    return pd.read_csv(file_path, on_bad_lines= "skip").replace(r'"', '', regex=True)

#Cleaning
def cleaning(df, has_label=True):

    binary_cols = ['Sex']  # keep message_id in test

    #remove low correlation cols
    df.drop(["Weight","Height"] , inplace = True , axis = 1 )

    #Encoding text to numbers
    df["Sex"] = df["Sex"].map({"male": 0, "female": 1}) #male into 0 women into 1

    #all are integers now
    for col in df.columns:
        df[col] = df[col].apply(helpers.try_word_to_num)
        df[col] = pd.to_numeric(df[col], errors='coerce')

    #replacing rows with much missing info
    threshold = 2
    df = df[df.isnull().sum(axis=1) < threshold] 

    #replacing missing numericals with their mean
    for col in df.columns:
        if col not in binary_cols and col != "id":
            df[col] = df[col].fillna(df[col].mean()) #all numbers from the type conversion or null , no need to lowercase , #that removed 120 k :O
        if col in binary_cols :
            missing_mask = df[col].isna()
            df.loc[missing_mask, col] = np.random.choice([0, 1], size=missing_mask.sum()) #in large datasets it won't effect bias

    #removing dumplicates (duplicates in values)
    if "id" in df.columns and has_label:    #keep duplicates to have the same amount of required rows
        df = df.drop_duplicates(subset=[col for col in df.columns if col != "id"])
    
    #fix skewness
    for col in df.columns :
        technique = None
        if col not in binary_cols and not "id" :
            minimum_skewness = skew(df[col]) 

            if abs(minimum_skewness) > 0.5 : #apply skewness fixes when its above 0.5 skew left or right
                skew_list = {"log" : skew(np.log(df[col])) , "sqrt" : skew(np.sqrt(df[col]))}
              
                if (df[col] > 0).all():       
                    transformed, _ = boxcox(df[col])                   #skip or shift values before boxcox --crushes when 0--c 
                    skew_list["boxcox"] = skew(transformed)            #appends it , boxcox returns transofmered data and lambda value

                for tech,value in skew_list.items() :
                    if abs(value) < abs(minimum_skewness) :            #find minimum skewness 
                        minimum_skewness = value 
                        technique = tech

                if technique =='log' :
                    df[col] = np.log1p(df[col])
                elif technique == "sqrt" :
                    df[col] = np.sqrt(df[col])
                elif technique == 'boxcox' :
                    df[col],_ = boxcox(df[col])
            
            if col == "Calories" :
                global operation_on_target          #keep it to undo the effect of sqrt later
                operation_on_target = technique     #sqrt in this case
                # print(technique)
        
    #removing outliers 
    for col in [c for c in df.columns if c != "id"]:
        if col != 'Calories' and col not in binary_cols:
            lower = df[col].quantile(0.001)
            upper = df[col].quantile(0.999)
            df[col] = df[col].clip(lower, upper)

    if has_label :
        if "id" in df.columns:
            df = df.drop("id", axis=1)
    else :
        global df_id
        if "id" in df.columns:
            df_id = df["id"]            #keep it if its a test
            print(df_id)
            print("\n\n")
            df = df.drop("id", axis=1)
            print(df_id)
    return df


###########################
def normalization (df,has_label = True,scaler = None):
    #Cols
    scaled_cols = ['Age', 'Duration','Heart_Rate','Body_Temp']
    binary_cols = ['Sex']
    target_col =  'Calories'
    
    #to avoid errors
    scaled_cols = [col for col in scaled_cols if col in df.columns]

    #Sperate accodring to if test of train
    if has_label :
        y = df[target_col]
        X = df.drop(columns = [target_col])
    else :
        y = None
        X = df 
    
    #applying scaler or setting it 
    if scaler is None :
        scaler = RobustScaler()
        X_scaled = scaler.fit_transform(X[scaled_cols])
    else :
        X_scaled = scaler.transform(X[scaled_cols])

    #rebuilding
    X_scaled_df = pd.DataFrame(X_scaled, columns=scaled_cols , index=X.index)
    X_scaled_df[binary_cols[0]] = X[binary_cols].values
    
    #adding label back
    if has_label :
        X_scaled_df[target_col] = y.values.ravel()
        # X_scaled_df = X_scaled_df.sample(frac=1).reset_index(drop=True)

    print(f"Normalized DataFrame columns: {X_scaled_df.columns.tolist()}")

    return X_scaled_df,y,scaler #tuble with both the df and the scaler

df = load_data("test.csv")
df = cleaning(df , has_label=False)