import pandas as pd 
import os
from sklearn.preprocessing import MinMaxScaler
import helpers

def load_data(file) :
    folder_path = os.path.dirname(os.path.abspath(__file__))
    file_path = os.path.join(folder_path,file)
    return pd.read_csv(file_path, on_bad_lines= "skip").replace(r'"', '', regex=True)

def cleaning(df, has_label=True):

    drop_cols = ['message_id' , 'num_words' , 'sender_score']  # keep message_id in test

    #feature col
    #df['link_word_ratio'] = df['num_links'] / (df['num_words'] + 1)


    #trying squaring
    #df['num_words'] = df['num_words'] ** 2  ##increased some stats by 1%

    #correct types , all cols are numerical 
    for col in df.columns:
        df[col] = df[col].apply(helpers.try_word_to_num)
        df[col] = pd.to_numeric(df[col], errors='coerce')

    #replacing rows with much missing info
    threshold = 2
    df = df[df.isnull().sum(axis=1) < threshold] 

    #replacing numericals with their mean
    for col in df.columns:
        if col not in drop_cols:
            df[col] = df[col].fillna(df[col].mean())

    #removing dumplicates
    df = df.drop_duplicates()  #all numbers from the type conversion or null , no need to lowercase

    #removing outliers 
    for col in df.columns:
        if col != 'is_spam' and col not in drop_cols:
            lower = df[col].quantile(0.002)
            upper = df[col].quantile(0.995)
            df[col] = df[col].clip(lower, upper)

    return df


###########################
def normalization (df,has_label = True,scaler = None):
    #Cols
    scaled_cols = ['num_links']
    binary_cols = ['has_offer', 'all_caps']
    target_col = 'is_spam'
    
    #Sperate accodring to if test of train
    if has_label :
        y = df[target_col]
        X = df.drop(columns = [target_col])
    else :
        y = None
        X = df 
    
    #applying scaler or setting it 
    if scaler is None :
        scaler = MinMaxScaler()
        X_scaled = scaler.fit_transform(X[scaled_cols])
    else :
        X_scaled = scaler.transform(X[scaled_cols])

    #rebuilding
    X_scaled_df = pd.DataFrame(X_scaled, columns=scaled_cols , index=X.index)
    X_scaled_df[binary_cols] = X[binary_cols].values

    #adding label back
    if has_label :
        X_scaled_df[target_col] = y.values.ravel()
        # X_scaled_df = X_scaled_df.sample(frac=1).reset_index(drop=True)

    return X_scaled_df,y,scaler #tuble with both the df and the scaler
