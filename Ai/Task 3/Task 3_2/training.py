import pandas as pd
import numpy as np
from sklearn.ensemble import GradientBoostingRegressor
from sklearn.model_selection import train_test_split, KFold
from sklearn.metrics import mean_squared_error
from sklearn.preprocessing import RobustScaler, PolynomialFeatures
import xgboost as xgb
import pickle
import os
import helpers
from scipy.stats import skew, boxcox
from preprocessing import *
from preprocessing import df_id


#saving training results
def save_ensemble_results(y_true, gbr_pred, xgb_pred, ensemble_pred, weight_gbr, rmse, fold=None):
    #undoing the sqrt done on normaliztion
    if operation_on_target == 'sqrt':
        y_true = np.square(y_true)
        gbr_pred = np.square(gbr_pred)
        xgb_pred = np.square(xgb_pred)
        ensemble_pred = np.square(ensemble_pred)
    
    results = pd.DataFrame({
        'Actual': y_true,
        'GBR_Pred': gbr_pred,
        'XGB_Pred': xgb_pred,
        'Ensemble_Pred': ensemble_pred,
        'Weight_GBR': weight_gbr,
        'RMSE': rmse,
        'Fold': fold if fold is not None else -1
    })

    filename = 'ensemble_results.csv'
    mode = 'a' if os.path.exists(filename) and weight_gbr > 0.3 else 'w'    #append if file exists , if not write
    header = weight_gbr == 0.3                                              
    results.to_csv(filename, mode=mode, header=header, index=False)
    print(f"Appended ensemble results for Weight_GBR={weight_gbr}, Fold={fold if fold is not None else -1} to '{filename}'")

#training
def train_ensemble():
    train_df = load_data('train.csv')
    train_df = cleaning(train_df, has_label=True)
    X, y, scaler = normalization(train_df, has_label=True)
    
    # Save feature names (excluding target 'Calories')
    if 'Calories' in X.columns:
        # ensure we don't accidentally train on the target as a feature
        X = X.drop(columns=['Calories'])
    feature_names = list(X.columns)
    with open('feature_names.pkl', 'wb') as f:
        pickle.dump(feature_names, f)
    
    #Tuned gbr
    gbr_params = {
        'n_estimators': 181,                  
        'learning_rate': 0.11416685200516473, 
        'max_depth': 12,                      
        'min_samples_split': 6,              
        'min_samples_leaf': 4,               
        'subsample': 0.8619076397167239,   
        'random_state': 42                   
    }

    #Tuned xbgr
    xgb_params = {
        'n_estimators': 238,                 
        'learning_rate': 0.06869763494360763,  
        'max_depth': 12,                     
        'subsample': 0.7596527212266415,   
        'colsample_bytree': 0.8184644554526709,  
        'objective': 'reg:squarederror',
        'random_state': 42
    }

    #5-fold cross-validation
    kf = KFold(n_splits=5, shuffle=True, random_state=42)
    weights_gbr = [0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9]  #tuning weights
    best_rmse = float('inf')
    best_weight_gbr = None
    filename = 'ensemble_results.csv'
    if os.path.exists(filename):
        os.remove(filename)                                           #writes in a new file everytime
    
    rmse_scores = []
    for fold, (train_idx, val_idx) in enumerate(kf.split(X)):
        X_train, X_val = X.iloc[train_idx], X.iloc[val_idx]
        y_train, y_val = y.iloc[train_idx], y.iloc[val_idx]
        
        gbr = GradientBoostingRegressor(**gbr_params)                 #passing a dic
        xgb_model = xgb.XGBRegressor(**xgb_params)
        gbr.fit(X_train, y_train)
        xgb_model.fit(X_train, y_train)
        
        gbr_pred = gbr.predict(X_val)
        xgb_pred = xgb_model.predict(X_val)

        #looping throught weights
        for weight_gbr in weights_gbr:
            weight_xgb = 1 - weight_gbr
            ensemble_pred = weight_gbr * gbr_pred + weight_xgb * xgb_pred   #total weighted pred

            rmse = np.sqrt(mean_squared_error(y_val, ensemble_pred))        #calculting rmse
            rmse_scores.append((rmse, weight_gbr, fold))                    #appending to rmse scores
            print(f"Fold {fold}, Weight_GBR={weight_gbr}, Weight_XGB={weight_xgb}, Validation RMSE (transformed): {rmse}")

            save_ensemble_results(y_val, gbr_pred, xgb_pred, ensemble_pred, weight_gbr, rmse, fold)
            if operation_on_target == 'sqrt':
                ensemble_pred_transformed = np.square(ensemble_pred)
                y_val_transformed = np.square(y_val)
                rmse_transformed = np.sqrt(mean_squared_error(y_val_transformed, ensemble_pred_transformed))
                print(f"Fold {fold}, Weight_GBR={weight_gbr}, Validation RMSE (original scale): {rmse_transformed}")
    
    #trainging final models on full training data
    gbr = GradientBoostingRegressor(**gbr_params)
    xgb_model = xgb.XGBRegressor(**xgb_params)
    gbr.fit(X, y)
    xgb_model.fit(X, y)
    with open('gbr_model.pkl', 'wb') as f:
        pickle.dump(gbr, f)                 #saving in folder the model
    with open('xgb_model.pkl', 'wb') as f:
        pickle.dump(xgb_model, f)
    with open('scaler.pkl', 'wb') as f:
        pickle.dump(scaler, f)              #saving scaler
    
    #best weight based on average rmse
    rmse_df = pd.DataFrame(rmse_scores, columns=['RMSE', 'Weight_GBR', 'Fold'])
    avg_rmse = rmse_df.groupby('Weight_GBR')['RMSE'].mean()
    best_weight_gbr = avg_rmse.idxmin()
    best_rmse = avg_rmse.min()
    print(f"Best Weight_GBR: {best_weight_gbr}, Average Validation RMSE (transformed): {best_rmse}")

    with open('best_weight.pkl', 'wb') as f:
        pickle.dump(best_weight_gbr, f) #returns best weight
    
    return gbr, xgb_model, scaler, best_weight_gbr

#submission logic
def generate_submission():
    #fetch data from model choices
    with open('gbr_model.pkl', 'rb') as f:
        gbr = pickle.load(f)
    with open('xgb_model.pkl', 'rb') as f:
        xgb_model = pickle.load(f)
    with open('scaler.pkl', 'rb') as f:
        scaler = pickle.load(f)
    with open('best_weight.pkl', 'rb') as f:
        weight_gbr = pickle.load(f)
    with open('feature_names.pkl', 'rb') as f:
        feature_names = pickle.load(f)
    
    test_df = load_data('test.csv')
    test_df = cleaning(test_df, has_label=False)
    X_test, _, _ = normalization(test_df, has_label=False, scaler=scaler)

    print(f"Expected features: {feature_names}")
    print(f"Test features before alignment: {X_test.columns.tolist()}")
    missing_features = [f for f in feature_names if f not in X_test.columns]
    extra_features = [f for f in X_test.columns if f not in feature_names]
    
    #Error-handling
    if missing_features or extra_features:
        print(f"Missing features in test data: {missing_features}")
        print(f"Extra features in test data: {extra_features}")
        if extra_features:
            print(f"Dropping extra features: {extra_features}")
            X_test = X_test.drop(columns=extra_features, errors='ignore')

        # Fill missing features with zeros (safer: use train means saved separately)
        if missing_features:
            print(f"Filling missing features with zeros: {missing_features}")
            for f in missing_features:
                X_test[f] = 0

    # Reorder to match training feature order exactly
    X_test = X_test[feature_names]
    print(f"Test features used: {X_test.columns.tolist()}")

    # Sanity-check: make sure the feature names/order exactly matches what model expects
    if list(X_test.columns) != feature_names:
        raise ValueError("Feature mismatch after alignment: test features do not match training features.")

    gbr_pred = gbr.predict(X_test)
    xgb_pred = xgb_model.predict(X_test)

    weight_xgb = 1 - weight_gbr                                     #total_weight = 1
    test_pred = weight_gbr * gbr_pred + weight_xgb * xgb_pred

    if operation_on_target == 'sqrt':
        test_pred = np.square(test_pred)

    submission = pd.DataFrame({'id': df_id, 'Calories': test_pred}) #building id,calories subm csv
    submission_file = 'sample_submission.csv'
    submission.to_csv(submission_file, index=False)
    print(f"Test predictions saved to '{submission_file}'")

# Main execution
if __name__ == "__main__":
    print("Starting training...")
    gbr, xgb_model, scaler, best_weight = train_ensemble()
    print("Generating submission...")
    generate_submission()
