# submitting.py
# Load saved models, transform test data using saved scaler and feature list,
# produce predictions and write 'sample_submission.csv'.

import pickle                                # load saved objects
import numpy as np                           # numeric operations
import pandas as pd                          # dataframes
import xgboost as xgb                        # if xgb model used (only for loading)
import preprocessing as prep                 # preprocessing functions & globals
import os                                    # filesystem operations

def generate_submission(use_gbr_only=False, use_stacking=True, use_lr=True):
    """Load models + scaler + feature names, prepare test set and save predictions."""
    # load trained models if present (use try/except for informative errors)
    gbr = None
    xgb_model = None
    lr_model = None
    meta_model = None

    if use_gbr_only or use_stacking:
        with open('gbr_model.pkl', 'rb') as f:
            gbr = pickle.load(f)                 # load gbr model

    if not use_gbr_only:
        with open('xgb_model.pkl', 'rb') as f:
            xgb_model = pickle.load(f)         # load xgb model

    if use_lr:
        with open('lr_model.pkl', 'rb') as f:
            lr_model = pickle.load(f)         # load lr model

    if use_stacking and not use_gbr_only and os.path.exists('meta_model.pkl'):
        with open('meta_model.pkl', 'rb') as f:
            meta_model = pickle.load(f)      # load meta stacking model if exists

    # load scaler, best weight/method, and feature names
    with open('scaler.pkl', 'rb') as f:
        scaler = pickle.load(f)
    with open('best_weight.pkl', 'rb') as f:
        weight_gbr = pickle.load(f)
    with open('best_method.pkl', 'rb') as f:
        best_method = pickle.load(f)
    with open('feature_names.pkl', 'rb') as f:
        feature_names = pickle.load(f)

    # load and preprocess test set
    test_df = prep.load_data('test.csv')                 # load CSV
    test_df = prep.cleaning(test_df, has_label=False)    # cleaning will set prep.df_id
    X_test, _, _ = prep.normalization(test_df, has_label=False, scaler=scaler)  # scale using saved scaler

    print(f"Expected features: {feature_names}")          # debug
    print(f"Test features before alignment: {X_test.columns.tolist()}")

    # ensure produced features match training features: drop extras and add missing if necessary
    missing_features = [f for f in feature_names if f not in X_test.columns]
    extra_features = [f for f in X_test.columns if f not in feature_names]
    if missing_features or extra_features:
        print(f"Missing features in test data: {missing_features}")
        print(f"Extra features in test data: {extra_features}")
        X_test = X_test.drop(columns=extra_features, errors='ignore')  # drop extras if any

    X_test = X_test[feature_names]                       # re-order columns to match training

    # get predictions from each model (if model exists)
    gbr_pred = gbr.predict(X_test) if gbr is not None else None
    xgb_pred = xgb_model.predict(X_test) if xgb_model is not None else None
    lr_pred = lr_model.predict(X_test) if lr_model is not None else None

    # decide final prediction according to best saved method or logic
    if use_stacking and not use_gbr_only and best_method == 'stacking' and gbr_pred is not None and xgb_pred is not None and meta_model is not None:
        meta_X = np.column_stack((gbr_pred, xgb_pred))
        test_pred = meta_model.predict(meta_X)
    elif use_lr and best_method == 'lr':
        test_pred = lr_pred
    elif best_method == 'gbr_only' or use_gbr_only:
        test_pred = gbr_pred
    else:
        # default ensemble: weighted average with saved weight
        test_pred = weight_gbr * gbr_pred + (1 - weight_gbr) * xgb_pred if gbr_pred is not None and xgb_pred is not None else gbr_pred

    # revert transformation on target if preprocessing recorded one
    if prep.operation_on_target == 'sqrt':
        test_pred = np.square(test_pred)

    # build submission using ids stored earlier by cleaning()
    submission = pd.DataFrame({'id': prep.df_id, 'Calories': test_pred})
    submission_file = 'sample_submission.csv'
    submission.to_csv(submission_file, index=False)       # save submission CSV
    print(f"Test predictions saved to '{submission_file}'")

if __name__ == "__main__":
    # If run directly, generate submission using the same defaults used in training.
    print("Generating submission...")
    generate_submission(use_gbr_only=False, use_stacking=True, use_lr=True)
    print("Submission generation complete.")
