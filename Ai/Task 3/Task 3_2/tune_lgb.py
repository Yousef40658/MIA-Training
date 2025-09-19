import lightgbm as lgb
import numpy as np
import pandas as pd
from sklearn.model_selection import KFold
from sklearn.metrics import mean_squared_error
import optuna
import preprocessing  
import csv
import os
import time

#saving to csv file
log_file = "lgb_tuning.csv"
if not os.path.exists(log_file):
    with open(log_file, mode="w", newline="") as f: 
        writer = csv.writer(f)
        writer.writerow([
            "trial_number", "num_leaves", "learning_rate", "num_boost_round",   #writes a row
            "val_rmse", "train_rmse", "val_rmse_std", "duration_sec"
        ])

#preprocessing
df = preprocessing.load_data("train.csv")   
df = preprocessing.cleaning(df, has_label=True)
df_scaled, y, _ = preprocessing.normalization(df, has_label=True)

X = df_scaled.drop(columns=["Calories"])    # separates labels
y = y.values

#K-folding
n_splits = 5
kf = KFold(n_splits=n_splits, shuffle=True, random_state=42)


#lbg
def objective(trial):
    params = {
        "objective": "regression",
        "metric": "rmse",
        "boosting_type": "gbdt",
        "num_leaves": trial.suggest_int("num_leaves", 15, 255),
        "learning_rate": trial.suggest_float("learning_rate", 0.01, 0.3, log=True) #randomize around floats
    }

    num_round = trial.suggest_int("num_boost_round", 100, 2000) #randomize around integers

    val_metrics = []
    train_metrics = []
    start = time.time()                                         #time comparison between models

    for train_index, val_index in kf.split(X):                  #splitting to training and validiting sets
        X_train, X_val = X.iloc[train_index], X.iloc[val_index]   
        y_train, y_val = y[train_index], y[val_index]

        train_data = lgb.Dataset(X_train, label=y_train)        #transoforming into a Dataset object
        val_data = lgb.Dataset(X_val, label=y_val)

        bst = lgb.train(params, train_data, num_round)

        # Predictions 
        y_val_pred = bst.predict(X_val)
        y_val_pred_original = np.square(y_val_pred)     #inversing the squaring we  did in preprocessing before error calc
        y_val_original = np.square(y_val)
        val_rmse = np.sqrt(mean_squared_error(y_val_original, y_val_pred_original))
        val_metrics.append(val_rmse)

        y_train_pred = bst.predict(X_train)
        y_train_pred_original = np.square(y_train_pred)
        y_train_original = np.square(y_train)
        train_rmse = np.sqrt(mean_squared_error(y_train_original, y_train_pred_original))
        train_metrics.append(train_rmse)

    end = time.time()
    duration = end - start

    avg_val_rmse = np.mean(val_metrics)
    avg_train_rmse = np.mean(train_metrics)
    std_val_rmse = np.std(val_metrics)

    #logs to file
    with open(log_file, mode="a", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            trial.number, params["num_leaves"], params["learning_rate"], num_round,
            avg_val_rmse, avg_train_rmse, std_val_rmse, duration
        ])

    return avg_val_rmse

# Run optimization
study = optuna.create_study(direction="minimize")   #automates hyperparameter tunning
study.optimize(objective, n_trials=30)

print("Best params:", study.best_params)
print("Best RMSE:", study.best_value)
