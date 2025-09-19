import pandas as pd
import numpy as np
import random
from sklearn.metrics import log_loss
import preprocessing  # your preprocessing.py
from logistic_regression import LogisticRegressionScratch  # your model
import os

# -------------------
# Load & preprocess data
# -------------------
train_df = preprocessing.load_data("train.csv")
train_df = preprocessing.cleaning(train_df, has_label=True)
train_df, y_train, scaler = preprocessing.normalization(train_df, has_label=True)

X_train = train_df.drop(columns=["is_spam"]).values
y_train = y_train.values

# -------------------
# Prepare CSV file for runtime saving
# -------------------
csv_file = "best_configs.csv"
if not os.path.exists(csv_file):
    pd.DataFrame(columns=["learning_rate", "epochs", "reg_l1", "reg_l2", "threshold", "log_loss"]).to_csv(csv_file, index=False)

# -------------------
# Random search
# -------------------
for i in range(500):
    # Randomly sample hyperparameters
    learning_rate = 10**random.uniform(-4, -3)  # 0.0001 to ~0.3
    epochs = random.randint(100, 3000)
    reg_l1 = 10**random.uniform(-4, 0)  # 0.0001 to 1
    reg_l2 = 10**random.uniform(-4, 0)  # 0.0001 to 1
    threshold = random.uniform(0.3, 0.7)

    # Create and train model
    model = LogisticRegressionScratch(
        learning_rate=learning_rate,
        epochs=epochs,
        reg_l1=reg_l1,
        reg_l2=reg_l2,
        threshold=threshold
    )

    model.fit_advanced(X_train, y_train,True,True,64)

    # Predictions for log loss
    y_pred_probs = model.predict_proba(X_train)
    loss = log_loss(y_train, y_pred_probs)

    # Save only good configs
    if loss <= 0.2:
        new_row = pd.DataFrame([{
            "learning_rate": learning_rate,
            "epochs": epochs,
            "reg_l1": reg_l1,
            "reg_l2": reg_l2,
            "threshold": threshold,
            "log_loss": loss
        }])
        new_row.to_csv(csv_file, mode='a', header=False, index=False)

    print(f"[{i+1}/500] Loss: {loss:.5f}")
