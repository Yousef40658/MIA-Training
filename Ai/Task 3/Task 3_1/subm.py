import numpy as np
import pandas as pd
from sklearn.utils import shuffle
from preprocessing import load_data, cleaning, normalization
from logistic_regression import LogisticRegressionScratch

# Load and preprocess training data
train_df = load_data("train.csv")
train_df = cleaning(train_df, has_label=True)
train_df = train_df.drop(columns=['message_id'])  # Drop message_id
X_train, y_train, scaler_train = normalization(train_df, has_label=True, scaler=None)
X_train = X_train.drop(columns=['is_spam'])  # Drop is_spam after normalization
X_train = X_train.values  # Convert to NumPy array
y_train = y_train.values.ravel()

# Load and preprocess test data
test_df = load_data("test.csv")
message_ids = test_df['message_id'].copy()  # Save message_id for submission
test_df = cleaning(test_df, has_label=False)
test_df = test_df.drop(columns=['message_id'])  # Drop message_id
X_test, _, scaler_test = normalization(test_df, has_label=False, scaler=scaler_train)
X_test = X_test.values  # Convert to NumPy array

# Verify feature consistency
assert X_train.shape[1] == X_test.shape[1], f"Feature mismatch: X_train has {X_train.shape[1]} features, X_test has {X_test.shape[1]} features"
print("X_train columns:", list(train_df.drop(columns=['is_spam']).columns))
print("X_test columns:", list(test_df.columns))

# Model configs (same as provided)
model_configs = [
    {"lr": 0.8, "ep": 1000, "l1": 0.0,   "l2": 0.0,   "th": 0.2},
    {"lr": 0.5, "ep": 1000, "l1": 0.0,   "l2": 0.0,   "th": 0.7}, 
    {"lr": 1.5, "ep": 1000, "l1": 0.00,  "l2": 0,     "th": 0.8},
    {"lr": 3,   "ep": 1000, "l1": 1e-4,  "l2": 1e-5,  "th": 0.5},
    {"lr": 4,   "ep": 1000, "l1": 0,     "l2": 1e-5,  "th": 0.5},
    {"lr": 0.1, "ep": 1000, "l1": 0.0,   "l2": 0,     "th": 0.4},
    {"lr": 0.05, "ep": 1500, "l1": 1e-5, "l2": 1e-5,  "th": 0.4},
    {"lr": 0.02, "ep": 2000, "l1": 0.0,  "l2": 0.0,   "th": 0.5},
    {"lr": 0.1,  "ep": 1500, "l1": 1e-4, "l2": 1e-4,  "th": 0.45},
    {"lr": 0.3,  "ep": 1200, "l1": 0.0,  "l2": 1e-4,  "th": 0.35},
    {"lr": 0.01, "ep": 2500, "l1": 1e-5, "l2": 1e-5, "th": 0.5},
]

# Train ensemble of models
models = []
probs_list = []

for i, cfg in enumerate(model_configs):
    X_shuffled, y_shuffled = shuffle(X_train, y_train, random_state=i)

    model = LogisticRegressionScratch(
        learning_rate=cfg["lr"],
        epochs=cfg["ep"],
        reg_l1=cfg["l1"],
        reg_l2=cfg["l2"],
        threshold=cfg["th"]
    )
    model.fit_advanced(X_shuffled, y_shuffled, shuffle=True, minibatch=True, batch_size=8)
    models.append(model)
    probs_list.append(model.predict_proba(X_test))  # Predict probabilities for test data

# Average the predicted probabilities
avg_probs = np.mean(probs_list, axis=0)

# Ensure probabilities are valid
avg_probs = np.clip(avg_probs, 0, 1)  # Clip to [0, 1] for LogLoss

# Create submission DataFrame
submission = pd.DataFrame({
    'message_id': message_ids,
    'is_spam': avg_probs
})

# Verify predictions are probabilities
assert all((submission['is_spam'] >= 0) & (submission['is_spam'] <= 1)), "Predictions are not valid probabilities"

# Save to CSV
submission.to_csv("submission.csv", index=False)
print("Saved to submission.csv")