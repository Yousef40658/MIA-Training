import numpy as np
import pandas as pd
import os
from preprocessing import load_data, cleaning, normalization
from logistic_regression import LogisticRegressionScratch
from sklearn.model_selection import StratifiedKFold
from sklearn.metrics import accuracy_score, precision_score, recall_score, f1_score, log_loss 

counter = 0

# 1) Load & preprocess
train_df = load_data("train.csv")
train_df = cleaning(train_df, has_label=True)
train_df, y_all, scaler = normalization(train_df, has_label=True)
X_all = train_df.drop(columns="is_spam").values
y_all = y_all.values.ravel()

# 2) Hyper-parameter grid
param_grid = {
    'learning_rate': [0.01, 0.1, 0.5, 1],
    'epochs':        [100, 200, 500, 1000],
    'reg_l1':        [0.0, 1e-3, 1e-2, 1e-1],
    'reg_l2':        [0.0, 1e-3, 1e-2, 1e-1],
    'threshold':     [0.3, 0.4, 0.5, 0.6, 0.7],
}

kf = StratifiedKFold(n_splits=5, shuffle=True, random_state=42)
results = []

best_overall = -1
best_config = None  

# CSV file to store best runs
best_csv_path = "best_configs_by_mini_64.csv"
if not os.path.exists(best_csv_path):
    pd.DataFrame(columns=[
        'learning_rate', 'epochs', 'reg_l1', 'reg_l2', 'threshold',
        'acc', 'prec', 'rec', 'f1', 'log_loss', 'overall'
    ]).to_csv(best_csv_path, index=False)

# Evaluation function
def evaluate_fold(train_idx, val_idx, lr, ep, l1, l2, th):
    X_train, X_val = X_all[train_idx], X_all[val_idx]
    y_train, y_val = y_all[train_idx], y_all[val_idx]

    model = LogisticRegressionScratch(
        learning_rate=lr,
        epochs=ep,
        tol=1e-6,
        reg_l1=l1,
        reg_l2=l2
    )
    model.fit_advanced(X_train, y_train, True, True, 64)

    prob_val = model.predict_proba(X_val)
    pred_val = (prob_val >= th).astype(int)

    return {
        'acc':       accuracy_score(y_val, pred_val),
        'prec':      precision_score(y_val, pred_val, zero_division=0),
        'rec':       recall_score(y_val, pred_val, zero_division=0),
        'f1':        f1_score(y_val, pred_val, zero_division=0),
        'log_loss':  log_loss(y_val, prob_val)
    }

# 3) Grid search
for lr in param_grid['learning_rate']:
    for ep in sorted(param_grid['epochs']):
        for l1 in param_grid['reg_l1']:
            for l2 in param_grid['reg_l2']:
                for th in param_grid['threshold']:
                    fold_indices = list(kf.split(X_all, y_all))
                    
                    fold_results = []
                    for train_idx, val_idx in fold_indices:
                        fold_results.append(evaluate_fold(train_idx, val_idx, lr, ep, l1, l2, th))

                    if counter % 20 == 0:
                        print(f"{counter} configurations evaluated...")

                    cv_metrics = {
                        'acc':      np.mean([r['acc']      for r in fold_results]),
                        'prec':     np.mean([r['prec']     for r in fold_results]),
                        'rec':      np.mean([r['rec']      for r in fold_results]),
                        'f1':       np.mean([r['f1']       for r in fold_results]),
                        'log_loss': np.mean([r['log_loss'] for r in fold_results])
                    }

                    if cv_metrics['rec'] < 0.1 or cv_metrics['prec'] == 0.0:
                        continue

                    counter += 1
                    result_entry = {
                        'learning_rate': lr,
                        'epochs':        ep,
                        'reg_l1':        l1,
                        'reg_l2':        l2,
                        'threshold':     th,
                        **cv_metrics
                    }
                    results.append(result_entry)
                    pd.DataFrame([result_entry]).to_csv("all_results_incremental_patch64.csv", mode='a', header=not os.path.exists("all_results_incremental_patch64.csv"), index=False)

                    # Compute normalized for overall score
                    temp_df = pd.DataFrame(results)
                    for m in ['acc', 'prec', 'rec', 'f1']:
                        mn, mx = temp_df[m].min(), temp_df[m].max()
                        temp_df[f'{m}_norm'] = (temp_df[m] - mn) / (mx - mn + 1e-12)
                    log_min, log_max = temp_df['log_loss'].min(), temp_df['log_loss'].max()
                    temp_df['log_loss_norm'] = (log_max - temp_df['log_loss']) / (log_max - log_min + 1e-12)

                    latest = temp_df.iloc[-1]
                    overall_score = (
                        0.15 * latest['acc_norm'] +
                        0.2 * latest['prec_norm'] +
                        0.15 * latest['rec_norm'] +
                        0.2 * latest['f1_norm'] +
                        0.3 * latest['log_loss_norm']
                    )

                    if overall_score > best_overall:
                        best_overall = overall_score
                        best_config = (lr, ep, l1, l2, th)
                        print(f"\n[NEW BEST OVERALL: {overall_score:.4f}] "
                              f"lr={lr}, ep={ep}, λ1={l1}, λ2={l2}, th={th}")

                        pd.DataFrame([{
                            'learning_rate': lr,
                            'epochs':        ep,
                            'reg_l1':        l1,
                            'reg_l2':        l2,
                            'threshold':     th,
                            'acc':           cv_metrics['acc'],
                            'prec':          cv_metrics['prec'],
                            'rec':           cv_metrics['rec'],
                            'f1':            cv_metrics['f1'],
                            'log_loss':      cv_metrics['log_loss'],
                            'overall':       overall_score
                        }]).to_csv(best_csv_path, mode='a', header=False, index=False)

                    print(f"lr={lr}, ep={ep}, λ1={l1}, λ2={l2}, th={th} → "
                          f"acc={cv_metrics['acc']:.4f}, prec={cv_metrics['prec']:.4f}, "
                          f"rec={cv_metrics['rec']:.4f}, f1={cv_metrics['f1']:.4f}, "
                          f"log_loss={cv_metrics['log_loss']:.4f}, overall={overall_score:.4f}")

# 4) Save all scored results ranked
df = pd.DataFrame(results)
for m in ['acc', 'prec', 'rec', 'f1']:
    mn, mx = df[m].min(), df[m].max()
    df[f'{m}_norm'] = (df[m] - mn) / (mx - mn + 1e-12)
log_min, log_max = df['log_loss'].min(), df['log_loss'].max()
df['log_loss_norm'] = (log_max - df['log_loss']) / (log_max - log_min + 1e-12)

df['overall'] = (
    0.2 * df['acc_norm'] +
    0.2 * df['prec_norm'] +
    0.15 * df['rec_norm'] +
    0.2 * df['f1_norm'] +
    0.27 * df['log_loss_norm']
)

df.to_csv("all_results_ranked.csv", index=False)

# Show top 5 configs
top5 = df.sort_values('overall', ascending=False).head(5)
print("\nTop 5 configurations:")
print(top5[[
    'learning_rate', 'epochs', 'reg_l1', 'reg_l2', 'threshold',
    'acc', 'prec', 'rec', 'f1', 'log_loss', 'overall'
]])
