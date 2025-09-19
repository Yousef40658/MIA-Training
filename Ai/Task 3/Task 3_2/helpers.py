from word2number import w2n
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import scipy.stats as stats

#detect "numerical" words in integers columns
def try_word_to_num(val):
    try:
        return w2n.word_to_num(val)
    except:
        return val  # keep as-is if it's not a number word
    
# Show percentage of BAD values per column
def empty_percentge(col):
    return col.astype(str).str.strip().str.lower().isin(['', '\\n', 'na', 'nan', 'null', '\n', '?']).mean()

def accuracy_score(y_true, y_pred):
    """
    Simple accuracy metric.
    """
    return np.mean(y_true == y_pred)

#PLOTTING
def plot_all_distributions(df):
    numeric_cols = df.select_dtypes(include=['number']).columns
    
    for col in numeric_cols:
        plt.figure(figsize=(8,4))
        sns.histplot(df[col], kde=True, bins=50)
        plt.title(f'Distribution of {col}')
        plt.xlabel(col)
        plt.ylabel('Frequency')
        plt.show()
