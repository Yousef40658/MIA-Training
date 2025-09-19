from word2number import w2n
import math

#detect "numerical" words in integers columns
def try_word_to_num(val):
    try:
        return w2n.word_to_num(val)
    except:
        return val  # keep as-is if it's not a number word
    
# Show percentage of BAD values per column
def empty_percentge(col):
    return col.astype(str).str.strip().str.lower().isin(['', '\\n', 'na', 'nan', 'null', '\n', '?']).mean()

class Matrix :
    def __init__(self,data):
        self.data = data
        self.rows = len(data)
        self.cols = len(data[0])

    def transpose(self):
        transposed = [[self.data[row][col] for row in range(self.rows)] for col in range(self.cols)]
        return Matrix(transposed)

        
    def multiply(self, other):
        if self.cols != other.rows:
            raise ValueError("Cols of first matrix should equal the rows of 2nd one")
        result = []
        for i in range(self.rows):
            row = []
            for j in range(other.cols):
                val = sum(self.data[i][k] * other.data[k][j] for k in range(self.cols))
                row.append(val)
            result.append(row)
        return Matrix(result)

    
    def standardize(self):
        standardized = []
        for j in range(self.cols):
            col = [self.data[i][j] for i in range(self.rows)]
            mean = sum(col) / len(col)
            std = (sum((x - mean) ** 2 for x in col) / (len(col) - 1)) ** 0.5  # ← Fix here
            for i in range(self.rows):
                if len(standardized) < i + 1:
                    standardized.append([])
                standardized[i].append((self.data[i][j] - mean) / std)
        return Matrix(standardized)
    
    def zscore_filter(self, threshold=3.0):
        filtered_data = []
        keep_indices = []
        for idx, row in enumerate(self.data):
            if all(abs(val) <= threshold for val in row):
                filtered_data.append(row)
                keep_indices.append(idx)
        return Matrix(filtered_data), keep_indices
                
    
    def covariance_matrix(self):
        X = self.data
        n = self.rows
        m = self.cols
        cov = [[0.0 for _ in range(m)] for _ in range(m)]
        for i in range(m):
            for j in range(m):
                cov[i][j] = sum(X[k][i] * X[k][j] for k in range(n)) / (n - 1)  # ← FIXED
        return Matrix(cov)

    
    def project(self, eigenvectors, n_components):
        reduced = [vec[:n_components] for vec in eigenvectors]
        return self.multiply(Matrix(reduced))

    
    def tolist(self):
        return self.data

    def shape(self):
        return (self.rows, self.cols)

    def __str__(self):
        return "\n".join(str(row) for row in self.data)

