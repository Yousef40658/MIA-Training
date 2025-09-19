import numpy as np
from collections import Counter

class KNNClassifier:
    def __init__(self, k=3):
        self.k = k
        self.X_train = None
        self.y_train = None

    def euclidean_distance(self, p, q):
        #sqrt the sum of squared differences 
        return np.sqrt(np.sum((p - q) ** 2))

    def fit(self, X, y):
        #transforming them to numpy arrays
        self.X_train = np.array(X)
        self.y_train = np.array(y)

    def predict_proba(self, X):
        X = np.array(X) #ensures the input is  a numpy array
        probs = []
        for x in X:
            distances = []  #stores distances with the label 
            for i in range(len(self.X_train)):
                dist = self.euclidean_distance(self.X_train[i], x)
                distances.append((dist, int(self.y_train[i])))
            distances.sort(key=lambda t: t[0])                      #sorts by closest first
            k_nearest = [label for _, label in distances[:self.k]]
            count = Counter(k_nearest)
            probs.append(count[1] / self.k)   #probability of class 1
        return np.array(probs)

    def predict(self, X, threshold=0.5):
        probs = self.predict_proba(X)
        return (probs >= threshold).astype(int)
