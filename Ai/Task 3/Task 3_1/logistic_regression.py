#refrence for some part :https://www.youtube.com/watch?v=S6iuhdYsGC8
import numpy as np

class LogisticRegressionScratch:
    def __init__(self, learning_rate=0.01, epochs=1000, tol=1e-7, reg_l1=0.01, reg_l2=0.05 , threshold = 0.5):
        self.learning_rate = learning_rate  # higher means lower log loss
        self.epochs = epochs    # log loss decreases with it increasing -> but large computation time
        self.tol = tol  
        self.theta = None  # includes weights and bias
        self.reg_1 = reg_l1  # strength
        self.reg_2 = reg_l2  # strength
        self.epsi = 1e-8                 # to avoid log(0)
        self.class_weights = {0: 1.0, 1: 1.0}  # default weights
        self.threshold = threshold

    def sigmoid(self, z):
        z = np.clip(z, -100, 100)
        return 1.0 / (1.0 + np.exp(-z))
    
  
    #batch gradiant decent #reads by epoch 
    def calculate_gradient(self, X_b, y):
        m = y.size
        predictions = self.sigmoid(X_b @ self.theta)    # projecting the features into X
        errors = predictions - y
        errors *= np.vectorize(self.class_weights.get)(y.astype(int))  # apply class weights
        grad = (X_b.T @ errors) / m          # calculating grad
        # L2 grad 
        grad[1:] += (self.reg_2 / m) * self.theta[1:]
        # l1
        grad[1:] += (self.reg_1 / m) * np.sign(self.theta[1:])
        return grad 

    def fit(self, X, y):
        X_b = np.c_[np.ones((X.shape[0], 1)), X]         # Add bias column (1s)
        self.theta = np.zeros(X_b.shape[1])              # same shape as x_biased -> explained further in fnd.md

        # class weighting
        unique, counts = np.unique(y, return_counts=True)
        total = y.shape[0]
        self.class_weights = {cls: total / (2 * count) for cls, count in zip(unique, counts)}

        for i in range(self.epochs):
            grad = self.calculate_gradient(X_b, y)       # calculates grad
            self.theta -= self.learning_rate * grad     

            if i % 100 == 0:                             # 100 element at a time
                y_pred = self.sigmoid(X_b @ self.theta)
                loss = self.compute_loss(y, y_pred)
                # detailed logging of each component
                print(
                    f"Epoch {i:4d}: base={self.base:.6f}, "
                    f"L1={self.l1_pen:.6f}, L2={self.l2_pen:.6f}, "
                    f"total={loss:.6f}"
                )

            if np.linalg.norm(grad) < self.tol:          # Stop if gradient is small
                break


    #ML - Stochastic Gradient Descent (SGD) #for each training example - not by packet
    #https://www.geeksforgeeks.org/machine-learning/ml-stochastic-gradient-descent-sgd/
    #Minibatch is faster GSD and more efficint than normal gd
    def fit_advanced(self, X, y, shuffle=True, minibatch=False, batch_size=1):
        X_b = np.c_[np.ones((X.shape[0], 1)), X]  # Add bias column (1s)
        self.theta = np.zeros(X_b.shape[1])       # Initialize weights
        m = X_b.shape[0]                           # Number of training samples

        # class weighting
        unique, counts = np.unique(y, return_counts=True)
        total = y.shape[0]
        self.class_weights = {cls: total / (2 * count) for cls, count in zip(unique, counts)}

        # Label smoothing
        epsilon = 0.05
        y = y * (1 - epsilon) + (epsilon / 2)

        for epoch in range(self.epochs):
            if shuffle:
                indices = np.arange(m)
                np.random.shuffle(indices)
                X_b = X_b[indices]
                y = y[indices]

            for i in range(0, m, batch_size):
                end = i + batch_size if minibatch else i + 1
                xi = X_b[i:end] if minibatch else X_b[i:i+1]
                yi = y[i:end] if minibatch else y[i:i+1]

                pred = self.sigmoid(xi @ self.theta)
                error = pred - yi
                error *= np.vectorize(self.class_weights.get)(yi.astype(int))  # apply class weights
                grad = xi.T @ error / len(xi)
                grad = grad.ravel()

                # Regularization
                grad[1:] += self.reg_2 * self.theta[1:]          # L2
                grad[1:] += self.reg_1 * np.sign(self.theta[1:]) # L1

                self.theta -= self.learning_rate * grad

            #
            if epoch % 100 == 0:
                y_pred = self.sigmoid(X_b @ self.theta)
                loss = self.compute_loss(y, y_pred)
                print(f"[SGD] Epoch {epoch:4d}: loss={loss:.6f}")

            # Optional: early stopping after full epoch
            if np.linalg.norm(grad) < self.tol:
                print(f"Converged at epoch {epoch}")
                break


    def predict_proba(self, X):
        X_b = np.c_[np.ones((X.shape[0], 1)), X]
        return self.sigmoid(X_b @ self.theta)

    def predict(self, X):               # above the threshold is considered a spam
        return (self.predict_proba(X) >= self.threshold).astype(int)

    def compute_loss(self, y_true, y_pred):
        m = y_true.size                  
        # base_loss
        self.base = -np.mean(
            y_true * np.log(y_pred + self.epsi) +
            (1 - y_true) * np.log(1 - y_pred + self.epsi)
        )  # the 2nd part collapses to zero when pred == label

        # l1 is usually for more features 
        self.l1_pen = (self.reg_1 / m) * np.sum(np.abs(self.theta[1:]))     # penalty based on the absolute value

        # L2 penalty 
        self.l2_pen = (self.reg_2 / (2*m) * np.sum(self.theta[1:]**2))# penalty based on the square of each coefficient

        return self.base + self.l1_pen + self.l2_pen
