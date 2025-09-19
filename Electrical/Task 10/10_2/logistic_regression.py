# reference for some part :https://www.youtube.com/watch?v=S6iuhdYsGC8
try:
    import cupy as cp
    xp = cp
    _HAS_CUPY = True
except Exception:
    import numpy as np
    xp = np
    _HAS_CUPY = False

import numpy as _np  # small internal alias 
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

    def _to_xp(self, arr):
        """Convert input to xp array (CuPy or NumPy) safely."""
        if _HAS_CUPY:
            # If arr already xp array, xp.array will be cheap; if numpy, convert
            return xp.array(arr)
        else:
            return _np.array(arr)

    def sigmoid(self, z):
        z = xp.clip(z, -100, 100)
        return 1.0 / (1.0 + xp.exp(-z))
    
    # batch gradient descent #reads by epoch 
    def calculate_gradient(self, X_b, y):
        m = y.size
        predictions = self.sigmoid(X_b @ self.theta)    # projecting the features into X
        errors = predictions - y

        # build per-sample weight array safely (python dict keys must be native ints)
        # yi may be xp array - convert small slice to numpy for indexing dict
        yi_cpu = xp.asnumpy(y.astype(int)) if _HAS_CUPY else y.astype(int)
        weights = xp.array([self.class_weights[int(v)] for v in yi_cpu])

        errors = errors * weights
        grad = (X_b.T @ errors) / m          # calculating grad
        # L2 grad 
        grad[1:] += (self.reg_2 / m) * self.theta[1:]
        # L1
        grad[1:] += (self.reg_1 / m) * xp.sign(self.theta[1:])
        return grad 

    def fit(self, X, y):
        # ensure xp arrays
        X = self._to_xp(X)
        y = self._to_xp(y).astype(float)
        X_b = xp.c_[xp.ones((X.shape[0], 1)), X]         # Add bias column (1s)
        self.theta = xp.zeros(X_b.shape[1])              # same shape as x_biased -> explained further in fnd.md

        # class weighting (make keys python ints)
        unique, counts = xp.unique(y.astype(int), return_counts=True)
        unique_cpu = xp.asnumpy(unique) if _HAS_CUPY else unique
        counts_cpu = xp.asnumpy(counts) if _HAS_CUPY else counts
        total = y.shape[0]
        self.class_weights = {int(cls): float(total) / (2.0 * int(count)) for cls, count in zip(unique_cpu, counts_cpu)}

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

            if xp.linalg.norm(grad) < self.tol:          # Stop if gradient is small
                break


    # ML - Stochastic Gradient Descent (SGD) #for each training example - not by packet
    # https://www.geeksforgeeks.org/machine-learning/ml-stochastic-gradient-descent-sgd/
    # Minibatch is faster GSD and more efficient than normal gd
    def fit_advanced(self, X, y, shuffle=True, minibatch=False, batch_size=1):
        # Accept numpy arrays too - convert to xp arrays internally
        X = self._to_xp(X)
        y = self._to_xp(y).astype(float)
        X_b = xp.c_[xp.ones((X.shape[0], 1)), X]  # Add bias column (1s)
        self.theta = xp.zeros(X_b.shape[1])       # Initialize weights
        m = X_b.shape[0]                           # Number of training samples

        # class weighting (python int keys)
        unique, counts = xp.unique(y.astype(int), return_counts=True)
        unique_cpu = xp.asnumpy(unique) if _HAS_CUPY else unique
        counts_cpu = xp.asnumpy(counts) if _HAS_CUPY else counts
        self.class_weights = {int(cls): float(m) / (2.0 * int(count)) for cls, count in zip(unique_cpu, counts_cpu)}

        # Label smoothing
        epsilon = 0.05
        y = y * (1 - epsilon) + (epsilon / 2)

        for epoch in range(self.epochs):
            if shuffle:
                indices = xp.arange(m)
                # shuffle in-place on xp
                xp.random.shuffle(indices)
                # reorder X_b and y - indices is xp array, but indexing works for xp arrays
                X_b = X_b[indices]
                y = y[indices]

            for i in range(0, m, batch_size):
                end = i + batch_size if minibatch else i + 1
                xi = X_b[i:end] if minibatch else X_b[i:i+1]
                yi = y[i:end] if minibatch else y[i:i+1]

                # ensure yi and xi are xp arrays (they are) and same type as theta
                # build per-sample weights safely (convert yi to CPU small list to index python dict)
                yi_int_cpu = xp.asnumpy(yi.astype(int)) if _HAS_CUPY else yi.astype(int)
                sample_weights = xp.array([self.class_weights[int(v)] for v in yi_int_cpu])

                pred = self.sigmoid(xi @ self.theta)
                error = pred - yi
                # apply weights
                error = error * sample_weights

                grad = (xi.T @ error) / len(xi)
                grad = grad.ravel()

                # Regularization
                grad[1:] += self.reg_2 * self.theta[1:]          # L2
                grad[1:] += self.reg_1 * xp.sign(self.theta[1:]) # L1

                self.theta -= self.learning_rate * grad

            #
            if epoch % 100 == 0:
                y_pred = self.sigmoid(X_b @ self.theta)
                loss = self.compute_loss(y, y_pred)
                print(f"[SGD] Epoch {epoch:4d}: loss={loss:.6f}")

            # Optional: early stopping after full epoch
            if xp.linalg.norm(grad) < self.tol:
                print(f"Converged at epoch {epoch}")
                break


    def predict_proba(self, X):
        X = self._to_xp(X)
        X_b = xp.c_[xp.ones((X.shape[0], 1)), X]
        return self.sigmoid(X_b @ self.theta)

    def predict(self, X):               # above the threshold is considered a spam
        return (self.predict_proba(X) >= self.threshold).astype(int)

    def compute_loss(self, y_true, y_pred):
        y_true = self._to_xp(y_true)
        m = y_true.size                  
        # base_loss
        self.base = -xp.mean(
            y_true * xp.log(y_pred + self.epsi) +
            (1 - y_true) * xp.log(1 - y_pred + self.epsi)
        )  # the 2nd part collapses to zero when pred == label

        # l1 is usually for more features 
        self.l1_pen = (self.reg_1 / m) * xp.sum(xp.abs(self.theta[1:]))     # penalty based on the absolute value

        # L2 penalty 
        self.l2_pen = (self.reg_2 / (2*m) * xp.sum(self.theta[1:]**2))# penalty based on the square of each coefficient

        return self.base + self.l1_pen + self.l2_pen
