import numpy as np
import matplotlib.pyplot as plt

# -----------------------------
# 1. Simulation settings
# -----------------------------

n_layers = 50         # Number of layers in our "network"
n_neurons = 100       # Neurons per layer
n_samples = 100       # Batch size (number of inputs)

# -----------------------------
# 2. Input data
# -----------------------------

# Random input data: mean 0, standard deviation 1
# This simulates a batch of inputs to the network
x = np.random.randn(n_samples, n_neurons) #creates an array of specifed shape and fills it with random values
#n_samples here represnets the dimension of the returned array

# -----------------------------
# 3. Function to run a forward pass
# -----------------------------
def forward_pass(std_dev):
    """
    Simulates passing data through multiple layers
    with given weight initialization std_dev.
    Returns list of activation standard deviations per layer.
    """
    layer_input = x.copy()
    activations_std = []  # store std dev of activations per layer

    for _ in range(n_layers):
        # Initialize weights for this layer
        # mean = 0, std = std_dev
        # multiplying by std_dev controls the spread so values are not so small or so large

        W = np.random.randn(n_neurons, n_neurons) * std_dev

        # Compute activations: here we just do a linear pass + ReLU
        layer_output = np.dot(layer_input, W)

        # Apply ReLU non-linearity (set negatives to zero)
        layer_output = np.maximum(0, layer_output)

        # Store standard deviation of activations
        activations_std.append(layer_output.std())

        # Pass output as input to the next layer
        layer_input = layer_output

    return activations_std

# -----------------------------
# 4. Run the simulation
# -----------------------------

# Case 1: Too small std_dev -> vanishing
stds_small = forward_pass(0.01) #falls to zero

# Case 2: Too large std_dev -> exploding
stds_large = forward_pass(1.0)  #sky rockets

# Case 3: "Good" std_dev for ReLU (He initialization)
# He init std_dev = sqrt(2 / fan_in) where fan_in = n_neurons
he_std = np.sqrt(2.0 / n_neurons)
stds_he = forward_pass(he_std) #flat line , healthy activation

# -----------------------------
# 5. Plot the results
# -----------------------------
plt.figure(figsize=(8, 5))
plt.plot(stds_small, label="Too small σ (0.01)")
plt.plot(stds_large, label="Too large σ (1.0)")
plt.plot(stds_he, label=f"He init σ ({he_std:.3f})")
plt.xlabel("Layer")
plt.ylabel("Activation std dev")
plt.title("Effect of weight initialization on activations")
plt.legend()
plt.grid(True)
plt.show()
