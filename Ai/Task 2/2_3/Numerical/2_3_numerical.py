import numpy as np
import matplotlib.pyplot as plt
from numpy.linalg import norm

#--------------
#Data
#--------------
Hassan = np.array([9, 8, 7, 6, 7, 8, 6])
teams = {
"Red_Bull" : np.array([10, 9, 6, 7, 6, 9, 5]),
"Ferrari" : np.array([9, 7, 6, 6, 7, 7, 5]),
"Mercedes" :  np.array([8, 6, 8, 9, 9, 5, 9])
}

results = {}
#--------------
#Similarities
#-------------- 
cosine_vals = []
euclidean_vals = []
manhattan_vals = []
correlation_vals = []
chebyshev_vals = []

for team_name , scores in teams.items() :
    
    #Cosine similarity
    #https://www.geeksforgeeks.org/python/how-to-calculate-cosine-similarity-in-python/
    """
    used to measure hot similar two vectors are regarding direction
    > in my research file i've studies that the dot product of two  vectors
    #define how in sync they're -moving in same direction-
    # """
    cosine_sim = np.dot(scores,Hassan) / (norm(scores) * norm(Hassan))
    cosine_vals.append(cosine_sim)
    
    #Euclidean Distance
    #https://www.datacamp.com/tutorial/euclidean-distance
    """
    the shortest distance between two points --used mainly in problems that
    involve space and distance - can be used to create clusters
    image processing - built a face recogantion using it before
    as it detects the edges and measures how much pixel colours change as you move inward or outward
    """
    euclidean_dist = norm(scores - Hassan)
    euclidean_vals.append(euclidean_dist)

    #Manhattan distance
    """
    simlliar to euclidean but with different calculations and approach
    manhatan doesn't take the square root therefore its faster to combute
    but it doesn't have the outsized effect the euclidean have
    -> the outsized effect can you help detect probems that are "big enough"
    faster and better because they'll have much more effect
    """
    manha_dist = np.sum(np.abs(scores-Hassan))
    manhattan_vals.append(manha_dist)

    #Pearson Correlation Coefficient
    #https://www.geeksforgeeks.org/maths/pearson-correlation-coefficient/
    """
    measures the strength and direction of linear relation ships 
    unlike cosine sim , magnitude matters 
    doesn't work well with non linear relationships
    """
    corr_matrix = np.corrcoef(Hassan,scores)
    corr = corr_matrix[0,1]                 #extract the coefficient 
    correlation_vals.append(corr)
    
    #Chebyshev Distance
    #https://www.datacamp.com/tutorial/chebyshev-distance
    """
    used in grid-based systems as it calculates the number of 
    grid-movments needed to move from one place to another
    same as chess -> the pieces move a square everytime 
    can be used in k-cells method in communications
    """
    chebyshev_dist = np.max(np.abs((scores) - (Hassan)))
    chebyshev_vals.append(chebyshev_dist)

    results[team_name] =\
    {
        "cosine similarity" : cosine_sim,
        "euclidean_distance": euclidean_dist,
        "manhatan_distance" : manha_dist,
        "correlation_coefficient" : corr,
        "chebyshev_distance" : chebyshev_dist
    }


#-----------------
#visualizing each function -can be used to find the "enough" methods needed
#-----------------

teams_names = list(teams.keys())
# Create subplots
fig, axs = plt.subplots(3, 2, figsize=(12, 10))
fig.suptitle('Similarity and Distance Metrics between Hassan and Teams')

# Plot Cosine Similarity
axs[0, 0].bar(teams_names, cosine_vals, color='skyblue')
axs[0, 0].set_title('Cosine Similarity')
axs[0, 0].set_ylim(0, 1.1)  # Cosine similarity ranges roughly between 0 and 1

# Plot Euclidean Distance
axs[0, 1].bar(teams_names, euclidean_vals, color='orange')
axs[0, 1].set_title('Euclidean Distance')

# Plot Manhattan Distance
axs[1, 0].bar(teams_names, manhattan_vals, color='green')
axs[1, 0].set_title('Manhattan Distance')

# Plot Pearson Correlation Coefficient
axs[1, 1].bar(teams_names, correlation_vals, color='purple')
axs[1, 1].set_title('Pearson Correlation')
axs[1, 1].set_ylim(-1, 1)  # Correlation ranges from -1 to 1

# Plot Chebyshev Distance
axs[2, 0].bar(teams_names, chebyshev_vals, color='red')
axs[2, 0].set_title('Chebyshev Distance')

# Hide the empty subplot (2,1)
axs[2, 1].axis('off')

plt.tight_layout(rect=[0, 0.03, 1, 0.95])  # Adjust layout to fit title
plt.show()

#-------------
#Recommendation
#-------------

#normalization to make them follow a simillar weight
#less distance is better while higher similarity is better

#distances
def normalize_distance(distances):
    max_dist = max(distances)
    return [1 - (d / max_dist) for d in distances]

ecelidean_norm = normalize_distance(euclidean_vals)
manhattan_norm = normalize_distance(manhattan_vals)
chebysex_norm  = normalize_distance(chebyshev_vals)

#similiarties
correlation_norm = [(c + 1) / 2 for c in correlation_vals]  #from -1 : 1 , to 0 : 1

cosine_norm = cosine_vals

#combined score for each team 
combined_scores = [] 
for i in range(len(teams_names)) :
    combined_score = np.mean([cosine_norm[i],ecelidean_norm[i],manhattan_norm[i]
                        ,chebysex_norm[i],correlation_norm[i]])
    combined_scores.append(combined_score)

best_index = np.argmax(combined_scores) #returns index to highest value
team = teams_names[best_index]

#final_results
plt.figure(figsize=(6, 5))
plt.bar(teams_names, combined_scores, color='goldenrod')
plt.title('Combined Similarity')
plt.ylabel('Score')
plt.ylim(0, 1)                   # scores are between 0 and 1
for i, score in enumerate(combined_scores):
    plt.text(i, score + 0.02, f"{score:.2f}", ha='center', fontweight='bold')

plt.tight_layout()
plt.show()