# AI Research & Reflection 🤖📚

This work focused on researching the mathematical foundations required for solid ML modeling rather than rushing to write lots of code documentation.  
I implemented models correctly and understood the code, but decided the time was better spent deepening math knowledge first — then I’ll come back and write detailed, accurate markdowns for each model. 🔁

---

## Highlights — topics studied 📌
- **Math foundations**: linear algebra, probability, and the math behind training choices. 📐  
- **Data prep & analysis**: cleaning, handling NaNs, detecting skewness, bias, outliers, and feature correlations. 🧹🔍  
- **Feature engineering**: when to drop low-relevance features vs. when they provide useful noise; encoding categorical values (e.g., male/female → numbers); clipping extreme values; creating new features to improve models. 🛠️  
- **Exploratory plotting**: using Matplotlib to visualize distributions, correlations, and model behavior. 📊  
- **Data structures**: differences between plain Python lists-of-lists and NumPy arrays (performance & semantics). 🧮  
- **Embeddings**: intuition for embeddings vs raw numbers (recommended explainer: 3Blue1Brown on transformers). 🌐  
- **Algorithms practiced**:  
  - Built-from-scratch **Logistic Regression** (training, parameter tuning). 🧠  
  - Tried **KNN**, **LightGBM (LGB)**, and pre-built logistic regression. ⚙️  
  - Hyperparameter search methods: **grid search**, **random search**, and **Bayesian search**. 🔎

---

## Practical workflow notes 🔧
- Always inspect data first: check missingness, skew, correlations, and suspicious IDs. ✅  
- Replace vs remove NaNs: decide per-feature based on distribution and business logic. ⚖️  
- Negative sampling / adding varied backgrounds (for vision tasks) reduces overfitting. 🚫🎯  
- Small *k* in KNN → higher overfitting risk; logistic regression can be more stable on smaller datasets. 📉  
- Use visualizations to guide feature decisions before modeling. 🖼️

---

## Outcome & plan 🎯
- I wrote correct model code and understood its operation, but I rerouted time to **study math deeply** (see `research/` folder). 📂  
- Next steps: finish math study → revisit each model and write a specific, high-quality markdown per model with full understanding. 🔁✅  
---

## Reference 🔗
- For embeddings intuition: 3Blue1Brown — *highly recommended*.  
- See the `research/` folder for detailed notes and readings.

