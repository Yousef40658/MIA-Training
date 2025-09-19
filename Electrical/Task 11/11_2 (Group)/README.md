# Task 11_2 — Classical Detection ⚡🔍  

---

## 📌 Overview  

In this task, we explored **classical computer vision methods** to detect and classify geometric shapes.  

At first, we tried **masking-based cropping**, but it failed in varied environments 🌍.  
Switching to **feature detection** yielded far better results — proving classical methods can be fast, reliable, and effective when time or data is limited.  

---

## 👨‍💻 My Contribution  

- Worked alongside a teammate to **write the feature detection + cropping code** ✍️  
- Helped test and refine the pipeline so it could handle noisy and diverse images robustly.  

---

## 🛠️ Key Concepts  

- **Feature Detection (ORB)** → Extract distinctive keypoints.  
- **Euclidean Distance** → Measure similarity between features.  
- **Lowe’s Ratio Test** → Filter strong matches.  
- **DBSCAN Clustering** → Group matched keypoints & extract regions.  
- **Contour Approximation** → Classify shapes by polygon side count.  

---

## 🚀 Pipeline  

### 🔹 Stage 1: Object Matching & Cropping  
1. Load object & compute ORB features.  
2. Match with dataset images (BFMatcher + Lowe’s Ratio Test).  
3. Cluster matches with DBSCAN → extract largest cluster.  
4. Crop & save detected objects.  

### 🔹 Stage 2: Shape Detection  
1. Preprocess images → resize, grayscale, blur, threshold.  
2. Extract contours & filter noise.  
3. Approximate polygons:  
   - 3 sides → **Triangle** 🔺  
   - 4 sides → **Quadrilateral** ⬛  
   - 5 sides → **Pentagon** 🔷  
   - 6 sides → **Hexagon** ⬢  
   - 12 sides → **Cross** ❌  
   - Otherwise → **Circle** ⚪  
4. Save results in `detected/`.  

---

## 📊 Evaluation  

System accuracy defined as:  

$$
\text{Accuracy} = \frac{TP}{TP + FP + FN}
$$  

Where:  
- **TP** → Correctly detected shapes ✅  
- **FP** → Non-shapes detected as shapes ❌  
- **FN** → Missed shapes ⚠️  

---
