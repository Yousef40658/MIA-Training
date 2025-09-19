# Task 13_1 — Introduction to PID Control 🎛️⚙️  

---

## 📌 Overview  

In this task, we explored the basics of **PID control** and why it’s superior to simple **proportional control** in many systems.  

- With proportional-only control, the system reacts **without caring about the consequences** of each step in real time.  
- **PID** improves this by **continuously checking consequences** and adjusting:  
  - ⚖️ **Kp (Proportional)** → reacts to present error  
  - ⏳ **Ki (Integral)** → accounts for past errors  
  - 🚀 **Kd (Derivative)** → predicts future errors  

This makes PID more robust, especially for systems where **behavior changes over time**.  

---

## 🧠 Key Insights  

- PID is best understood as:  
  👉 *“Changing things while continuously checking the consequences of each change.”*  
- Systems without PID either **don’t check at all** or check only after **large steps**.  
- Although full tuning was time-consuming and not feasible in this task, I gained intuition for **manual tuning** to achieve acceptable results.  

---

## 👨‍💻 My Contribution  

- Using the **PID values set by a teammate**, I:  
  - Integrated the system with **RViz**.  
  - Enabled **receiving setpoints** directly from RViz to drive the PID loop.  

---
