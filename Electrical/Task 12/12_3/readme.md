# Task 12_3 — Kinematics & Robotic Arm 🤖🦾  

---

## 📌 Overview  

In this task, I explored kinematics further by **modifying the MIA team’s robot** to include:  
- A **2-DOF robotic arm** 🦿  
- A **gripper as the end-effector** ✋  

The goal was to maximize understanding of **how the arm should be placed software-wise** to ensure reliable real-life assembly.  

---

## 🧠 arm_brain Node  

- Implemented an **`arm_brain` node** 🧠 that checks:  
  - If the joints can grab the target ✅  
  - Or if the target is unreachable ❌  
- This lays the foundation for **automating the robot** (previously manually controlled in Task 1).  

---

## 🛠️ Technical Work  

- Built a **launch file** to run the robot in both **Gazebo** and **RViz**:  
  - Option to enable/disable **LiDAR sensor** 🔦  
  - Gained understanding of **launch file syntax**.  
- Controlled robot movement in **Gazebo** using **self-built nodes**.  
- Calculated **wheel speeds** using **mecanum wheel equations** ⚙️.  
  - Used nodes such as key_binding and moving which showed again that keeping reusable files is highly important . 

---

## ✨ Reflection  

- The task description was detailed and well-prepared 📋.  
- With a strong conceptual understanding, the coding part became much more straightforward 💡.  

---

## 🎬 Demo  
- [Demo](https://drive.google.com/file/d/1vJKwi4fatV2Gjm2CwD5lU-j4IyQFHIH4/view?usp=drive_link)  