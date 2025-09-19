# Task 7: ROS-Based Robot Control 🤖  

This task introduced me to **ROS (Robot Operating System)** and gave me hands-on experience with:  
- **Topics, Subscribers, Publishers**  
- **Services and Clients**  
- Implementing both **individual ROS tasks** and a **group project**  

It was also my **first group task**, which made coordination a challenge at first, but it taught me valuable collaboration skills 🧑‍🤝‍🧑.  

---

## 👥 My Contribution  
- Helped implement the **keyboard-to-robot control flow** (mapping keys → topics → robot actions).  
- Worked on the **movement node** to ensure correct velocity/angular limits.  
- Assisted in debugging the **arm and gripper nodes** to sync them with `/orders` input.  

---

## 🚀 Features  

### 🤖 Robot Movement  
**Controls:** `W A S D`  
- Forward / Left movement  
- **Rotation** 🔄  
- Velocity gradually increases (capped at **0.33**)  

---

### 🦾 Arm Movement  
**Controls:** `I J K L U O`  
- **3 Degrees of Freedom (DoF)** → efficient enough to hold blocks  

---

### ✋ Gripper  
**Controls:** `C Z`  
- `C` → Catch ✊  
- `Z` → Drop 🖐️  

---

## ⚙️ How It Works  

1. **Keyboard Input** ⌨️  
   - Publishes the ASCII character of the pressed key over the `/orders` topic.  

2. **Movement Node** 🚗  
   - Subscribes to `/orders` and publishes velocity & angular commands (limited to `0.33`) on `/cmd_vel`.  
   - Robot subscribes to `/cmd_vel` → executes motion.  

3. **Arm Joints Node** 🦾  
   - Subscribes to `/orders` and moves arm joints.  
   - 3 DoF controlled efficiently with 3 buttons.  

4. **Gripper Node** ✋  
   - Opens/closes gripper according to input key.  

---

## 🎬 Demo  
- [Demo 1](https://drive.google.com/file/d/1l53LkXEmmGMmrrj6njCe8kzIjO3Gq4c5/view?usp=drive_link)  
- [Demo 2](https://drive.google.com/file/d/1R2i-TYUqNVVUHNKyZny1OY6OE9nG-r__/view?usp=drive_link)  

---

## 📂 GitHub  During Task
👉 [Task 7 ROS Repository](https://github.com/Yousef40658/Task-7_ROS) *(latest files were not uploaded)*  
