# Task 14 — Competition 🏆🤖  

---

## 👤 My Contribution  

- Integrated **ultrasonic sensors** into the robot model at correct positions, and created simulation topics that mirrored the real robot.  
- Built a **maze with real specifications**, ensuring that simulation results closely matched what would happen on hardware — essential since we had **limited access to the real robot**.  
- Implemented **letter detection** using a pre-trained model:  
  - Masked the background of letters.  
  - Appended letters with **green background** to a text file.  
  - Ignored letters with **red background** or environmental letters (maze was yellow → risky to misclassify).  
- Worked with **Mahmoud** pulling an **all-nighter**, programming and testing the movement code in Gazebo until multiple successful runs were achieved.  
- Final hardware tests required **only minor tweaks**, and the robot matched simulation expectations closely.  

---

## 🔧 Technical Overview  

### 🖥️ Simulation  
- Used a **scaled Shato model** with a **customized maze map**.  
- Added **ultrasonic sensors** on both sides and front.  
- Built simulation topics:  
  - `/yaw` (from `/imu`)  
  - `/ultrasonics` to closely mirror real hardware readings.  

### 📷 Camera  
- Integrated camera with **OpenCV + masking techniques** for letter and sign detection.  

### 🤖 Movement  
- Tested several approaches, including **PID** and **robot centralizing**, but found limitations for the simple maze.  
- Final movement solution: a **straightforward ROS node** using `if` statements, `while` loops, and sensor readings — robust enough for multiple runs.  

---

## ⚠️ Challenges & Solutions  

- **Camera delay** → mitigated by capturing only when necessary, reducing GPU load and lag (thanks to Dandy’s insight).  
- **Slow movement** → lower speeds minimized errors; higher speeds would increase efficiency but required careful balancing.  
- **Limited hardware access** → simulation maze closely matched real maze specifications to maximize accuracy.  

---

## 🔬 Further Research  

- Investigated **left/right-hand follower method**.  
- Explored **dead-end filling strategy** for maze navigation optimization.  

---

## 👥 Non-Technical Overview  

- Improved **GitHub workflow**: branching, merging, and working in a shared file efficiently.  
- Learned to **coordinate within a single file**, which was challenging at first.  
- Realized the importance of **parallel documentation/presentation preparation** next time to avoid last-minute work.  

---

## 🥇 Result  

- Minimal hardware testing required since simulation provided accurate predictions.  
- Completed the maze with **full letter and sign detection** in **8m 4s**.  
- 🥇 Secured **#1 position in Technical Rating** 🎉  
