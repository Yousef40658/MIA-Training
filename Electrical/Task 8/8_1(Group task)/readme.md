# 🍽️ Task 8_1: ROS Restaurant Ordering System  

This project simulates a **restaurant ordering workflow** using **ROS (Robot Operating System)**, `actionlib`, and a **CustomTkinter GUI**.  
It models the full process: customers place orders → restaurant confirms via service → kitchen executes via action server → results and feedback returned.  

---

## 👥 My Contribution  
- Designed the **food file and menus** 📖  
- Implemented the **restaurant node** 🏢  
- Assisted with the **customer node**, ensuring compatibility with restaurant logic 👤  

This task helped me:  
- Understand the difference between a **Service** (instantaneous ⚡) and an **Action** (ongoing with feedback 🔁)  
- Apply **Object-Oriented Programming (OOP)** in a practical ROS project 💡  
- Explore **threading** 🧵 to run multiple operations in parallel  

---

## 🚀 Features  

### 🧾 Menu Management  
- `Item` class supports **sizes (S, M, L)** with adjusted price & cooking time  
- Extras (cheese 🧀, sauce, fries 🍟, etc.) increase both cost and time  

### 🏢 Restaurant Backend (`restaurant.py`)  
- Provides a **ROS Service** (`confirm_order`) to confirm orders  
- Runs a **ROS Action Server** (`execute_order`) to simulate cooking  
- Supports **multiple chefs in parallel** (default = 3)  
- Queues orders if all chefs are busy  

### 👤 Customer Node (`customer.py`)  
- Listens to `/orders` from GUI or CLI  
- Calls service to confirm orders  
- Extracts order ID and sends it to the action server  
- Receives **feedback & results** from the kitchen (20%, 40%, …, ✅ Completed)  

### 🖥️ GUI (`gui.py`)  
- Built with **CustomTkinter**  
- Displays menu categories & items  
- Popup windows for **size & extras** selection  
- Quick-order manual input supported  
- Publishes orders on `/orders` topic  

---

## 📦 Dependencies  
- **ROS (Noetic recommended)**  
- `rospy`  
- `actionlib`  
- `std_msgs`  
- `colorama` 🌈 (for colorful logs)  
- `customtkinter` (for GUI)  

---

## 🔄 Order Flow  

1. User selects item in GUI  
2. GUI publishes order (e.g., *M Pizza + cheese + fries*) → `/orders`  
3. Customer node calls `confirm_order` service → gets confirmation + ID  
4. Customer node sends ID to `execute_order` action server  
5. Restaurant simulates cooking with **feedback** (20%, 40%, …)  
6. Once finished → result sent back ✅ *Order Completed!*  

---

## 🎯 Outcome  
- Learned to combine **Services, Actions, OOP, and threading** inside ROS  
- Built a complete **simulation of a real-world workflow** (ordering → confirmation → execution)  
- Strengthened my skills in **parallel execution** and **ROS system design**  


## Demo
- [Demo](https://drive.google.com/file/d/1uL58Zymz-pGN6gnFIaUU3PowD_rh0-Ix/view=drive_link)  

## 📂 GitHub  During Task
👉 [Task 8 Repository](https://github.com/Yousef40658/Task_8)