# Task: Autonomous Grid Robot 🤖🗺️

In this task, I built a **simple robot** that can navigate on a **grid map** toward a given **destination** 📍 while avoiding obstacles 🚧.  

### How It Works ⚙️
- The map is represented in a **grid style**, where both the **destination** and the **obstacles (with their specs)** are defined.  
- The robot uses **two wheels** controlled by **PWM signals** ⚡.  
- Based on obstacle distance, the wheels adjust their movement speed.  
- The **A* Search algorithm** is used by the **first Arduino** to calculate the **next grid block** the robot should move to ➡️.  
- This movement instruction is then sent to the **second Arduino**, which controls the **two servo motors**.  
- Once the robot reaches its destination, the motors stop 🚦.  

### Outcome 🏁
- Implemented **A* pathfinding** on Arduino.  
- Learned to coordinate between **two microcontrollers** (one for path planning, one for motor control).  
- Applied **PWM control** for smooth motor movement.  
- Built the foundation for **more advanced autonomous robots** 🤝.  
