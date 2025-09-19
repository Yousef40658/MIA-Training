# Task 9 — PCB Design & Fabrication ⚡🔧  

In this task, we worked on **PCB design** and **Circuit protection methods** using Altium Designer and also fabricated a **voltage regulation circuit** in real life.  
It was a practical step from design → hardware, where I even messed up my first soldering attempt 😅, but got it right the next two times ✅ — making me excited to build more projects involving **self-soldering** and **PCB design**.  

---

## 👥 My Contribution  
- Assisted on the **schematic of the sensors board**.  
- Designed the **PCB layout for the sensors board**.    
- Designed the  **Actuators board schematic design**.  

---

## 🛡️ General Protection Methods  
- **P-channel MOSFET**: Prevents reverse polarity — faster response & lower voltage drop ⚡  
- **TVS Diode**: Protects against voltage spikes (better than Zener at high current) ✅  
- **Ceramic Capacitor**: Handles fast, low fluctuations 🟢  
- **Aluminum Bulk Capacitor**: Handles larger, slower fluctuations 🟡  

**Design Note**: Capacitors must be placed close to components to:  
1️⃣ Provide instant voltage during fluctuations.  
2️⃣ Avoid inductance issues from longer tracks.  

**Indicators** → give visual feedback, always protected by resistors 💡  

---

# 📋 Boards Overview  

## 1️⃣ Actuator Board  
### Features  
- Buck converter → powers servo 🔧  
- Voltage regulator → powers STM32 💻  
- USB Type B → upload code 🖥️  
- Motor driving circuit → powered by STM GPIO ⚡  

### Design Notes  
- Servo pins → PWM signals 🔄  
- USB Type B → digital connection 🔌  

### Protection Highlight  
- Separate filter protects 12V input 🛡️  

---

## 2️⃣ Sensor Board  
### Features  
- 3.3V voltage regulator → powers MPU6050, BNO055, and ultrasonic sensor ⚡  
- USB Type B 🖥️  

### Design Notes  
- Ultrasonic echo pin → mapped to interrupt ⏱️  
- Sensor placement avoids blocked readings 📏  

### Protection Highlight  
- Voltage divider keeps Echo safe (<3.3V for STM32) ⚡  
- I2C lines protected by TVS Diode + filtering 🟢  

---

## 3️⃣ Power Distribution Board  
### Features  
- 2 Buck converters → regulate 12V to 8.4V & 5V ⚡  

### Design Notes  
- Output voltage tuned by resistors ⚡  

### Protection Highlights  
- Input diode instead of J-MOSFET → drop not critical ✅  
- Output MOSFET ensures reliable 5V ⚡  
