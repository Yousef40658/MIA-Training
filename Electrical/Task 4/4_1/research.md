# research.md

### ⛓️ Bridge Between Components
- The bridge between components controls transmission speed to match the **slowest device**, while the **fastest device** keeps its regular speed.  
  (e.g., USB hub with mixed-speed devices, or a shared bus).

---

### 🚀 DMA (Direct Memory Access)
- DMA is special hardware that allows **peripherals ↔ memory data transfer without CPU**.  
- Normally peripherals interrupt the CPU → extra load.  
- With DMA:
  - CPU is free for other tasks.  
  - Transfers are faster and efficient.  
  - Used in **real-time systems**.  
- DMA controller signals CPU only when done (via interrupt).  

---

### ⚡ GPIO Modes
- **OUTPUT** → signal out of microcontroller.  
- **INPUT** → signal into microcontroller.  
- **INPUT PULL-UP** → prevents floating input (internally tied to Vcc with high resistance).  
- **INPUT PULL-DOWN** → tied to GND with high resistance.  
- **HIGH IMPEDANCE (Hi-Z)** → input with no pull-up/down.  
  - Used for shared buses and bidirectional lines.  
  - Prevents interference on unused pins.  

💡 Why Hi-Z?  
- If one device drives HIGH and another LOW → **short circuit 🔥**.  
- Solution: only master drives the line, others stay Hi-Z (like disconnected).  

---

### 🖧 I²C (Inter-Integrated Circuit)
- Shared bus: **SDA** (data) + **SCL** (clock).  
- **Open-drain outputs**:
  - Devices only pull line LOW (0).  
  - To send HIGH (1) → device releases line (Hi-Z).  
- **Pull-up resistors** keep line HIGH when no device pulls it LOW.  
- Multiple devices share bus → only one talks at a time.  

---

### ⏱️ Interrupts vs Polling
- **Interrupt** → CPU stops main() flow, runs ISR (interrupt service routine), then resumes main.  
- **Interrupt controller**:  
  - Detects and prioritizes interrupts.  
  - Status register tracks requests.  
  - CPU acknowledges, clears status, and jumps to ISR (via vector table).  

🔄 **Steps of Interrupt Handling**  
1. Interrupt occurs.  
2. Interrupts disabled + context saved (where code stopped).  
3. ISR executes (CPU jumps to vector table address).  
4. Context restored, interrupts re-enabled, main program resumes.  

- **Polling** → CPU keeps checking in loop if event happened → wastes time.  
- **Interrupts** → CPU only reacts when needed → efficient.  

📌 Terms:  
- **Interrupt latency** → delay from request start to ISR execution.  
- **Interrupt response** → request start to end of ISR.  

---

### ⏲️ Timers
- Hardware components that **count clock cycles** to measure time or generate events.  
- Clock period = time for one cycle.  
- On overflow → interrupt triggered.  

---

### 🔉 PWM (Pulse Width Modulation)
- Digital signal rapidly switched ON/OFF to simulate analog control.  
- Controls **motor speed**, **LED brightness**, etc.  
- **Duty cycle** = % of time signal is HIGH in one period.  

---

### 🤖Basics
- **delay(ms)** → stops everything for given time.  
- Good for simple code.  
- Bad if you want multitasking (e.g., do something every 5s without freezing everything).  
  → Use **timers / millis()** instead of delay().  
