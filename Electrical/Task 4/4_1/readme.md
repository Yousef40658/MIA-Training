# Task X: Interrupts and PWM for Sensor-Based Control

In this task, I explored **communication methods between devices**, the concept of **interrupts**, and the role of **timers and PWM** in controlling motors and LEDs. ⚙️

### Key Learnings
- **Interrupts** ⏸️  
  - Understood how interrupts can stop normal execution and immediately respond to critical events.  
  - Learned the benefits of interrupts for fast, efficient response to sensors.  

- **Timers and PWM** ⚡  
  - Gained knowledge of generating PWM signals for precise control of motors and LEDs.  
  - Understood how PWM duty cycle affects motor speed and LED brightness.  

### Implementation
- **Motion Sensor with Interrupts** 🚨  
  - A motion sensor was connected to an interrupt pin on Arduino.  
  - On detection, it stopped all running functions and activated an LED 💡.  

- **Ultrasonic Sensor with PWM** 📏  
  - Distance readings from an ultrasonic sensor controlled two motors via PWM pins.  
  - The farther the target, the faster the motors ran 🏎️.  
  - Established the groundwork for **PID control concepts**.  

### Outcome
- Built a system combining **interrupt-driven events** and **PWM motor control** 🔄.  
- Gained a practical foundation for **real-time control systems**.  
