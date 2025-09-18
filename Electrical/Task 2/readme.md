# Task 2: Motor Driver Circuit with Filter and Protection

In this task, I designed a filter and motor driver circuit based on college knowledge of power electronics. The circuit steps down and stabilizes the supply to run a **24 V DC motor** safely and efficiently.

### Key Features
- **Filter Circuit**  
  - Converts AC to DC using a **full bridge rectifier**.  
  - **Capacitor** used to reduce ripple voltage and stabilize the DC output.  

- **Switching Control**  
  - A **transistor** combined with a **J-MOSFET** acts as an electronic switch.  
  - The motor is activated when **5 V** is applied between the transistor’s N-side and ground.  

- **Protection Components**  
  - **TVS diodes** regulate voltage and suppress spikes from the AC power supply.  
  - A **2A fuse** in series with the motor protects against overcurrent conditions.  
  - A **TVS diode in parallel with the motor** mitigates switching spikes from inductive loads.  
  - An **LED (with series resistor)** can be added as a status indicator.  

### Outcome
- Achieved a **24 V DC output with low ripple voltage**.  
- Successfully drove a **24 V DC motor** using a 5 V control signal.  
- Ensured safe and reliable operation with proper filtering and protective components.

