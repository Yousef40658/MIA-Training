#ref for first points
#https://www.youtube.com/watch?v=6ojuNMZiAds
import time
import matplotlib.pyplot as plt
#-------------
#PID class
#-------------
class PidController() :
    def __init__(self , Kp , Ki , Kd , setpoint):
        #PID weights
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.setpoint = setpoint #desired value

        self.integral   = 0
        self.prev_error = 0 
        self.prev_time  = None

    #update desired target
    def set_setpoint(self, setpoint) :
        self.set_setpoint = setpoint
        self.reset()

    #resets integral and prev error when restarting
    def reset(self) :
        self.integral   = 0
        self.prev_error = 0 
        self.prev_time  = 0 

    def compute(self,current_val ,dt):
        error = self.setpoint - current_val 
        self.integral += error * dt
        self.deriv = ((error - self.prev_error) / dt) if dt > 0 else 0

        self.prev_error = error #store error for next loop

        output = self.Kp * error + self.Ki * self.integral + self.Kd * self.deriv
        return output 


if __name__ == "__main__":
    #-------------
    #Simulation -> robot moving in 1 line
    #-------------
    pid = PidController(Kp=20 , Ki= 1.556 , Kd= 0.08 , setpoint=10)

    #initial states 
    position = 0
    velocity = 0
    dt = 0.01       #time step
    steps = 120 

    positions = []
    times     = []

    #for plotting
    for i in range(steps) :
        control_signal = pid.compute(position , dt)

        velocity = control_signal
        position += velocity * dt      #d = v * t

        positions.append(position)
        times.append(i * dt) 
        time.sleep(0.05)

    # ---------------- Plot results ----------------
    plt.plot(times, positions, label="Robot Position")
    plt.axhline(y=pid.setpoint, color='r', linestyle='--', label="Target")
    plt.xlabel("Time (s)")
    plt.ylabel("Position")
    plt.legend()
    plt.grid()
    plt.show()