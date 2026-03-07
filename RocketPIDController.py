class RocketPIDController:
    """
    PID controller specifically for rocket airbrakes using apogee prediction.
    
    Uses PREDICTED apogee (not current altitude) as the process variable
    Updates based on prediction error
    """
    
    def __init__(self, rocket_sim, target_apogee, kp, ki, kd, dt=0.01):
        """
        Initialize rocket PID controller.
        
        Parameters:
        -----------
        rocket_sim : RocketAltitudeSimulator
            The rocket simulator
        target_apogee : float
            Desired final altitude (m)
        kp, ki, kd : float
            PID gains
        dt : float
            Time step (s)
        """
        self.rocket = rocket_sim
        self.target = target_apogee
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        
        # PID state
        self.integral = 0.0
        self.prev_error = 0.0
    
    def step(self):
        """
        Take one control step:
        1. Get current predicted apogee
        2. Calculate error from target
        3. Compute PID adjustment
        4. Update brake position
        5. Step the simulator forward
        """
        # Get predicted apogee with current brake position
        predicted_apogee = self.rocket.get_output()
        
        # Error: predicted - target (positive = overshooting, negative = undershooting)
        error = predicted_apogee - self.target  # ← CHANGED: flipped the sign
        
        # PID terms
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        
        # PID output (how much to CHANGE brake position)
        adjustment = self.kp * error + self.ki * self.integral + self.kd * derivative
        
        # Get current brake position and adjust it
        current_brake_pos = self.rocket.brake_position
        new_brake_pos = current_brake_pos + adjustment  # ← CHANGED: plus instead of minus
        
        # Clamp to valid range [0, 1]
        new_brake_pos = max(0.0, min(1.0, new_brake_pos))
        
        # Set new brake position
        self.rocket.set_input(new_brake_pos)
        
        # Step the simulation forward
        self.rocket.step(self.dt)
        
        # Update for next iteration
        self.prev_error = error
        
        return predicted_apogee, error, new_brake_pos
