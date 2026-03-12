import numpy as np
from Simulator import Simulator


def air_density(altitude):
    """Calculate air density at altitude."""
    rho_0 = 1.06  # sea level density (kg/m³)
    scale_height = 8500  # meters
    return rho_0 * np.exp(-altitude / scale_height)


def predict_apogee(h_current, v_current, brake_position, mass, Cd, A_base, A_max, dt=0.01):
    """
    Predict final apogee from current state.
    
    Runs a fast simulation forward to see where the rocket will end up
    with the given brake position.
    
    Parameters:
    -----------
    h_current : float
        Current altitude (m)
    v_current : float
        Current velocity (m/s)
    brake_position : float
        Brake position 0.0 (closed) to 1.0 (open)
    mass : float
        Rocket mass (kg)
    Cd : float
        Drag coefficient
    A_base : float
        Base drag area (m²)
    A_max : float
        Max drag area with brakes (m²)
    dt : float
        Timestep for prediction (s)
    
    Returns:
    --------
    float : Predicted apogee altitude (m)
    """
    h = h_current
    v = v_current
    g = 9.81
    
    # Calculate drag area for this brake position
    A = A_base + (A_max - A_base) * brake_position
    
    # Simulate forward until apogee
    max_steps = 1000  # safety limit
    for _ in range(max_steps):
        if v <= 0:  # reached apogee
            break
        
        rho = air_density(h)
        F_drag = 0.5 * rho * Cd * A * v**2
        a = -g - F_drag / mass
        
        v = v + a * dt
        h = h + v * dt
        
        if h < 0:
            break
    
    return h


class RocketAltitudeSimulator(Simulator):
    """
    Rocket simulator that outputs PREDICTED apogee.
    
    Allows PID to control based on where the rocket will end up
    
    Input: brake_position (0.0 = closed, 1.0 = fully open)
    Output: predicted apogee (meters)
    """
    
    def __init__(self, h0, v0, mass, Cd, A_base, A_max):
        """
        Initialize rocket.
        
        Parameters:
        -----------
        h0 : float
            Initial altitude (m)
        v0 : float
            Initial velocity (m/s, positive = upward)
        mass : float
            Rocket mass (kg)
        Cd : float
            Drag coefficient
        A_base : float
            Base drag area with brakes closed (m²)
        A_max : float
            Maximum drag area with brakes fully open (m²)
        """
        super().__init__(min_input=0.0, max_input=1.0)
        
        # Rocket parameters
        self.mass = mass
        self.Cd = Cd
        self.A_base = A_base
        self.A_max = A_max
        self.g = 9.81  # m/s²
        
        # Initial conditions (for reset)
        self.h0 = h0
        self.v0 = v0
        
        # Current state
        self.h = h0
        self.v = v0
        self.brake_position = 0.0
    
    def set_input(self, brake_position):
        """
        Set airbrake position.
        
        Parameters:
        -----------
        brake_position : float
            Brake position from 0.0 (closed) to 1.0 (fully open)
        """
        self.brake_position = np.clip(brake_position, 0.0, 1.0)
    
    def get_output(self):
        """
        Returns:
        --------
        float : Predicted final apogee (m)
        """
        return predict_apogee(
            self.h, 
            self.v, 
            self.brake_position,
            self.mass, 
            self.Cd, 
            self.A_base, 
            self.A_max
        )
    
    def step(self, dt):
        """
        Step simulation forward by dt seconds.
        Updates the rocket's actual position and velocity based on
        current brake position.
        
        Parameters:
        -----------
        dt : float
            Time step (seconds)
        """
        # Calculate current drag area based on brake position
        A = self.A_base + (self.A_max - self.A_base) * self.brake_position
        
        # Get air density at current altitude
        rho = air_density(self.h)
        
        # Calculate drag force (opposes motion)
        F_drag = 0.5 * rho * self.Cd * A * self.v**2
        
        # Net acceleration (negative = downward)
        a = -self.g - F_drag / self.mass
        
        # Update velocity and altitude using Euler integration
        self.v = self.v + a * dt
        self.h = self.h + self.v * dt
        
        # Don't go below ground
        if self.h < 0:
            self.h = 0
            self.v = 0
    
    def get_altitude(self):
        """
        This is useful for tracking real position during simulation.
        
        Returns:
        --------
        float : Current altitude (m)
        """
        return self.h
    
    def get_velocity(self):
        """
        Returns:
        --------
        float : Current velocity (m/s, positive = upward)
        """
        return self.v
    
    def reset(self):
        """Reset simulator to initial conditions."""
        self.h = self.h0
        self.v = self.v0
        self.brake_position = 0.0
