class State:
    """Holds the current state of the rocket (altitude, velocity, acceleration, etc)."""
    def __init__(self, altitude=0, velocity=0, acceleration=0, time=0):
        self.altitude = altitude
        self.velocity = velocity
        self.acceleration = acceleration
        self.time = time
    
    def update(self, altitude, velocity, acceleration):
        self.altitude = altitude
        self.velocity = velocity
        self.acceleration = acceleration