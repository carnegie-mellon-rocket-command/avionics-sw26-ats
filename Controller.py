from State import State
from ATS import ATS

class Controller: # this is where loop will happen
    def __init__(self, target_apogee, sim):
        self.target_apogee = target_apogee
        self.sim = sim #simulation or real data

    def compute_command(self, state: State):
        """Given sensor data, output the airbrake position we want"""
        pass
        
