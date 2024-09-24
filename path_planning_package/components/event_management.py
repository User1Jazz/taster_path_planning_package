from time import time, sleep
from math import sqrt

class EventManager():

    # TRACK ATTRIBUTES
    max_laps = 0
    lap_counter = 0
    
    # TIMEOUT ATTRIBUTES
    timeout = 0
    timer = 0
    started = False

    # FRAME ATTRIBUTES
    stop_zone_counter = 0
    stop_zone_counter_max = 0
    lap_timer = 0
    
    def __init__(self, _timeout = float(5), _max_laps=1, _stop_zone_counter_max=0):
        self.timeout = _timeout
        self.max_laps = _max_laps
        self.stop_zone_counter_max = _stop_zone_counter_max
        print("Event manager initialised")

    """
    Initialise timer
    """
    def set_timer(self, _timeout):
        self.timeout = _timeout
        self.started = False
        
    """
    Function to check the timer (this is also used to initiate it)
    """
    def check_timeout(self):
        if not self.started:
            self.timer = time()
            self.started = True
        delta = time() - self.timer
        self.timeout -= delta
        self.timer = time()
        if self.timeout > 0:
            return False
        return True
    
    """
    Function to increase lap count
    """
    def increase_lap(self):
        print(f"Lap before: {self.lap_counter}")
        self.lap_counter = self.lap_counter + 1
        self.print_laps()
    
    """
    Function to check the lap count
    """
    def check_lap(self):
        if self.lap_counter > self.max_laps: # +1 for first lap
            return True
        return False
        
    
    """
    Function to print lap
    """
    def print_laps(self):
        print(f"LAP {self.lap_counter-1} OUT OF {self.max_laps} REACHED")
    
    """
    Function to increase frame count
    """
    def increase_frame(self):
        self.stop_zone_counter += 1

    """
    Function to checkthe frame count
    """
    def check_frame_count(self):
        if self.stop_zone_counter >= self.stop_zone_counter_max:
            return True
        return False