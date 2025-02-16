from Motor import *
from Ultrasonic import *
import time
import random

class SmartNavigator:
    def __init__(self):
        self.motor_controller = Motor()
        self.distance_sensor = Ultrasonic()
        self.min_safe_distance = 25  # Increased safe distance to 25cm
        self.scan_delay = 0.2
        self.movement_speeds = {
            'forward': 800,
            'backward': -1200,
            'turn': 1800
        }
        
    def scan_surroundings(self):
        """Scan environment and return best direction to move"""
        left_dist = self.distance_sensor.get_distance()
        time.sleep(self.scan_delay)
        center_dist = self.distance_sensor.get_distance()
        time.sleep(self.scan_delay)
        right_dist = self.distance_sensor.get_distance()
        
        return {
            'left': left_dist,
            'center': center_dist,
            'right': right_dist
        }
    
    def execute_movement(self, movement_type, duration=0.5):
        """Execute a specific movement pattern"""
        if movement_type == 'forward':
            speed = self.movement_speeds['forward']
            self.motor_controller.setMotorModel(speed, speed, speed, speed)
        elif movement_type == 'backward':
            speed = self.movement_speeds['backward']
            self.motor_controller.setMotorModel(speed, speed, speed, speed)
        elif movement_type == 'turn_left':
            speed = self.movement_speeds['turn']
            self.motor_controller.setMotorModel(-speed, -speed, speed, speed)
        elif movement_type == 'turn_right':
            speed = self.movement_speeds['turn']
            self.motor_controller.setMotorModel(speed, speed, -speed, -speed)
            
        time.sleep(duration)
        self.motor_controller.setMotorModel(0, 0, 0, 0)
    
    def navigate(self):
        while True:
            surroundings = self.scan_surroundings()
            
            # If path is clear, move forward
            if all(dist > self.min_safe_distance for dist in surroundings.values()):
                self.execute_movement('forward', 0.3)
                continue
                
            # Obstacle detected - implement avoidance
            self.execute_movement('backward', 0.4)
            
            # Choose turn direction based on available space
            if surroundings['left'] > surroundings['right']:
                self.execute_movement('turn_left', 0.6)
            else:
                self.execute_movement('turn_right', 0.6)
            
            time.sleep(0.1)  # Brief pause between actions
    
    def shutdown(self):
        """Safely stop all motors"""
        self.motor_controller.setMotorModel(0, 0, 0, 0)

if __name__ == '__main__':
    navigator = SmartNavigator()
    try:
        print("Starting autonomous navigation...")
        navigator.navigate()
    except KeyboardInterrupt:
        print("\nStopping navigation...")
        navigator.shutdown() 