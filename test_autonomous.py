import time
import cv2
import numpy as np
from mapping import Mapper
from Motor import Motor
import signal
import sys

class AutonomousTestSystem:
    def __init__(self):
        print("Initializing autonomous system...")
        self.running = True
        self.detected_objects = []
        
        # Configure test parameters
        self.test_duration = 120  # 2 minutes test duration
        self.start_time = None
        
        try:
            self.mapper = Mapper()
            self.motor = Motor()
        except Exception as e:
            print(f"Error initializing hardware: {e}")
            sys.exit(1)
        
        # Set up emergency stop
        signal.signal(signal.SIGINT, self.emergency_stop)
        
    def emergency_stop(self, signum, frame):
        """Emergency stop handler"""
        print("\nEmergency Stop Activated!")
        self.running = False
        try:
            self.motor.setMotorModel(0, 0, 0, 0)
        except:
            pass
        sys.exit(0)
        
    def print_status(self):
        """Print current status information"""
        print("\n=== Autonomous System Status ===")
        print(f"Time Elapsed: {time.time() - self.start_time:.1f} seconds")
        print(f"Current Position: ({self.mapper.car_pos[0]:.1f}, {self.mapper.car_pos[1]:.1f})")
        print(f"Current Angle: {self.mapper.car_angle:.1f} degrees")
        if self.mapper.goal is not None:
            print(f"Distance to Goal: {np.linalg.norm(self.mapper.car_pos - self.mapper.goal):.1f} cm")
        print("==============================\n")
        
    def test_movement(self):
        """Test basic movement capabilities"""
        print("Testing basic movements...")
        
        # Test forward movement
        print("Moving forward...")
        self.motor.setMotorModel(1000, 1000, 1000, 1000)
        time.sleep(2)
        self.motor.setMotorModel(0, 0, 0, 0)
        time.sleep(1)
        
        # Test rotation
        print("Testing rotation...")
        self.motor.setMotorModel(-1000, -1000, 1000, 1000)
        time.sleep(1)
        self.motor.setMotorModel(0, 0, 0, 0)
        time.sleep(1)
        
        print("Basic movement test completed")
        
    def run_test(self):
        """Run the autonomous navigation test"""
        print("\nInitializing Autonomous Test System...")
        print("Place the car at the starting position and ensure obstacles are set up.")
        input("Press Enter to begin the test...")
        
        self.start_time = time.time()
        
        try:
            # First test basic movements
            self.test_movement()
            
            # Set goal position (5 feet north, 5 feet east)
            self.mapper.set_goal(5, 5)
            print(f"Goal set to: {self.mapper.goal}")
            
            while self.running and (time.time() - self.start_time) < self.test_duration:
                # Perform initial scan
                print("Scanning surroundings...")
                self.mapper.scan_surroundings()
                
                # Find path to goal
                path = self.mapper.find_path()
                if path:
                    print("Path found! Starting navigation...")
                    
                    # Follow path while continuously monitoring for obstacles
                    while len(self.mapper.path) > 1 and self.running:
                        # Move to next waypoint
                        if not self.mapper.move_to_next_waypoint():
                            print("Failed to reach waypoint")
                            break
                        
                        # Print status
                        self.print_status()
                        
                        # Short pause to allow for sensor readings
                        time.sleep(0.5)
                        
                        # Scan for new obstacles
                        self.mapper.scan_surroundings()
                        
                else:
                    print("No path found to goal")
                    time.sleep(1)
                
                # Check if we've reached the goal
                if np.linalg.norm(self.mapper.car_pos - self.mapper.goal) < 30:  # Within 30cm
                    print("\nGoal reached successfully!")
                    break
                    
        except Exception as e:
            print(f"Error during test: {e}")
        finally:
            # Stop all motors
            try:
                self.motor.setMotorModel(0, 0, 0, 0)
            except:
                pass
            print("\nTest completed!")
            print(f"Final position: ({self.mapper.car_pos[0]:.1f}, {self.mapper.car_pos[1]:.1f})")
            print(f"Total time: {time.time() - self.start_time:.1f} seconds")

if __name__ == "__main__":
    print("Starting autonomous test system...")
    try:
        test_system = AutonomousTestSystem()
        test_system.run_test()
    except Exception as e:
        print(f"Fatal error: {e}")
        sys.exit(1) 