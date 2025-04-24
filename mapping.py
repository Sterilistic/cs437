#!/usr/bin/python3.11
from __future__ import print_function, division
import numpy as np
import math
from servo import *
import time
from queue import PriorityQueue  # Python 3 queue
from Motor import *
import cv2
import os
from detect import detect_stop_sign
import RPi.GPIO as GPIO
from Ultrasonic import *
import io
import struct
from threading import Condition
from picamera2 import Picamera2  # Only use picamera2
from picamera2.encoders import JpegEncoder
from picamera2.outputs import FileOutput
from threading import Thread
import traceback

class StreamingOutput(io.BufferedIOBase):  # Use Python 3 class inheritance
    def __init__(self):
        self.frame = None
        self.condition = Condition()

    def write(self, buf):
        with self.condition:
            self.frame = buf
            self.condition.notify_all()

class Mapper:  # Use Python 3 class syntax
    def __init__(self, map_size=300):
        print("Starting Mapper initialization...")
        try:
            # Initialize map
            self.map_size = map_size
            self.map = np.zeros((map_size, map_size))
            self.car_pos = np.array([map_size//2, map_size//2])
            self.car_angle = 0
            self.goal = None
            
            # Initialize hardware first
            print("Initializing hardware...")
            try:
                self.ultrasonic = Ultrasonic()
                print("Ultrasonic sensor initialized")
                
                test_distance = self.ultrasonic.get_distance()
                print(f"Initial distance reading: {test_distance:.1f} cm")
                
                self.servo = Servo()
                print("Servo initialized")
                self.motor = Motor()
                print("Motor initialized")
                
            except Exception as e:
                print(f"Error initializing hardware: {e}")
                raise

            # Initialize camera with simpler approach
            print("Initializing camera...")
            try:
                self.camera = Picamera2()
                preview_config = self.camera.create_preview_configuration()
                self.camera.configure(preview_config)
                self.camera.start()
                print("Camera started successfully")
                time.sleep(2)  # Wait for camera to stabilize
                
                # Create camera window
                self.camera_window = 'Camera Feed'
                cv2.namedWindow(self.camera_window, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(self.camera_window, 800, 600)
                
                # Display initial frame to verify camera works
                initial_frame = self.camera.capture_array()
                if initial_frame is not None:
                    cv2.putText(initial_frame, "Camera initialized", (10, 30), 
                               cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
                    cv2.imshow(self.camera_window, initial_frame)
                    cv2.waitKey(1)
                    print("Camera preview shown")
                
            except Exception as e:
                print(f"Error initializing camera: {e}")
                raise
            
            # Configure scanning parameters
            self.scan_step = 15
            self.max_scan_angle = 120
            self.min_scan_angle = 30
            
            # Movement parameters
            self.step_size = 10
            self.planning_step_size = 5
            
            # Initialize servo position
            self.servo.setServoPwm('0', 75)  # Center position
            self.servo.setServoPwm('1', 40)  # Tilt down slightly
            time.sleep(0.5)
            
            # Add test obstacles
            self.map[100:200, 100:110] = 1  # Vertical wall
            self.map[100:110, 100:200] = 1  # Horizontal wall
            
            # Display initial map in terminal
            self.print_map_to_terminal()
            
            print("Initialization complete!")
            
        except Exception as e:
            print(f"Error in Mapper initialization: {e}")
            raise

    def get_distance(self):
        """Get distance reading in cm"""
        try:
            distance = self.ultrasonic.get_distance()
            if distance > 300:
                return 300
            return distance
        except Exception as e:
            print(f"Error reading distance: {e}")
            return 300  # Return max distance on error

    def scan_surroundings(self):
        """Enhanced scanning with multiple angles and averaging"""
        print("\nScanning surroundings...")
        
        # Clear immediate surroundings
        clear_radius = 30
        cx, cy = self.car_pos.astype(int)
        for x in range(max(0, cx-clear_radius), min(self.map_size, cx+clear_radius)):
            for y in range(max(0, cy-clear_radius), min(self.map_size, cy+clear_radius)):
                self.map[y, x] = 0

        # Scan with more granular steps
        for angle in range(self.min_scan_angle, self.max_scan_angle, self.scan_step):
            self.servo.setServoPwm('0', angle)
            time.sleep(0.1)  # Wait for servo to settle
            
            # Take multiple readings for accuracy
            distances = []
            for _ in range(5):  # Increased from 3 to 5 readings
                distance = self.get_distance()
                if 0 < distance < 300:  # Valid range
                    distances.append(distance)
                time.sleep(0.02)
            
            if distances:
                # Use median of readings
                distance = sorted(distances)[len(distances)//2]
                print(f"Angle: {angle}°, Distance: {distance:.1f}cm")
                
                # Convert to global coordinates with improved accuracy
                scan_angle = self.car_angle + (angle - 75)
                rad_angle = math.radians(scan_angle)
                
                # Mark multiple points along the beam
                for d in range(10, int(distance), 10):
                    obstacle_x = int(self.car_pos[0] + d * math.cos(rad_angle))
                    obstacle_y = int(self.car_pos[1] - d * math.sin(rad_angle))
                    
                    if (0 <= obstacle_x < self.map_size and 
                        0 <= obstacle_y < self.map_size):
                        # Mark as clear space
                        self.map[obstacle_y, obstacle_x] = 0
                
                # Mark the actual obstacle point
                obstacle_x = int(self.car_pos[0] + distance * math.cos(rad_angle))
                obstacle_y = int(self.car_pos[1] - distance * math.sin(rad_angle))
                
                if (0 <= obstacle_x < self.map_size and 
                    0 <= obstacle_y < self.map_size):
                    # Mark obstacle and surrounding area
                    for dx in range(-5, 6):
                        for dy in range(-5, 6):
                            mark_x = obstacle_x + dx
                            mark_y = obstacle_y + dy
                            if (0 <= mark_x < self.map_size and 
                                0 <= mark_y < self.map_size):
                                self.map[mark_y, mark_x] = 1

        # Return servo to center
        self.servo.setServoPwm('0', 75)
        print("Scan completed")

    def set_goal(self, x, y):
        """Set the goal position"""
        self.goal = np.array([x, y])
        print(f"New goal set: ({x}, {y})")

    def heuristic(self, a, b):
        """Manhattan distance heuristic for A*"""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def get_neighbors(self, pos, clearance=0):
        """Get valid neighboring positions for A*"""
        neighbors = []
        directions = [
            (0, -1),    # North
            (1, -1),    # Northeast
            (1, 0),     # East
            (1, 1),     # Southeast
            (0, 1),     # South
            (-1, 1),    # Southwest
            (-1, 0),    # West
            (-1, -1),   # Northwest
        ]
        
        for dx, dy in directions:
            new_x = pos[0] + dx * self.planning_step_size
            new_y = pos[1] + dy * self.planning_step_size
            
            # Check bounds with clearance
            if not (clearance <= new_x < self.map_size - clearance and 
                   clearance <= new_y < self.map_size - clearance):
                continue
            
            # Check if path is clear
            if self.is_path_clear((pos[0], pos[1]), (new_x, new_y), clearance):
                neighbors.append((new_x, new_y))
        
        return neighbors

    def is_path_clear(self, start, end, clearance):
        """Check if path between two points is clear"""
        x1, y1 = start
        x2, y2 = end
        distance = math.sqrt((x2-x1)**2 + (y2-y1)**2)
        
        # Check points along the path
        steps = max(int(distance / 5), 1)
        for i in range(steps + 1):
            t = i / steps
            x = int(x1 + t * (x2-x1))
            y = int(y1 + t * (y2-y1))
            
            # Check area around point
            for dx in range(-clearance, clearance + 1):
                for dy in range(-clearance, clearance + 1):
                    check_x = x + dx
                    check_y = y + dy
                    if (0 <= check_x < self.map_size and 
                        0 <= check_y < self.map_size and 
                        self.map[check_y, check_x] == 1):
                        return False
        return True

    def find_path(self, clearance=10):
        """A* pathfinding algorithm"""
        if self.goal is None:
            return None
            
        start = tuple(self.car_pos.astype(int))
        goal = tuple(self.goal.astype(int))
        
        frontier = PriorityQueue()
        frontier.put((0, start))
        came_from = {start: None}
        cost_so_far = {start: 0}
        
        while not frontier.empty():
            current = frontier.get()[1]
            
            if current == goal:
                break
                
            for next_pos in self.get_neighbors(current, clearance):
                new_cost = cost_so_far[current] + math.sqrt(
                    (next_pos[0] - current[0])**2 + 
                    (next_pos[1] - current[1])**2
                )
                
                if next_pos not in cost_so_far or new_cost < cost_so_far[next_pos]:
                    cost_so_far[next_pos] = new_cost
                    priority = new_cost + self.heuristic(next_pos, goal)
                    frontier.put((priority, next_pos))
                    came_from[next_pos] = current
        
        if goal not in came_from:
            return None
            
        # Reconstruct path
        path = []
        current = goal
        while current != start:
            path.append(current)
            current = came_from[current]
        path.append(start)
        path.reverse()
        return path

    def check_for_stop_sign(self):
        """Check camera feed for stop sign"""
        try:
            # Get frame directly from camera
            frame = self.camera.capture_array()
            
            if frame is None:
                print("Warning: Null frame in stop sign detection")
                return False
            
            # Use detect module directly
            stop_sign_detected = detect_stop_sign(frame)
            
            # Add visualization
            if stop_sign_detected:
                # Draw red box and text
                cv2.rectangle(frame, (10, 10), (200, 50), (0, 0, 255), -1)
                cv2.putText(frame, 'STOP SIGN DETECTED!', (20, 35),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Add navigation info
            cv2.putText(frame, f"Pos: ({int(self.car_pos[0])}, {int(self.car_pos[1])})",
                       (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(frame, f"Angle: {int(self.car_angle)}°",
                       (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            if self.goal is not None:
                cv2.putText(frame, f"Goal: ({int(self.goal[0])}, {int(self.goal[1])})",
                           (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Show the frame and refresh
            cv2.imshow(self.camera_window, frame)
            cv2.waitKey(1)
            
            return stop_sign_detected
            
        except Exception as e:
            print(f"Error during detection: {e}")
            traceback.print_exc()
            return False

    def handle_stop_sign(self):
        """Handle stop sign detection with gradual stop"""
        print("\nSTOP SIGN DETECTED - Coming to a gradual stop!")
        
        # Gradually slow down
        for speed in range(1500, 0, -300):
            self.motor.setMotorModel(speed, speed, speed, speed)
            time.sleep(0.1)
        
        # Full stop
        self.motor.setMotorModel(0, 0, 0, 0)
        
        print("Waiting at stop sign for 3 seconds...")
        time.sleep(3)
        
        # Double check if stop sign is still visible
        if self.check_for_stop_sign():
            print("Stop sign still visible, scanning surroundings...")
            self.scan_surroundings()  # Scan for obstacles while waiting
            time.sleep(2)
        
        print("Proceeding after stop...")

    def navigate_to_goal(self):
        """Main navigation loop with stop sign detection"""
        if self.goal is None:
            print("No goal set!")
            return False
            
        print(f"\nNavigating to goal: ({self.goal[0]}, {self.goal[1]})")
        print(f"Current position: ({int(self.car_pos[0])}, {int(self.car_pos[1])})")
        
        while True:
            # Scan surroundings and check for stop sign
            print("\nScanning for obstacles and stop signs...")
            self.scan_surroundings()
            # Print updated map
            self.print_map_to_terminal()
            
            if self.check_for_stop_sign():
                self.handle_stop_sign()
            
            # Find path to goal
            print("Finding path to goal...")
            path = self.find_path(clearance=10)
            
            if path is None:
                print("No path found to goal!")
                return False
                
            print(f"Found path with {len(path)} waypoints")
            
            # Follow path
            for i, point in enumerate(path[1:], 1):
                print(f"\nMoving to waypoint {i}/{len(path)-1}: ({point[0]}, {point[1]})")
                
                # Update camera feed
                frame = self.camera.capture_array()
                if frame is not None:
                    cv2.putText(frame, f"Moving to point {i}: ({point[0]}, {point[1]})", 
                               (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    cv2.imshow(self.camera_window, frame)
                    cv2.waitKey(1)
                
                # Check for stop sign before movement
                if self.check_for_stop_sign():
                    self.handle_stop_sign()
                
                # Calculate movement
                # Convert point to numpy array
                point_array = np.array(point)
                direction = point_array - self.car_pos
                distance = np.linalg.norm(direction)
                target_angle = math.degrees(math.atan2(-direction[1], direction[0]))
                
                # Rotate to face target
                self.rotate_to_angle(target_angle)
                
                # Check for stop sign after rotation
                if self.check_for_stop_sign():
                    self.handle_stop_sign()
                
                # Move forward
                self.move_forward(distance)
                
                # Update position
                self.car_pos = point_array
                
                # After moving to the waypoint, print map
                if i % 2 == 0:  # Update map every other waypoint to avoid too much output
                    self.print_map_to_terminal()
                
                # Check if we've reached the goal
                if np.linalg.norm(self.car_pos - self.goal) < 20:
                    print("Goal reached!")
                    self.print_map_to_terminal()  # Final map
                    return True
                    
        return False

    def rotate_to_angle(self, target_angle):
        """Rotate car to face target angle"""
        angle_diff = target_angle - self.car_angle
        # Normalize to [-180, 180]
        angle_diff = (angle_diff + 180) % 360 - 180
        
        print(f"Rotating {angle_diff:.1f} degrees")
        
        # Rotate
        rotation_speed = 1500
        if angle_diff > 0:
            self.motor.setMotorModel(-rotation_speed, -rotation_speed, 
                                   rotation_speed, rotation_speed)
        else:
            self.motor.setMotorModel(rotation_speed, rotation_speed, 
                                   -rotation_speed, -rotation_speed)
        
        # Calculate rotation time
        rotation_time = abs(angle_diff) / 90.0 * 0.7
        time.sleep(rotation_time)
        self.motor.setMotorModel(0, 0, 0, 0)
        time.sleep(0.3)
        
        self.car_angle = target_angle

    def move_forward(self, distance):
        """Move forward by specified distance"""
        print(f"Moving forward {distance:.1f}cm")
        
        forward_speed = 1500
        movement_time = distance / 30.0  # Assuming 30cm/sec at this speed
        
        self.motor.setMotorModel(forward_speed, forward_speed, 
                               forward_speed, forward_speed)
        time.sleep(movement_time)
        self.motor.setMotorModel(0, 0, 0, 0)
        time.sleep(0.2)

    def sendvideo(self):
        """Camera streaming thread function"""
        while True:
            try:
                # Capture frame directly like in test_camera.py
                frame = self.camera.capture_array()
                
                if frame is None:
                    print("Warning: Null frame captured")
                    continue
                    
                # Convert from BGR to RGB
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                
                # Add overlay information
                cv2.putText(frame, f"Pos: ({int(self.car_pos[0])}, {int(self.car_pos[1])})", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
                cv2.putText(frame, f"Angle: {int(self.car_angle)}°", 
                           (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
                
                if self.goal is not None:
                    cv2.putText(frame, f"Goal: ({int(self.goal[0])}, {int(self.goal[1])})",
                               (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
                
                # Show frame
                cv2.imshow(self.camera_window, frame)
                key = cv2.waitKey(1)
                if key == ord('q'):
                    break
                
                time.sleep(0.05)  # Control frame rate
                
            except Exception as e:
                print(f"Error in video stream: {e}")
                traceback.print_exc()
                time.sleep(0.1)

    def cleanup(self):
        """Clean up resources"""
        print("Cleaning up...")
        try:
            if hasattr(self, 'motor'):
                self.motor.setMotorModel(0, 0, 0, 0)
            if hasattr(self, 'camera'):
                self.camera.stop()
            cv2.destroyAllWindows()
            GPIO.cleanup()
        except Exception as e:
            print(f"Error during cleanup: {e}")

    def __del__(self):
        """Destructor to ensure cleanup"""
        self.cleanup()

    def mouse_callback(self, event, x, y, flags, param):
        """Handle mouse clicks on map"""
        if event == cv2.EVENT_LBUTTONDOWN:
            # Set clicked point as destination
            self.set_goal(x, y)
            print(f"Destination set to: ({x}, {y})")

    def update_displays(self):
        """Update camera feed and print map to terminal"""
        print("Display thread started")
        while True:
            try:
                # Get camera frame
                frame = self.camera.capture_array()
                
                if frame is not None:
                    # Add overlay information
                    cv2.putText(frame, f"Car pos: ({int(self.car_pos[0])}, {int(self.car_pos[1])})", 
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
                    cv2.putText(frame, f"Angle: {int(self.car_angle)}°", 
                            (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
                    if self.goal is not None:
                        cv2.putText(frame, f"Goal: ({int(self.goal[0])}, {int(self.goal[1])})",
                                (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
                    
                    # Show frame
                    cv2.imshow(self.camera_window, frame)
                
                # Handle key press
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                
                time.sleep(0.1)  # Faster update rate
                
            except Exception as e:
                print(f"Display error: {e}")
                traceback.print_exc()
                time.sleep(0.1)

    def print_map_to_terminal(self):
        """Print ASCII map to terminal"""
        print("\033[2J\033[H")  # Clear terminal
        print("Obstacle Map (■=obstacle, ·=free, C=car, G=goal):")
        print("-" * (self.map_size//10 + 2))
        
        for y in range(0, self.map_size, 10):
            line = "|"
            for x in range(0, self.map_size, 10):
                if abs(x - self.car_pos[0]) < 5 and abs(y - self.car_pos[1]) < 5:
                    line += "C"
                elif self.goal is not None and abs(x - self.goal[0]) < 5 and abs(y - self.goal[1]) < 5:
                    line += "G"
                elif self.map[y:y+10, x:x+10].any():
                    line += "■"
                else:
                    line += "·"
            line += "|"
            print(line)
        print("-" * (self.map_size//10 + 2))
        print("\nCar position:", self.car_pos)
        if self.goal is not None:
            print("Goal position:", self.goal)

if __name__ == '__main__':
    try:
        print("Starting mapper initialization...")
        mapper = Mapper()
        print("\nMapper initialized successfully!")
        
        print("\nInstructions:")
        print("1. Enter 'g x y' to set destination (e.g., 'g 150 150')")
        print("2. Press Ctrl+C to exit")
        
        while True:
            try:
                command = input("\nEnter command: ").strip()
                if command.startswith('g '):
                    try:
                        _, x, y = command.split()
                        x, y = int(x), int(y)
                        if 0 <= x < mapper.map_size and 0 <= y < mapper.map_size:
                            mapper.set_goal(x, y)
                            print(f"Goal set to ({x}, {y})")
                            success = mapper.navigate_to_goal()
                            if success:
                                print("\nDestination reached!")
                            else:
                                print("\nNavigation failed!")
                        else:
                            print(f"Coordinates must be between 0 and {mapper.map_size}")
                    except ValueError:
                        print("Invalid format. Use: g x y (e.g., g 150 150)")
                else:
                    print("Unknown command. Use 'g x y' to set destination")
                
            except Exception as e:
                print(f"\nError: {e}")
                traceback.print_exc()
                
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        if 'mapper' in locals():
            mapper.cleanup() 