import numpy as np
import math
from Ultrasonic import *
from servo import *
import time
from queue import PriorityQueue
from Motor import Motor

class Mapper:
    def __init__(self, map_size=300):  # Increased map size to accommodate feet measurements
        # Initialize map as 300x300 grid where each cell is 1x1 cm
        self.map_size = map_size
        self.map = np.zeros((map_size, map_size))
        self.car_pos = np.array([map_size//2, map_size//2])  # Car starts at center
        self.car_angle = 0  # Car starts facing 0 degrees (positive x-axis)
        
        # Initialize hardware
        self.ultrasonic = Ultrasonic()
        self.servo = Servo()
        
        # Configure scanning parameters
        self.scan_step = 5  # Degrees between each scan
        self.max_scan_angle = 120  # Maximum servo angle
        self.min_scan_angle = 30   # Minimum servo angle
        
        # Path planning parameters
        self.goal = None
        self.path = []
        
        # Initialize servo to front position
        self.servo.setServoPwm('0', 75)
        self.servo.setServoPwm('1', 40)
        time.sleep(0.5)
        
        # Initialize motor control
        self.motor = Motor()
        self.movement_time = 1  # Base time for movements in seconds
        
        # Add new parameters for path planning
        self.replanning_interval = 5  # Number of steps before replanning
        self.steps_since_replan = 0
        self.person_detected = False
        self.movement_directions = {
            'up': (0, -1),
            'right': (1, 0),
            'down': (0, 1),
            'left': (-1, 0)
        }
        
    def set_goal(self, north_feet, east_feet):
        """Set goal position relative to current position in feet"""
        # Convert feet to centimeters (1 foot = 30.48 cm)
        north_cm = north_feet * 30.48
        east_cm = east_feet * 30.48
        
        # Calculate goal position in map coordinates
        goal_x = self.car_pos[0] + east_cm
        goal_y = self.car_pos[1] - north_cm  # Subtract because y increases downward
        
        self.goal = np.array([goal_x, goal_y])
        
    def detect_person(self, detection_result):
        """Check if a person is detected in front of the car"""
        for detection in detection_result.detections:
            category = detection.categories[0]
            if category.category_name == "person" and category.score > 0.5:
                bbox = detection.bounding_box
                # Check if person is in the center third of the image
                center_x = bbox.origin_x + bbox.width/2
                if bbox.width > 100 and (213 < center_x < 426):  # For 640x480 image
                    self.person_detected = True
                    return True
        self.person_detected = False
        return False

    def heuristic(self, a, b):
        """A* heuristic: Euclidean distance plus penalty for proximity to obstacles"""
        base_cost = np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
        
        # Add penalty for being close to obstacles
        obstacle_penalty = 0
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                check_x = a[0] + dx
                check_y = a[1] + dy
                if (0 <= check_x < self.map_size and 
                    0 <= check_y < self.map_size and 
                    self.map[check_y, check_x] == 1):
                    obstacle_penalty += 10  # Penalty for each nearby obstacle
                    
        return base_cost + obstacle_penalty

    def get_neighbors(self, pos):
        """Get valid neighboring positions"""
        neighbors = []
        for direction in self.movement_directions.values():
            new_pos = (pos[0] + direction[0], pos[1] + direction[1])
            
            # Check bounds and obstacles
            if (0 <= new_pos[0] < self.map_size and 
                0 <= new_pos[1] < self.map_size and 
                self.map[new_pos[1], new_pos[0]] == 0):
                
                # Check for diagonal obstacles (prevent corner cutting)
                if direction[0] != 0 and direction[1] != 0:
                    if (self.map[pos[1], new_pos[0]] == 1 or 
                        self.map[new_pos[1], pos[0]] == 1):
                        continue
                        
                neighbors.append(new_pos)
        return neighbors

    def find_path(self):
        """Enhanced A* pathfinding with obstacle avoidance"""
        if self.goal is None:
            return None
            
        start = tuple(self.car_pos.astype(int))
        goal = tuple(self.goal.astype(int))
        
        if not (0 <= goal[0] < self.map_size and 0 <= goal[1] < self.map_size):
            print("Goal is outside map boundaries")
            return None
            
        frontier = PriorityQueue()
        frontier.put((0, start))
        came_from = {start: None}
        cost_so_far = {start: 0}
        
        while not frontier.empty():
            current = frontier.get()[1]
            
            if current == goal:
                break
                
            for next_pos in self.get_neighbors(current):
                new_cost = cost_so_far[current] + 1
                
                if next_pos not in cost_so_far or new_cost < cost_so_far[next_pos]:
                    cost_so_far[next_pos] = new_cost
                    priority = new_cost + self.heuristic(next_pos, goal)
                    frontier.put((priority, next_pos))
                    came_from[next_pos] = current
        
        # Reconstruct path
        if goal not in came_from:
            return None
            
        current = goal
        path = []
        while current != start:
            path.append(current)
            current = came_from[current]
        path.append(start)
        path.reverse()
        self.path = path
        return path
        
    def polar_to_cartesian(self, distance, angle):
        """Convert polar coordinates (distance, angle) to cartesian (x,y)"""
        x = distance * math.cos(math.radians(angle))
        y = distance * math.sin(math.radians(angle))
        return np.array([x, y])
        
    def update_map(self, point, is_obstacle=True):
        """Update map with obstacle at given point"""
        # Transform point to map coordinates (shift by car position)
        map_x = int(point[0] + self.car_pos[0])
        map_y = int(point[1] + self.car_pos[1])
        
        # Check if point is within map bounds
        if 0 <= map_x < self.map_size and 0 <= map_y < self.map_size:
            self.map[map_y, map_x] = 1 if is_obstacle else 0
            
    def interpolate_points(self, p1, p2):
        """Interpolate points between two measurements"""
        x1, y1 = p1
        x2, y2 = p2
        
        # Calculate number of points to interpolate
        distance = math.sqrt((x2-x1)**2 + (y2-y1)**2)
        num_points = int(distance)
        
        if num_points > 1:
            x_points = np.linspace(x1, x2, num_points)
            y_points = np.linspace(y1, y2, num_points)
            return list(zip(x_points, y_points))
        return []
        
    def scan_surroundings(self):
        """Perform a full scan and update map"""
        prev_point = None
        
        # Scan from min to max angle
        for angle in range(self.min_scan_angle, self.max_scan_angle, self.scan_step):
            # Move servo to angle
            self.servo.setServoPwm('0', angle)
            time.sleep(0.1)  # Wait for servo to move
            
            # Get distance reading
            distance = self.ultrasonic.get_distance()
            
            if distance < 300:  # Filter out readings that are too far
                # Convert to cartesian coordinates
                point = self.polar_to_cartesian(distance, angle)
                
                # Update map with current point
                self.update_map(point)
                
                # Interpolate between previous and current point
                if prev_point is not None:
                    interpolated = self.interpolate_points(prev_point, point)
                    for p in interpolated:
                        self.update_map(np.array(p))
                
                prev_point = point
                
    def update_position(self, velocity, dt):
        """Update car's position based on velocity readings"""
        # Update position based on velocity and time
        displacement = velocity * dt
        dx = displacement * math.cos(math.radians(self.car_angle))
        dy = displacement * math.sin(math.radians(self.car_angle))
        
        self.car_pos += np.array([dx, dy])
        
    def get_map(self):
        """Return current map"""
        return self.map

    def move_to_next_waypoint(self):
        """Move to the next waypoint in the path"""
        if not self.path or len(self.path) < 2:
            print("No path to follow")
            return False
            
        current = np.array(self.path[0])
        next_point = np.array(self.path[1])
        
        # Calculate direction vector
        direction = next_point - current
        
        # Calculate angle to target
        target_angle = math.degrees(math.atan2(direction[1], direction[0]))
        angle_diff = target_angle - self.car_angle
        
        # Normalize angle difference to [-180, 180]
        angle_diff = (angle_diff + 180) % 360 - 180
        
        # First rotate to face the target
        if abs(angle_diff) > 5:  # If angle difference is significant
            self.rotate(angle_diff)
            
        # Calculate distance to move
        distance = np.linalg.norm(direction)
        
        # Move forward
        self.move_forward(distance)
        
        # Update position and remove the waypoint we just reached
        self.car_pos = np.array(self.path[1])
        self.path.pop(0)
        
        return True
        
    def rotate(self, angle):
        """Rotate the car by the specified angle in degrees"""
        # Positive angle = counterclockwise rotation
        print(f"Rotating {angle} degrees")
        
        # Adjust motor speed based on angle magnitude
        rotation_speed = 1500  # Base rotation speed
        
        if angle > 0:
            # Rotate counterclockwise
            self.motor.setMotorModel(-rotation_speed, -rotation_speed, 
                                   rotation_speed, rotation_speed)
        else:
            # Rotate clockwise
            self.motor.setMotorModel(rotation_speed, rotation_speed, 
                                   -rotation_speed, -rotation_speed)
            
        # Calculate rotation time based on angle
        rotation_time = abs(angle) / 90 * self.movement_time
        time.sleep(rotation_time)
        
        # Stop motors
        self.motor.setMotorModel(0, 0, 0, 0)
        
        # Update car's angle
        self.car_angle = (self.car_angle + angle) % 360
        
    def move_forward(self, distance):
        """Move forward by specified distance in cm"""
        print(f"Moving forward {distance} cm")
        
        # Convert distance to movement time
        # This needs calibration based on your car's speed
        movement_time = distance / 50 * self.movement_time  # Assuming 50cm/sec speed
        
        # Move forward
        self.motor.setMotorModel(2000, 2000, 2000, 2000)
        time.sleep(movement_time)
        
        # Stop motors
        self.motor.setMotorModel(0, 0, 0, 0)
        
    def follow_path(self):
        """Enhanced path following with periodic replanning and person detection"""
        if not self.path:
            print("No path available")
            return False
            
        while len(self.path) > 1:
            # Check for persons
            if self.person_detected:
                print("Person detected! Waiting...")
                self.motor.setMotorModel(0, 0, 0, 0)
                time.sleep(1)
                continue
                
            # Scan surroundings and replan periodically
            self.steps_since_replan += 1
            if self.steps_since_replan >= self.replanning_interval:
                print("Replanning route...")
                self.scan_surroundings()
                new_path = self.find_path()
                if new_path:
                    self.path = new_path
                    print("New path found")
                else:
                    print("No new path available")
                    return False
                self.steps_since_replan = 0
                
            # Move to next waypoint
            if not self.move_to_next_waypoint():
                break
                
            time.sleep(0.5)
            
        return len(self.path) == 1

# Test code
if __name__ == '__main__':
    mapper = Mapper()
    try:
        while True:
            mapper.scan_surroundings()
            print("Map updated")
            print(mapper.get_map())
            time.sleep(1)
    except KeyboardInterrupt:
        print("Mapping stopped") 