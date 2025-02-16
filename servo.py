import time
from PCA9685 import PCA9685

class ServoController:
    def __init__(self):
        self.pwm_driver = PCA9685(0x40, debug=True)
        self.pwm_driver.setPWMFreq(50)
        
        # Initialize default positions
        self._init_servos()
        
        # Servo configuration
        self.servo_config = {
            '0': {'channel': 8, 'min_pulse': 500, 'max_pulse': 2500, 'direction': -1},
            '1': {'channel': 9, 'min_pulse': 500, 'max_pulse': 2500, 'direction': 1},
            '2': {'channel': 10, 'min_pulse': 500, 'max_pulse': 2500, 'direction': 1},
            '3': {'channel': 11, 'min_pulse': 500, 'max_pulse': 2500, 'direction': 1},
            '4': {'channel': 12, 'min_pulse': 500, 'max_pulse': 2500, 'direction': 1},
            '5': {'channel': 13, 'min_pulse': 500, 'max_pulse': 2500, 'direction': 1},
            '6': {'channel': 14, 'min_pulse': 500, 'max_pulse': 2500, 'direction': 1},
            '7': {'channel': 15, 'min_pulse': 500, 'max_pulse': 2500, 'direction': 1}
        }
    
    def _init_servos(self):
        """Initialize servos to center position"""
        self.pwm_driver.setServoPulse(8, 1500)
        self.pwm_driver.setServoPulse(9, 1500)
    
    def _calculate_pulse(self, angle, config, offset=10):
        """Calculate PWM pulse for given angle"""
        angle = int(angle)
        if config['direction'] < 0:
            return config['max_pulse'] - int((angle + offset) / 0.09)
        return config['min_pulse'] + int((angle + offset) / 0.09)
    
    def set_angle(self, servo_id, angle, offset=10):
        """Set servo to specified angle"""
        if servo_id not in self.servo_config:
            raise ValueError(f"Invalid servo ID: {servo_id}")
            
        config = self.servo_config[servo_id]
        pulse = self._calculate_pulse(angle, config, offset)
        self.pwm_driver.setServoPulse(config['channel'], pulse)

def test_servo_movement():
    print("Initializing servo test...")
    print("Servos will move to test positions.")
    print("Press Ctrl+C to exit.")
    
    controller = ServoController()
    
    try:
        while True:
            # Test movement of first two servos
            controller.set_angle('0', 75)
            controller.set_angle('1', 40)
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\nTest complete - shutting down")

if __name__ == '__main__':
    test_servo_movement()

    

    
       



    
