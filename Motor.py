import time
import math
from PCA9685 import PCA9685
from ADC import *

class WheelController:
    def __init__(self):
        self.pwm_controller = PCA9685(0x40, debug=True)
        self.pwm_controller.setPWMFreq(50)
        self.rotation_coefficient = 3  # Calibration factor for rotation
        self.power_monitor = Adc()
        self.MAX_DUTY = 4095
        
        # PWM channel mappings for each wheel
        self.wheel_channels = {
            'front_left': {'forward': 1, 'reverse': 0},
            'front_right': {'forward': 7, 'reverse': 6},
            'rear_left': {'forward': 2, 'reverse': 3},
            'rear_right': {'forward': 5, 'reverse': 4}
        }
    
    def _clamp_duty_cycle(self, value):
        """Ensure duty cycle stays within valid range"""
        return max(min(value, self.MAX_DUTY), -self.MAX_DUTY)
    
    def _control_wheel(self, channels, power):
        """Control individual wheel direction and power"""
        if power > 0:
            self.pwm_controller.setMotorPwm(channels['reverse'], 0)
            self.pwm_controller.setMotorPwm(channels['forward'], power)
        elif power < 0:
            self.pwm_controller.setMotorPwm(channels['forward'], 0)
            self.pwm_controller.setMotorPwm(channels['reverse'], abs(power))
        else:
            self.pwm_controller.setMotorPwm(channels['forward'], self.MAX_DUTY)
            self.pwm_controller.setMotorPwm(channels['reverse'], self.MAX_DUTY)
    
    def drive(self, fl_power, rl_power, fr_power, rr_power):
        """Control all wheels with power values"""
        # Clamp all power values
        powers = map(self._clamp_duty_cycle, [fl_power, rl_power, fr_power, rr_power])
        fl, rl, fr, rr = powers
        
        # Control each wheel
        self._control_wheel(self.wheel_channels['front_left'], fl)
        self._control_wheel(self.wheel_channels['rear_left'], rl)
        self._control_wheel(self.wheel_channels['front_right'], fr)
        self._control_wheel(self.wheel_channels['rear_right'], rr)
    
    def rotate_vehicle(self, angle):
        """Rotate vehicle by specified angle"""
        battery_factor = 7.5 / (self.power_monitor.recvADC(2) * 3)
        
        while True:
            base_power = 2000
            
            # Calculate vector components
            y_vector = int(base_power * math.cos(math.radians(angle)))
            x_vector = -int(base_power * math.sin(math.radians(angle)))
            rotation = base_power
            
            # Calculate wheel powers
            fr = y_vector - x_vector + rotation
            fl = y_vector + x_vector - rotation
            rl = y_vector - x_vector - rotation
            rr = y_vector + x_vector + rotation
            
            self.drive(fl, rl, fr, rr)
            time.sleep(5 * self.rotation_coefficient * battery_factor / 1000)
            angle -= 5

# Create controller instance
controller = WheelController()

def test_movements():
    # Forward
    controller.drive(2000, 2000, 2000, 2000)
    time.sleep(3)
    
    # Reverse
    controller.drive(-2000, -2000, -2000, -2000)
    time.sleep(3)
    
    # Left turn
    controller.drive(-500, -500, 2000, 2000)
    time.sleep(3)
    
    # Right turn
    controller.drive(2000, 2000, -500, -500)
    time.sleep(3)
    
    # Stop
    controller.drive(0, 0, 0, 0)

def cleanup():
    controller.drive(0, 0, 0, 0)

if __name__ == '__main__':
    try:
        test_movements()
    except KeyboardInterrupt:
        cleanup()
