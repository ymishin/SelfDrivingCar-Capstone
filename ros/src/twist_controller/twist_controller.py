import rospy
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

class Controller(object):

    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit,
            wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):

        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio        
        self.max_lat_accel = max_lat_accel
        self.max_steer_angle = max_steer_angle

        # Yaw controller
        min_speed = 0.1
        self.yaw_controller = YawController(wheel_base, steer_ratio, 
                min_speed, max_lat_accel, max_steer_angle)

        # Throttle controller
        kp = 1.0
        ki = 0.0002
        kd = 0.1
        min_throttle = 0.0
        max_throttle = 1.0
        self.throttle_controller = PID(kp, ki, kd, min_throttle, max_throttle)

        # LPF for velocity
        tau = 0.5
        ts = 0.02
        self.vel_lpf = LowPassFilter(tau, ts)

        self.last_time = rospy.get_time()

        pass

    def control(self, dbw_enabled, current_vel, linear_vel, angular_vel):

        if not dbw_enabled:

            self.throttle_controller.reset()
            return 0., 0., 0.

        else:

            # Filter noise
            current_vel = self.vel_lpf.filt(current_vel)
            steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)
           
            # Calculate throttle
            vel_error = linear_vel - current_vel
            current_time = rospy.get_time()
            sample_time = current_time - self.last_time
            self.last_time = current_time
            throttle = self.throttle_controller.step(vel_error, sample_time)
            brake = 0.

            # Apply brake if necessary
            if linear_vel == 0. and current_vel < 0.1:
                throttle = 0.
                brake = 700.
            elif throttle < 1. and vel_error < 0.:
                throttle = 0.
                decel = max(vel_error, self.decel_limit)
                brake = abs(decel) * self.vehicle_mass * self.wheel_radius

            return throttle, brake, steering
