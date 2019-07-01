
GAS_DENSITY = 2.858
ONE_MPH = 0.44704

class Controller(object):

    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit,
            wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):

        # Yaw controller
        min_speed = 0.1
        self.yaw_controller = YawController(wheel_base, steer_ratio, 
                min_speed, max_lat_accel, max_steer_angle)

        # Throttle controller
        kp = 0.3
        ki = 0.1
        kd = 0.0
        min_throttle = 0.0
        max_throttle = 0.2
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
            
            error = linear_vel - current_vel

            current_time = rospy.get_time()
            sample_time = current_time - self.last_time
            self.last_time = current_time

            throttle = self.throttle_controller.step(error, sample_time)
            brake = 0.

            if linear_vel == 0. and current_vel < 0.1:
                throttle = 0.
                brake = 400.

            elif throttle < 1. and error < 0.:
                throttle = 0.
                decel = max(error, self.decel_limit)
                brake = abs(decel)*self.vehicle_mass*self.wheel_radius

            return throttle, brake, steering

