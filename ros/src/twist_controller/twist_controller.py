from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter
from rospy import get_time

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, 
    			 decel_limit, accel_limit, wheel_radius, wheel_base,
    			 steer_ratio, max_lat_accel, max_steer_angle):

        self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1, 
        									max_lat_accel, max_steer_angle)

        kp = 0.3
        ki = 0.1
        kd = 0.
        mn = 0.
        mx = 0.5
        self.throttle_controller = PID(kp, kd, ki, mn, mx)

        tau = 0.5
        ts = 0.02
        self.velocity_lpf = LowPassFilter(tau, ts)

        self.vehicle_mass = vehicle_mass
        self.wheel_radius = wheel_radius

        self.last_time = get_time()

    def control(self, current_velocity, req_linear_velocity, req_angular_velocity, 
    			dbw_enabled):

        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        if not dbw_enabled:
        	self.throttle_controller.reset()
        	return 0., 0., 0.

        current_velocity = self.velocity_lpf.filt(current_velocity)

        steering = self.yaw_controller.get_steering(req_linear_velocity, 
        											req_angular_velocity, 
        							   				current_velocity)

        vel_error = req_linear_velocity - current_velocity
        dt = get_time() - self.last_time
        self.last_time = get_time()

        throttle = self.throttle_controller.step(vel_error, dt)
        brake = 0

        if req_linear_velocity == 0. and current_velocity < 0.1:
        	throttle = 0
        	brake = 700

        elif throttle < .1 and vel_error < 0:
        	throttle = 0
        	decel = max(vel_error, self.decel_limit)
        	brake = abs(decel) * self.vehicle_mass * self.wheel_radius

        return throttle, brake, steering 
