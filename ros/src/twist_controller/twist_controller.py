from pid import PID
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, **kwargs):
        # TODO: Implement
        self.vehicle_mass = kwargs["vehicle_mass"]
        self.fuel_capacity = kwargs["fuel_capacity"]
        self.brake_deadband = kwargs["brake_deadband"]
        self.decel_limit = kwargs["decel_limit"]
        self.accel_limit = kwargs["accel_limit"]
        self.wheel_radius = kwargs["wheel_radius"]
        self.rate = kwargs["rate"]
        self.time = 1.0/self.rate
        self.mass = self.vehicle_mass + self.fuel_capacity * GAS_DENSITY


        self.lin_vel_pid = PID(0.8, 0.05, 0.4, mn = -abs(self.decel_limit), mx = self.accel_limit) #0.9 0.1 0.4
        self.ang_vel_pid = PID(1.0, 0.1, 0.5, mn = -0.43, mx = 0.43)
        self.accel_pid = PID(0.3, 0.1, 0.0, mn = 0.0, mx = 1.0)
        self.yawcontroller = YawController(kwargs['wheel_base'], kwargs['steer_ratio'], 10.0, kwargs['max_lat_accel'], kwargs['max_steer_angle'])

    def control(self, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        linear_velocity_future = kwargs["linear_velocity_future"]
        angular_velocity_future = kwargs["angular_velocity_future"]
        linear_velocity_current = kwargs["linear_velocity_current"]
        #angular_velocity_future = kwargs["angular_velocity_future"]
        acceleration_current = kwargs["acceleration_current"]

        vel_gap = linear_velocity_future - linear_velocity_current

        acceleration = self.lin_vel_pid.step(vel_gap,self.time)
        #steer = self.ang_vel_pid.step(angular_velocity_future, self.time)
        steer = self.yawcontroller.get_steering(linear_velocity_future, angular_velocity_future, linear_velocity_current)

        if(acceleration < 0):
            self.accel_pid.reset()
            throttle = 0
        else:
            throttle = self.accel_pid.step(acceleration - acceleration_current, self.time) 

        if(acceleration < -abs(self.brake_deadband)):
            brake = acceleration * self.mass * self.wheel_radius
        else:
            brake = 0

        return throttle, brake, steer

    def reset(self):
        self.lin_vel_pid.reset()
        self.ang_vel_pid.reset()
