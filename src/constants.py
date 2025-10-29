from wpimath.units import feet_per_second, inch, radians_per_second, radians_per_second_squared

class SwerveModuleConstants:
    MAX_WHEEL_SPEED: feet_per_second = 8.0
    WHEEL_RADIUS: inch = 2.0
    DRIVE_GEAR_RATIO: float = 36000.0/5880.0
    DRIVE_RATIO_SCALE: float = 1.0

    class SteerPIDConstants:
        Kp: float = 0.5
        Ki: float = 0.0
        Kd: float = 0.0
        MAX_SPEED: radians_per_second = 12.0
        MAX_ACCELERATION: radians_per_second_squared = 100.0