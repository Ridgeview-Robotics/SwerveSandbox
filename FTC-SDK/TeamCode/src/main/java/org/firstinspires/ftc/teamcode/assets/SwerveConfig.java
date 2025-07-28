package org.firstinspires.ftc.teamcode.assets;

import com.acmerobotics.dashboard.config.Config;

@Config
public class SwerveConfig {
    
    // Drive Motor Configuration
    public static double DRIVE_GEAR_RATIO = 1.0; // Adjust based on your gearbox
    public static double WHEEL_RADIUS = 2.0; // inches
    public static double DRIVE_MAX_VEL = 50.0; // inches per second
    public static double DRIVE_MAX_ACCEL = 30.0; // inches per second squared
    
    // Servo Configuration
    public static double SERVO_MIN_ANGLE = 0.0; // degrees
    public static double SERVO_MAX_ANGLE = 360.0; // degrees
    public static double SERVO_DEGREES_PER_PULSE = 0.18; // AXON Max spec
    
    // PID Coefficients for Swerve Module Rotation
    public static double SWERVE_kP = 0.8;
    public static double SWERVE_kI = 0.0;
    public static double SWERVE_kD = 0.1;
    public static double SWERVE_kF = 0.0;
    
    // Angular PID for robot rotation
    public static double ANGULAR_kP = 2.0;
    public static double ANGULAR_kI = 0.0;
    public static double ANGULAR_kD = 0.2;
    
    // Deadband for gamepad inputs
    public static double DRIVE_DEADBAND = 0.05;
    public static double ROTATION_DEADBAND = 0.05;
    
    // Maximum angular velocity (rad/s)
    public static double MAX_ANG_VEL = Math.PI; // 180 degrees per second
    
    // Servo optimization settings
    public static double SERVO_TOLERANCE = 2.0; // degrees
    public static double SERVO_SETTLE_TIME = 0.1; // seconds
    
    // Performance optimization
    public static boolean USE_BULK_READS = true;
    public static int CONTROL_HUB_CACHE_TIME = 5; // milliseconds
    public static boolean OPTIMIZE_SERVO_WRITES = true;
    
    // Module positions relative to robot center (inches)
    public static double TRACK_WIDTH = 12.0; // distance between left and right wheels
    public static double WHEELBASE = 12.0; // distance between front and rear wheels
    
    // Calculated module positions
    public static final double MODULE_X_OFFSET = TRACK_WIDTH / 2.0;
    public static final double MODULE_Y_OFFSET = WHEELBASE / 2.0;
    
    // Safety limits
    public static double MAX_DRIVE_POWER = 1.0;
    public static double MAX_TURN_POWER = 0.8;
    
    // Encoder configuration
    public static double ENCODER_TICKS_PER_REV = 8192.0; // Adjust for your encoder
    public static boolean REVERSE_ENCODER = false;
    
    // Auto-alignment settings
    public static double AUTO_ALIGN_TOLERANCE = 1.0; // degrees
    public static double AUTO_ALIGN_MAX_POWER = 0.3;
}