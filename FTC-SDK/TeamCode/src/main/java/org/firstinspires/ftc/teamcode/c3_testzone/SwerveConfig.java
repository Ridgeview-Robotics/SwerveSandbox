package org.firstinspires.ftc.teamcode.c3_testzone;

import com.acmerobotics.dashboard.config.Config;

@Config
public class SwerveConfig {
    
    // Drive Motor Configuration
    public static double DRIVE_GEAR_RATIO = 1.0; // Adjust based on your gearbox
    public static double WHEEL_RADIUS = 2.0; // inches
    public static double DRIVE_MAX_VEL = 50.0; // inches per second
    public static double DRIVE_MAX_ACCEL = 30.0; // inches per second squared
    
    // Continuous Servo Configuration
    public static double SERVO_STOP_POWER = 0.5; // Power value that stops the servo
    public static double SERVO_MAX_POWER = 1.0; // Maximum servo power
    public static double SERVO_MIN_POWER = 0.0; // Minimum servo power
    
    // Analog Encoder Configuration (0-3.3V = 0-2π radians)
    public static double ENCODER_MAX_VOLTAGE = 3.3; // Maximum voltage from analog input
    public static double ENCODER_MIN_VOLTAGE = 0.0; // Minimum voltage from analog input
    public static double VOLTAGE_TO_RADIANS = (2.0 * Math.PI) / ENCODER_MAX_VOLTAGE;
    
    // PID Coefficients for Swerve Module Rotation (output is servo power)
    public static double SWERVE_kP = 0.5; // Higher P for continuous servo control
    public static double SWERVE_kI = 0.0;
    public static double SWERVE_kD = 0.2;
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
    
    // Continuous Servo optimization settings
    public static double SERVO_DEADBAND = 0.02; // Deadband around stop power (0.5)
    public static double SERVO_TOLERANCE = 0.05; // radians (about 3 degrees)
    public static double SERVO_SETTLE_TIME = 0.1; // seconds
    
    // Encoder offset calibration values (radians)
    public static double FL_ENCODER_OFFSET = 0.0;
    public static double FR_ENCODER_OFFSET = 0.0;
    public static double BL_ENCODER_OFFSET = 0.0;
    public static double BR_ENCODER_OFFSET = 0.0;
    
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
    
    // Drive motor encoder configuration
    public static double ENCODER_TICKS_PER_REV = 537.7; // GoBilda 5203/5204 motors
    public static boolean REVERSE_ENCODER = false;
    
    // Wheel alignment settings
    public static boolean AUTO_ALIGN_ON_INIT = true; // Straighten wheels on initialization
    public static double AUTO_ALIGN_TIMEOUT = 3.0; // seconds to wait for alignment
}