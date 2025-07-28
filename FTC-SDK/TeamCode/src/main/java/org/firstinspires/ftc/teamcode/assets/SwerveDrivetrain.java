package org.firstinspires.ftc.teamcode.assets;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.lynx.LynxModule;

import java.util.List;

/**
 * Swerve drivetrain controller for FTC
 * Manages 4 swerve modules with robot-centric control
 * Optimized for performance with bulk reads and minimal processing
 */
public class SwerveDrivetrain {
    
    // Swerve modules (FL, FR, BL, BR)
    private SwerveModule frontLeft;
    private SwerveModule frontRight;
    private SwerveModule backLeft;
    private SwerveModule backRight;
    
    // Performance optimization
    private List<LynxModule> allHubs;
    private ElapsedTime bulkReadTimer = new ElapsedTime();
    
    // Control variables
    private double robotAngle = 0.0; // Current robot heading
    private SwerveModulePID rotationPID;
    
    // Module positions for kinematics
    private final double moduleX = SwerveConfig.MODULE_X_OFFSET;
    private final double moduleY = SwerveConfig.MODULE_Y_OFFSET;
    
    public SwerveDrivetrain(HardwareMap hardwareMap) {
        initializeHardware(hardwareMap);
        setupPerformanceOptimizations(hardwareMap);
        
        // Initialize rotation PID
        rotationPID = new SwerveModulePID(
            SwerveConfig.ANGULAR_kP,
            SwerveConfig.ANGULAR_kI,
            SwerveConfig.ANGULAR_kD
        );
    }
    
    /**
     * Initialize all hardware components
     */
    private void initializeHardware(HardwareMap hardwareMap) {
        // Front Left Module
        DcMotorEx flDrive = hardwareMap.get(DcMotorEx.class, "frontLeftDrive");
        Servo flSteer = hardwareMap.get(Servo.class, "frontLeftSteer");
        AnalogInput flEncoder = hardwareMap.get(AnalogInput.class, "frontLeftEncoder");
        frontLeft = new SwerveModule(flDrive, flSteer, flEncoder, "FL", 0.0);
        
        // Front Right Module
        DcMotorEx frDrive = hardwareMap.get(DcMotorEx.class, "frontRightDrive");
        Servo frSteer = hardwareMap.get(Servo.class, "frontRightSteer");
        AnalogInput frEncoder = hardwareMap.get(AnalogInput.class, "frontRightEncoder");
        frontRight = new SwerveModule(frDrive, frSteer, frEncoder, "FR", 0.0);
        
        // Back Left Module
        DcMotorEx blDrive = hardwareMap.get(DcMotorEx.class, "backLeftDrive");
        Servo blSteer = hardwareMap.get(Servo.class, "backLeftSteer");
        AnalogInput blEncoder = hardwareMap.get(AnalogInput.class, "backLeftEncoder");
        backLeft = new SwerveModule(blDrive, blSteer, blEncoder, "BL", 0.0);
        
        // Back Right Module
        DcMotorEx brDrive = hardwareMap.get(DcMotorEx.class, "backRightDrive");
        Servo brSteer = hardwareMap.get(Servo.class, "backRightSteer");
        AnalogInput brEncoder = hardwareMap.get(AnalogInput.class, "backRightEncoder");
        backRight = new SwerveModule(brDrive, brSteer, brEncoder, "BR", 0.0);
    }
    
    /**
     * Setup performance optimizations
     */
    private void setupPerformanceOptimizations(HardwareMap hardwareMap) {
        if (SwerveConfig.USE_BULK_READS) {
            // Get all Lynx modules (Control Hub and Expansion Hub)
            allHubs = hardwareMap.getAll(LynxModule.class);
            
            // Set bulk caching mode for better performance
            for (LynxModule module : allHubs) {
                module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            }
        }
        bulkReadTimer.reset();
    }
    
    /**
     * Main drive method - call this in your OpMode loop
     * @param xSpeed Forward/backward speed (-1.0 to 1.0)
     * @param ySpeed Left/right speed (-1.0 to 1.0)  
     * @param rotation Rotational speed (-1.0 to 1.0)
     */
    public void drive(double xSpeed, double ySpeed, double rotation) {
        // Performance optimization - bulk read sensors
        updateBulkReads();
        
        // Apply deadbands
        xSpeed = applyDeadband(xSpeed, SwerveConfig.DRIVE_DEADBAND);
        ySpeed = applyDeadband(ySpeed, SwerveConfig.DRIVE_DEADBAND);
        rotation = applyDeadband(rotation, SwerveConfig.ROTATION_DEADBAND);
        
        // Scale rotation speed
        rotation *= SwerveConfig.MAX_ANG_VEL;
        
        // Calculate swerve module states using kinematics
        SwerveModuleState[] moduleStates = calculateModuleStates(xSpeed, ySpeed, rotation);
        
        // Normalize wheel speeds if any exceed maximum
        normalizeWheelSpeeds(moduleStates);
        
        // Update each module
        frontLeft.update(moduleStates[0].speed, moduleStates[0].angle);
        frontRight.update(moduleStates[1].speed, moduleStates[1].angle);
        backLeft.update(moduleStates[2].speed, moduleStates[2].angle);
        backRight.update(moduleStates[3].speed, moduleStates[3].angle);
    }
    
    /**
     * Drive using gamepad input
     */
    public void driveWithGamepad(Gamepad gamepad) {
        double xSpeed = -gamepad.left_stick_y;  // Forward/backward (inverted)
        double ySpeed = gamepad.left_stick_x;   // Left/right
        double rotation = gamepad.right_stick_x; // Rotation
        
        drive(xSpeed, ySpeed, rotation);
    }
    
    /**
     * Calculate swerve module states using inverse kinematics
     */
    private SwerveModuleState[] calculateModuleStates(double xSpeed, double ySpeed, double rotation) {
        SwerveModuleState[] states = new SwerveModuleState[4];
        
        // Front Left
        double flX = xSpeed - rotation * moduleY;
        double flY = ySpeed + rotation * moduleX;
        states[0] = new SwerveModuleState(Math.hypot(flX, flY), Math.toDegrees(Math.atan2(flY, flX)));
        
        // Front Right  
        double frX = xSpeed - rotation * moduleY;
        double frY = ySpeed - rotation * moduleX;
        states[1] = new SwerveModuleState(Math.hypot(frX, frY), Math.toDegrees(Math.atan2(frY, frX)));
        
        // Back Left
        double blX = xSpeed + rotation * moduleY;
        double blY = ySpeed + rotation * moduleX;  
        states[2] = new SwerveModuleState(Math.hypot(blX, blY), Math.toDegrees(Math.atan2(blY, blX)));
        
        // Back Right
        double brX = xSpeed + rotation * moduleY;
        double brY = ySpeed - rotation * moduleX;
        states[3] = new SwerveModuleState(Math.hypot(brX, brY), Math.toDegrees(Math.atan2(brY, brX)));
        
        return states;
    }
    
    /**
     * Normalize wheel speeds to prevent any from exceeding maximum
     */
    private void normalizeWheelSpeeds(SwerveModuleState[] moduleStates) {
        double maxSpeed = 0.0;
        
        // Find maximum speed
        for (SwerveModuleState state : moduleStates) {
            maxSpeed = Math.max(maxSpeed, Math.abs(state.speed));
        }
        
        // Scale down if necessary
        if (maxSpeed > 1.0) {
            for (SwerveModuleState state : moduleStates) {
                state.speed /= maxSpeed;
            }
        }
    }
    
    /**
     * Apply deadband to input value
     */
    private double applyDeadband(double value, double deadband) {
        if (Math.abs(value) < deadband) {
            return 0.0;
        }
        return value;
    }
    
    /**
     * Update bulk reads for performance optimization
     */
    private void updateBulkReads() {
        if (SwerveConfig.USE_BULK_READS && 
            bulkReadTimer.milliseconds() >= SwerveConfig.CONTROL_HUB_CACHE_TIME) {
            
            // Clear and update bulk cache
            for (LynxModule module : allHubs) {
                module.clearBulkCache();
            }
            bulkReadTimer.reset();
        }
    }
    
    /**
     * Stop all modules
     */
    public void stop() {
        frontLeft.setDriveSpeed(0.0);
        frontRight.setDriveSpeed(0.0);
        backLeft.setDriveSpeed(0.0);
        backRight.setDriveSpeed(0.0);
    }
    
    /**
     * Update PID coefficients from config
     */
    public void updatePIDCoefficients() {
        frontLeft.updatePIDCoefficients();
        frontRight.updatePIDCoefficients();
        backLeft.updatePIDCoefficients();
        backRight.updatePIDCoefficients();
        
        rotationPID.setPID(SwerveConfig.ANGULAR_kP, SwerveConfig.ANGULAR_kI, SwerveConfig.ANGULAR_kD);
    }
    
    /**
     * Get module states for telemetry
     */
    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            new SwerveModuleState(frontLeft.getDrivePower(), frontLeft.getCurrentAngle()),
            new SwerveModuleState(frontRight.getDrivePower(), frontRight.getCurrentAngle()),
            new SwerveModuleState(backLeft.getDrivePower(), backLeft.getCurrentAngle()),
            new SwerveModuleState(backRight.getDrivePower(), backRight.getCurrentAngle())
        };
    }
    
    /**
     * Get individual modules for advanced control
     */
    public SwerveModule getFrontLeft() { return frontLeft; }
    public SwerveModule getFrontRight() { return frontRight; }
    public SwerveModule getBackLeft() { return backLeft; }
    public SwerveModule getBackRight() { return backRight; }
    
    /**
     * Inner class to represent module state
     */
    public static class SwerveModuleState {
        public double speed;
        public double angle;
        
        public SwerveModuleState(double speed, double angle) {
            this.speed = speed;
            this.angle = angle;
        }
    }
}