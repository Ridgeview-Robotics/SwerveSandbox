package org.firstinspires.ftc.teamcode.assets;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.lynx.LynxModule;

import java.util.List;

/**
 * Swerve drivetrain controller for FTC
 * Manages 4 swerve modules with robot-centric control
 * Uses continuous rotation servos with analog encoder feedback
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
    private double robotAngle = 0.0; // Current robot heading (for future field-centric)
    private SwerveModulePID rotationPID;

    // Module positions for kinematics (inches from robot center)
    private final double moduleX = SwerveConfig.MODULE_X_OFFSET;
    private final double moduleY = SwerveConfig.MODULE_Y_OFFSET;

    // Previous drive state for "sticky" wheel positions
    private double lastXSpeed = 0.0;
    private double lastYSpeed = 0.0;
    private double lastRotation = 0.0;
    private boolean hadInput = false;

    public SwerveDrivetrain(HardwareMap hardwareMap) {
        initializeHardware(hardwareMap);
        setupPerformanceOptimizations(hardwareMap);

        // Initialize rotation PID (for future use)
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
        CRServo flSteer = hardwareMap.get(CRServo.class, "frontLeftSteer");
        AnalogInput flEncoder = hardwareMap.get(AnalogInput.class, "frontLeftEncoder");
        frontLeft = new SwerveModule(flDrive, flSteer, flEncoder, "FL", SwerveConfig.FL_ENCODER_OFFSET);

        // Front Right Module
        DcMotorEx frDrive = hardwareMap.get(DcMotorEx.class, "frontRightDrive");
        CRServo frSteer = hardwareMap.get(CRServo.class, "frontRightSteer");
        AnalogInput frEncoder = hardwareMap.get(AnalogInput.class, "frontRightEncoder");
        frontRight = new SwerveModule(frDrive, frSteer, frEncoder, "FR", SwerveConfig.FR_ENCODER_OFFSET);

        // Back Left Module
        DcMotorEx blDrive = hardwareMap.get(DcMotorEx.class, "backLeftDrive");
        CRServo blSteer = hardwareMap.get(CRServo.class, "backLeftSteer");
        AnalogInput blEncoder = hardwareMap.get(AnalogInput.class, "backLeftEncoder");
        backLeft = new SwerveModule(blDrive, blSteer, blEncoder, "BL", SwerveConfig.BL_ENCODER_OFFSET);

        // Back Right Module
        DcMotorEx brDrive = hardwareMap.get(DcMotorEx.class, "backRightDrive");
        CRServo brSteer = hardwareMap.get(CRServo.class, "backRightSteer");
        AnalogInput brEncoder = hardwareMap.get(AnalogInput.class, "backRightEncoder");
        backRight = new SwerveModule(brDrive, brSteer, brEncoder, "BR", SwerveConfig.BR_ENCODER_OFFSET);

        // Initialize all modules (this will align wheels if configured)
        frontLeft.initialize();
        frontRight.initialize();
        backLeft.initialize();
        backRight.initialize();
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
     * Implements "sticky" wheel behavior - wheels maintain position when no input
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

        // Check if we have any input
        boolean hasInput = (Math.abs(xSpeed) > 0 || Math.abs(ySpeed) > 0 || Math.abs(rotation) > 0);

        // If no input and we previously had input, maintain wheel angles but stop driving
        if (!hasInput && hadInput) {
            // Stop all drive motors but maintain wheel angles
            frontLeft.setDriveSpeed(0.0);
            frontRight.setDriveSpeed(0.0);
            backLeft.setDriveSpeed(0.0);
            backRight.setDriveSpeed(0.0);

            // Keep the steering servos at their last positions by not updating them
            return;
        }

        // If we have input, calculate new module states
        if (hasInput) {
            // Scale rotation speed
            rotation *= SwerveConfig.MAX_ANG_VEL;

            // Calculate swerve module states using kinematics
            SwerveModuleState[] moduleStates = calculateModuleStates(xSpeed, ySpeed, rotation);

            // Normalize wheel speeds if any exceed maximum
            normalizeWheelSpeeds(moduleStates);

            // Apply module optimization (flipping) to minimize rotation
            optimizeModuleStates(moduleStates);

            // Update each module
            frontLeft.update(moduleStates[0].speed, moduleStates[0].angle);
            frontRight.update(moduleStates[1].speed, moduleStates[1].angle);
            backLeft.update(moduleStates[2].speed, moduleStates[2].angle);
            backRight.update(moduleStates[3].speed, moduleStates[3].angle);

            // Store current input state
            lastXSpeed = xSpeed;
            lastYSpeed = ySpeed;
            lastRotation = rotation;
        }

        hadInput = hasInput;
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
     * Returns angles in radians and speeds normalized to [-1, 1]
     */
    private SwerveModuleState[] calculateModuleStates(double xSpeed, double ySpeed, double rotation) {
        SwerveModuleState[] states = new SwerveModuleState[4];

        // Convert rotation from rad/s back to normalized for calculations
        double normalizedRotation = rotation / SwerveConfig.MAX_ANG_VEL;

        // Front Left
        double flX = xSpeed - normalizedRotation * moduleY;
        double flY = ySpeed + normalizedRotation * moduleX;
        double flSpeed = Math.hypot(flX, flY);
        double flAngle = Math.atan2(flY, flX);
        if (flAngle < 0) flAngle += 2 * Math.PI; // Normalize to 0-2π
        states[0] = new SwerveModuleState(flSpeed, flAngle);

        // Front Right
        double frX = xSpeed - normalizedRotation * moduleY;
        double frY = ySpeed - normalizedRotation * moduleX;
        double frSpeed = Math.hypot(frX, frY);
        double frAngle = Math.atan2(frY, frX);
        if (frAngle < 0) frAngle += 2 * Math.PI;
        states[1] = new SwerveModuleState(frSpeed, frAngle);

        // Back Left
        double blX = xSpeed + normalizedRotation * moduleY;
        double blY = ySpeed + normalizedRotation * moduleX;
        double blSpeed = Math.hypot(blX, blY);
        double blAngle = Math.atan2(blY, blX);
        if (blAngle < 0) blAngle += 2 * Math.PI;
        states[2] = new SwerveModuleState(blSpeed, blAngle);

        // Back Right
        double brX = xSpeed + normalizedRotation * moduleY;
        double brY = ySpeed - normalizedRotation * moduleX;
        double brSpeed = Math.hypot(brX, brY);
        double brAngle = Math.atan2(brY, brX);
        if (brAngle < 0) brAngle += 2 * Math.PI;
        states[3] = new SwerveModuleState(brSpeed, brAngle);

        return states;
    }

    /**
     * Optimize module states to minimize rotation (handle module flipping)
     */
    private void optimizeModuleStates(SwerveModuleState[] moduleStates) {
        for (SwerveModuleState state : moduleStates) {
            // Check if we should flip the module (rotate 180° and reverse speed)
            // This will be handled in the SwerveModule.optimizeAngle() method
            // So we don't need to do it here, just pass the states through
        }
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
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();

        // Reset input tracking
        hadInput = false;
        lastXSpeed = 0.0;
        lastYSpeed = 0.0;
        lastRotation = 0.0;
    }

    /**
     * Manually align all wheels to straight position
     */
    public void alignWheelsStraight() {
        frontLeft.alignToStraight();
        frontRight.alignToStraight();
        backLeft.alignToStraight();
        backRight.alignToStraight();
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
     * Get module states in degrees for easier reading
     */
    public SwerveModuleState[] getModuleStatesDegrees() {
        return new SwerveModuleState[] {
                new SwerveModuleState(frontLeft.getDrivePower(), Math.toDegrees(frontLeft.getCurrentAngle())),
                new SwerveModuleState(frontRight.getDrivePower(), Math.toDegrees(frontRight.getCurrentAngle())),
                new SwerveModuleState(backLeft.getDrivePower(), Math.toDegrees(backLeft.getCurrentAngle())),
                new SwerveModuleState(backRight.getDrivePower(), Math.toDegrees(backRight.getCurrentAngle()))
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
     * Get diagnostic information
     */
    public String getDiagnosticInfo() {
        StringBuilder sb = new StringBuilder();
        sb.append("Swerve Drivetrain Status:\n");
        sb.append("Had Input: ").append(hadInput).append("\n");
        sb.append("Last Input: X=").append(String.format("%.2f", lastXSpeed))
                .append(", Y=").append(String.format("%.2f", lastYSpeed))
                .append(", R=").append(String.format("%.2f", lastRotation)).append("\n");

        SwerveModuleState[] states = getModuleStatesDegrees();
        sb.append("FL: Speed=").append(String.format("%.2f", states[0].speed))
                .append(", Angle=").append(String.format("%.1f°", states[0].angle)).append("\n");
        sb.append("FR: Speed=").append(String.format("%.2f", states[1].speed))
                .append(", Angle=").append(String.format("%.1f°", states[1].angle)).append("\n");
        sb.append("BL: Speed=").append(String.format("%.2f", states[2].speed))
                .append(", Angle=").append(String.format("%.1f°", states[2].angle)).append("\n");
        sb.append("BR: Speed=").append(String.format("%.2f", states[3].speed))
                .append(", Angle=").append(String.format("%.1f°", states[3].angle)).append("\n");

        return sb.toString();
    }

    /**
     * Inner class to represent module state
     */
    public static class SwerveModuleState {
        public double speed; // -1.0 to 1.0
        public double angle; // radians (0 to 2π) or degrees for display

        public SwerveModuleState(double speed, double angle) {
            this.speed = speed;
            this.angle = angle;
        }

        @Override
        public String toString() {
            return String.format("Speed: %.2f, Angle: %.1f", speed, angle);
        }
    }
}