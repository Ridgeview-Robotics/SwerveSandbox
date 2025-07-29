package org.firstinspires.ftc.teamcode.c3_testzone;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Individual swerve module controller - DEBUG VERSION
 * Manages one drive motor and one continuous rotation servo with analog encoder feedback
 */
public class SwerveModule {

    private DcMotorEx driveMotor;
    private CRServo steerServo;
    private AnalogInput steerEncoder;

    private SwerveModulePID steerPID;

    private double targetAngle = 0.0; // radians
    private double currentAngle = 0.0; // radians
    private double lastServoPower = SwerveConfig.SERVO_STOP_POWER;
    private ElapsedTime servoTimer = new ElapsedTime();

    private final String moduleName;
    private final double encoderOffset;

    // Performance optimization variables
    private double lastDrivePower = 0.0;
    private boolean isInitialized = false;

    // Debug variables
    private double lastPIDOutput = 0.0;
    private double lastAngleError = 0.0;
    private double rawVoltage = 0.0;

    public SwerveModule(DcMotorEx driveMotor, CRServo steerServo, AnalogInput steerEncoder,
                        String moduleName, double encoderOffset) {
        this.driveMotor = driveMotor;
        this.steerServo = steerServo;
        this.steerEncoder = steerEncoder;
        this.moduleName = moduleName;
        this.encoderOffset = encoderOffset;

        // Initialize PID controller
        steerPID = new SwerveModulePID(
                SwerveConfig.SWERVE_kP,
                SwerveConfig.SWERVE_kI,
                SwerveConfig.SWERVE_kD
        );

        // Configure drive motor
        driveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Stop the steering servo initially
        steerServo.setPower(SwerveConfig.SERVO_STOP_POWER);

        servoTimer.reset();
    }

    /**
     * Initialize module - align to straight position if configured
     */
    public void initialize() {
        if (SwerveConfig.AUTO_ALIGN_ON_INIT) {
            alignToStraight();
        }
        isInitialized = true;
    }

    /**
     * Align module to straight position (0 radians)
     */
    public void alignToStraight() {
        ElapsedTime alignTimer = new ElapsedTime();
        alignTimer.reset();

        while (!isAtTargetAngle(0.0) && alignTimer.seconds() < SwerveConfig.AUTO_ALIGN_TIMEOUT) {
            updateSteering(0.0); // Target 0 radians (straight)
            try {
                Thread.sleep(20); // Small delay to prevent overwhelming the servo
            } catch (InterruptedException e) {
                break;
            }
        }

        // Stop the servo
        steerServo.setPower(SwerveConfig.SERVO_STOP_POWER);
        targetAngle = 0.0;
    }

    /**
     * Update the module with desired speed and angle
     * @param speed Drive speed (-1.0 to 1.0)
     * @param angle Target angle in radians
     */
    public void update(double speed, double angle) {
        // Update current angle reading
        getCurrentAngle();

        // Optimize angle to minimize rotation (stay within current position if no input)
        angle = optimizeAngle(angle);

        // Update steering angle
        setTargetAngle(angle);
        updateSteering(angle);

        // Update drive speed
        setDriveSpeed(speed);
    }

    /**
     * Set the target steering angle in radians
     */
    public void setTargetAngle(double angle) {
        // Normalize angle to 0-2π range
        targetAngle = normalizeAngle(angle);
    }

    /**
     * Get current steering angle from analog encoder in radians
     * SIMPLIFIED VERSION - less complex wraparound logic
     */
    public double getCurrentAngle() {
        rawVoltage = steerEncoder.getVoltage();

        // Simple conversion from voltage to radians
        double rawAngle = rawVoltage * SwerveConfig.VOLTAGE_TO_RADIANS;

        // Apply offset and normalize
        currentAngle = normalizeAngle(rawAngle - encoderOffset);

        return currentAngle;
    }

    /**
     * Update servo steering using PID control - SIMPLIFIED WITH DEBUG
     */
    private void updateSteering(double targetAngle) {
        double currentAngle = getCurrentAngle();

        // Calculate angle error manually for debugging
        lastAngleError = angleError(targetAngle, currentAngle);

        // Calculate PID output (this will be servo power adjustment)
        lastPIDOutput = steerPID.calculate(targetAngle, currentAngle);

        // Convert PID output to servo power
        // PID output is the adjustment from the stop power (0.5)
        double servoPower = SwerveConfig.SERVO_STOP_POWER + lastPIDOutput;

        // Clamp servo power to valid range
        servoPower = Range.clip(servoPower, SwerveConfig.SERVO_MIN_POWER, SwerveConfig.SERVO_MAX_POWER);

        // MODIFIED: Only apply deadband if error is very small AND we're close to stop power
        // This prevents the servo from getting "stuck" when it needs to move
        if (Math.abs(lastAngleError) < SwerveConfig.SERVO_TOLERANCE &&
                Math.abs(servoPower - SwerveConfig.SERVO_STOP_POWER) < SwerveConfig.SERVO_DEADBAND) {
            servoPower = SwerveConfig.SERVO_STOP_POWER;
        }

        // ALWAYS update servo in debug mode (disable optimization temporarily)
        steerServo.setPower(servoPower);
        lastServoPower = servoPower;
        servoTimer.reset();

        /* Original optimization code - disabled for debugging
        if (SwerveConfig.OPTIMIZE_SERVO_WRITES) {
            if (Math.abs(servoPower - lastServoPower) > 0.01 ||
                servoTimer.seconds() > SwerveConfig.SERVO_SETTLE_TIME) {
                steerServo.setPower(servoPower);
                lastServoPower = servoPower;
                servoTimer.reset();
            }
        } else {
            steerServo.setPower(servoPower);
            lastServoPower = servoPower;
        }
        */
    }

    /**
     * Set drive motor speed
     */
    public void setDriveSpeed(double speed) {
        speed = Range.clip(speed, -SwerveConfig.MAX_DRIVE_POWER, SwerveConfig.MAX_DRIVE_POWER);

        // Always update in debug mode
        driveMotor.setPower(speed);
        lastDrivePower = speed;

        /* Original optimization - disabled for debugging
        if (Math.abs(speed - lastDrivePower) > 0.01) {
            driveMotor.setPower(speed);
            lastDrivePower = speed;
        }
        */
    }

    /**
     * Normalize angle to 0-2π range
     */
    private double normalizeAngle(double angle) {
        return ((angle % (2 * Math.PI)) + (2 * Math.PI)) % (2 * Math.PI);
    }

    /**
     * SIMPLIFIED angle optimization - less aggressive
     */
    private double optimizeAngle(double requestedAngle) {
        // For now, just return the requested angle normalized
        // This removes the complex optimization that might be causing issues
        return normalizeAngle(requestedAngle);

        /* Original complex optimization - disabled for debugging
        if (!isInitialized) {
            return requestedAngle;
        }

        double angleDifference = requestedAngle - targetAngle;
        while (angleDifference > Math.PI) angleDifference -= 2 * Math.PI;
        while (angleDifference < -Math.PI) angleDifference += 2 * Math.PI;

        if (Math.abs(angleDifference) > 0.1) {
            double currentAngle = getCurrentAngle();
            double difference = requestedAngle - currentAngle;

            while (difference > Math.PI) difference -= 2 * Math.PI;
            while (difference < -Math.PI) difference += 2 * Math.PI;

            if (Math.abs(difference) > Math.PI / 2) {
                if (difference > Math.PI / 2) {
                    requestedAngle -= Math.PI;
                } else {
                    requestedAngle += Math.PI;
                }
            }

            return normalizeAngle(requestedAngle);
        } else {
            return targetAngle;
        }
        */
    }

    /**
     * Check if module is at target angle within tolerance
     */
    public boolean isAtTargetAngle() {
        return isAtTargetAngle(targetAngle);
    }

    /**
     * Check if module is at specified angle within tolerance
     */
    public boolean isAtTargetAngle(double angle) {
        double error = Math.abs(angleError(angle, getCurrentAngle()));
        return error < SwerveConfig.SERVO_TOLERANCE;
    }

    /**
     * Calculate angle error with proper wraparound handling
     */
    private double angleError(double target, double current) {
        double error = target - current;
        while (error > Math.PI) error -= 2 * Math.PI;
        while (error < -Math.PI) error += 2 * Math.PI;
        return error;
    }

    /**
     * Get drive motor position for odometry
     */
    public double getDrivePosition() {
        return driveMotor.getCurrentPosition();
    }

    /**
     * Get drive motor velocity
     */
    public double getDriveVelocity() {
        return driveMotor.getVelocity();
    }

    /**
     * Reset drive encoder
     */
    public void resetDriveEncoder() {
        driveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Stop the module
     */
    public void stop() {
        driveMotor.setPower(0.0);
        steerServo.setPower(SwerveConfig.SERVO_STOP_POWER);
    }

    /**
     * Get module name for debugging
     */
    public String getModuleName() {
        return moduleName;
    }

    /**
     * Get current target angle in radians
     */
    public double getTargetAngle() {
        return targetAngle;
    }

    /**
     * Get current drive power
     */
    public double getDrivePower() {
        return lastDrivePower;
    }

    /**
     * Get current servo power
     */
    public double getServoPower() {
        return lastServoPower;
    }

    /**
     * Update PID coefficients from config
     */
    public void updatePIDCoefficients() {
        steerPID.setPID(SwerveConfig.SWERVE_kP, SwerveConfig.SWERVE_kI, SwerveConfig.SWERVE_kD);
    }

    /**
     * Get raw encoder voltage for calibration
     */
    public double getRawEncoderVoltage() {
        return rawVoltage;
    }

    /**
     * Get current angle in degrees for display
     */
    public double getCurrentAngleDegrees() {
        return Math.toDegrees(getCurrentAngle());
    }

    /**
     * Get target angle in degrees for display
     */
    public double getTargetAngleDegrees() {
        return Math.toDegrees(targetAngle);
    }

    // DEBUG METHODS

    /**
     * Get last PID output for debugging
     */
    public double getLastPIDOutput() {
        return lastPIDOutput;
    }

    /**
     * Get last angle error for debugging
     */
    public double getLastAngleError() {
        return lastAngleError;
    }

    /**
     * Get last angle error in degrees for debugging
     */
    public double getLastAngleErrorDegrees() {
        return Math.toDegrees(lastAngleError);
    }
}