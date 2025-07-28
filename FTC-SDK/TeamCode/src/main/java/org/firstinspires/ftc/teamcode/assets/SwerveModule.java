package org.firstinspires.ftc.teamcode.assets;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Individual swerve module controller
 * Manages one drive motor and one steering servo with encoder feedback
 */
public class SwerveModule {
    
    private DcMotorEx driveMotor;
    private Servo steerServo;
    private AnalogInput steerEncoder;
    
    private SwerveModulePID steerPID;
    
    private double targetAngle = 0.0;
    private double currentAngle = 0.0;
    private double lastServoPosition = 0.0;
    private ElapsedTime servoTimer = new ElapsedTime();
    
    private final String moduleName;
    private final double encoderOffset;
    
    // Performance optimization variables
    private double lastDrivePower = 0.0;
    private boolean driveMotorChanged = false;
    
    public SwerveModule(DcMotorEx driveMotor, Servo steerServo, AnalogInput steerEncoder, 
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
        
        servoTimer.reset();
    }
    
    /**
     * Update the module with desired speed and angle
     * @param speed Drive speed (-1.0 to 1.0)
     * @param angle Target angle in degrees (0-360)
     */
    public void update(double speed, double angle) {
        // Optimize angle to minimize rotation
        angle = optimizeAngle(angle, getCurrentAngle());
        
        // Update steering angle
        setTargetAngle(angle);
        updateSteering();
        
        // Update drive speed
        setDriveSpeed(speed);
    }
    
    /**
     * Set the target steering angle
     */
    public void setTargetAngle(double angle) {
        // Normalize angle to 0-360 range
        targetAngle = normalizeAngle(angle);
    }
    
    /**
     * Get current steering angle from encoder
     */
    public double getCurrentAngle() {
        double voltage = steerEncoder.getVoltage();
        double maxVoltage = steerEncoder.getMaxVoltage();
        
        // Convert voltage to angle (0-360 degrees)
        double rawAngle = (voltage / maxVoltage) * 360.0;
        
        // Apply offset and normalize
        currentAngle = normalizeAngle(rawAngle - encoderOffset);
        
        return currentAngle;
    }
    
    /**
     * Update servo steering using PID control
     */
    private void updateSteering() {
        double currentAngle = getCurrentAngle();
        
        // Calculate PID output
        double pidOutput = steerPID.calculate(targetAngle, currentAngle);
        
        // Convert to servo position (0.0 to 1.0)
        double servoPosition = angleToServoPosition(targetAngle);
        
        // Only update servo if position changed significantly (performance optimization)
        if (SwerveConfig.OPTIMIZE_SERVO_WRITES) {
            if (Math.abs(servoPosition - lastServoPosition) > 0.01 || 
                servoTimer.seconds() > SwerveConfig.SERVO_SETTLE_TIME) {
                steerServo.setPosition(servoPosition);
                lastServoPosition = servoPosition;
                servoTimer.reset();
            }
        } else {
            steerServo.setPosition(servoPosition);
        }
    }
    
    /**
     * Set drive motor speed
     */
    public void setDriveSpeed(double speed) {
        speed = Range.clip(speed, -SwerveConfig.MAX_DRIVE_POWER, SwerveConfig.MAX_DRIVE_POWER);
        
        // Performance optimization - only update if changed
        if (Math.abs(speed - lastDrivePower) > 0.01) {
            driveMotor.setPower(speed);
            lastDrivePower = speed;
            driveMotorChanged = true;
        }
    }
    
    /**
     * Convert angle to servo position (0.0 to 1.0)
     */
    private double angleToServoPosition(double angle) {
        return Range.clip(angle / 360.0, 0.0, 1.0);
    }
    
    /**
     * Normalize angle to 0-360 range
     */
    private double normalizeAngle(double angle) {
        while (angle < 0) angle += 360;
        while (angle >= 360) angle -= 360;
        return angle;
    }
    
    /**
     * Optimize angle to minimize rotation distance
     */
    private double optimizeAngle(double targetAngle, double currentAngle) {
        double difference = targetAngle - currentAngle;
        
        // Normalize difference to -180 to +180
        while (difference > 180) difference -= 360;
        while (difference < -180) difference += 360;
        
        // If we need to rotate more than 90 degrees, flip the module
        if (Math.abs(difference) > 90) {
            if (difference > 90) {
                targetAngle -= 180;
            } else {
                targetAngle += 180;
            }
            // Also reverse the drive direction (handled in SwerveDrivetrain)
        }
        
        return normalizeAngle(targetAngle);
    }
    
    /**
     * Check if module is at target angle within tolerance
     */
    public boolean isAtTargetAngle() {
        double error = Math.abs(targetAngle - getCurrentAngle());
        return error < SwerveConfig.SERVO_TOLERANCE;
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
     * Get module name for debugging
     */
    public String getModuleName() {
        return moduleName;
    }
    
    /**
     * Get current target angle
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
     * Update PID coefficients from config
     */
    public void updatePIDCoefficients() {
        steerPID.setPID(SwerveConfig.SWERVE_kP, SwerveConfig.SWERVE_kI, SwerveConfig.SWERVE_kD);
    }
}