package org.firstinspires.ftc.teamcode.assets;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Lightweight PID controller optimized for swerve module steering
 * Handles angle wrapping and minimizes computation overhead
 */
public class SwerveModulePID {
    
    private double kP, kI, kD;
    private double previousError = 0.0;
    private double integral = 0.0;
    private ElapsedTime timer = new ElapsedTime();
    
    // Integral windup prevention
    private final double integralMax = 0.5;
    private final double integralMin = -0.5;
    
    public SwerveModulePID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        timer.reset();
    }
    
    /**
     * Calculate PID output for angle control
     * @param setpoint Target angle (degrees)
     * @param measurement Current angle (degrees)
     * @return PID output
     */
    public double calculate(double setpoint, double measurement) {
        double deltaTime = timer.seconds();
        timer.reset();
        
        // Calculate error with angle wrapping
        double error = angleError(setpoint, measurement);
        
        // Proportional term
        double proportional = kP * error;
        
        // Integral term with windup protection
        integral += error * deltaTime;
        integral = Math.max(integralMin, Math.min(integralMax, integral));
        double integralTerm = kI * integral;
        
        // Derivative term
        double derivative = 0.0;
        if (deltaTime > 0) {
            derivative = kD * (error - previousError) / deltaTime;
        }
        
        previousError = error;
        
        return proportional + integralTerm + derivative;
    }
    
    /**
     * Calculate angle error with proper wrapping
     * Returns error in range [-180, 180]
     */
    private double angleError(double setpoint, double measurement) {
        double error = setpoint - measurement;
        
        // Wrap error to [-180, 180] range
        while (error > 180) error -= 360;
        while (error < -180) error += 360;
        
        return error;
    }
    
    /**
     * Update PID coefficients
     */
    public void setPID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }
    
    /**
     * Reset PID controller state
     */
    public void reset() {
        previousError = 0.0;
        integral = 0.0;
        timer.reset();
    }
    
    /**
     * Get current PID coefficients
     */
    public double getKP() { return kP; }
    public double getKI() { return kI; }
    public double getKD() { return kD; }
    
    /**
     * Get current integral value for debugging
     */
    public double getIntegral() { return integral; }
    
    /**
     * Get last error for debugging
     */
    public double getLastError() { return previousError; }
}