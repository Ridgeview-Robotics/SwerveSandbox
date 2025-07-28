package org.firstinspires.ftc.teamcode.assets;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Lightweight PID controller optimized for swerve module steering
 * Handles angle wrapping and outputs servo power adjustments
 * Output range is designed for continuous servo control
 */
public class SwerveModulePID {
    
    private double kP, kI, kD;
    private double previousError = 0.0;
    private double integral = 0.0;
    private ElapsedTime timer = new ElapsedTime();
    
    // Integral windup prevention and output limits
    private final double integralMax = 0.3;
    private final double integralMin = -0.3;
    private final double outputMax = 0.4; // Maximum servo power adjustment from 0.5
    private final double outputMin = -0.4; // Minimum servo power adjustment from 0.5
    
    public SwerveModulePID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        timer.reset();
    }
    
    /**
     * Calculate PID output for angle control
     * @param setpoint Target angle (radians)
     * @param measurement Current angle (radians)
     * @return PID output (servo power adjustment from 0.5)
     */
    public double calculate(double setpoint, double measurement) {
        double deltaTime = timer.seconds();
        timer.reset();
        
        // Calculate error with angle wrapping (radians)
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
        
        // Calculate total output and clamp to servo power range
        double output = proportional + integralTerm + derivative;
        return Math.max(outputMin, Math.min(outputMax, output));
    }
    
    /**
     * Calculate angle error with proper wrapping for radians
     * Returns error in range [-π, π]
     */
    private double angleError(double setpoint, double measurement) {
        double error = setpoint - measurement;
        
        // Wrap error to [-π, π] range
        while (error > Math.PI) error -= 2 * Math.PI;
        while (error < -Math.PI) error += 2 * Math.PI;
        
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