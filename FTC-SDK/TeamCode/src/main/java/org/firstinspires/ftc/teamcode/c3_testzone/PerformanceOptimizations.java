package org.firstinspires.ftc.teamcode.c3_testzone;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

/**
 * FTC-Legal Performance Optimizations for Control Hub
 * 
 * Research-based optimizations that are legal for FTC competition:
 * - Bulk sensor reads to reduce I2C overhead
 * - Efficient motor control patterns
 * - Optimized servo update rates
 * - Memory management techniques
 * - Loop time monitoring and optimization
 */
public class PerformanceOptimizations {
    
    private List<LynxModule> allHubs;
    private ElapsedTime bulkReadTimer = new ElapsedTime();
    private ElapsedTime performanceTimer = new ElapsedTime();
    
    // Performance monitoring
    private double averageLoopTime = 0.0;
    private int loopCount = 0;
    private long totalCycles = 0;
    
    // Optimization flags
    private boolean bulkReadsEnabled = true;
    private boolean servoOptimizationEnabled = true;
    private boolean motorOptimizationEnabled = true;
    
    // Cache management
    private static final int BULK_READ_INTERVAL_MS = 5;
    private static final int SERVO_UPDATE_THRESHOLD = 10; // milliseconds
    private static final double MOTOR_POWER_THRESHOLD = 0.01;
    
    public PerformanceOptimizations(HardwareMap hardwareMap) {
        initializeOptimizations(hardwareMap);
    }
    
    /**
     * Initialize all FTC-legal performance optimizations
     */
    private void initializeOptimizations(HardwareMap hardwareMap) {
        setupBulkReads(hardwareMap);
        performanceTimer.reset();
        bulkReadTimer.reset();
    }
    
    /**
     * Setup bulk reads for all Lynx modules
     * This is the most impactful legal optimization for FTC
     */
    private void setupBulkReads(HardwareMap hardwareMap) {
        // Get all Lynx modules (Control Hub and any Expansion Hubs)
        allHubs = hardwareMap.getAll(LynxModule.class);
        
        for (LynxModule module : allHubs) {
            // Set to MANUAL mode for maximum performance
            // This allows us to control when sensor reads happen
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        
        bulkReadsEnabled = !allHubs.isEmpty();
    }
    
    /**
     * Update bulk reads - call this at the start of each loop iteration
     * Reduces I2C overhead by batching sensor reads
     */
    public void updateBulkReads() {
        if (!bulkReadsEnabled) return;
        
        // Only update if enough time has passed
        if (bulkReadTimer.milliseconds() >= BULK_READ_INTERVAL_MS) {
            for (LynxModule module : allHubs) {
                // Clear the cache to force fresh reads
                module.clearBulkCache();
            }
            bulkReadTimer.reset();
        }
    }
    
    /**
     * Optimized motor power setting
     * Only updates motor if power changed significantly
     */
    public static class OptimizedMotorController {
        private double lastPower = Double.NaN;
        private ElapsedTime updateTimer = new ElapsedTime();
        
        public boolean setPowerIfChanged(com.qualcomm.robotcore.hardware.DcMotor motor, double power) {
            // Check if power changed significantly
            if (Double.isNaN(lastPower) || Math.abs(power - lastPower) > MOTOR_POWER_THRESHOLD) {
                motor.setPower(power);
                lastPower = power;
                updateTimer.reset();
                return true;
            }
            return false;
        }
        
        public double getLastPower() { return lastPower; }
        public double getTimeSinceUpdate() { return updateTimer.seconds(); }
    }
    
    /**
     * Optimized servo controller
     * Reduces servo update frequency to prevent jitter and save processing
     */
    public static class OptimizedServoController {
        private double lastPosition = Double.NaN;
        private ElapsedTime updateTimer = new ElapsedTime();
        private static final double POSITION_THRESHOLD = 0.005; // Minimum change to update
        
        public boolean setPositionIfChanged(com.qualcomm.robotcore.hardware.Servo servo, double position) {
            // Check if position changed significantly or enough time passed
            boolean significantChange = Double.isNaN(lastPosition) || 
                Math.abs(position - lastPosition) > POSITION_THRESHOLD;
            boolean timeForUpdate = updateTimer.milliseconds() > SERVO_UPDATE_THRESHOLD;
            
            if (significantChange || timeForUpdate) {
                servo.setPosition(position);
                lastPosition = position;
                updateTimer.reset();
                return true;
            }
            return false;
        }
        
        public double getLastPosition() { return lastPosition; }
        public double getTimeSinceUpdate() { return updateTimer.seconds(); }
    }
    
    /**
     * Loop time monitoring for performance analysis
     */
    public void recordLoopTime(double loopTime) {
        loopCount++;
        totalCycles++;
        
        // Calculate running average
        averageLoopTime = (averageLoopTime * (loopCount - 1) + loopTime) / loopCount;
        
        // Reset statistics periodically to prevent overflow
        if (loopCount > 1000) {
            loopCount = 500; // Keep some history
            averageLoopTime = averageLoopTime; // Maintain current average
        }
    }
    
    /**
     * Get performance statistics
     */
    public PerformanceStats getPerformanceStats() {
        return new PerformanceStats(
            averageLoopTime,
            loopCount,
            totalCycles,
            bulkReadsEnabled,
            allHubs.size()
        );
    }
    
    /**
     * Force garbage collection (use sparingly)
     * Only call during non-critical periods like initialization
     */
    public static void forceGarbageCollection() {
        System.gc();
        System.runFinalization();
    }
    
    /**
     * Optimize thread priority for main OpMode thread
     * This is legal but should be used carefully
     */
    public static void optimizeThreadPriority() {
        Thread currentThread = Thread.currentThread();
        currentThread.setPriority(Thread.MAX_PRIORITY);
    }
    
    /**
     * Memory usage optimization hints
     */
    public static class MemoryOptimizations {
        
        /**
         * Reuse objects when possible to reduce garbage collection
         */
        public static void reuseObjects() {
            // Example: Pre-allocate arrays and reuse them
            // Instead of: double[] array = new double[4];
            // Use: reusableArray[0] = value; (where reusableArray is a field)
        }
        
        /**
         * Use primitive collections when possible
         * TIntArrayList instead of ArrayList<Integer>, etc.
         */
        public static void usePrimitiveCollections() {
            // Reduces boxing/unboxing overhead
        }
        
        /**
         * Minimize string concatenation in loops
         */
        public static StringBuilder getStringBuilder() {
            return new StringBuilder(256); // Pre-size for efficiency
        }
    }
    
    /**
     * Hardware-specific optimizations
     */
    public static class HardwareOptimizations {
        
        /**
         * Configure motors for optimal performance
         */
        public static void optimizeMotor(com.qualcomm.robotcore.hardware.DcMotorEx motor) {
            // Set motor to use internal PID for velocity control when possible
            motor.setZeroPowerBehavior(com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE);
            
            // Use RUN_WITHOUT_ENCODER for direct power control (fastest)
            // Use RUN_USING_ENCODER for velocity control with built-in PID
            motor.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        
        /**
         * Configure servos for optimal performance
         */
        public static void optimizeServo(com.qualcomm.robotcore.hardware.Servo servo) {
            // Servos are optimized by reducing update frequency (handled in OptimizedServoController)
        }
        
        /**
         * Configure sensors for bulk reads
         */
        public static void configureSensorForBulkReads(com.qualcomm.robotcore.hardware.HardwareDevice sensor) {
            // Sensors automatically benefit from bulk reads when using LynxModule caching
            // No additional configuration needed
        }
    }
    
    /**
     * Performance statistics data class
     */
    public static class PerformanceStats {
        public final double averageLoopTime;
        public final int loopCount;
        public final long totalCycles;
        public final boolean bulkReadsEnabled;
        public final int hubCount;
        
        public PerformanceStats(double averageLoopTime, int loopCount, long totalCycles, 
                              boolean bulkReadsEnabled, int hubCount) {
            this.averageLoopTime = averageLoopTime;
            this.loopCount = loopCount;
            this.totalCycles = totalCycles;
            this.bulkReadsEnabled = bulkReadsEnabled;
            this.hubCount = hubCount;
        }
        
        public double getFrequencyHz() {
            return averageLoopTime > 0 ? 1.0 / averageLoopTime : 0.0;
        }
        
        public boolean isPerformanceGood() {
            return averageLoopTime < 0.020; // Less than 20ms (50Hz)
        }
        
        public String getPerformanceRating() {
            double freq = getFrequencyHz();
            if (freq > 100) return "EXCELLENT";
            else if (freq > 50) return "GOOD";
            else if (freq > 25) return "FAIR";
            else return "POOR";
        }
    }
    
    /**
     * Emergency performance recovery
     * Call this if loop times become excessive
     */
    public void emergencyPerformanceRecovery() {
        // Force garbage collection
        forceGarbageCollection();
        
        // Reset performance counters
        loopCount = 0;
        averageLoopTime = 0.0;
        
        // Ensure bulk reads are working
        if (bulkReadsEnabled) {
            for (LynxModule module : allHubs) {
                module.clearBulkCache();
            }
        }
    }
    
    /**
     * Diagnostic information for troubleshooting
     */
    public String getDiagnosticInfo() {
        StringBuilder sb = new StringBuilder();
        sb.append("Performance Diagnostics:\n");
        sb.append("Bulk Reads: ").append(bulkReadsEnabled ? "ENABLED" : "DISABLED").append("\n");
        sb.append("Hub Count: ").append(allHubs.size()).append("\n");
        sb.append("Average Loop Time: ").append(String.format("%.2f ms", averageLoopTime * 1000)).append("\n");
        sb.append("Loop Frequency: ").append(String.format("%.1f Hz", 1.0 / averageLoopTime)).append("\n");
        sb.append("Total Cycles: ").append(totalCycles).append("\n");
        
        return sb.toString();
    }
}