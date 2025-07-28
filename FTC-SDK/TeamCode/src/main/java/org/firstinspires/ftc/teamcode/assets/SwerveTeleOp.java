package org.firstinspires.ftc.teamcode.assets;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.assets.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.assets.SwerveConfig;

@TeleOp(name="Swerve Drive TeleOp", group="Swerve")
public class SwerveTeleOp extends OpMode {
    
    private SwerveDrivetrain drivetrain;
    private FtcDashboard dashboard;
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime loopTimer = new ElapsedTime();
    
    // Performance monitoring
    private double averageLoopTime = 0.0;
    private int loopCount = 0;
    private double maxLoopTime = 0.0;
    
    // Control mode flags
    private boolean fieldCentric = false;
    private boolean slowMode = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    
    @Override
    public void init() {
        // Initialize drivetrain
        drivetrain = new SwerveDrivetrain(hardwareMap);
        
        // Initialize dashboard
        dashboard = FtcDashboard.getInstance();
        
        // Reset timers
        runtime.reset();
        loopTimer.reset();
        
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Mode", "Robot Centric");
        telemetry.update();
    }
    
    @Override
    public void init_loop() {
        // Update PID coefficients in case they were changed on dashboard
        drivetrain.updatePIDCoefficients();
        
        telemetry.addData("Status", "Ready to Start");
        telemetry.addData("Runtime", "%.1f seconds", runtime.seconds());
        telemetry.update();
    }
    
    @Override
    public void start() {
        runtime.reset();
        loopTimer.reset();
    }
    
    @Override
    public void loop() {
        // Performance monitoring
        double loopTime = loopTimer.seconds();
        loopTimer.reset();
        
        updateLoopTimeStats(loopTime);
        
        // Handle control mode switching
        handleControlModes();
        
        // Get drive inputs with optional speed scaling
        double speedMultiplier = slowMode ? 0.3 : 1.0;
        double xSpeed = -gamepad1.left_stick_y * speedMultiplier;
        double ySpeed = gamepad1.left_stick_x * speedMultiplier;
        double rotation = gamepad1.right_stick_x * speedMultiplier;
        
        // Apply exponential curve for finer control (optional)
        if (gamepad1.left_bumper) {
            xSpeed = applyExponentialCurve(xSpeed, 2.0);
            ySpeed = applyExponentialCurve(ySpeed, 2.0);
            rotation = applyExponentialCurve(rotation, 2.0);
        }
        
        // Drive the robot
        drivetrain.drive(xSpeed, ySpeed, rotation);
        
        // Update dashboard and telemetry
        updateTelemetry();
        updateDashboard();
    }
    
    @Override
    public void stop() {
        drivetrain.stop();
    }
    
    /**
     * Handle control mode switching
     */
    private void handleControlModes() {
        // Toggle field centric mode (currently robot centric only)
        boolean currentDpadUp = gamepad1.dpad_up;
        if (currentDpadUp && !lastDpadUp) {
            fieldCentric = !fieldCentric;
        }
        lastDpadUp = currentDpadUp;
        
        // Toggle slow mode
        boolean currentDpadDown = gamepad1.dpad_down;
        if (currentDpadDown && !lastDpadDown) {
            slowMode = !slowMode;
        }
        lastDpadDown = currentDpadDown;
        
        // Emergency stop
        if (gamepad1.start && gamepad1.back) {
            drivetrain.stop();
        }
    }
    
    /**
     * Apply exponential curve to input for finer control
     */
    private double applyExponentialCurve(double input, double exponent) {
        return Math.signum(input) * Math.pow(Math.abs(input), exponent);
    }
    
    /**
     * Update loop time statistics
     */
    private void updateLoopTimeStats(double loopTime) {
        loopCount++;
        averageLoopTime = (averageLoopTime * (loopCount - 1) + loopTime) / loopCount;
        maxLoopTime = Math.max(maxLoopTime, loopTime);
    }
    
    /**
     * Update telemetry data
     */
    private void updateTelemetry() {
        // Basic status
        telemetry.addData("Status", "Running");
        telemetry.addData("Runtime", "%.1f sec", runtime.seconds());
        
        // Control modes
        telemetry.addData("Drive Mode", fieldCentric ? "Field Centric" : "Robot Centric");
        telemetry.addData("Speed Mode", slowMode ? "SLOW" : "NORMAL");
        
        // Performance metrics
        telemetry.addData("Loop Time", "%.1f ms (avg: %.1f, max: %.1f)", 
            loopTimer.milliseconds(), 
            averageLoopTime * 1000, 
            maxLoopTime * 1000);
        
        // Drive inputs
        telemetry.addData("Drive Input", "X: %.2f, Y: %.2f, Rot: %.2f",
            -gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        
        // Module states
        SwerveDrivetrain.SwerveModuleState[] states = drivetrain.getModuleStates();
        telemetry.addLine("\nModule States:");
        telemetry.addData("FL", "Speed: %.2f, Angle: %.1f°", states[0].speed, states[0].angle);
        telemetry.addData("FR", "Speed: %.2f, Angle: %.1f°", states[1].speed, states[1].angle);
        telemetry.addData("BL", "Speed: %.2f, Angle: %.1f°", states[2].speed, states[2].angle);
        telemetry.addData("BR", "Speed: %.2f, Angle: %.1f°", states[3].speed, states[3].angle);
        
        // Configuration values
        telemetry.addLine("\nConfig (Live from Dashboard):");
        telemetry.addData("Drive PID", "P: %.3f, I: %.3f, D: %.3f", 
            SwerveConfig.SWERVE_kP, SwerveConfig.SWERVE_kI, SwerveConfig.SWERVE_kD);
        telemetry.addData("Max Speeds", "Drive: %.1f, Turn: %.1f", 
            SwerveConfig.MAX_DRIVE_POWER, SwerveConfig.MAX_TURN_POWER);
        
        // Controls help
        telemetry.addLine("\nControls:");
        telemetry.addData("Drive", "Left stick");
        telemetry.addData("Rotate", "Right stick X");
        telemetry.addData("Precision", "Left bumper");
        telemetry.addData("Slow Mode", "DPad Down");
        telemetry.addData("Emergency Stop", "Start + Back");
        
        telemetry.update();
    }
    
    /**
     * Update FTC Dashboard
     */
    private void updateDashboard() {
        TelemetryPacket packet = new TelemetryPacket();
        
        // Performance data
        packet.put("Loop Time (ms)", loopTimer.milliseconds());
        packet.put("Average Loop Time (ms)", averageLoopTime * 1000);
        packet.put("Max Loop Time (ms)", maxLoopTime * 1000);
        
        // Drive inputs
        packet.put("X Speed", -gamepad1.left_stick_y);
        packet.put("Y Speed", gamepad1.left_stick_x);
        packet.put("Rotation", gamepad1.right_stick_x);
        
        // Module data
        SwerveDrivetrain.SwerveModuleState[] states = drivetrain.getModuleStates();
        packet.put("FL Speed", states[0].speed);
        packet.put("FL Angle", states[0].angle);
        packet.put("FR Speed", states[1].speed);
        packet.put("FR Angle", states[1].angle);
        packet.put("BL Speed", states[2].speed);
        packet.put("BL Angle", states[2].angle);
        packet.put("BR Speed", states[3].speed);
        packet.put("BR Angle", states[3].angle);
        
        // Control modes
        packet.put("Field Centric", fieldCentric);  
        packet.put("Slow Mode", slowMode);
        
        // Module angles for visualization
        packet.put("FL Current Angle", drivetrain.getFrontLeft().getCurrentAngle());
        packet.put("FR Current Angle", drivetrain.getFrontRight().getCurrentAngle());
        packet.put("BL Current Angle", drivetrain.getBackLeft().getCurrentAngle());
        packet.put("BR Current Angle", drivetrain.getBackRight().getCurrentAngle());
        
        packet.put("FL Target Angle", drivetrain.getFrontLeft().getTargetAngle());
        packet.put("FR Target Angle", drivetrain.getFrontRight().getTargetAngle());
        packet.put("BL Target Angle", drivetrain.getBackLeft().getTargetAngle());
        packet.put("BR Target Angle", drivetrain.getBackRight().getTargetAngle());
        
        dashboard.sendTelemetryPacket(packet);
    }
}