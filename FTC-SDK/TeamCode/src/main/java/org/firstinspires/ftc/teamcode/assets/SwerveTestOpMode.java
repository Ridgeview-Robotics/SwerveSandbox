package org.firstinspires.ftc.teamcode.assets;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.assets.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.assets.SwerveModule;
import org.firstinspires.ftc.teamcode.assets.SwerveConfig;

@TeleOp(name="Swerve Test & Calibration", group="Swerve")
public class SwerveTestOpMode extends OpMode {
    
    private SwerveDrivetrain drivetrain;
    private FtcDashboard dashboard;
    private ElapsedTime runtime = new ElapsedTime();
    
    // Test mode variables
    private int selectedModule = 0; // 0=FL, 1=FR, 2=BL, 3=BR
    private boolean testMode = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastY = false;
    private boolean lastX = false;
    
    // Test angles for calibration
    private final double[] testAngles = {0, 45, 90, 135, 180, 225, 270, 315};
    private int currentTestAngle = 0;
    
    private String[] moduleNames = {"Front Left", "Front Right", "Back Left", "Back Right"};
    
    @Override
    public void init() {
        drivetrain = new SwerveDrivetrain(hardwareMap);
        dashboard = FtcDashboard.getInstance();
        runtime.reset();
        
        telemetry.addData("Status", "Initialized - Swerve Test Mode");
        telemetry.addData("Instructions", "Y = Test Mode, X = Exit Test Mode");
        telemetry.addData("Selected Module", moduleNames[selectedModule]);
        telemetry.update();
    }
    
    @Override
    public void init_loop() {
        drivetrain.updatePIDCoefficients();
        
        telemetry.addData("Status", "Ready - Use Y to enter test mode");
        telemetry.addData("Selected Module", moduleNames[selectedModule]);
        telemetry.update();
    }
    
    @Override
    public void start() {
        runtime.reset();
    }
    
    @Override
    public void loop() {
        handleTestModeControls();
        
        if (testMode) {
            runTestMode();
        } else {
            runNormalMode();
        }
        
        updateTelemetry();
        updateDashboard();
    }
    
    /**
     * Handle test mode control switching
     */
    private void handleTestModeControls() {
        // Enter test mode
        boolean currentY = gamepad1.y;
        if (currentY && !lastY) {
            testMode = true;
        }
        lastY = currentY;
        
        // Exit test mode
        boolean currentX = gamepad1.x;
        if (currentX && !lastX) {
            testMode = false;
            drivetrain.stop();
        }
        lastX = currentX;
        
        // Module selection
        boolean currentDpadLeft = gamepad1.dpad_left;
        if (currentDpadLeft && !lastDpadLeft) {
            selectedModule = (selectedModule - 1 + 4) % 4;
        }
        lastDpadLeft = currentDpadLeft;
        
        boolean currentDpadRight = gamepad1.dpad_right;
        if (currentDpadRight && !lastDpadRight) {
            selectedModule = (selectedModule + 1) % 4;
        }
        lastDpadRight = currentDpadRight;
    }
    
    /**
     * Run individual module testing
     */
    private void runTestMode() {
        SwerveModule module = getSelectedModule();
        
        // Angle control with right stick X
        double angleInput = gamepad1.right_stick_x;
        if (Math.abs(angleInput) > 0.1) {
            double targetAngle = (angleInput + 1.0) * 180.0; // 0-360 degrees
            module.setTargetAngle(targetAngle);
        }
        
        // Preset angles with dpad
        if (gamepad1.dpad_up) {
            currentTestAngle = (currentTestAngle + 1) % testAngles.length;
            module.setTargetAngle(testAngles[currentTestAngle]);
        }
        if (gamepad1.dpad_down) {
            currentTestAngle = (currentTestAngle - 1 + testAngles.length) % testAngles.length;
            module.setTargetAngle(testAngles[currentTestAngle]);
        }
        
        // Drive speed control with left stick Y
        double driveSpeed = -gamepad1.left_stick_y * 0.5; // Limited speed for testing
        module.setDriveSpeed(driveSpeed);
        
        // Update the module
        module.update(driveSpeed, module.getTargetAngle());
        
        // Stop other modules
        stopOtherModules();
    }
    
    /**
     * Run normal drive mode
     */
    private void runNormalMode() {
        double xSpeed = -gamepad1.left_stick_y * 0.7;
        double ySpeed = gamepad1.left_stick_x * 0.7;
        double rotation = gamepad1.right_stick_x * 0.7;
        
        drivetrain.drive(xSpeed, ySpeed, rotation);
    }
    
    /**
     * Get the currently selected module
     */
    private SwerveModule getSelectedModule() {
        switch (selectedModule) {
            case 0: return drivetrain.getFrontLeft();
            case 1: return drivetrain.getFrontRight();
            case 2: return drivetrain.getBackLeft();
            case 3: return drivetrain.getBackRight();
            default: return drivetrain.getFrontLeft();
        }
    }
    
    /**
     * Stop all modules except the selected one
     */
    private void stopOtherModules() {
        if (selectedModule != 0) drivetrain.getFrontLeft().setDriveSpeed(0);
        if (selectedModule != 1) drivetrain.getFrontRight().setDriveSpeed(0);
        if (selectedModule != 2) drivetrain.getBackLeft().setDriveSpeed(0);
        if (selectedModule != 3) drivetrain.getBackRight().setDriveSpeed(0);
    }
    
    /**
     * Update telemetry
     */
    private void updateTelemetry() {
        telemetry.addData("Status", testMode ? "TEST MODE" : "NORMAL MODE");
        telemetry.addData("Runtime", "%.1f sec", runtime.seconds());
        
        if (testMode) {
            SwerveModule module = getSelectedModule();
            telemetry.addLine("\n=== TEST MODE ===");
            telemetry.addData("Selected Module", moduleNames[selectedModule]);
            telemetry.addData("Current Angle", "%.1f°", module.getCurrentAngle());
            telemetry.addData("Target Angle", "%.1f°", module.getTargetAngle());
            telemetry.addData("Drive Power", "%.2f", module.getDrivePower());
            telemetry.addData("At Target", module.isAtTargetAngle() ? "YES" : "NO");
            telemetry.addData("Test Angle", "%.0f° (%d/%d)", 
                testAngles[currentTestAngle], currentTestAngle + 1, testAngles.length);
            
            telemetry.addLine("\nTest Controls:");
            telemetry.addData("Drive", "Left stick Y");
            telemetry.addData("Steer", "Right stick X");
            telemetry.addData("Next Test Angle", "DPad Up");
            telemetry.addData("Prev Test Angle", "DPad Down");
            telemetry.addData("Next Module", "DPad Right");
            telemetry.addData("Prev Module", "DPad Left");
            telemetry.addData("Exit Test", "X button");
        } else {
            telemetry.addLine("\n=== NORMAL MODE ===");
            SwerveDrivetrain.SwerveModuleState[] states = drivetrain.getModuleStates();
            telemetry.addData("FL", "Speed: %.2f, Angle: %.1f°", states[0].speed, states[0].angle);
            telemetry.addData("FR", "Speed: %.2f, Angle: %.1f°", states[1].speed, states[1].angle);
            telemetry.addData("BL", "Speed: %.2f, Angle: %.1f°", states[2].speed, states[2].angle);
            telemetry.addData("BR", "Speed: %.2f, Angle: %.1f°", states[3].speed, states[3].angle);
            
            telemetry.addLine("\nNormal Controls:");
            telemetry.addData("Drive", "Left stick");
            telemetry.addData("Rotate", "Right stick X");
            telemetry.addData("Enter Test", "Y button");
        }
        
        // Configuration display
        telemetry.addLine("\nPID Configuration:");
        telemetry.addData("kP", "%.3f", SwerveConfig.SWERVE_kP);
        telemetry.addData("kI", "%.3f", SwerveConfig.SWERVE_kI);
        telemetry.addData("kD", "%.3f", SwerveConfig.SWERVE_kD);
        telemetry.addData("Tolerance", "%.1f°", SwerveConfig.SERVO_TOLERANCE);
        
        telemetry.update();
    }
    
    /**
     * Update dashboard
     */
    private void updateDashboard() {
        TelemetryPacket packet = new TelemetryPacket();
        
        packet.put("Test Mode", testMode);
        packet.put("Selected Module", selectedModule);
        packet.put("Module Name", moduleNames[selectedModule]);
        
        if (testMode) {
            SwerveModule module = getSelectedModule();
            packet.put("Test Current Angle", module.getCurrentAngle());
            packet.put("Test Target Angle", module.getTargetAngle());
            packet.put("Test Drive Power", module.getDrivePower());
            packet.put("Test At Target", module.isAtTargetAngle());
            packet.put("Test Angle Index", currentTestAngle);
            packet.put("Test Angle Value", testAngles[currentTestAngle]);
        }
        
        // All module data for monitoring
        packet.put("FL Current", drivetrain.getFrontLeft().getCurrentAngle());
        packet.put("FR Current", drivetrain.getFrontRight().getCurrentAngle());
        packet.put("BL Current", drivetrain.getBackLeft().getCurrentAngle());
        packet.put("BR Current", drivetrain.getBackRight().getCurrentAngle());
        
        packet.put("FL Target", drivetrain.getFrontLeft().getTargetAngle());
        packet.put("FR Target", drivetrain.getFrontRight().getTargetAngle());
        packet.put("BL Target", drivetrain.getBackLeft().getTargetAngle());
        packet.put("BR Target", drivetrain.getBackRight().getTargetAngle());
        
        // Configuration values
        packet.put("Config kP", SwerveConfig.SWERVE_kP);
        packet.put("Config kI", SwerveConfig.SWERVE_kI);
        packet.put("Config kD", SwerveConfig.SWERVE_kD);
        packet.put("Config Tolerance", SwerveConfig.SERVO_TOLERANCE);
        
        dashboard.sendTelemetryPacket(packet);
    }
    
    @Override
    public void stop() {
        drivetrain.stop();
    }
}