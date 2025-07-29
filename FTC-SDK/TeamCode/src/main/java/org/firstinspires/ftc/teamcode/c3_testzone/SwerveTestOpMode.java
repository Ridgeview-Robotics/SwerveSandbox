package org.firstinspires.ftc.teamcode.c3_testzone;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Swerve Test & Calibration FIXED", group="Swerve")
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

    // Test angles for calibration - NOW IN RADIANS
    private final double[] testAngles = {
            0,                    // 0°
            Math.PI / 4,         // 45°
            Math.PI / 2,         // 90°
            3 * Math.PI / 4,     // 135°
            Math.PI,             // 180°
            5 * Math.PI / 4,     // 225°
            3 * Math.PI / 2,     // 270°
            7 * Math.PI / 4      // 315°
    };
    private int currentTestAngle = 0;

    private String[] moduleNames = {"Front Left", "Front Right", "Back Left", "Back Right"};

    @Override
    public void init() {
        drivetrain = new SwerveDrivetrain(hardwareMap);
        
        // Force dashboard initialization
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25); // 40Hz update rate
        
        // Clear any existing packets
        TelemetryPacket packet = new TelemetryPacket();
        dashboard.sendTelemetryPacket(packet);
        
        runtime.reset();
        
        telemetry.addData("Status", "Initialized - Swerve Test Mode FIXED");
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
    
        // Reduce angle control sensitivity and add deadzone
        double angleInput = gamepad1.right_stick_x;
        if (Math.abs(angleInput) > 0.15) { // Increased deadzone
            double targetAngle = (angleInput + 1.0) * Math.PI;
            // Add smoothing to prevent sudden movements
            double currentAngle = module.getCurrentAngle();
            double smoothedAngle = currentAngle + Math.signum(targetAngle - currentAngle) * 0.1;
            module.setTargetAngle(smoothedAngle);
        }
    
        // Reduce drive speed even further for testing
        double driveSpeed = -gamepad1.left_stick_y * 0.3; // Reduced from 0.5 to 0.3
        module.setDriveSpeed(driveSpeed);


        // Preset angles with dpad - NOW USING RADIANS
        if (gamepad1.dpad_up) {
            currentTestAngle = (currentTestAngle + 1) % testAngles.length;
            module.setTargetAngle(testAngles[currentTestAngle]);
        }
        if (gamepad1.dpad_down) {
            currentTestAngle = (currentTestAngle - 1 + testAngles.length) % testAngles.length;
            module.setTargetAngle(testAngles[currentTestAngle]);
        }

        // Drive speed control with left stick Y
        driveSpeed = -gamepad1.left_stick_y * 0.5; // Limited speed for testing
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
            telemetry.addData("Current Angle", "%.1f° (%.3f rad)",
                    Math.toDegrees(module.getCurrentAngle()), module.getCurrentAngle());
            telemetry.addData("Target Angle", "%.1f° (%.3f rad)",
                    Math.toDegrees(module.getTargetAngle()), module.getTargetAngle());
            telemetry.addData("Drive Power", "%.2f", module.getDrivePower());
            telemetry.addData("Servo Power", "%.3f", module.getServoPower());
            telemetry.addData("Raw Encoder V", "%.3f", module.getRawEncoderVoltage());
            telemetry.addData("At Target", module.isAtTargetAngle() ? "YES" : "NO");
            telemetry.addData("Test Angle", "%.0f° (%d/%d)",
                    Math.toDegrees(testAngles[currentTestAngle]), currentTestAngle + 1, testAngles.length);

            // DEBUG: Show angle error
            double angleError = module.getTargetAngle() - module.getCurrentAngle();
            while (angleError > Math.PI) angleError -= 2 * Math.PI;
            while (angleError < -Math.PI) angleError += 2 * Math.PI;
            telemetry.addData("Angle Error", "%.1f° (%.3f rad)", Math.toDegrees(angleError), angleError);

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
            SwerveDrivetrain.SwerveModuleState[] states = drivetrain.getModuleStatesDegrees();
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
        telemetry.addData("Tolerance", "%.1f° (%.3f rad)",
                Math.toDegrees(SwerveConfig.SERVO_TOLERANCE), SwerveConfig.SERVO_TOLERANCE);
        telemetry.addData("Servo Deadband", "%.3f", SwerveConfig.SERVO_DEADBAND);

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
            packet.put("Test Current Angle (deg)", Math.toDegrees(module.getCurrentAngle()));
            packet.put("Test Target Angle (deg)", Math.toDegrees(module.getTargetAngle()));
            packet.put("Test Current Angle (rad)", module.getCurrentAngle());
            packet.put("Test Target Angle (rad)", module.getTargetAngle());
            packet.put("Test Drive Power", module.getDrivePower());
            packet.put("Test Servo Power", module.getServoPower());
            packet.put("Test Raw Encoder", module.getRawEncoderVoltage());
            packet.put("Test At Target", module.isAtTargetAngle());
            packet.put("Test Angle Index", currentTestAngle);
            packet.put("Test Angle Value (deg)", Math.toDegrees(testAngles[currentTestAngle]));
        }

        // All module data for monitoring
        packet.put("FL Current (deg)", Math.toDegrees(drivetrain.getFrontLeft().getCurrentAngle()));
        packet.put("FR Current (deg)", Math.toDegrees(drivetrain.getFrontRight().getCurrentAngle()));
        packet.put("BL Current (deg)", Math.toDegrees(drivetrain.getBackLeft().getCurrentAngle()));
        packet.put("BR Current (deg)", Math.toDegrees(drivetrain.getBackRight().getCurrentAngle()));

        packet.put("FL Target (deg)", Math.toDegrees(drivetrain.getFrontLeft().getTargetAngle()));
        packet.put("FR Target (deg)", Math.toDegrees(drivetrain.getFrontRight().getTargetAngle()));
        packet.put("BL Target (deg)", Math.toDegrees(drivetrain.getBackLeft().getTargetAngle()));
        packet.put("BR Target (deg)", Math.toDegrees(drivetrain.getBackRight().getTargetAngle()));

        // Raw encoder voltages for all modules
        packet.put("FL Raw V", drivetrain.getFrontLeft().getRawEncoderVoltage());
        packet.put("FR Raw V", drivetrain.getFrontRight().getRawEncoderVoltage());
        packet.put("BL Raw V", drivetrain.getBackLeft().getRawEncoderVoltage());
        packet.put("BR Raw V", drivetrain.getBackRight().getRawEncoderVoltage());

        // Configuration values
        packet.put("Config kP", SwerveConfig.SWERVE_kP);
        packet.put("Config kI", SwerveConfig.SWERVE_kI);
        packet.put("Config kD", SwerveConfig.SWERVE_kD);
        packet.put("Config Tolerance", SwerveConfig.SERVO_TOLERANCE);
        packet.put("Config Deadband", SwerveConfig.SERVO_DEADBAND);

        dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void stop() {
        drivetrain.stop();
    }
}