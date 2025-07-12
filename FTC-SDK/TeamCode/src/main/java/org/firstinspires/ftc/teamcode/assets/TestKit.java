package org.firstinspires.ftc.teamcode.assets;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class TestKit extends OpMode {

    Module testModule;

    double Lx;
    double Ly;
    double solvedTarget;

    @Override
    public void init() { //Test Module Runs on "Front Left" Module
        testModule = new Module(hardwareMap, "fl3m", "fle", "fls");

        telemetry.addLine("Ready to Start!");
        telemetry.update();
    }

    @Override
    public void loop() {
        Lx = gamepad1.left_stick_x;
        Ly = gamepad1.left_stick_y;
        solvedTarget = Math.hypot(Lx, Ly);
        testModule.setServoTarget(solvedTarget);

        //

        telemetry.addData("Target Angle", solvedTarget);
        telemetry.addData("Current Angle", testModule.getEncPosition());
        telemetry.update();
    }
}
