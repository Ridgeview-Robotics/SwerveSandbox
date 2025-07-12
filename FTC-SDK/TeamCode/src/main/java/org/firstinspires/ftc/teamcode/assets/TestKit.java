package org.firstinspires.ftc.teamcode.assets;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TestKit")
public class TestKit extends OpMode {

    Module testModule;

    double Lx;
    double Ly;
    double solvedTarget;

    @Override
    public void init() { //Test Module Runs on "Front Left" Module
        testModule = new Module(hardwareMap, "flm", "fle", "fls");

        telemetry.addLine("Ready to Start!");
        telemetry.update();
    }

    @Override
    public void loop() {
        testModule.getEncPosition();

        Lx = gamepad1.left_stick_x;
        Ly = gamepad1.left_stick_y;
        solvedTarget = (Math.abs(Math.atan2(Ly, Lx))*2);
        testModule.setServoTarget(solvedTarget);

        //

        telemetry.addData("Target Angle", solvedTarget);
        telemetry.addData("Current Angle", testModule.getEncPosition());
        telemetry.addData("PID Output", testModule.getPIDCalc());
        telemetry.update();
    }
}
