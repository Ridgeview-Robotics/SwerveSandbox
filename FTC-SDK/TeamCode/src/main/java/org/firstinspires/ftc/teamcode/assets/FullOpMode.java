package org.firstinspires.ftc.teamcode.assets;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

public class FullOpMode extends OpMode {

    Drivetrain d;

    @Override
    public void init() {
        d = new Drivetrain(hardwareMap);

        telemetry.addLine("ready to start!");
        telemetry.update();
    }

    @Override
    public void loop() {
        double Lx = gamepad1.left_stick_x;
        double Ly = gamepad1.left_stick_y;
        double Rx = gamepad1.right_stick_x;
        d.update(Lx, Ly, Rx);


    }
}
