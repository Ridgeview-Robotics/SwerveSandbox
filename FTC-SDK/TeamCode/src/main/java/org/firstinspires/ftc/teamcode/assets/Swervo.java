package org.firstinspires.ftc.teamcode.assets;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class Swervo {
    CRServo s;
    SwerveAE e;

    public Swervo(HardwareMap hardwareMap, String sn, String en){
        s = hardwareMap.get(CRServo.class, sn);
        e = new SwerveAE(hardwareMap.analogInput.get(en));
    }

    public void setPower(double power){
        Range.clip(power, -1, 1);
        s.setPower(power);
    }

    public double getPosition(){
        return e.getEncoderRad();
    }
}
