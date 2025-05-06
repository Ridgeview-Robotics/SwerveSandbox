package org.firstinspires.ftc.teamcode.assets;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Module {

    private final Swervo s;
    private final DcMotorEx m;

    public Module(HardwareMap hardwareMap, String mn, String en, String sn){
        m = hardwareMap.get(DcMotorEx.class, mn);
        s = new Swervo(hardwareMap, sn, en);
    }

    public void setMotorPower(double power){
        m.setPower(power);
    }

    public double getEncPosition(){
        return s.getPosition();
    }

    public void setServoPower(double power){
        s.setPower(power);
    }

    public void setServoTarget(double targ){
        e.set
    }

    


}
