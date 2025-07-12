package org.firstinspires.ftc.teamcode.assets;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Module {

    private final Swervo s;
    private final DcMotorEx m;
    private final PIDC pid;
    public static double p = 1;
    public static double i = 0.0001;
    public static double d = 0.001;

    private double mNwb;
    private double mNtw;
    private double x;
    private double y;
    private double tA;
    private double tV;
    private double solvedTarget;

    public Module(HardwareMap hardwareMap, String mn, String en, String sn){
        m = hardwareMap.get(DcMotorEx.class, mn);
        s = new Swervo(hardwareMap, sn, en);
        pid = new PIDC(p, i, d);
    }

    public void setMotorPower(double power){
        m.setPower(power);
    }

    public void setServoPower(double power){
        s.setPower(power);
    }

    public double getEncPosition(){
        return s.getPosition();
    }

    public double getPIDCalc(){
        return pid.calculate(getEncPosition());
    }

    public void setServoTarget(double targ){
        solvedTarget = targ;
        pid.setTarg(solvedTarget);
        s.setPower(getPIDCalc());
    }


    public void setValues(double ta, double tv){
        setServoTarget(ta);
        setMotorPower(tv);
    }


    public void solve(double Lx, double Ly, double Rx){
        double vRotX = -Rx * y; // Perpendicular to radius
        double vRotY = Rx * x; // Tangential to radius

        double vTotalX = Lx + vRotX;
        double vTotalY = Ly + vRotY;

        double Angle= Math.atan2(vTotalY, vTotalX) - Math.PI/2;
        double Speed= Math.sqrt((vTotalX * vTotalX) + (vTotalY * vTotalY));

        if(Ly > 0){
            m.setDirection(DcMotorEx.Direction.REVERSE);
        }

        tA = Angle;
        tV = Speed;

        setValues(tA, tV);
    }







    


}
