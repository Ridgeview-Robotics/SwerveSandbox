package org.firstinspires.ftc.teamcode.assets;


import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain {

    Module FL;
    Module BL;
    Module BR;
    Module FR;

    public String[] fl = new String[]{"flm", "fle", "fls"};
    public String[] bl = new String[]{"blm", "ble", "bls"};
    public String[] br = new String[]{"brm", "bre", "brs"};
    public String[] fr = new String[]{"frm", "fre", "frs"};

    public Drivetrain(HardwareMap hardwareMap) {
        FL = new Module(hardwareMap, fl[0], fl[1], fl[2]);
        BL = new Module(hardwareMap, bl[0], bl[1], bl[2]);
        BR = new Module(hardwareMap, br[0], br[1], br[2]);
        FR = new Module(hardwareMap, fr[0], fr[1], fr[2]);
    }

    public void update(double Lx, double Ly, double Rx){
        FL.solve(Lx, Ly, Rx);
        BL.solve(Lx, Ly, Rx);
        BR.solve(Lx, Ly, Rx);
        FR.solve(Lx, Ly, Rx);
    }


}
