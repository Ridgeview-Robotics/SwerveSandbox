package org.firstinspires.ftc.teamcode.assets;

public class Drivetrain {

    Module[] modules;

    public Drivetrain(Module[] modules) {
        this.modules = modules;
    }

    public void setModulePowers(double FL, double FR, double BL, double BR){
        modules[0].setMotorPower(FL);
        modules[1].setMotorPower(FR);
        modules[2].setMotorPower(BL);
        modules[3].setMotorPower(BR);
    }

    public void setModuleAngles(double FL, double FR, double BL, double BR){
        modules[0].setServoPower(FL);
        modules[1].setServoPower(FR);
        modules[2].setServoPower(BL);
        modules[3].setServoPower(BR);
    }

}
