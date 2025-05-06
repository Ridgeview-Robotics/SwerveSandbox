package org.firstinspires.ftc.teamcode.assets;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

public class SwerveAE {
    public final AnalogInput encoder;
    public double offset;
    private final double radianMultiplier = 1.90399554763;
    public double analogRange = 3.3;
    public boolean inverted;
    

    public SwerveAE(AnalogInput encInput){
        encoder = encInput;
        offset = 0;
        inverted = false;
    }
    public SwerveAE zero(double off){
        offset = off;
        return this;
    }
    public SwerveAE setInverted(boolean invert){
        inverted = invert;
        return this;
    }

    public double getEncoderRad(){
        return getVoltage() * radianMultiplier;
    }

    public double getVoltage(){
        return encoder.getVoltage();
    }

}
